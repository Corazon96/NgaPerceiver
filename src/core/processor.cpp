#include "core/processor.h"
#include "core/config.h"
#include "core/logger.h"

#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <iostream>
#include <memory>
#include <chrono>

static const size_t BATCH_SIZE = 32; // 每次处理多个帧以提高吞吐

PointCloudProcessor::PointCloudProcessor() : running_(false), count_(0)
{
	// worker线程在 start() 中启动
}

PointCloudProcessor::~PointCloudProcessor()
{
	stop();
}

void PointCloudProcessor::addFilter(Linger::FilterPtr filter, bool is_post_process)
{
	std::lock_guard<std::mutex> lock(filter_mutex_);
	if (is_post_process) {
		post_filters_.push_back(filter);
	} else {
		pre_filters_.push_back(filter);
	}
}

void PointCloudProcessor::clearFilters()
{
	std::lock_guard<std::mutex> lock(filter_mutex_);
	pre_filters_.clear();
	post_filters_.clear();
}

void PointCloudProcessor::start()
{
	if (running_.load())
		return;
	running_ = true;
	worker_ = std::thread([this]()
						  {
		// 每秒聚合用于诊断
		uint64_t s_frames = 0;
		uint64_t s_in_points = 0;
		uint64_t s_published = 0;
		auto last_log = std::chrono::steady_clock::now();

		/**
		 * @brief 设置为 30ms (约 33Hz)。
		 * 略高于 30Hz 的生产速率可以补偿系统抖动，确保 Renderer 总是有数据可取。
		 * 配合 Renderer 的异步丢帧机制，可以实现流畅的 30 FPS。
		 */
		const std::chrono::milliseconds publish_interval_ms(LingerConfig::PROCESSOR_PUBLISH_INTERVAL_MS); 
		auto last_publish = std::chrono::steady_clock::now();

		while (running_) {
			std::pair<PointCloudPtr, Pose> item;
			{
				std::unique_lock<std::mutex> lock(mutex_);
				cond_.wait(lock, [this] { return count_.load() >0 || !running_.load(); });
				if (!running_ && count_.load() ==0) break;
				if (!queue_.pop(item)) {
					continue;
				}
				// 成功 pop 一项，调整计数
				count_.fetch_sub(1, std::memory_order_relaxed);
			}

			// 批量收集（非阻塞）
			std::vector<std::pair<PointCloudPtr, Pose>> batch;
			batch.reserve(BATCH_SIZE);
			batch.push_back(item);
			for (size_t b =1; b < BATCH_SIZE; ++b) {
				std::pair<PointCloudPtr, Pose> it;
				if (queue_.pop(it)) {
					batch.push_back(it);
					count_.fetch_sub(1, std::memory_order_relaxed);
				} else break;
			}

			bool do_publish = (std::chrono::steady_clock::now() - last_publish) >= publish_interval_ms;

			// 复制滤波器列表以避免在循环中长时间持有锁
			std::vector<Linger::FilterPtr> current_pre_filters;
			std::vector<Linger::FilterPtr> current_post_filters;
			{
				std::lock_guard<std::mutex> lock(filter_mutex_);
				current_pre_filters = pre_filters_;
				current_post_filters = post_filters_;
			}

			// 始终使用原始模式：转换点并追加到 raw_buffer_
			for (auto &pr : batch) {
				s_frames++;
				PointCloudPtr pc = pr.first; // 注意：这里可能需要修改 pc，所以不用 const&
				const Pose& pose = pr.second;
				if (!pc) continue;
				s_in_points += pc->points.size();

				// 1. 预处理滤波器（传感器帧）
				// 例如：距离过滤、去噪
				size_t frame_filtered = 0;
				for (auto& filter : current_pre_filters) {
					// 原地过滤
					filter->filter(pc, pc);
					frame_filtered += filter->getLastFilteredCount();
				}
				// if (pc->empty()) continue; // 不要丢弃空帧，否则会丢失 filtered_count 统计

				uint64_t frame_ts = pose.timestamp_ns;
				if (frame_ts ==0) {
					auto now = std::chrono::system_clock::now().time_since_epoch();
					frame_ts = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
				}

				// 根据配置决定是否应用姿态变换
				PointCloudPtr transformed_pc;
				if (LingerConfig::ENABLE_POSE_TRANSFORM) {
					// 将点云从雷达坐标系变换到世界坐标系
					float qx = pose.qx, qy = pose.qy, qz = pose.qz, qw = pose.qw;
					float tx = pose.x, ty = pose.y, tz = pose.z;
					auto rotate_translate = [&](const Point& p, Point& out) {
						float vx = p.x; float vy = p.y; float vz = p.z;
						float t_x =2.0f * (qy * vz - qz * vy);
						float t_y =2.0f * (qz * vx - qx * vz);
						float t_z =2.0f * (qx * vy - qy * vx);
						float c_x = qy * t_z - qz * t_y;
						float c_y = qz * t_x - qx * t_z;
						float c_z = qx * t_y - qy * t_x;
						out.x = vx + qw * t_x + c_x + tx;
						out.y = vy + qw * t_y + c_y + ty;
						out.z = vz + qw * t_z + c_z + tz;
						out.intensity = p.intensity;
					};

					transformed_pc = std::make_shared<pcl::PointCloud<Point>>();
					size_t num_points = pc->points.size();
					transformed_pc->points.resize(num_points);
					
					for (size_t i = 0; i < num_points; ++i) {
						rotate_translate(pc->points[i], transformed_pc->points[i]);
					}
					transformed_pc->width = static_cast<uint32_t>(num_points);
					transformed_pc->height = 1;
				} else {
					// 不进行姿态变换，直接使用原始点云（雷达本地坐标系）
					transformed_pc = std::make_shared<pcl::PointCloud<Point>>(*pc);
				}

				// 3. 后处理滤波器（世界帧）
				// 例如：ROI 裁剪、高度过滤
				for (auto& filter : current_post_filters) {
					filter->filter(transformed_pc, transformed_pc);
					frame_filtered += filter->getLastFilteredCount();
				}
				// if (transformed_pc->empty()) continue; // 不要丢弃空帧，否则会丢失 filtered_count 统计

				// 追加到帧缓冲区
				{
					std::lock_guard<std::mutex> lock(raw_mutex_);
					frame_buffer_.push_back({transformed_pc, frame_ts, frame_filtered});
				}
			}

			// 清理旧帧
			{
				std::lock_guard<std::mutex> lk(raw_mutex_);
				auto now_ns = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
				
				// 从队列前端移除过期的帧
				while (!frame_buffer_.empty()) {
					if (now_ns > frame_buffer_.front().timestamp_ns && (now_ns - frame_buffer_.front().timestamp_ns) > retention_ns_.load()) {
						frame_buffer_.pop_front();
					} else {
						break;
					}
				}
			}

			if (do_publish) {
				// 从 frame_buffer_ 构建要发布的快照列表，并应用保留策略
				std::vector<PointCloudPtr> published_snapshot;
				size_t total_points = 0;
				size_t total_filtered = 0;
				{
					std::lock_guard<std::mutex> lk(raw_mutex_);
					
					// 收集有效帧
					published_snapshot.reserve(frame_buffer_.size());
					for (const auto& frame : frame_buffer_) {
						published_snapshot.push_back(frame.cloud);
						total_points += frame.cloud->points.size();
						total_filtered += frame.filtered_count;
					}
				}

				// 安全地发布
				{
					std::lock_guard<std::mutex> lk(snapshot_mutex_);
					latest_snapshot_ = published_snapshot;
				}

				try {
					if (mapUpdateCallback_) 
					    mapUpdateCallback_(published_snapshot, total_filtered);
				} catch (const std::exception &e) {
					LOG_ERROR("PointCloudProcessor: mapUpdateCallback threw: {}", e.what());
				} catch (...) {
					LOG_ERROR("PointCloudProcessor: mapUpdateCallback threw unknown exception");
				}

				last_publish = std::chrono::steady_clock::now();
				s_published += total_points;
			}

			// 周期性打印诊断信息
			auto now = std::chrono::steady_clock::now();
			auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count();
			if (elapsed >=1000) {
				uint64_t frames = s_frames; s_frames = 0;
				uint64_t in_pts = s_in_points; s_in_points = 0;
				uint64_t pub = s_published; s_published = 0;
				LOG_DEBUG("[Processor Stats] frames/s={} in_pts/s={} published_pts/s={}", frames, in_pts, pub);
				last_log = now;
			}
		} });
}

void PointCloudProcessor::stop()
{
	if (!running_)
		return;
	
	running_ = false;
	cond_.notify_all();
	if (worker_.joinable())
		worker_.join();
}

void PointCloudProcessor::enqueue(PointCloudPtr pc, const Pose &pose)
{
	if (!running_)
		return;
	
	std::pair<PointCloudPtr, Pose> q{pc, pose};
	if (queue_.push(q))
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			count_.fetch_add(1, std::memory_order_relaxed);
		}
		cond_.notify_one();
		return;
	}
	// 队列满，丢弃新帧（使用限流宏，每秒最多打印一次）
	LOG_WARN_THROTTLED(1000, "[Processor] Frame queue full. Dropping frame. (Queue size: {})", queue_.capacity());
}

void PointCloudProcessor::setMapUpdateCallback(std::function<void(const std::vector<PointCloudPtr>&, size_t)> cb)
{
	mapUpdateCallback_ = cb;
}

// 设置点保留时间（秒）
void PointCloudProcessor::setRetentionSeconds(double s)
{
	if (s < 0.0)
		return;
	uint64_t ns = static_cast<uint64_t>(s * 1e9);
	retention_ns_.store(ns);
}

double PointCloudProcessor::getRetentionSeconds() const
{
	uint64_t ns = retention_ns_.load();
	return static_cast<double>(ns) / 1e9;
}

std::vector<PointCloudPtr> PointCloudProcessor::takeLatestSnapshot()
{
	std::lock_guard<std::mutex> lk(snapshot_mutex_);
	return latest_snapshot_;
}
