#include "acquisition/streamer.h"
#include "core/logger.h"
#include "core/config.h"
#include <cstring>
#include <filesystem>

static const char kMagic[] = "LGV1";
static const uint32_t kVersion = 1;

Streamer::Streamer() {}

Streamer::~Streamer() {
	closeWrite();
	closeRead();
}

bool Streamer::openWrite(const std::string& filepath) {
	std::lock_guard<std::mutex> lk(file_mutex_);
	if (is_writing_.load() || is_reading_.load()) return false;

	// 使用 u8path 支持 Windows 下的 UTF-8 路径
	ofs_.open(std::filesystem::u8path(filepath), std::ios::binary | std::ios::out | std::ios::trunc);
	if (!ofs_.is_open()) return false;

	// 写入文件头
	FileHeader header;
	std::memcpy(header.magic, kMagic, 4);
	header.version = kVersion;
	ofs_.write(reinterpret_cast<const char*>(&header), sizeof(FileHeader));

	is_writing_ = true;
	// 启动写入线程
	record_thread_ = std::thread(&Streamer::recordWorker_, this);
	return true;
}

void Streamer::writeFrame(const PointCloudPtr& cloud, const Pose& pose) {
	if (!is_writing_ || !cloud) return;

	{
		std::lock_guard<std::mutex> lk(queue_mutex_);
		// 简单的背压控制：如果队列过大（例如磁盘太慢），丢弃旧帧以避免内存爆满
		if (write_queue_.size() > LingerConfig::STREAMER_QUEUE_SIZE) {
			write_queue_.pop_front();
			LOG_WARN("[Streamer] Warning: Write queue full, dropping frame.");
			if (onWriteQueueFull) {
				onWriteQueueFull();
			}
		}
		write_queue_.push_back({cloud, pose});
	}
	queue_cv_.notify_one();
}

void Streamer::recordWorker_() {
	while (true) {
		WriteTask task;
		{
			std::unique_lock<std::mutex> lk(queue_mutex_);
			queue_cv_.wait(lk, [this] { return !write_queue_.empty() || !is_writing_; });
			
			if (write_queue_.empty() && !is_writing_) {
				break; // 停止且队列空
			}
			
			if (write_queue_.empty()) continue; // 虚假唤醒

			task = write_queue_.front();
			write_queue_.pop_front();
		}

		// 执行实际写入（无需持有 queue_mutex，但需持有 file_mutex 保护 ofs_）
		std::lock_guard<std::mutex> lk(file_mutex_);
		if (!ofs_.is_open()) break;

		// 1. 写入 Pose
		ofs_.write(reinterpret_cast<const char*>(&task.pose), sizeof(Pose));

		// 2. 写入点数
		uint32_t count = static_cast<uint32_t>(task.cloud->points.size());
		ofs_.write(reinterpret_cast<const char*>(&count), sizeof(uint32_t));

		// 3. 写入点数据
		if (count > 0) {
			buffer_.clear();
			if (buffer_.capacity() < count) {
				buffer_.reserve(count);
			}
			for (const auto& p : task.cloud->points) {
				buffer_.push_back({p.x, p.y, p.z, p.intensity});
			}
			ofs_.write(reinterpret_cast<const char*>(buffer_.data()), count * sizeof(PackedPoint));
		}
	}
}

void Streamer::closeWrite() {
	// 1. 标记停止
	is_writing_ = false;
	queue_cv_.notify_all();

	// 2. 等待线程结束（确保所有队列中的数据都写完）
	if (record_thread_.joinable()) {
		record_thread_.join();
	}

	// 3. 关闭文件
	std::lock_guard<std::mutex> lk(file_mutex_);
	if (ofs_.is_open()) {
		ofs_.close();
	}
}

bool Streamer::openRead(const std::string& filepath, std::atomic<bool>* running_flag) {
	std::lock_guard<std::mutex> lk(file_mutex_);
	if (is_writing_.load() || is_reading_.load()) return false;

	// 使用 u8path 支持 Windows 下的 UTF-8 路径
	ifs_.open(std::filesystem::u8path(filepath), std::ios::binary | std::ios::in);
	if (!ifs_.is_open()) return false;

	// 校验文件头
	FileHeader header;
	ifs_.read(reinterpret_cast<char*>(&header), sizeof(FileHeader));
	if (ifs_.gcount() != sizeof(FileHeader)) {
		ifs_.close();
		return false;
	}

	if (std::strncmp(header.magic, kMagic, 4) != 0 || header.version != kVersion) {
		LOG_ERROR("Invalid LGV file format or version mismatch.");
		ifs_.close();
		return false;
	}

	// 构建索引
	frame_index_.clear();
	LOG_INFO("[Streamer] Indexing file...");
	
	while (ifs_.peek() != EOF) {
		// 检查运行标志，如果为 false 则中止索引
		if (running_flag && !running_flag->load()) {
			LOG_INFO("[Streamer] Indexing aborted.");
			ifs_.close();
			return false;
		}

		std::streampos current_pos = ifs_.tellg();
		Pose p;
		if (!ifs_.read(reinterpret_cast<char*>(&p), sizeof(Pose))) break;
		
		uint32_t count = 0;
		if (!ifs_.read(reinterpret_cast<char*>(&count), sizeof(uint32_t))) break;
		
		frame_index_.push_back({p.timestamp_ns, current_pos});
		
		// 跳过点数据
		if (count > 0) {
			ifs_.seekg(count * sizeof(PackedPoint), std::ios::cur);
		}
	}
	
	// 重置到第一帧位置
	ifs_.clear();
	if (!frame_index_.empty()) {
		ifs_.seekg(frame_index_[0].second);
	}
	
	LOG_INFO("[Streamer] Indexed {} frames. Duration: {:.2f}s", frame_index_.size(), 
		frame_index_.empty() ? 0.0 : (frame_index_.back().first - frame_index_.front().first) * 1e-9);

	is_reading_.store(true);
	return true;
}

bool Streamer::seekToFrame(size_t index) {
	std::lock_guard<std::mutex> lk(file_mutex_);
	if (!is_reading_.load() || index >= frame_index_.size()) return false;
	
	ifs_.clear(); // 清除 EOF 标志
	ifs_.seekg(frame_index_[index].second);
	return true;
}

size_t Streamer::getFrameCount() const {
	return frame_index_.size();
}

uint64_t Streamer::getStartTime() const {
	if (frame_index_.empty()) return 0;
	return frame_index_.front().first;
}

uint64_t Streamer::getEndTime() const {
	if (frame_index_.empty()) return 0;
	return frame_index_.back().first;
}

size_t Streamer::getFrameIndexAtTime(uint64_t timestamp) const {
	if (frame_index_.empty()) return 0;
	
	auto it = std::lower_bound(frame_index_.begin(), frame_index_.end(), timestamp, 
		[](const std::pair<uint64_t, std::streampos>& a, uint64_t val) {
			return a.first < val;
		});
		
	if (it == frame_index_.end()) return frame_index_.size() - 1;
	if (it == frame_index_.begin()) return 0;
	
	// 比较 it 和 it-1 哪个更近
	auto it_prev = it - 1;
	if ((timestamp - it_prev->first) < (it->first - timestamp)) {
		return std::distance(frame_index_.begin(), it_prev);
	}
	return std::distance(frame_index_.begin(), it);
}

bool Streamer::readFrame(PointCloudPtr& out_cloud, Pose& out_pose) {
	std::lock_guard<std::mutex> lk(file_mutex_);
	if (!is_reading_.load()) return false;

	// 1. 读取 Pose
	if (!ifs_.read(reinterpret_cast<char*>(&out_pose), sizeof(Pose))) {
		return false; // EOF or Error
	}

	// 2. 读取点数
	uint32_t count = 0;
	if (!ifs_.read(reinterpret_cast<char*>(&count), sizeof(uint32_t))) {
		return false;
	}

	// 3. 读取点数据
	out_cloud = std::make_shared<PointCloud>();
	out_cloud->points.reserve(count);
	out_cloud->width = count;
	out_cloud->height = 1;
	out_cloud->is_dense = false;

	if (count > 0) {
		buffer_.resize(count);
		if (!ifs_.read(reinterpret_cast<char*>(buffer_.data()), count * sizeof(PackedPoint))) {
			return false;
		}
		
		for (const auto& pp : buffer_) {
			Point p;
			p.x = pp.x;
			p.y = pp.y;
			p.z = pp.z;
			p.intensity = pp.intensity;
			out_cloud->points.push_back(p);
		}
	}

	return true;
}

void Streamer::closeRead() {
	std::lock_guard<std::mutex> lk(file_mutex_);
	if (is_reading_.load()) {
		ifs_.close();
		is_reading_.store(false);
		frame_index_.clear();
	}
}
