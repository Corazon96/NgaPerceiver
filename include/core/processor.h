#pragma once

#include <functional>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <utility>
#include <vector>
#include <deque>
#include "core/queue.h"
#include "preprocessing/filter.h"
#include "core/config.h"

/**
 * @brief PointCloudProcessor负责：接收逐帧点云（含 Pose），在后台线程进行变换、合并/缓存
 * 并以回调的形式发布合并后的地图快照（PointCloudPtr）
 * 目前实现保留原始变换后的点并按保留时间发布快照（raw publish）
 * 线程模型说明：
 * - enqueue(...) 可从任意线程（例如 DeviceManager 的 frameDispatcher_）调用，须尽快返回
 * - Processor 内部启动一个 worker线程来处理队列并调用 mapUpdateCallback（在 worker线程中调用）
 * - mapUpdateCallback 的接收方（例如 Renderer）应使用 std::atomic_load 来读取并供 UI/渲染线程安全地使用快照
 */
class PointCloudProcessor
{
public:
	PointCloudProcessor();
	~PointCloudProcessor();

	/** @brief 启动处理线程。调用 start() 后才能 enqueue() */
	void start();

	/** @brief 停止处理线程并等待退出（会 join worker线程） */
	void stop();

	/** @brief 将帧入队以便后台处理。 */
	void enqueue(PointCloudPtr pc, const Pose &pose);

	/**
	 * @brief 注册地图快照更新回调：接收者应注意线程上下文并尽快返回。
	 * 增加 filtered_count 参数，表示当前快照中被过滤掉的点数
	 */
	void setMapUpdateCallback(std::function<void(const std::vector<PointCloudPtr>&, size_t)> cb);

	/** @brief 可配置点保留时间（秒），默认0.1s；过期点在发布时会被过滤与清除 */
	void setRetentionSeconds(double s);
	double getRetentionSeconds() const;

	/**
	 * @brief 原子读取最新发布的快照（从 worker 中使用原子存储发布）
	 * 修改：返回点云块列表
	 */
	std::vector<PointCloudPtr> takeLatestSnapshot();

	/**
	 * @brief 添加滤波器到处理管线
	 * is_post_process: true 表示在坐标变换后(World Frame)执行，false 表示在坐标变换前(Sensor Frame)执行
	 */
	void addFilter(Linger::FilterPtr filter, bool is_post_process = false);

	/** @brief 清除所有滤波器 */
	void clearFilters();

	/** @brief 清空内部缓存（用于 Seek 或重置） */
	void clear();

private:
	/** @brief 内部队列：使用 pair<PointCloudPtr, Pose>作为队列元素以保持时序一致性 */
	SpscRingBuffer<std::pair<PointCloudPtr, Pose>> queue_{LingerConfig::PROCESSOR_QUEUE_SIZE};
	std::mutex mutex_;
	std::condition_variable cond_;
	std::thread processor_worker_;
	std::atomic<bool> running_{false};
	std::atomic<size_t> count_{0};
	std::function<void(const std::vector<PointCloudPtr>&, size_t)> mapUpdateCallback_;
	/** @brief 保留时间（纳秒） */
	std::atomic<uint64_t> retention_ns_{100000000ULL}; // 默认0.1s

	/**
	 * @brief 最新发布的快照指针（使用 std::atomic_store / std::atomic_load进行原子操作）
	 * 注意：std::atomic<shared_ptr> 在 C++20 前不完全支持，这里使用 mutex 保护或者 atomic_load/store 的特化
	 * 为了简单起见，我们这里使用 mutex 保护这个 snapshot 列表的读写
	 */
	std::vector<PointCloudPtr> latest_snapshot_;
	std::mutex snapshot_mutex_;

	/** @brief 存储最近变换后的原始点以按 retention 发布 */
	std::deque<StampedCloud> frame_buffer_;
	std::mutex raw_mutex_;

	/** @brief 滤波器链 */
	std::vector<Linger::FilterPtr> pre_filters_;  // 变换前(Sensor Frame)
	std::vector<Linger::FilterPtr> post_filters_; // 变换后(World Frame)
	std::mutex filter_mutex_;
};
