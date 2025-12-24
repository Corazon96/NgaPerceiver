#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <condition_variable>
#include <deque>
#include "core/common.h"

/**
 * @brief 紧凑的点结构，用于文件存储 (16 bytes)
 * 避免直接写入 pcl::PointXYZI 可能带来的 padding 问题
 */
struct PackedPoint {
	float x, y, z, intensity;
};

/**
 * @brief 文件头结构
 */
struct FileHeader {
	char magic[4]; // "LGV1"
	uint32_t version;
};

class Streamer {
public:
	Streamer();
	~Streamer();

	// --- 写入接口 ---
	bool openWrite(const std::string& filepath);
	void writeFrame(const PointCloudPtr& cloud, const Pose& pose);
	void closeWrite();

	// --- 读取接口 ---
	bool openRead(const std::string& filepath, std::atomic<bool>* running_flag = nullptr);
	/** 
	 * @brief 读取下一帧
	 * @return true 成功读取, false 文件结束或错误
	 */
	bool readFrame(PointCloudPtr& out_cloud, Pose& out_pose);
	
	/** @brief 跳转到指定帧索引 */
	bool seekToFrame(size_t index);
	/** @brief 获取总帧数 */
	size_t getFrameCount() const;
	/** @brief 获取起始时间 */
	uint64_t getStartTime() const;
	/** @brief 获取结束时间 */
	uint64_t getEndTime() const;
	/** @brief 根据时间戳查找最近的帧索引 */
	size_t getFrameIndexAtTime(uint64_t timestamp) const;

	void closeRead();

	bool isWriting() const { return is_writing_; }
	bool isReading() const { return is_reading_; }

	/** @brief 当写入队列满导致丢帧时的回调 */
	std::function<void()> onWriteQueueFull;

private:
	void recordWorker_();

	std::ofstream ofs_;
	std::ifstream ifs_;
	std::atomic<bool> is_writing_{false};
	std::atomic<bool> is_reading_{false};
	std::mutex file_mutex_;

	// 读取索引：pair<timestamp, file_offset>
	std::vector<std::pair<uint64_t, std::streampos>> frame_index_;

	// 异步写入队列
	struct WriteTask {
		PointCloudPtr cloud;
		Pose pose;
	};
	std::deque<WriteTask> write_queue_;
	std::mutex queue_mutex_;
	std::condition_variable queue_cv_;
	std::thread record_thread_;

	/** @brief 复用缓冲区以避免频繁内存分配 */
	std::vector<PackedPoint> buffer_;
};
