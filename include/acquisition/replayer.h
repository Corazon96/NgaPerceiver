#pragma once

#include <string>
#include <thread>
#include <atomic>
#include <functional>
#include <mutex>

#include "core/common.h"
#include "streamer.h"

/**
 * @brief 独立的回放控制器
 * 负责读取 LGV 文件、控制播放进度、倍速、暂停等
 */
class Replayer
{
public:
	Replayer();
	~Replayer();

	/**
	 * @brief 启动回放
	 * @param filepath 文件路径
	 * @return true 成功启动, false 失败（如文件不存在或正在运行）
	 */
	bool start(const std::string& filepath);

	/** @brief 停止回放 */
	void stop();

	bool isPlaying() const;
	bool isPaused() const;

	// --- 播放控制 ---
	void setSpeed(float speed);
	void pause();
	void resume();
	
	/** @brief 跳转进度 (0.0 - 1.0) */
	void seek(double progress);
	
	/** @brief 设置Seek时的预加载时长（毫秒）*/
	void setRetentionTime(int ms);

	// --- 录制控制 ---
	/**
	 * @brief 启动录制
	 * @param filepath 文件路径
	 * @return true 成功启动, false 失败（如正在回放或录制）
	 */
	bool startRecording(const std::string& filepath);

	/** @brief 停止录制（异步） */
	void stopRecording();

	/** @brief 是否正在录制 */
	bool isRecording() const;

	/** @brief 写入一帧（仅在录制状态下有效） */
	void writeFrame(const PointCloudPtr& cloud, const Pose& pose);

	// --- 回调接口 ---
	
	/** @brief 输出点云帧的回调 */
	std::function<void(PointCloudPtr, const Pose&)> onFrame;

	/** @brief 进度回调 (current_ns, total_duration_ns) */
	std::function<void(uint64_t, uint64_t)> onProgress;

	/** @brief 播放结束回调 */
	std::function<void()> onFinished;

	/** @brief 录制结束回调 */
	std::function<void()> onRecordingFinished;

	/** @brief 录制丢帧回调 */
	std::function<void()> onRecordingDrop;

	/** @brief 错误回调 */
	std::function<void(const std::string&)> onError;

private:
	/**
	 * @brief 处理 Seek 请求及预加载逻辑
	 * 
	 * 当发起 Seek 时，不仅跳转文件指针，还会预读取一段时间的数据（Retention Time）
	 * 并快速发送，以便在暂停状态下也能看到点云“拖尾"效果
	 */
	void handleSeek_(uint64_t start_ts_file, uint64_t duration_ns, 
                     std::chrono::steady_clock::time_point& start_time_sys, 
                     double& current_speed, bool& eof_reached);

	void replayWorker_(std::string filepath);

	// 数据源
	Streamer streamer_;

	// 回放线程
	std::thread replay_worker_;
	std::atomic<bool> running_{false};
	std::atomic<bool> paused_{false};
	std::atomic<float> speed_{1.0f};
	
	// Seek 控制 - 使用互斥锁保护以避免竞争
	struct SeekRequest {
		bool requested = false;
		double target = 0.0;
	};
	SeekRequest seek_request_;
	std::mutex seek_mutex_;
	std::atomic<int> retention_ms_{100};

	/** @brief 速度变化检测阈值，用于判断是否需要重新计算时间基准*/
	static constexpr double SPEED_EPSILON = 1e-4;
	
	/** @brief 进度回调触发间隔 (每处理多少帧触发一次) */
	static constexpr int PROGRESS_UPDATE_INTERVAL = 10; 
	
	/** @brief Seek 时最大预加载帧数，防止内存溢出*/
	static constexpr int MAX_PRELOAD_FRAMES = 3000;
	
	/** @brief 忙等待阈值(微秒)，小于此值时使用忙等待以提高定时精度 */
	static constexpr int SPIN_WAIT_THRESHOLD_US = 500;

	// 录制控制
	std::thread recording_stop_thread_;
	std::atomic<bool> is_stopping_recording_{false};
};
