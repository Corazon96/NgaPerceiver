#pragma once

#include <deque>
#include <mutex>
#include <atomic>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "core/common.h"
#include "core/config.h"

/**
 * @brief 数据处理器
 * 负责 IMU 融合、姿态解算、时间同步等核心算法
 */
class DataProcessor
{
public:
	DataProcessor();
	~DataProcessor();

	/**
	 * @brief 时间同步：计算并应用时间偏移
	 * 将设备时间戳转换为系统统一时间戳
	 */
	uint64_t syncTimestamp(uint64_t timestamp_ns);

	/** @brief 将 IMU 样本插入时间序列并通过陀螺仪积分计算方向 */
	void addImuSample(uint64_t timestamp_ns, const IMUData &imu);

	/**
	 * @brief 获取在时间点的插值姿态（四元数）与插值 IMU 数据
	 */
	bool getPoseAt(uint64_t timestamp_ns, Pose &out_pose, uint64_t max_delta_ns = LingerConfig::POSE_INTERPOLATION_MAX_DELTA_NS);

	/** @brief 更新最近一次已知的 Pose（通常来自 IMU 回调的最新数据） */
	void updateLastPose(const Pose &p);
	
	/** @brief 获取最近一次已知的 Pose */
	Pose getLastPose();

private:
	/** @brief 姿态积分核心算法 */
	void integratePose_(const Pose& prev, Pose& cur);

	/**
	 * @brief IMU 时间序列样本（按 timestamp 升序）
	 */
	std::deque<Pose> imu_samples_;
	std::mutex imu_mutex_;
	size_t imu_capacity_ = LingerConfig::IMU_SAMPLE_CAPACITY;

	/** @brief 时间同步相关 */
	std::atomic<bool> time_sync_initialized_{false};
	std::atomic<int64_t> time_offset_ns_{0};
	std::mutex time_sync_mutex_;

	/** @brief IMU 校准相关 */
	Eigen::Vector3f gyro_bias_{0.0f, 0.0f, 0.0f};
	int calibration_count_{0};
	bool is_calibrated_{false};

	/** @brief 缓存最近一帧 IMU/位姿信息 */
	Pose last_pose_;
	std::mutex last_pose_mutex_;
};
