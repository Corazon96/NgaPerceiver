#pragma once

/**
 * @file device_manager.h
 * @brief 设备管理器 - Livox LiDAR 数据采集、录制与回放的 Facade 模块
 * 
 * DeviceManager 是系统的核心协调层，负责：
 * 1. **硬件层**：管理 Livox SDK 设备发现、连接、启动/停止采集
 * 2. **数据处理**：委托 DataProcessor 进行 IMU 融合、时间同步、姿态插值
 * 3. **录制回放**：委托 Replayer 管理 Streamer 进行文件 I/O、回放控制
 * 4. **数据分发**：frameDispatcher_ 线程将点云帧与 Pose 配对后发送给下游
 * 
 * @section thread_model 线程模型
 * - **SDK 回调线程**（Livox SDK 内部，多个）：
 *   调用 enqueuePacket() 快速入队原始数据包，避免阻塞 SDK
 * - **frameDispatcher_ 线程**（单线程）：
 *   从 SPSC 队列取出原始包 → 解析点云 → 匹配 Pose → 触发 onFrameWithPose/录制
 * - **Replayer 内部线程**：
 *   回放模式下，读取文件并模拟帧回调
 * 
 * @section data_flow 数据流
 * 【采集模式】
 * Livox SDK → LivoxPointCloudObserver/LivoxImuObserver → enqueuePacket() → 
 * frame_queue_ (SPSC) → frameDispatcher_() → DataProcessor::getPoseAt() → 
 * onFrameWithPose(cloud, pose) → 下游（Processor/Renderer）
 * 
 * 【回放模式】
 * Replayer::worker_thread_ → Replayer::onFrame → DeviceManager::onFrameWithPose → 下游
 * 
 * @section time_sync 时间同步
 * 所有时间戳通过 DataProcessor::syncTimestamp() 对齐到系统时间（当 lidar 时钟 < MIN_VALID_EPOCH_NS 时）
 */

#include <functional>
#include <thread>
#include <atomic>
#include <mutex>
#include <deque>
#include <condition_variable>
#include <memory>

#include "core/common.h"
#include "core/queue.h"
#include "core/config.h"
#include "streamer.h"
#include "replayer.h"
#include "data_processor.h"

/**
 * @brief 设备信息结构体
 * 用于 UI 展示和设备状态管理
 */
struct DeviceInfo {
	uint32_t handle;          ///< Livox SDK 分配的设备句柄（唯一标识符）
	std::string sn;           ///< 设备序列号
	std::string ip;           ///< 设备 IP 地址
	uint8_t type;             ///< 设备类型（如 Mid-360）
	bool is_connected;        ///< 逻辑连接状态（是否为当前活动设备）
	bool is_acquiring;        ///< 数据采集状态（是否正在发送数据流）
};

class DeviceManager
{
public:
	DeviceManager();
	~DeviceManager();

	// --- 系统生命周期 ---

	/**
	 * @brief 初始化 SDK 并启动设备发现
	 * @param config_path Livox SDK 配置文件路径 (JSON)
	 * @return true 初始化成功
	 * @return false 初始化失败（如配置文件不存在、SDK 错误）
	 */
	bool start(const std::string& config_path = "./mid360_config.json");

	/**
	 * @brief 停止 SDK，断开所有连接并清理资源
	 * 阻塞直到内部线程退出
	 */
	void stop();

	// --- 设备连接与采集管理 ---

	/**
	 * @brief 获取当前发现的所有设备列表
	 * 线程安全
	 */
	std::vector<DeviceInfo> getDevices() const;

	/**
	 * @brief 连接指定设备（逻辑连接）
	 * 切换当前活动设备，断开之前的设备
	 * @param handle 设备句柄
	 */
	void connectDevice(uint32_t handle);

	/**
	 * @brief 断开当前活动设备
	 */
	void disconnectDevice();

	/**
	 * @brief 开始采集点云数据
	 * 仅当有活动设备且未在回放时有效
	 */
	void startAcquisition();

	/**
	 * @brief 停止采集点云数据
	 */
	void stopAcquisition();

	/**
	 * @brief 检查指定句柄是否为当前活动设备
	 */
    bool isActiveDevice(uint32_t handle) const {
		return active_device_handle_.load() == handle;
	}

	/**
	 * @brief 检查是否正在采集数据（设备模式）
	 */
	bool isAcquiring() const {
		return is_acquiring_.load();
	}

	// --- 录制控制 ---

	/**
	 * @brief 开始录制点云数据到文件
	 * @param filepath 输出文件路径 (.lgv)
	 * @return true 成功开始录制
	 */
	bool startRecording(const std::string& filepath);

	/**
	 * @brief 停止录制
	 * 异步操作，完成后触发 onRecordingFinished
	 */
	void stopRecording();

	/**
	 * @brief 检查是否正在录制
	 */
	bool isRecording() const;

	// --- 回放控制 ---

	/**
	 * @brief 开始回放录制文件
	 * 会自动停止当前的设备采集
	 * @param filepath 录制文件路径 (.lgv)
	 * @return true 成功开始回放
	 */
	bool startReplay(const std::string& filepath);

	/**
	 * @brief 停止回放
	 */
	void stopReplay();

	/**
	 * @brief 检查是否正在回放
	 */
	bool isReplaying() const;
	
	/**
	 * @brief 设置回放速度倍率
	 * @param speed 0.5 - 8.0
	 */
	void setReplaySpeed(float speed);

	/**
	 * @brief 暂停回放
	 */
	void pauseReplay();

	/**
	 * @brief 恢复回放
	 */
	void resumeReplay();

	/**
	 * @brief 检查回放是否暂停
	 */
	bool isReplayPaused() const;

	/** 
	 * @brief 跳转回放进度 
	 * @param progress 0.0 - 1.0 之间的比例值
	 */
	void seekReplay(double progress);
	
	/** 
	 * @brief 设置回放时的积分时间（用于 Seek 时的预加载时长）
	 * @param ms 毫秒数
	 */
	void setRetentionTime(int ms);

	// --- 回调函数接口 ---

    /**
	 * @brief 核心数据回调：传递点云与对应 Pose
	 * 在 frameDispatcher_ 线程中调用，非 SDK 回调线程
	 * Pose 包含时间戳和 IMU/位姿信息
	 */
	std::function<void(PointCloudPtr, const Pose &)> onFrameWithPose;

	/** @brief 回放进度回调 (current_ts_ns, total_duration_ns) */
	std::function<void(uint64_t, uint64_t)> onReplayProgress;

	/** @brief 回放结束回调 */
	std::function<void()> onReplayFinished;

    /** @brief 错误回调 */
	std::function<void(const std::string&)> onError;

    /** @brief 录制结束回调（异步停止完成后触发） */
	std::function<void()> onRecordingFinished;

	/** @brief 录制丢帧回调 */
	std::function<void()> onRecordingDrop;

	/** @brief 信息提示回调 */
	std::function<void(const std::string&)> onInfo;

	/** @brief 当设备列表变化或状态改变时的回调 */
	std::function<void(const std::vector<DeviceInfo>&)> onDeviceListUpdated;

	// --- 内部/高级接口 ---

	/**
	 * @brief 将 SDK 回调的原始数据包快速入队
	 * 由 SDK 回调线程调用，非阻塞，存入 SPSC 队列
	 * @param data 原始数据指针
	 * @param length 数据长度
	 * @param timestamp_ns 纳秒时间戳
	 */
	void enqueuePacket(const uint8_t* data, size_t length, uint64_t timestamp_ns);

	/** @brief 处理 Livox 点云数据回调（内部实现） */
	void handlePointCloud(const void* data);

	/** @brief 处理 Livox IMU 数据回调（内部实现） */
	void handleImu(const void* data);

	/** @brief Livox SDK 信息变更的内部回调 */
	void handleDeviceInfoChange(uint32_t handle, const void* info);

	/** @brief 获取 DataProcessor 实例（用于 IMU 处理等） */
	DataProcessor* getDataProcessor() { return data_processor_.get(); }

private:
	/** 
	 * @brief 原始数据包结构，用于解耦 SDK 回调与帧处理
	 * SDK 回调线程快速拷贝原始数据到此结构并入队。
	 * frameDispatcher_ 线程异步处理
	 */
	struct RawPacket {
		std::vector<uint8_t> data;  ///< 原始以太网数据包（LivoxLidarEthernetPacket）
		uint64_t timestamp_ns;      ///< 已同步的时间戳（纳秒）
	};

    /** 
	 * @brief 帧分发线程函数
	 * 负责从 frame_queue_ 取出原始包 → 解析点云 → 匹配 Pose → 触发回调
	 */
	void frameDispatcher_();

	// --- 生命周期管理 ---
	
	/** @brief 模块运行状态标志，控制 frameDispatcher_ 线程退出 */
	std::atomic<bool> dm_running_;

	// --- 子模块委托 ---
	
	/** 
	 * @brief 回放模块（兼录制）
	 * 管理 Streamer 进行文件 I/O、回放控制（速度/暂停/跳转）
	 */
	std::unique_ptr<Replayer> replayer_;

	/** 
	 * @brief 数据处理模块（IMU/时间同步）
	 * 负责：
	 * - IMU 样本积分与姿态推算
	 * - 时间戳同步（lidar 时钟 → 系统时钟）
	 * - Pose 插值（根据时间戳查询对应姿态）
	 */
	std::unique_ptr<DataProcessor> data_processor_;

	// --- 设备状态管理 ---
	
	/** @brief 保护 devices_ 列表的互斥锁 */
	mutable std::mutex devices_mutex_;
	
	/** @brief 当前发现的所有设备列表 */
	std::vector<DeviceInfo> devices_;
	
	/** @brief 当前活动设备句柄（0 表示无活动设备） */
	std::atomic<uint32_t> active_device_handle_{0};
	
	/** @brief 是否正在采集数据（设备模式，非回放模式） */
	std::atomic<bool> is_acquiring_{false};

	// --- 帧队列与线程 ---
	
	/** 
	 * @brief 原始数据帧队列 (SPSC lock-free)
	 * 生产者：SDK 回调线程（enqueuePacket）
	 * 消费者：frameDispatcher_ 线程
	 * 容量：LingerConfig::FRAME_QUEUE_SIZE (2048, 约 1 秒缓冲)
	 */
	SpscRingBuffer<RawPacket> frame_queue_{LingerConfig::FRAME_QUEUE_SIZE};
	
	/** 
	 * @brief 帧分发工作线程
	 * 负责从 frame_queue_ 取出原始包 → 解析点云 → 匹配 Pose → 触发回调
	 */
	std::thread frame_worker_;

	/** @brief 条件变量：当有新帧到达时唤醒 frameDispatcher_ 线程 */
	std::condition_variable frame_cv_;
	
	/** @brief frame_cv_ 关联的互斥锁 */
	std::mutex frame_cv_mutex_;
};
