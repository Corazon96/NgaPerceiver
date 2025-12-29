#pragma once

/**
 * @file config.h
 * @brief 系统编译期配置参数
 * 
 * 包含系统架构参数、资源限制、算法核心阈值等。
 * 修改此文件中的参数通常需要重新编译项目。
 * 
 * 主要配置项：
 * - DeviceManager: 时间同步、IMU 校准、队列大小
 * - Streamer: 写入队列大小
 * - Processor: 发布频率、内部队列
 * - Filter: 滤波器默认参数
 * - Renderer: 渲染帧率限制
 */

#include <cstdint>
#include <cstddef>

namespace LingerConfig {

    // --- DeviceManager 配置 ---
    
    // 时间同步参数
    /** @brief 最小有效时间戳 (纳秒)，早于 2000-01-01 00:00:00 UTC 的时间戳视为无效 */
    inline constexpr uint64_t MIN_VALID_EPOCH_NS = 946684800000000000ULL;
    /** @brief 时间同步漂移阈值 (纳秒)，超过此值触发重新同步 */
    inline constexpr int64_t TIME_SYNC_DRIFT_THRESHOLD_NS = 10000000000LL; // 10秒

    // IMU 校准参数
    /** @brief IMU 校准样本数（约 1-2 秒，取决于 IMU 频率） */
    inline constexpr int IMU_CALIBRATION_SAMPLES = 200;
    /** @brief IMU 死区阈值 (rad/s) */
    inline constexpr float IMU_DEADZONE_THRESHOLD = 1e-4f;
    /** @brief IMU 小角度阈值 (rad) */
    inline constexpr double IMU_SMALL_ANGLE_THRESHOLD = 1e-12;

    // 重力校正参数（运行时持续校正姿态漂移，宽容度较高，允许运动状态）
    /** @brief IMU 重力校正最小加速度 (g) - 运行时重力方向校正的加速度范围 */
    inline constexpr float IMU_GRAVITY_CORRECTION_MIN_ACC = 0.9f;
    /** @brief IMU 重力校正最大加速度 (g) - 运行时重力方向校正的加速度范围 */
    inline constexpr float IMU_GRAVITY_CORRECTION_MAX_ACC = 1.1f;
    /** @brief IMU 重力校正增益 (alpha) - 互补滤波融合系数（1% 加速度计 + 99% 陀螺仪） */
    inline constexpr float IMU_GRAVITY_CORRECTION_ALPHA = 0.01f;
    
    // 静止检测参数（仅启动校准阶段使用，要求严格静止，同时满足角速度和加速度条件）
    /** @brief IMU 静止检测：最大角速度 (rad/s) - 用于陀螺仪零偏校准时的静止判断 */
    inline constexpr float IMU_STATIC_GYRO_THRESHOLD = 0.5f;
    /** @brief IMU 静止检测：最大加速度偏差 (g) - 用于陀螺仪零偏校准时的静止判断 */
    inline constexpr float IMU_STATIC_ACC_DEVIATION = 0.3f;

    // 姿态与队列参数
    /** @brief 姿态插值最大容差 (纳秒) */
    inline constexpr uint64_t POSE_INTERPOLATION_MAX_DELTA_NS = 20000000ULL; // 20ms
    /** @brief UDP 数据包队列大小（Mid-360 典型速率 200-1000 包/秒，2048 提供约 2-10 秒缓冲） */
    inline constexpr size_t FRAME_QUEUE_SIZE = 2048;
    /** @brief IMU 样本队列容量 (200Hz 下，1000 提供约 5 秒缓冲，足够姿态插值查询) */
    inline constexpr size_t IMU_SAMPLE_CAPACITY = 1000;

    /** @brief 是否启用点云姿态变换（将点云从雷达坐标系变换到世界坐标系）
     *  - true: 点云变换到世界坐标系（雷达旋转时，世界中的物体保持静止）
     *  - false: 点云保持在雷达坐标系（雷达旋转时，点云跟着旋转）
     *  对于靠泊检测，通常设为 false，因为需要检测物体相对于（雷达的）位置
     */
    inline constexpr bool ENABLE_POSE_TRANSFORM = false;


    // --- Streamer 配置 ---

    /** @brief 录制写入队列大小 */
    inline constexpr size_t STREAMER_QUEUE_SIZE = 5000;


    // --- Processor 配置 ---

    /** @brief 处理器内部队列大小 */
    inline constexpr size_t PROCESSOR_QUEUE_SIZE = 8192;

    /** @brief 处理器单次批处理大小 (帧数)，用于提高吞吐量 */
    inline constexpr size_t PROCESSOR_BATCH_SIZE = 32;

    /** 
     * @brief 处理器发布间隔 (毫秒)
     * 30ms -> 约 33Hz，略高于 30FPS 以补偿抖动
     */
    inline constexpr int PROCESSOR_PUBLISH_INTERVAL_MS = 30;


    // --- Filter 配置 ---

    /** @brief 距离滤波器默认最小值 (m) */
    inline constexpr float FILTER_DISTANCE_MIN_DEFAULT = 0.1f;
    /** @brief 距离滤波器默认最大值 (m) */
    inline constexpr float FILTER_DISTANCE_MAX_DEFAULT = 100.0f;

    /** @brief 体素滤波器默认大小 (m) */
    inline constexpr float FILTER_VOXEL_SIZE_DEFAULT = 0.1f;

    /** @brief ROI 裁剪默认范围 (米，船体/世界坐标系) */
    inline constexpr float ROI_X_MIN_DEFAULT = -10.0f;
    inline constexpr float ROI_X_MAX_DEFAULT = 80.0f;
    inline constexpr float ROI_Y_MIN_DEFAULT = -25.0f;
    inline constexpr float ROI_Y_MAX_DEFAULT = 25.0f;
    inline constexpr float ROI_Z_MIN_DEFAULT = -3.0f;
    inline constexpr float ROI_Z_MAX_DEFAULT = 15.0f;

    /** @brief 海面滤除：海面高度与安全裕度 (米) */
    inline constexpr float SEA_LEVEL_Z_DEFAULT = 0.0f;
    inline constexpr float SEA_MARGIN_DEFAULT   = 0.5f;

    /** @brief 统计离群点滤波默认参数 */
    inline constexpr int   STAT_OUTLIER_MEAN_K_DEFAULT     = 20;
    inline constexpr float STAT_OUTLIER_STDDEV_MUL_DEFAULT = 1.0f;


    // --- Renderer 配置 ---

    /**
     * @brief 最小帧间隔 (毫秒)，用于限制最大 FPS
     * 25.0ms -> 40 FPS
     * 16.6ms -> 60 FPS
     * 8.3ms  -> 120 FPS
     */
    inline constexpr double RENDERER_MIN_FRAME_INTERVAL_MS = 25.0;

    /** @brief 并行处理阈值 (点数) */
    inline constexpr size_t RENDERER_PARALLEL_THRESHOLD = 40000;

}
