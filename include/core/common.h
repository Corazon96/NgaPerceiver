#pragma once

/**
 * @file common.h
 * @brief 项目通用类型定义与常量
 * 
 * 定义了全项目通用的数据结构，包括：
 * - Point/PointCloud: PCL 点云类型别名
 * - Pose: 包含位置、旋转（四元数）和时间戳的位姿结构
 * - IMUData: 原始 IMU 数据结构
 * - StampedCloud: 带时间戳的点云帧
 */

#include <memory>
#include <vector>
#include <array>
#include <cstdint>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// --- PCL 类型别名 ---
using Point = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<Point>;
using PointCloudPtr = std::shared_ptr<PointCloud>;

/** @brief 可选的保留时间配置（毫秒），用于 UI 下拉框 */
inline constexpr std::array<int, 5> kRetentionValues = {100, 200, 500, 1000, 3000};

/**
 * @brief 带时间戳的点云帧
 * 用于在模块间传递（如 DeviceManager -> Processor）
 */
struct StampedCloud
{
    PointCloudPtr cloud;        ///< 点云数据指针
    uint64_t timestamp_ns = 0;  ///< 采集时间戳（纳秒）
    size_t filtered_count = 0;  ///< 被过滤掉的点数（统计用）
};

/**
 * @brief 简单的 IMU 原始数据结构
 * 用于在采集端传递
 */
struct IMUData
{
    /** @brief 陀螺仪读数，单位 rad/s */
    float gyro_x = 0.0f;
    float gyro_y = 0.0f;
    float gyro_z = 0.0f;
    
    /** @brief 加速度计读数，单位 m/s² */
    float acc_x = 0.0f;
    float acc_y = 0.0f;
    float acc_z = 0.0f;
};

/**
 * @brief 姿态结构体
 * 包含时间戳、位置、旋转（四元数）及关联的 IMU 数据
 */
struct Pose
{
    /** @brief 时间戳，单位纳秒 */
    uint64_t timestamp_ns = 0; 
    
    /** 
     * @brief 平移（米）
     * 注意：当前系统仅使用 IMU 进行姿态估计，无法准确计算位移。
     * 因此 x, y, z 默认为 0（或代表雷达安装偏移）。
     * 若需绝对定位，需融合 GPS/RTK 或里程计数据。
     */
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    
    /** @brief 旋转（单位四元数） */
    float qx = 0.0f;
    float qy = 0.0f;
    float qz = 0.0f;
    float qw = 1.0f;
    
    /** @brief 可附带最近一帧 IMU 原始读数（可选） */
    IMUData imu{};
};
