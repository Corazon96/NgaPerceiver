#pragma once

/**
 * @file app_config.h
 * @brief 应用运行时配置管理
 * 
 * 定义 AppConfig 结构体及 JSON 序列化/反序列化接口。
 * 这里的参数可以在运行时通过修改 app_config.json 文件进行调整，
 * 无需重新编译。主要包含：
 * - 日志配置
 * - 各类滤波器参数 (距离、ROI、海面、离群点、体素)
 */

#include <string>

namespace Linger {

/**
 * @brief 应用配置结构体，用于保存滤波器参数、日志设置等
 */
struct AppConfig {
    // --- 距离滤波器 (Distance Filter) ---
    float distance_min = 0.1f;
    float distance_max = 100.0f;
    bool distance_enabled = false;

    // --- ROI 滤波器 (CropBox Filter) ---
    float roi_x_min = -10.0f;
    float roi_x_max = 80.0f;
    float roi_y_min = -25.0f;
    float roi_y_max = 25.0f;
    float roi_z_min = -3.0f;
    float roi_z_max = 15.0f;
    bool roi_enabled = false;

    // --- 海面滤波器 (Sea Surface Filter) ---
    float sea_level_z = 0.0f;
    float sea_margin = 0.5f;
    bool sea_enabled = false;

    // --- 统计离群点滤波器 (Statistical Outlier Removal) ---
    int outlier_mean_k = 20;
    float outlier_stddev_mul = 1.0f;
    bool outlier_enabled = false;

    // --- 体素滤波器 (Voxel Grid Filter) ---
    float voxel_leaf_size = 0.1f;
    bool voxel_enabled = false;

    // --- 密度滤波器 (Density Filter) ---
    float density_voxel_size = 0.3f;
    int density_min_points = 3;
    bool density_enabled = false;

    // --- 运动滤波器 (Motion Filter) ---
    float motion_cell_size = 0.5f;
    float motion_threshold = 0.3f;
    bool motion_output_static = true;
    bool motion_enabled = false;

    // --- 靠泊检测 - 最近区域 (Nearest Region Detection) ---
    float nr_sector_x_min = 0.0f;
    float nr_sector_x_max = 50.0f;
    float nr_sector_y_min = -15.0f;
    float nr_sector_y_max = 15.0f;
    float nr_sector_z_min = -1.0f;
    float nr_sector_z_max = 10.0f;
    float nr_percentile = 5.0f;
    bool nr_enabled = true;

    // --- 靠泊检测 - 码头边缘 (Dock Edge Detection) ---
    float edge_x_min = 0.0f;
    float edge_x_max = 30.0f;
    float edge_y_min = -15.0f;
    float edge_y_max = 15.0f;
    float edge_z_min = 0.3f;
    float edge_z_max = 1.5f;
    float edge_ransac_dist = 0.15f;
    bool edge_enabled = true;

    // --- 时序滤波 (Temporal Filter) ---
    // 注意：时序滤波是 DockingAlgorithm 的内部参数，不在 UI 中暴露
    // 用于抑制靠泊距离的异常跳变
    float temporal_max_jump = 2.0f;     // 允许的最大距离跳变(m)
    bool temporal_enabled = true;       // 是否启用时序滤波

    // --- UDP 通信 (Docking Publisher) ---
    std::string udp_target_ip = "127.0.0.1";
    uint16_t udp_target_port = 5000;
    uint8_t udp_sensor_id = 1;  // 1=前, 2=左舷, 3=右舷
    bool udp_enabled = false;
};

/**
 * @brief 从 JSON 文件加载配置
 * @param path JSON 文件路径
 * @param cfg 输出配置对象
 * @return 成功返回 true，失败返回 false
 */
bool LoadAppConfig(const std::string& path, AppConfig& cfg);

/**
 * @brief 保存配置到 JSON 文件
 * @param path JSON 文件路径
 * @param cfg 配置对象
 * @return 成功返回 true，失败返回 false
 */
bool SaveAppConfig(const std::string& path, const AppConfig& cfg);

/**
 * @brief 获取默认配置文件路径
 * 通常为当前目录下的 app_config.json
 */
std::string GetDefaultAppConfigPath();

} // namespace Linger
