#pragma once

#include <cstdint>
#include <vector>

namespace Linger {

/**
 * @brief 检测状态码
 */
enum class DockingStatus : uint8_t {
    NOT_DETECTED = 0,   ///< 未检测到
    NORMAL = 1,         ///< 正常检测
    LOW_CONFIDENCE = 2, ///< 低置信度警告
    INVALID = 3         ///< 无效/错误
};

/**
 * @brief 2D 直线参数（Ax + By + C = 0，归一化 A²+B²=1）
 */
struct Line2D {
    float a = 0.0f;  ///< 直线系数 A
    float b = 0.0f;  ///< 直线系数 B
    float c = 0.0f;  ///< 直线系数 C
    
    // 直线端点（用于可视化）
    float x1 = 0.0f, y1 = 0.0f;
    float x2 = 0.0f, y2 = 0.0f;
};

//=============================================================================
// 最近区域距离检测 (Nearest Region Distance)
//=============================================================================

/**
 * @brief 最近区域距离检测配置
 * 
 * 参考系说明：
 * - 当前实现以雷达坐标系原点为参考点
 * - 如果雷达安装在船上，原点即为雷达安装位置
 * - X轴指向前方，Y轴指向左侧，Z轴向上
 * 
 * 推荐参数值（根据场景调整）：
 * - sector_x: [0, 50] 检测前方50米
 * - sector_y: [-15, 15] 左右15米
 * - sector_z: [-1, 10] 水面以上
 * - percentile: 5-10% 使用最近的5-10%点
 * - min_points: 20-50 最少需20-50个点才输出
 */
struct NearestRegionConfig {
    // 检测扇区（雷达坐标系）
    float sector_x_min = 0.0f;          ///< X最小值(m)，推荐0（只检测前方）
    float sector_x_max = 50.0f;         ///< X最大值(m)，推荐0-80（根据码头距离）
    float sector_y_min = -15.0f;        ///< Y最小值(m)，推荐-10~-20（右侧范围）
    float sector_y_max = 15.0f;         ///< Y最大值(m)，推荐0~20（左侧范围）
    float sector_z_min = -1.0f;         ///< Z最小值(m)，推荐-1~0（略低于水面）
    float sector_z_max = 10.0f;         ///< Z最大值(m)，推荐~15（码头高度）
    
    // 最近距离计算参数
    float percentile = 5.0f;            ///< 最近点百分比(%), 推荐5-10，取最近N%点的平均
    size_t min_points = 30;             ///< 最小点数要求，少于此数不输出
    
    // 输出平滑
    float smoothing_alpha = 0.3f;       ///< 平滑系数(0-1)，推荐0.2-0.4，越小越平滑
    bool enable_smoothing = true;       ///< 是否启用平滑
    
    // 连续性检查
    bool check_continuity = true;       ///< 是否检查点云连续性
    float continuity_threshold = 2.0f;  ///< 连续性阈值(m)，超过此范围视为不连续
    
    // 开启
    bool enabled = true;                ///< 是否启用此检测
    bool show_sector = true;            ///< 是否显示检测扇区（可视化）
};

/**
 * @brief 最近区域距离检测结果
 */
struct NearestRegionResult {
    bool valid = false;                 ///< 是否有效
    float distance_m = 0.0f;            ///< 最近区域距离(m)
    float nearest_x = 0.0f;             ///< 最近区域中心X
    float nearest_y = 0.0f;             ///< 最近区域中心Y
    size_t point_count = 0;             ///< 扇区内点数
    size_t used_points = 0;             ///< 用于计算的点数（percentile）
    float continuity_score = 0.0f;      ///< 连续性评分 (0-1)
    uint8_t confidence = 0;             ///< 置信度0-100
};

//=============================================================================
// 码头边缘检测 (Dock Edge Detection)
//=============================================================================

/**
 * @brief 码头边缘检测配置
 * 
 * 原理：
 * - 码头边缘是特定高度范围内的水平线状结构
 * - 通过Z轴切片RANSAC拟合直线
 * 
 * 推荐参数值：
 * - edge_z: [0.3, 1.5] 码头边缘的典型高度
 * - sector_x: [0, 30] 只检测较近距离
 * - ransac_dist: 0.1-0.2m
 * - min_inlier_ratio: 0.3-0.5
 */
struct DockEdgeConfig {
    // 检测扇区（雷达坐标系）
    float sector_x_min = 0.0f;          ///< X最小值(m)
    float sector_x_max = 30.0f;         ///< X最大值(m)，边缘检测范围较近
    float sector_y_min = -15.0f;        ///< Y最小值(m)
    float sector_y_max = 15.0f;         ///< Y最大值(m)
    // 码头边缘高度范围（关键参数）
    float edge_z_min = 0.3f;            ///< 边缘Z最小值(m)，推荐0.2-0.5
    float edge_z_max = 1.5f;            ///< 边缘Z最大值(m)，推荐0.0-2.0
    
    // RANSAC 参数
    float ransac_distance_threshold = 0.15f;  ///< 内点距离阈值(m)，推荐0.1-0.2
    int ransac_max_iterations = 200;          ///< 最大迭代次数
    float ransac_min_inlier_ratio = 0.3f;     ///< 最小内点比例，推荐0.3-0.5
    size_t min_points = 50;                   ///< 最小点数要求
    
    // 几何约束
    float expected_angle_deg = 0.0f;    ///< 预期码头角度(°)
    float angle_tolerance_deg = 45.0f;  ///< 允许的角度偏差(°)
    float min_edge_length = 2.0f;       ///< 最小边缘长度(m)
    
    // 输出平滑
    float smoothing_alpha = 0.3f;       ///< 平滑系数(0-1)
    bool enable_smoothing = true;       ///< 是否启用平滑
    
    // 开启
    bool enabled = true;                ///< 是否启用此检测（默认开启以支持交叉验证）
    bool show_region = true;            ///< 是否显示检测区域（可视化）
};

/**
 * @brief 码头边缘检测结果
 */
struct DockEdgeResult {
    bool valid = false;                 ///< 是否有效
    float distance_m = 0.0f;            ///< 到边缘的垂直距离(m)
    float angle_deg = 0.0f;             ///< 边缘与X轴的夹角(°)
    Line2D edge_line;                   ///< 边缘直线
    size_t inlier_count = 0;            ///< 内点数
    size_t total_points = 0;            ///< 切片内总点数
    float mean_error = 0.0f;            ///< 内点平均误差
    float geometry_score = 0.0f;        ///< 几何合理性评分 (0-1)
    uint8_t confidence = 0;             ///< 置信度0-100
    
    // Layer 2: 多假设支持
    std::vector<Line2D> alternative_lines; ///< 备选直线（用于 Layer 3 智能融合）
};

//=============================================================================
// 时序滤波配置
//=============================================================================

struct TemporalFilterConfig {
    bool enabled = true;
    float max_jump_m = 2.0f;            ///< 允许的最大距离跳变(m)
    float velocity_limit = 5.0f;        ///< 最大速度限制(m/s)
    float process_noise = 0.1f;         ///< 过程噪声
    float measurement_noise = 0.5f;     ///< 测量噪声
};

//=============================================================================
// 综合靠泊状态
//=============================================================================

/**
 * @brief 综合靠泊状态
 */
struct DockingState {
    uint64_t timestamp_ns = 0;          ///< 时间戳（纳秒）
    
    // 最近区域检测结果
    NearestRegionResult nearest;
    
    // 码头边缘检测结果
    DockEdgeResult edge;
    
    // 综合状态（取两者中有效且距离最近的结果）
    DockingStatus status = DockingStatus::NOT_DETECTED;
    float final_distance_m = 0.0f;      ///< 最终输出距离(m)
    float final_angle_deg = 0.0f;       ///< 最终输出角度（仅边缘检测有效时）
    
    bool is_valid() const {
        return status != DockingStatus::NOT_DETECTED && status != DockingStatus::INVALID;
    }
};

/**
 * @brief 完整靠泊配置
 */
struct DockingConfig {
    NearestRegionConfig nearest;        ///< 最近区域配置
    DockEdgeConfig edge;                ///< 码头边缘配置
    TemporalFilterConfig temporal;      ///< 时序滤波配置
    // 注意：预处理功能已重构为独立 Filter (DensityFilter, MotionFilter)
    //       请在 Processor 管线中配置
};

} // namespace Linger
