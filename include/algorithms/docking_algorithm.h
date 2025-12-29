#pragma once

#include "core/common.h"
#include "algorithms/docking_types.h"
#include <mutex>
#include <atomic>
#include <functional>

namespace Linger {

/**
 * @brief 靠泊检测算法
 * 
 * 包含两种检测模式：
 * 1. 最近区域距离检测 - 简单高效，计算扇区内最近点的距离
 * 2. 码头边缘检测 - 通过Z轴切片RANSAC拟合边缘直线
 * 
 * 参考系说明：
 * - 所有检测以雷达坐标系原点为参考点
 * - X轴指向前方，Y轴指向左侧，Z轴向上
 * 
 * 线程安全说明：
 * - processMultiple() 可从任意线程调用
 * - 内部使用 mutex 保护状态
 */
class DockingAlgorithm {
public:
    DockingAlgorithm();
    ~DockingAlgorithm();

    /**
     * @brief 处理多个点云块（避免合并拷贝）
     * @param clouds 输入点云块列表
     * @param timestamp_ns 时间戳（纳秒）
     * @return 综合靠泊状态
     */
    DockingState processMultiple(const std::vector<PointCloudPtr>& clouds, uint64_t timestamp_ns);

    /**
     * @brief 设置配置
     */
    void setConfig(const DockingConfig& config);
    DockingConfig getConfig() const;
    
    /**
     * @brief 分别设置两种检测的配置
     */
    void setNearestConfig(const NearestRegionConfig& config);
    void setEdgeConfig(const DockEdgeConfig& config);
    NearestRegionConfig getNearestConfig() const;
    DockEdgeConfig getEdgeConfig() const;

    /**
     * @brief 获取最近一次的检测结果
     */
    DockingState getLastState() const;

    /**
     * @brief 获取当前的稳定性评分 (0-1)
     * @return 稳定性评分，1 表示非常稳定，0 表示不稳定
     * @note 可用于判断是否可以开始自动靠泊等操作
     */
    float getStabilityScore() const;
    
    /**
     * @brief 获取连续有效帧数
     */
    int getConsecutiveValidFrames() const;

    /**
     * @brief 重置状态
     */
    void reset();

    /**
     * @brief 状态更新回调
     */
    std::function<void(const DockingState&)> onStateUpdated;

private:
    //=========================================================================
    // Layer 2: 多假设边缘检测
    //=========================================================================
    
    /**
     * @brief 直线假设（RANSAC 候选）
     */
    struct LineHypothesis {
        Line2D line;                    ///< 直线参数
        std::vector<size_t> inliers;   ///< 内点索引
        float inlier_ratio = 0.0f;      ///< 内点比例
        float mean_error = 0.0f;        ///< 平均拟合误差
        float geometry_score = 1.0f;    ///< 几何约束评分
        float history_score = 0.0f;     ///< 历史一致性评分
        float total_score = 0.0f;       ///< 综合评分
    };
    
    /**
     * @brief 多假设 RANSAC 直线拟合（输出 Top-N 候选）
     * @param points 输入 2D 点集
     * @param cfg 边缘检测配置
     * @param max_hypotheses 最大候选数量
     * @return 按评分排序的候选直线列表
     */
    std::vector<LineHypothesis> fitLineRansacMulti(
        const std::vector<std::pair<float, float>>& points,
        const DockEdgeConfig& cfg,
        int max_hypotheses = 3);
    
    /**
     * @brief 计算直线的几何约束评分
     * @param hypothesis 直线假设
     * @param cfg 边缘检测配置
     */
    void evaluateGeometry(LineHypothesis& hypothesis, const DockEdgeConfig& cfg);
    
    /**
     * @brief 计算直线与历史的一致性评分
     * @param hypothesis 直线假设
     */
    void evaluateHistoryConsistency(LineHypothesis& hypothesis);
    
    /**
     * @brief 判断两条直线是否足够不同（避免重复候选）
     * @param line1 直线1
     * @param line2 直线2
     * @return true 如果差异足够大
     */
    bool areLinesDistinct(const Line2D& line1, const Line2D& line2) const;

    //=========================================================================
    // 最近区域距离检测
    //=========================================================================
    
    /**
     * @brief 执行最近区域距离检测
     */
    NearestRegionResult detectNearestRegion(const std::vector<PointCloudPtr>& clouds);

    /**
     * @brief 计算最近区域统计信息（内部辅助函数）
     */
    NearestRegionResult computeNearestStatistics(
        const std::vector<float>& distances, 
        const std::vector<std::pair<float, float>>& positions,
        const NearestRegionConfig& cfg);
    
    //=========================================================================
    // 码头边缘检测
    //=========================================================================
    
    /**
     * @brief 执行码头边缘检测
     */
    DockEdgeResult detectDockEdge(const std::vector<PointCloudPtr>& clouds);
    
    /**
     * @brief 使用最小二乘法精化直线
     */
    void refineLine(const std::vector<std::pair<float, float>>& points,
                    const std::vector<size_t>& inliers,
                    Line2D& line);
    
    /**
     * @brief 计算点到直线的距离
     */
    static float pointToLineDistance(float x, float y, const Line2D& line);
    
    /**
     * @brief 计算直线端点（用于可视化）
     */
    void computeLineEndpoints(const std::vector<std::pair<float, float>>& points,
                              const std::vector<size_t>& inliers,
                              Line2D& line);
    
    /**
     * @brief 交叉验证融合
     */
    void crossValidate(DockingState& state);

    /**
     * @brief 时序滤波更新
     */
    void updateTemporalFilter(DockingState& state);

    //=========================================================================
    // 内部状态
    //=========================================================================
    
    mutable std::mutex mutex_;
    DockingConfig config_;
    DockingState last_state_;
    
    // Layer 2 多假设历史数据
    Line2D last_valid_edge_line_;           ///< 上一次有效的边缘直线
    bool has_valid_edge_history_ = false;   ///< 是否有历史边缘数据
    
    /**
     * @brief 增强的时序滤波器 (基于卡尔曼滤波思想)
     * 
     * 作用：
     * 1. 状态预测 - 根据历史速度预测下一时刻的距离和角度
     * 2. 异常检测 - 检测距离和角度的异常跳变
     * 3. 融合更新 - 用测量值修正预测值，平滑输出
     * 4. 稳定性评估 - 跟踪连续有效帧数，评估长期稳定性
     * 
     * 原理：
     * - 预测: dist[k] = dist[k-1] + velocity * dt
     * - 检测: 如果 |测量值 - 预测值| > max_jump → 忽略测量值
     * - 更新: dist[k] = 预测值 + K * (测量值 - 预测值)
     *         其中 K 是卡尔曼增益，在0-1之间，控制信任度
     * 
     * 解决的问题：
     * - 防止单次测量误差导致距离/角度突变
     * - 当检测暂时丢失时，用预测值填充，保持输出连续性
     * - 通过稳定性评分支持上层决策（如自动靠泊）
     * 
     * 适用时机：
     * - 在两种检测算法融合之后应用
     * - 对 final_distance_m 和 final_angle_deg 进行全局时序滤波
     */
    struct TemporalFilter {
        // 距离滤波
        float predicted_dist = 0.0f;       ///< 预测的距离值 (m)
        float predicted_velocity = 0.0f;   ///< 估计的接近速度 (m/s，正值表示靠近)
        float dist_error_cov = 1.0f;       ///< 距离预测误差协方差
        
        // 角度滤波
        float predicted_angle = 0.0f;      ///< 预测的角度值 (°)
        float predicted_angle_rate = 0.0f; ///< 估计的角度变化率 (°/s)
        float angle_error_cov = 1.0f;      ///< 角度预测误差协方差
        
        // 稳定性跟踪
        int consecutive_valid_frames = 0;   ///< 连续有效帧数
        int consecutive_invalid_frames = 0; ///< 连续无效帧数
        int consecutive_jump_frames = 0;    ///< 连续跳变帧数
        float stability_score = 0.0f;       ///< 稳定性评分 (0-1)，1表示非常稳定
        
        // 通用状态
        uint64_t last_update_ns = 0;       ///< 上次更新的时间戳（纳秒）
        bool initialized = false;          ///< 是否已初始化（第一次测量后为true）
        
        void reset() {
            predicted_dist = 0.0f;
            predicted_velocity = 0.0f;
            dist_error_cov = 1.0f;
            predicted_angle = 0.0f;
            predicted_angle_rate = 0.0f;
            angle_error_cov = 1.0f;
            consecutive_valid_frames = 0;
            consecutive_invalid_frames = 0;
            consecutive_jump_frames = 0;
            stability_score = 0.0f;
            last_update_ns = 0;
            initialized = false;
        }
    } filter_;

    /**
     * @brief 平滑滤波历史 (指数移动平均, Exponential Moving Average)
     * 
     * 作用：
     * 1. 减少高频噪声 - 平滑单帧测量的抖动
     * 2. 独立处理 - 分别对两种检测算法的输出进行平滑
     * 3. 快速响应 - 通过 smoothing_alpha 调节响应速度
     * 
     * 原理：
     * - smooth[k] = α * measure[k] + (1-α) * smooth[k-1]
     * - α ∈ [0,1]: α越大响应越快，越小越平滑
     * - 默认 α=0.3: 保留30%新测量 + 70%历史平均
     * 
     * 与 TemporalFilter 的区别：
     * - 平滑滤波: 简单的指数平均，无预测能力，只是平滑历史值
     * - 时序滤波: 卡尔曼滤波，有预测能力，能检测和拒绝异常值
     * 
     * 执行顺序：
     * 1. 先在各检测算法内部应用平滑滤波（Nearest/Edge独立）
     * 2. 融合后再应用时序滤波（全局）
     * 
     * 适用场景：
     * - 当测量值本身较稳定，只需减少噪声时
     * - 需要保留原始检测的响应速度时
     */
    float smoothed_nearest_dist_ = 0.0f;   ///< 最近区域检测的平滑距离 (m)
    float smoothed_edge_dist_ = 0.0f;      ///< 边缘检测的平滑距离 (m)
    float smoothed_edge_angle_ = 0.0f;     ///< 边缘检测的平滑角度 (°)
    bool has_previous_nearest_ = false;    ///< 是否有最近区域检测的历史值
    bool has_previous_edge_ = false;       ///< 是否有边缘检测的历史值
};

} // namespace Linger
