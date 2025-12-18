#pragma once

#include "core/common.h"
#include "algorithms/docking_types.h"
#include <mutex>
#include <atomic>
#include <functional>

namespace Linger {

/**
 * @brief 靠泊检测算�?
 * 
 * 包含两种检测模式：
 * 1. 最近区域距离检�?- 简单高效，计算扇区内最近点的距�?
 * 2. 码头边缘检�?- 通过Z轴切�?RANSAC拟合边缘直线
 * 
 * 参考系�?
 * - 所有检测以雷达坐标系原点为参�?
 * - X轴指向前方，Y轴指向左侧，Z轴向�?
 * 
 * 线程安全�?
 * - process() 可从任意线程调用
 * - 内部使用 mutex 保护状�?
 */
class DockingAlgorithm {
public:
    DockingAlgorithm();
    ~DockingAlgorithm();

    /**
     * @brief 处理一帧点�?
     * @param cloud 输入点云
     * @param timestamp_ns 时间�?
     * @return 综合靠泊状�?
     */
    DockingState process(const PointCloudPtr& cloud, uint64_t timestamp_ns);

    /**
     * @brief 处理多个点云块（避免合并拷贝�?
     * @param clouds 输入点云块列�?
     * @param timestamp_ns 时间�?
     * @return 综合靠泊状�?
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
     * @brief 获取最近一次的检测结�?
     */
    DockingState getLastState() const;

    /**
     * @brief 重置状�?
     */
    void reset();

    /**
     * @brief 状态更新回�?
     */
    std::function<void(const DockingState&)> onStateUpdated;

private:
    //=========================================================================
    // 最近区域距离检�?
    //=========================================================================
    
    /**
     * @brief 执行最近区域距离检�?
     */
    NearestRegionResult detectNearestRegion(const PointCloudPtr& cloud);
    
    /**
     * @brief 执行最近区域距离检测（多点云块版本�?
     */
    NearestRegionResult detectNearestRegionMultiple(const std::vector<PointCloudPtr>& clouds);
    
    //=========================================================================
    // 码头边缘检�?
    //=========================================================================
    
    /**
     * @brief 执行码头边缘检�?
     */
    DockEdgeResult detectDockEdge(const PointCloudPtr& cloud);
    
    /**
     * @brief 执行码头边缘检测（多点云块版本�?
     */
    DockEdgeResult detectDockEdgeMultiple(const std::vector<PointCloudPtr>& clouds);
    
    /**
     * @brief RANSAC 2D 直线拟合
     */
    bool fitLineRansac(const std::vector<std::pair<float, float>>& points,
                       const DockEdgeConfig& cfg,
                       Line2D& line,
                       std::vector<size_t>& inliers);
    
    /**
     * @brief 使用最小二乘法精化直线
     */
    void refineLine(const std::vector<std::pair<float, float>>& points,
                    const std::vector<size_t>& inliers,
                    Line2D& line);
    
    /**
     * @brief 计算点到直线的距�?
     */
    static float pointToLineDistance(float x, float y, const Line2D& line);
    
    /**
     * @brief 计算直线端点（用于可视化�?
     */
    void computeLineEndpoints(const std::vector<std::pair<float, float>>& points,
                              const std::vector<size_t>& inliers,
                              Line2D& line);
    
    //=========================================================================
    // 内部状�?
    //=========================================================================
    
    mutable std::mutex mutex_;
    DockingConfig config_;
    DockingState last_state_;
    
    // 平滑滤波历史
    float smoothed_nearest_dist_ = 0.0f;
    float smoothed_edge_dist_ = 0.0f;
    float smoothed_edge_angle_ = 0.0f;
    bool has_previous_nearest_ = false;
    bool has_previous_edge_ = false;
};

} // namespace Linger
