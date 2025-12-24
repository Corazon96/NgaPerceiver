#pragma once

#include "preprocessing/filter.h"
#include <mutex>
#include <atomic>
#include <map>
#include <tuple>

namespace Linger {

/**
 * @brief 运动滤波器（动静分离）
 * 
 * 通过帧间差分检测运动物体，可配置为：
 * - 输出静态点（过滤动态目标）
 * - 输出动态点（提取运动目标）
 * 
 * 原理：
 * - 使用空间网格索引历史帧
 * - 对当前帧的每个点，在历史帧的邻域内搜索匹配点
 * - 如果找到匹配（距离 < 阈值），认为是静态点
 * 
 * 注意：
 * - 此滤波器有状态（保存历史帧），适合在 Sensor Frame 执行
 * - 计算开销较大（O(n) 空间索引构建 + O(n×27) 邻域搜索）
 */
class MotionFilter : public Filter {
public:
    /**
     * @brief 构造函数
     * @param cell_size 空间网格大小(m)，默认0.5m
     * @param motion_threshold 运动阈值(m)，默认0.3m
     */
    MotionFilter(float cell_size = 0.5f, float motion_threshold = 0.3f);
    ~MotionFilter() override = default;

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;
    std::string name() const override { return "Motion Filter"; }
    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

    /** @brief 设置网格大小 */
    void setCellSize(float size);
    float getCellSize() const;

    /** @brief 设置运动阈值 */
    void setMotionThreshold(float threshold);
    float getMotionThreshold() const;

    /** @brief 启用/禁用滤波器 */
    void setEnabled(bool enabled);
    bool isEnabled() const;

    /** @brief 设置输出模式：true=输出静态点，false=输出动态点 */
    void setOutputStatic(bool output_static);
    bool getOutputStatic() const;

    /** @brief 清空历史帧 */
    void reset();

    /** @brief 获取最近一次检测到的动态点数 */
    size_t getLastDynamicCount() const { return dynamic_count_.load(); }

    /** @brief 获取最近一次检测到的静态点数 */
    size_t getLastStaticCount() const { return static_count_.load(); }

private:
    using GridKey = std::tuple<int, int, int>;
    using GridMap = std::map<GridKey, std::vector<Point>>;

    mutable std::mutex mutex_;
    float cell_size_;
    float motion_threshold_;
    bool enabled_{false};       // 默认关闭
    bool output_static_{true};  // 默认输出静态点
    
    // 历史帧网格索引
    GridMap history_grid_;
    PointCloudPtr previous_cloud_;
    
    std::atomic<size_t> filtered_count_{0};
    std::atomic<size_t> dynamic_count_{0};
    std::atomic<size_t> static_count_{0};
};

} // namespace Linger
