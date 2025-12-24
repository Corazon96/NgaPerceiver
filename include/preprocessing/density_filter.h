#pragma once

#include "preprocessing/filter.h"
#include <mutex>
#include <atomic>

namespace Linger {

/**
 * @brief 密度滤波器
 * 
 * 使用体素网格统计，保留密度足够的区域内的点。
 * 与 VoxelFilter 的区别：
 * - VoxelFilter: 每个体素只保留一个代表点（降采样）
 * - DensityFilter: 保留密度足够的体素内的所有点（去除稀疏噪声）
 * 
 * 适用场景：
 * - 去除远距离稀疏噪声点
 * - 去除多路径反射等离散噪点
 */
class DensityFilter : public Filter {
public:
    /**
     * @brief 构造函数
     * @param voxel_size 体素大小(m)，默认0.3m
     * @param min_points 体素内最少点数，默认3
     */
    DensityFilter(float voxel_size = 0.3f, int min_points = 3);
    ~DensityFilter() override = default;

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;
    std::string name() const override { return "Density Filter"; }
    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

    /** @brief 设置体素大小 */
    void setVoxelSize(float size);
    float getVoxelSize() const;

    /** @brief 设置每个体素的最小点数 */
    void setMinPoints(int min_points);
    int getMinPoints() const;

    /** @brief 启用/禁用滤波器 */
    void setEnabled(bool enabled);
    bool isEnabled() const;

private:
    mutable std::mutex mutex_;
    float voxel_size_;
    int min_points_;
    bool enabled_{false};  // 默认关闭
    std::atomic<size_t> filtered_count_{0};
};

} // namespace Linger
