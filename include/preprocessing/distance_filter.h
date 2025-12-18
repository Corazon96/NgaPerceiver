#pragma once

#include "preprocessing/filter.h"
#include <mutex>
#include <atomic>

namespace Linger {

/**
 * @brief 简单的距离滤波器 (Distance/Range Filter)
 * 过滤掉距离原点过近或过远的点。通常用于去除雷达自身的盲区噪点或过远的不稳定点。
 * 适用于 Sensor Frame (雷达坐标系)。
 */
class DistanceFilter : public Filter {
public:
    DistanceFilter(float min_dist, float max_dist);

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;

    std::string name() const override { return "DistanceFilter"; }

    void setMinDistance(float d);
    void setMaxDistance(float d);
    void setEnabled(bool enabled);

    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

private:
    float min_dist_sq_;
    float max_dist_sq_;
    bool enabled_{false};
    mutable std::mutex mutex_;
    std::atomic<size_t> filtered_count_{0};
};

} // namespace Linger
