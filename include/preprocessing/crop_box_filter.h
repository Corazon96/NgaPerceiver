#pragma once

#include "preprocessing/filter.h"
#include <mutex>
#include <atomic>

namespace Linger {

/**
 * @brief 轴对齐 ROI 裁剪过滤器（世界/船体坐标系）。
 * 通过设定 x/y/z 范围保留兴趣区域内的点。
 */
class CropBoxFilter : public Filter {
public:
    CropBoxFilter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    ~CropBoxFilter() override = default;

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;
    std::string name() const override { return "CropBoxFilter"; }
    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

    void setBounds(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max);
    void setEnabled(bool enabled);

private:
    std::mutex mutex_;
    float x_min_;
    float x_max_;
    float y_min_;
    float y_max_;
    float z_min_;
    float z_max_;
    bool enabled_{true};
    std::atomic<size_t> filtered_count_{0};
};

} // namespace Linger
