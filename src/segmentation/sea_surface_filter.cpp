#include "segmentation/sea_surface_filter.h"

using namespace Linger;

SeaSurfaceFilter::SeaSurfaceFilter(float sea_level_z, float margin)
    : sea_level_z_(sea_level_z), margin_(margin) {}

void SeaSurfaceFilter::setSeaLevel(float sea_level_z)
{
    std::lock_guard<std::mutex> lock(mutex_);
    sea_level_z_ = sea_level_z;
}

void SeaSurfaceFilter::setMargin(float margin)
{
    std::lock_guard<std::mutex> lock(mutex_);
    margin_ = margin;
}

void SeaSurfaceFilter::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

void SeaSurfaceFilter::filter(const PointCloudPtr& input, PointCloudPtr& output)
{
    if (!input) {
        return;
    }

    if (!output) {
        output = std::make_shared<PointCloud>();
    }

    // 获取参数快照（短时间持锁）
    float current_sea_level;
    float current_margin;
    bool current_enabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_sea_level = sea_level_z_;
        current_margin = margin_;
        current_enabled = enabled_;
    }

    if (!current_enabled) {
        if (input != output) {
            *output = *input;
        }
        filtered_count_.store(0);
        return;
    }

    PointCloudPtr out = output;
    if (!out || out == input) {
        out = std::make_shared<pcl::PointCloud<Point>>();
    }

    const float threshold = current_sea_level + current_margin;
    const size_t in_size = input->points.size();

    out->points.clear();
    out->points.reserve(input->points.size());

    for (const auto& p : input->points) {
        if (p.z >= threshold) {
            out->points.push_back(p);
        }
    }

    out->width = static_cast<uint32_t>(out->points.size());
    out->height = 1;
    out->is_dense = input->is_dense;

    size_t out_size = out->points.size();
    filtered_count_.store(in_size > out_size ? in_size - out_size : 0);

    output = out;
}
