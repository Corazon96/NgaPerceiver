#pragma once

#include "preprocessing/filter.h"
#include <pcl/filters/voxel_grid.h>
#include <mutex>
#include <atomic>

namespace Linger {

class VoxelFilter : public Filter {
public:
    VoxelFilter(float leaf_size = 0.1f);
    ~VoxelFilter() override = default;

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;
    std::string name() const override { return "VoxelGrid Filter"; }
    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

    void setLeafSize(float s);
    void setEnabled(bool enabled);

private:
    std::mutex mutex_;
    float leaf_size_;
    bool enabled_{false}; // 默认关闭，由 UI 控制开启
    std::atomic<size_t> filtered_count_{0};
};

} // namespace Linger
