#include "preprocessing/voxel_filter.h"

namespace Linger {

VoxelFilter::VoxelFilter(float leaf_size) : leaf_size_(leaf_size)
{
}

void VoxelFilter::setLeafSize(float s)
{
    std::lock_guard<std::mutex> lock(mutex_);
    leaf_size_ = s;
}

void VoxelFilter::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

void VoxelFilter::filter(const PointCloudPtr& input, PointCloudPtr& output)
{
    if (!input) {
        return;
    }

    // 确保 output 可用，避免未启用时的空指针拷贝
    if (!output) {
        output = std::make_shared<PointCloud>();
    }

    float current_leaf_size;
    bool current_enabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_leaf_size = leaf_size_;
        current_enabled = enabled_;
    }

    if (!current_enabled) {
        // 如果未启用，直接透传
        if (input != output) {
            *output = *input;
        }
        filtered_count_.store(0);
        return;
    }

    if (input->empty()) {
        output->clear();
        output->width = 0;
        output->height = 1;
        filtered_count_.store(0);
        return;
    }

    // 执行体素滤波
    pcl::VoxelGrid<Point> voxel_grid;
    voxel_grid.setInputCloud(input);
    voxel_grid.setLeafSize(current_leaf_size, current_leaf_size, current_leaf_size);

    // 记录原始大小，因为如果是原地过滤，input 的大小会被修改
    size_t original_size = input->points.size();

    // PCL 的 filter 函数处理原地过滤 (input == output) 是安全的
    voxel_grid.filter(*output);

    size_t out_size = output->points.size();
    if (original_size >= out_size) {
        filtered_count_.store(original_size - out_size);
    } else {
        filtered_count_.store(0);
    }
}

} // namespace Linger
