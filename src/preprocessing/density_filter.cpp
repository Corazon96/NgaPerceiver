#include "preprocessing/density_filter.h"
#include "core/logger.h"
#include <map>
#include <cmath>

namespace Linger {

DensityFilter::DensityFilter(float voxel_size, int min_points)
    : voxel_size_(voxel_size)
    , min_points_(min_points)
{
}

void DensityFilter::setVoxelSize(float size)
{
    std::lock_guard<std::mutex> lock(mutex_);
    voxel_size_ = size;
}

float DensityFilter::getVoxelSize() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return voxel_size_;
}

void DensityFilter::setMinPoints(int min_points)
{
    std::lock_guard<std::mutex> lock(mutex_);
    min_points_ = min_points;
}

int DensityFilter::getMinPoints() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return min_points_;
}

void DensityFilter::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

bool DensityFilter::isEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

void DensityFilter::filter(const PointCloudPtr& input, PointCloudPtr& output)
{
    if (!input) {
        return;
    }

    if (!output) {
        output = std::make_shared<PointCloud>();
    }

    float current_voxel_size;
    int current_min_points;
    bool current_enabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_voxel_size = voxel_size_;
        current_min_points = min_points_;
        current_enabled = enabled_;
    }

    if (!current_enabled) {
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

    size_t original_size = input->points.size();

    // 体素键结构
    struct VoxelKey {
        int x, y, z;
        
        bool operator<(const VoxelKey& other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return z < other.z;
        }
    };

    // 第一遍：分配点到体素
    std::map<VoxelKey, std::vector<size_t>> voxel_map;
    
    for (size_t i = 0; i < input->points.size(); ++i) {
        const auto& pt = input->points[i];
        VoxelKey key{
            static_cast<int>(std::floor(pt.x / current_voxel_size)),
            static_cast<int>(std::floor(pt.y / current_voxel_size)),
            static_cast<int>(std::floor(pt.z / current_voxel_size))
        };
        voxel_map[key].push_back(i);
    }

    // 第二遍：保留密度足够的体素中的点
    // 如果是原地过滤，需要先收集索引
    std::vector<size_t> keep_indices;
    keep_indices.reserve(input->points.size());

    for (const auto& [key, indices] : voxel_map) {
        if (static_cast<int>(indices.size()) >= current_min_points) {
            for (size_t idx : indices) {
                keep_indices.push_back(idx);
            }
        }
    }

    // 构建输出点云
    if (input == output) {
        // 原地过滤：需要创建临时存储
        PointCloud temp;
        temp.points.reserve(keep_indices.size());
        for (size_t idx : keep_indices) {
            temp.points.push_back(input->points[idx]);
        }
        output->points = std::move(temp.points);
    } else {
        output->points.clear();
        output->points.reserve(keep_indices.size());
        for (size_t idx : keep_indices) {
            output->points.push_back(input->points[idx]);
        }
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = false;

    size_t filtered = original_size > output->points.size() ? 
                      original_size - output->points.size() : 0;
    filtered_count_.store(filtered);

    LOG_TRACE("[DensityFilter] Input: {}, Output: {}, Filtered: {}", 
              original_size, output->points.size(), filtered);
}

} // namespace Linger
