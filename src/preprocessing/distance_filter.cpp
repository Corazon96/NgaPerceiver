#include "preprocessing/distance_filter.h"
#include <cmath>

namespace Linger {

DistanceFilter::DistanceFilter(float min_dist, float max_dist) 
    : min_dist_sq_(min_dist * min_dist), max_dist_sq_(max_dist * max_dist) 
{
}

void DistanceFilter::setMinDistance(float d) {
    std::lock_guard<std::mutex> lock(mutex_);
    min_dist_sq_ = d * d;
}

void DistanceFilter::setMaxDistance(float d) {
    std::lock_guard<std::mutex> lock(mutex_);
    max_dist_sq_ = d * d;
}

void DistanceFilter::setEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

void DistanceFilter::filter(const PointCloudPtr& input, PointCloudPtr& output) 
{
    if (!input) return;

    // 获取当前的阈值快照
    float current_min_sq, current_max_sq;
    bool current_enabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_min_sq = min_dist_sq_;
        current_max_sq = max_dist_sq_;
        current_enabled = enabled_;
    }

    // 如果 output 未初始化，创建一个新的
    if (!output) {
        output = std::make_shared<pcl::PointCloud<Point>>();
    }

    if (!current_enabled) {
        // 如果未启用，直接透传
        if (input != output) {
            *output = *input;
        }
        filtered_count_.store(0);
        return;
    }

    /**
     * @brief 如果 input 和 output 是同一个对象，我们需要原地过滤
     * 预分配空间 (最坏情况)
     */
    output->points.reserve(input->points.size());
    output->header = input->header;
    output->width = 0;
    output->height = 1;
    output->is_dense = input->is_dense;

    // 如果是同一个指针，我们需要小心处理
    if (input == output) {
        // 原地过滤：使用双指针法
        size_t original_size = input->points.size();
        size_t keep_idx = 0;
        for (size_t i = 0; i < input->points.size(); ++i) {
            const auto& pt = input->points[i];
            float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            
            if (dist_sq >= current_min_sq && dist_sq <= current_max_sq) {
                if (keep_idx != i) {
                    input->points[keep_idx] = pt;
                }
                keep_idx++;
            }
        }
        input->points.resize(keep_idx);
        input->width = static_cast<uint32_t>(keep_idx);
        filtered_count_.store(original_size - keep_idx);
    } else {
        // 异地过滤
        output->points.clear();
        size_t filtered = 0;
        for (const auto& pt : input->points) {
            float dist_sq = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
            
            if (dist_sq >= current_min_sq && dist_sq <= current_max_sq) {
                output->points.push_back(pt);
            } else {
                filtered++;
            }
        }
        output->width = static_cast<uint32_t>(output->points.size());
        filtered_count_.store(filtered);
    }
}

} // namespace Linger
