#include "preprocessing/motion_filter.h"
#include "core/logger.h"
#include <cmath>

namespace Linger {

MotionFilter::MotionFilter(float cell_size, float motion_threshold)
    : cell_size_(cell_size)
    , motion_threshold_(motion_threshold)
{
}

void MotionFilter::setCellSize(float size)
{
    std::lock_guard<std::mutex> lock(mutex_);
    cell_size_ = size;
}

float MotionFilter::getCellSize() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return cell_size_;
}

void MotionFilter::setMotionThreshold(float threshold)
{
    std::lock_guard<std::mutex> lock(mutex_);
    motion_threshold_ = threshold;
}

float MotionFilter::getMotionThreshold() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return motion_threshold_;
}

void MotionFilter::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

bool MotionFilter::isEnabled() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return enabled_;
}

void MotionFilter::setOutputStatic(bool output_static)
{
    std::lock_guard<std::mutex> lock(mutex_);
    output_static_ = output_static;
}

bool MotionFilter::getOutputStatic() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return output_static_;
}

void MotionFilter::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    history_grid_.clear();
    previous_cloud_.reset();
}

void MotionFilter::filter(const PointCloudPtr& input, PointCloudPtr& output)
{
    if (!input) {
        return;
    }

    if (!output) {
        output = std::make_shared<PointCloud>();
    }

    float current_cell_size;
    float current_threshold;
    bool current_enabled;
    bool current_output_static;
    GridMap current_history;
    PointCloudPtr current_previous;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_cell_size = cell_size_;
        current_threshold = motion_threshold_;
        current_enabled = enabled_;
        current_output_static = output_static_;
        current_history = history_grid_;  // 复制历史（避免长时间持锁）
        current_previous = previous_cloud_;
    }

    if (!current_enabled) {
        if (input != output) {
            *output = *input;
        }
        filtered_count_.store(0);
        dynamic_count_.store(0);
        static_count_.store(input->points.size());
        
        // 更新历史帧（即使未启用也要更新，以便启用时有历史数据）
        {
            std::lock_guard<std::mutex> lock(mutex_);
            previous_cloud_ = input;
            // 重建网格索引
            history_grid_.clear();
            for (const auto& pt : input->points) {
                int gx = static_cast<int>(std::floor(pt.x / current_cell_size));
                int gy = static_cast<int>(std::floor(pt.y / current_cell_size));
                int gz = static_cast<int>(std::floor(pt.z / current_cell_size));
                history_grid_[{gx, gy, gz}].push_back(pt);
            }
        }
        return;
    }

    if (input->empty()) {
        output->clear();
        output->width = 0;
        output->height = 1;
        filtered_count_.store(0);
        dynamic_count_.store(0);
        static_count_.store(0);
        return;
    }

    size_t original_size = input->points.size();

    // 如果没有历史帧，全部视为静态
    if (!current_previous || current_previous->empty() || current_history.empty()) {
        if (current_output_static) {
            if (input != output) {
                *output = *input;
            }
        } else {
            output->clear();
        }
        
        filtered_count_.store(current_output_static ? 0 : original_size);
        dynamic_count_.store(0);
        static_count_.store(original_size);

        // 保存当前帧作为历史
        {
            std::lock_guard<std::mutex> lock(mutex_);
            previous_cloud_ = input;
            history_grid_.clear();
            for (const auto& pt : input->points) {
                int gx = static_cast<int>(std::floor(pt.x / current_cell_size));
                int gy = static_cast<int>(std::floor(pt.y / current_cell_size));
                int gz = static_cast<int>(std::floor(pt.z / current_cell_size));
                history_grid_[{gx, gy, gz}].push_back(pt);
            }
        }
        return;
    }

    // 分离静态和动态点
    PointCloudPtr static_cloud = std::make_shared<PointCloud>();
    PointCloudPtr dynamic_cloud = std::make_shared<PointCloud>();
    static_cloud->points.reserve(original_size);
    dynamic_cloud->points.reserve(original_size / 10);  // 预计动态点较少

    const float threshold_sq = current_threshold * current_threshold;

    for (const auto& pt : input->points) {
        int gx = static_cast<int>(std::floor(pt.x / current_cell_size));
        int gy = static_cast<int>(std::floor(pt.y / current_cell_size));
        int gz = static_cast<int>(std::floor(pt.z / current_cell_size));

        bool is_static = false;

        // 搜索 27 邻域
        for (int dx = -1; dx <= 1 && !is_static; ++dx) {
            for (int dy = -1; dy <= 1 && !is_static; ++dy) {
                for (int dz = -1; dz <= 1 && !is_static; ++dz) {
                    auto it = current_history.find({gx + dx, gy + dy, gz + dz});
                    if (it != current_history.end()) {
                        for (const auto& hist_pt : it->second) {
                            float dist_sq = (pt.x - hist_pt.x) * (pt.x - hist_pt.x) +
                                           (pt.y - hist_pt.y) * (pt.y - hist_pt.y) +
                                           (pt.z - hist_pt.z) * (pt.z - hist_pt.z);
                            if (dist_sq < threshold_sq) {
                                is_static = true;
                                break;
                            }
                        }
                    }
                }
            }
        }

        if (is_static) {
            static_cloud->points.push_back(pt);
        } else {
            dynamic_cloud->points.push_back(pt);
        }
    }

    // 根据输出模式选择结果
    if (current_output_static) {
        output->points.swap(static_cloud->points);
        filtered_count_.store(dynamic_cloud->points.size());
    } else {
        output->points.swap(dynamic_cloud->points);
        filtered_count_.store(static_cloud->points.size());
    }

    output->width = output->points.size();
    output->height = 1;
    output->is_dense = false;

    dynamic_count_.store(dynamic_cloud->points.size());
    static_count_.store(static_cloud->points.size());

    LOG_TRACE("[MotionFilter] Input: {}, Static: {}, Dynamic: {}", 
              original_size, static_count_.load(), dynamic_count_.load());

    // 更新历史帧
    {
        std::lock_guard<std::mutex> lock(mutex_);
        previous_cloud_ = input;
        history_grid_.clear();
        for (const auto& pt : input->points) {
            int gx = static_cast<int>(std::floor(pt.x / current_cell_size));
            int gy = static_cast<int>(std::floor(pt.y / current_cell_size));
            int gz = static_cast<int>(std::floor(pt.z / current_cell_size));
            history_grid_[{gx, gy, gz}].push_back(pt);
        }
    }
}

} // namespace Linger
