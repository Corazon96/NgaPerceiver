#pragma once

#include "preprocessing/filter.h"
#include <mutex>
#include <atomic>

namespace Linger {

/**
 * @brief 海面滤除：丢弃低于 sea_level + margin 的点。
 */
class SeaSurfaceFilter : public Filter {
public:
    SeaSurfaceFilter(float sea_level_z, float margin);
    ~SeaSurfaceFilter() override = default;

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;
    std::string name() const override { return "SeaSurfaceFilter"; }
    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

    void setSeaLevel(float sea_level_z);
    void setMargin(float margin);
    void setEnabled(bool enabled);

private:
    std::mutex mutex_;
    float sea_level_z_;
    float margin_;
    bool enabled_{true};
    std::atomic<size_t> filtered_count_{0};
};

} // namespace Linger
