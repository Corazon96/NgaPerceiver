#pragma once

#include "preprocessing/filter.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <mutex>
#include <atomic>

namespace Linger {

class StatisticalOutlierFilter : public Filter {
public:
    StatisticalOutlierFilter(int mean_k, float stddev_mul);
    ~StatisticalOutlierFilter() override = default;

    void filter(const PointCloudPtr& input, PointCloudPtr& output) override;
    std::string name() const override { return "StatisticalOutlierFilter"; }
    size_t getLastFilteredCount() const override { return filtered_count_.load(); }

    void setParameters(int mean_k, float stddev_mul);
    void setEnabled(bool enabled);

private:
    std::mutex mutex_;
    int mean_k_;
    float stddev_mul_;
    bool enabled_{true};
    std::atomic<size_t> filtered_count_{0};
};

} // namespace Linger
