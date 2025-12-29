#include "preprocessing/statistical_outlier_filter.h"
#include <algorithm>

using namespace Linger;

StatisticalOutlierFilter::StatisticalOutlierFilter(int mean_k, float stddev_mul)
    : mean_k_(mean_k), stddev_mul_(stddev_mul) {}

void StatisticalOutlierFilter::setParameters(int mean_k, float stddev_mul)
{
    std::lock_guard<std::mutex> lock(mutex_);
    mean_k_ = mean_k;
    stddev_mul_ = stddev_mul;
}

void StatisticalOutlierFilter::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

void StatisticalOutlierFilter::filter(const PointCloudPtr& input, PointCloudPtr& output)
{
    if (!input) {
        return;
    }

    if (!output) {
        output = std::make_shared<PointCloud>();
    }

    // 获取参数快照（短时间持锁）
    int current_mean_k;
    float current_stddev_mul;
    bool current_enabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_mean_k = mean_k_;
        current_stddev_mul = stddev_mul_;
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

    const size_t in_size = input->points.size();

    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(input);
    sor.setMeanK(std::max(1, current_mean_k));
    sor.setStddevMulThresh(current_stddev_mul);
    sor.filter(*out);

    size_t out_size = out->points.size();
    filtered_count_.store(in_size > out_size ? in_size - out_size : 0);

    output = out;
}
