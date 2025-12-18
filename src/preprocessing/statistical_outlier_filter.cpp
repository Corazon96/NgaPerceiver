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
    std::lock_guard<std::mutex> lock(mutex_);
    filtered_count_.store(0);

    if (!input) {
        output.reset();
        return;
    }
    if (!enabled_) {
        output = input;
        return;
    }

    PointCloudPtr out = output;
    if (!out || out == input) {
        out = std::make_shared<pcl::PointCloud<Point>>();
    }

    const size_t in_size = input->points.size();

    pcl::StatisticalOutlierRemoval<Point> sor;
    sor.setInputCloud(input);
    sor.setMeanK(std::max(1, mean_k_));
    sor.setStddevMulThresh(stddev_mul_);
    sor.filter(*out);

    size_t out_size = out->points.size();
    filtered_count_.store(in_size > out_size ? in_size - out_size : 0);

    output = out;
}
