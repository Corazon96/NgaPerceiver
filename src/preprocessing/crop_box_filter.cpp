#include "preprocessing/crop_box_filter.h"
#include <pcl/filters/crop_box.h>
#include <Eigen/Core>

using namespace Linger;

CropBoxFilter::CropBoxFilter(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
    : x_min_(x_min), x_max_(x_max), y_min_(y_min), y_max_(y_max), z_min_(z_min), z_max_(z_max) {}

void CropBoxFilter::setBounds(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max)
{
    std::lock_guard<std::mutex> lock(mutex_);
    x_min_ = x_min; x_max_ = x_max;
    y_min_ = y_min; y_max_ = y_max;
    z_min_ = z_min; z_max_ = z_max;
}

void CropBoxFilter::setEnabled(bool enabled)
{
    std::lock_guard<std::mutex> lock(mutex_);
    enabled_ = enabled;
}

void CropBoxFilter::filter(const PointCloudPtr& input, PointCloudPtr& output)
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

    pcl::CropBox<Point> crop;
    crop.setMin(Eigen::Vector4f(x_min_, y_min_, z_min_, 1.0f));
    crop.setMax(Eigen::Vector4f(x_max_, y_max_, z_max_, 1.0f));
    crop.setInputCloud(input);
    crop.filter(*out);

    size_t out_size = out->points.size();
    filtered_count_.store(in_size > out_size ? in_size - out_size : 0);

    output = out;
}
