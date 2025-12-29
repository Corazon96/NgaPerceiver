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
    if (!input) {
        return;
    }

    if (!output) {
        output = std::make_shared<PointCloud>();
    }

    // 获取参数快照（短时间持锁）
    float current_x_min, current_x_max;
    float current_y_min, current_y_max;
    float current_z_min, current_z_max;
    bool current_enabled;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_x_min = x_min_; current_x_max = x_max_;
        current_y_min = y_min_; current_y_max = y_max_;
        current_z_min = z_min_; current_z_max = z_max_;
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

    pcl::CropBox<Point> crop;
    crop.setMin(Eigen::Vector4f(current_x_min, current_y_min, current_z_min, 1.0f));
    crop.setMax(Eigen::Vector4f(current_x_max, current_y_max, current_z_max, 1.0f));
    crop.setInputCloud(input);
    crop.filter(*out);

    size_t out_size = out->points.size();
    filtered_count_.store(in_size > out_size ? in_size - out_size : 0);

    output = out;
}
