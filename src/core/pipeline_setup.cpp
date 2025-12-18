#include "core/pipeline_setup.h"
#include "core/processor.h"
#include "ui/point_cloud_wgt.h"
#include "preprocessing/distance_filter.h"
#include "preprocessing/voxel_filter.h"
#include "preprocessing/crop_box_filter.h"
#include "segmentation/sea_surface_filter.h"
#include "preprocessing/statistical_outlier_filter.h"
#include "core/config.h"
#include <memory>
#include <QObject>

void SetupFilterPipeline(PointCloudProcessor& proc, PointCloudWgt& w)
{
    /**
     * @brief 添加一个简单的距离滤波器 (Sensor Frame)
     * 过滤掉距离小于 0.1m 和大于 100m 的点
     */
    auto dist_filter = std::make_shared<Linger::DistanceFilter>(LingerConfig::FILTER_DISTANCE_MIN_DEFAULT, LingerConfig::FILTER_DISTANCE_MAX_DEFAULT);
    proc.addFilter(dist_filter, false); // false = pre-process (sensor frame)

    /**
     * @brief ROI 裁剪 (World Frame)
     */
    auto crop_filter = std::make_shared<Linger::CropBoxFilter>(
        LingerConfig::ROI_X_MIN_DEFAULT, LingerConfig::ROI_X_MAX_DEFAULT,
        LingerConfig::ROI_Y_MIN_DEFAULT, LingerConfig::ROI_Y_MAX_DEFAULT,
        LingerConfig::ROI_Z_MIN_DEFAULT, LingerConfig::ROI_Z_MAX_DEFAULT);
    crop_filter->setEnabled(false); // 先默认关闭，避免裁掉全部点
    proc.addFilter(crop_filter, true);
    QObject::connect(&w, &PointCloudWgt::roiBoundsChanged, [crop_filter](double xmin, double xmax, double ymin, double ymax, double zmin, double zmax){
        crop_filter->setBounds(static_cast<float>(xmin), static_cast<float>(xmax), static_cast<float>(ymin), static_cast<float>(ymax), static_cast<float>(zmin), static_cast<float>(zmax));
    });
    QObject::connect(&w, &PointCloudWgt::roiEnabledChanged, [crop_filter](bool enabled){ crop_filter->setEnabled(enabled); });

    /**
     * @brief 海面滤除 (World Frame)
     */
    auto sea_filter = std::make_shared<Linger::SeaSurfaceFilter>(
        LingerConfig::SEA_LEVEL_Z_DEFAULT,
        LingerConfig::SEA_MARGIN_DEFAULT);
    sea_filter->setEnabled(false); // 默认关闭，现场按安装高度再开启
    proc.addFilter(sea_filter, true);
    QObject::connect(&w, &PointCloudWgt::seaLevelChanged, [sea_filter](double z){ sea_filter->setSeaLevel(static_cast<float>(z)); });
    QObject::connect(&w, &PointCloudWgt::seaMarginChanged, [sea_filter](double m){ sea_filter->setMargin(static_cast<float>(m)); });
    QObject::connect(&w, &PointCloudWgt::seaEnabledChanged, [sea_filter](bool enabled){ sea_filter->setEnabled(enabled); });

    /**
     * @brief 统计离群点滤波 (World Frame)
     */
    auto stat_filter = std::make_shared<Linger::StatisticalOutlierFilter>(
        LingerConfig::STAT_OUTLIER_MEAN_K_DEFAULT,
        LingerConfig::STAT_OUTLIER_STDDEV_MUL_DEFAULT);
    stat_filter->setEnabled(false); // 默认关闭，防止小样本全被剔除
    proc.addFilter(stat_filter, true);
    QObject::connect(&w, &PointCloudWgt::outlierParamsChanged, [stat_filter](int k, double s){ stat_filter->setParameters(k, static_cast<float>(s)); });
    QObject::connect(&w, &PointCloudWgt::outlierEnabledChanged, [stat_filter](bool enabled){ stat_filter->setEnabled(enabled); });

    /**
     * @brief 添加体素滤波器 (World Frame / Post-Process)
     * 默认 0.1m, 默认关闭 (由 UI 控制)
     */
    auto voxel_filter = std::make_shared<Linger::VoxelFilter>(LingerConfig::FILTER_VOXEL_SIZE_DEFAULT);
    proc.addFilter(voxel_filter, true); // true = post-process (world frame)

    /**
     * @brief 连接距离滤波控制
     * 注意：dist_filter 被 lambda 捕获（shared_ptr 拷贝），保证了生命周期安全
     */
    QObject::connect(&w, &PointCloudWgt::minDistChanged, [dist_filter](double d) {
        dist_filter->setMinDistance(static_cast<float>(d));
    });
    QObject::connect(&w, &PointCloudWgt::maxDistChanged, [dist_filter](double d) {
        dist_filter->setMaxDistance(static_cast<float>(d));
    });
    QObject::connect(&w, &PointCloudWgt::distEnabledChanged, [dist_filter](bool enabled) {
        dist_filter->setEnabled(enabled);
    });

    /**
     * @brief 连接体素滤波控制
     */
    QObject::connect(&w, &PointCloudWgt::voxelSizeChanged, [voxel_filter](double s) {
        voxel_filter->setLeafSize(static_cast<float>(s));
    });
    QObject::connect(&w, &PointCloudWgt::voxelEnabledChanged, [voxel_filter](bool enabled) {
        voxel_filter->setEnabled(enabled);
    });
}
