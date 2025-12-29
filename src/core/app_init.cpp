#include "core/app_init.h"
#include "core/logger.h"
#include "core/config.h"

#include "preprocessing/distance_filter.h"
#include "preprocessing/crop_box_filter.h"
#include "segmentation/sea_surface_filter.h"
#include "preprocessing/statistical_outlier_filter.h"
#include "preprocessing/voxel_filter.h"
#include "preprocessing/density_filter.h"
#include "preprocessing/motion_filter.h"

#include <filesystem>
#include <chrono>

namespace Linger {

void SetupFiltersFromConfig(PointCloudProcessor& processor, const AppConfig& config)
{
    // Sensor 坐标系滤波器（世界变换前）
    auto dist_filter = std::make_shared<DistanceFilter>(config.distance_min, config.distance_max);
    dist_filter->setEnabled(config.distance_enabled);
    processor.addFilter(dist_filter, false);

    // World 坐标系滤波器（世界变换后）
    auto crop_filter = std::make_shared<CropBoxFilter>(
        config.roi_x_min, config.roi_x_max,
        config.roi_y_min, config.roi_y_max,
        config.roi_z_min, config.roi_z_max);
    crop_filter->setEnabled(config.roi_enabled);
    processor.addFilter(crop_filter, true);

    auto sea_filter = std::make_shared<SeaSurfaceFilter>(
        config.sea_level_z,
        config.sea_margin);
    sea_filter->setEnabled(config.sea_enabled);
    processor.addFilter(sea_filter, true);

    auto stat_filter = std::make_shared<StatisticalOutlierFilter>(
        config.outlier_mean_k,
        config.outlier_stddev_mul);
    stat_filter->setEnabled(config.outlier_enabled);
    processor.addFilter(stat_filter, true);

    auto voxel_filter = std::make_shared<VoxelFilter>(config.voxel_leaf_size);
    voxel_filter->setEnabled(config.voxel_enabled);
    processor.addFilter(voxel_filter, true);

    auto density_filter = std::make_shared<DensityFilter>(
        config.density_voxel_size,
        config.density_min_points);
    density_filter->setEnabled(config.density_enabled);
    processor.addFilter(density_filter, true);

    auto motion_filter = std::make_shared<MotionFilter>(
        config.motion_cell_size,
        config.motion_threshold);
    motion_filter->setOutputStatic(config.motion_output_static);
    motion_filter->setEnabled(config.motion_enabled);
    processor.addFilter(motion_filter, true);

    LOG_INFO("Filters initialized: Distance={}, ROI={}, Sea={}, Outlier={}, Voxel={}, Density={}, Motion={}",
             config.distance_enabled ? "ON" : "OFF",
             config.roi_enabled ? "ON" : "OFF",
             config.sea_enabled ? "ON" : "OFF",
             config.outlier_enabled ? "ON" : "OFF",
             config.voxel_enabled ? "ON" : "OFF",
             config.density_enabled ? "ON" : "OFF",
             config.motion_enabled ? "ON" : "OFF");
}

void SetupDockingFromConfig(DockingAlgorithm& docking, const AppConfig& config)
{
    DockingConfig dockingConfig;
    
    // 最近区域检测配置
    dockingConfig.nearest.sector_x_min = config.nr_sector_x_min;
    dockingConfig.nearest.sector_x_max = config.nr_sector_x_max;
    dockingConfig.nearest.sector_y_min = config.nr_sector_y_min;
    dockingConfig.nearest.sector_y_max = config.nr_sector_y_max;
    dockingConfig.nearest.sector_z_min = config.nr_sector_z_min;
    dockingConfig.nearest.sector_z_max = config.nr_sector_z_max;
    dockingConfig.nearest.percentile = config.nr_percentile;
    dockingConfig.nearest.enabled = config.nr_enabled;
    
    // 码头边缘检测配置
    dockingConfig.edge.sector_x_min = config.edge_x_min;
    dockingConfig.edge.sector_x_max = config.edge_x_max;
    dockingConfig.edge.sector_y_min = config.edge_y_min;
    dockingConfig.edge.sector_y_max = config.edge_y_max;
    dockingConfig.edge.edge_z_min = config.edge_z_min;
    dockingConfig.edge.edge_z_max = config.edge_z_max;
    dockingConfig.edge.ransac_distance_threshold = config.edge_ransac_dist;
    dockingConfig.edge.enabled = config.edge_enabled;
    
    // 时序滤波配置
    dockingConfig.temporal.max_jump_m = config.temporal_max_jump;
    dockingConfig.temporal.enabled = config.temporal_enabled;
    
    docking.setConfig(dockingConfig);
    
    LOG_INFO("Docking algorithm initialized: Nearest={}, Edge={}, Temporal={}",
             dockingConfig.nearest.enabled ? "ON" : "OFF",
             dockingConfig.edge.enabled ? "ON" : "OFF",
             dockingConfig.temporal.enabled ? "ON" : "OFF");
}

bool SetupPublisherFromConfig(DockingPublisher& publisher, const AppConfig& config)
{
    UdpConfig udpConfig;
    udpConfig.target_ip = config.udp_target_ip;
    udpConfig.target_port = config.udp_target_port;
    udpConfig.sensor_id = config.udp_sensor_id;
    udpConfig.enabled = config.udp_enabled;
    
    publisher.setConfig(udpConfig);
    
    if (udpConfig.enabled) {
        publisher.start();
        LOG_INFO("UDP Publisher started: target={}:{}, sensor_id={}",
                 udpConfig.target_ip, udpConfig.target_port, udpConfig.sensor_id);
        return true;
    } else {
        LOG_INFO("UDP Publisher disabled");
        return false;
    }
}

void ConnectDockingCallbacks(
    DockingAlgorithm& docking,
    DockingPublisher& publisher,
    std::function<void(const DockingState&)> renderer_callback)
{
    docking.onStateUpdated = [&publisher, renderer_callback](const DockingState& state) {
        // 发送 UDP（如果启用）
        publisher.publish(state);
        // 更新渲染器（如果提供了回调）
        if (renderer_callback) {
            renderer_callback(state);
        }
    };
}

void SetupMapUpdateCallback(
    PointCloudProcessor& processor,
    DockingAlgorithm& docking,
    std::function<void(const std::vector<PointCloudPtr>&, size_t)> renderer_callback)
{
    processor.setMapUpdateCallback([&docking, renderer_callback](
        const std::vector<PointCloudPtr>& clouds, size_t filtered_count) {
        
        // 提交给渲染器（如果提供了回调）
        if (renderer_callback) {
            renderer_callback(clouds, filtered_count);
        }
        
        // 执行靠泊检测
        if (!clouds.empty()) {
            auto now_ns = static_cast<uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count());
            
            // 执行靠泊检测（内部会触发 onStateUpdated 回调）
            docking.processMultiple(clouds, now_ns);
        }
    });
}

std::string FindLivoxConfig(const std::string& cmd_config)
{
    // 如果命令行指定了配置，直接使用
    if (!cmd_config.empty()) {
        return cmd_config;
    }
    
    // 按优先级搜索默认路径（优先从 config 目录查找）
    const std::vector<std::string> search_paths = {
        "./config/mid360_config.json",    // config 子目录（推荐位置）
        "./mid360_config.json",           // 当前目录（兼容旧版）
        "../mid360_config.json",          // 上级目录（开发时常用）
    };
    
    for (const auto& path : search_paths) {
        if (std::filesystem::exists(std::filesystem::u8path(path))) {
            LOG_INFO("Found Livox config at: {}", path);
            return path;
        }
    }
    
    // 如果都找不到，使用默认值
    LOG_WARN("Livox config not found in search paths, using default: ./mid360_config.json");
    return "./mid360_config.json";
}

std::function<void()> MakeConfigSaver(const std::string& config_path, AppConfig& config)
{
    return [&config, config_path]() {
        SaveAppConfig(config_path, config);
    };
}

} // namespace Linger
