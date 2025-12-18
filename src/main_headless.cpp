#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
#include <string>
#include <iostream>
#include <memory>
#include <filesystem>

#include "core/logger.h"
#include "acquisition/device_manager.h"
#include "core/processor.h"
#include "preprocessing/distance_filter.h"
#include "preprocessing/crop_box_filter.h"
#include "segmentation/sea_surface_filter.h"
#include "preprocessing/statistical_outlier_filter.h"
#include "preprocessing/voxel_filter.h"
#include "core/config.h"
#include "core/app_config.h"

// 简单的退出标志，响应 Ctrl+C/CTRL_BREAK
static std::atomic<bool> g_running{true};

static void HandleSignal(int /*signum*/)
{
    g_running.store(false);
}

int main(int argc, char** argv)
{
    std::string config_path = "mid360_config.json";
    std::string app_config_path = "./config/app_config.json";
    std::string log_path = "log_headless.txt";

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--app-config" && i + 1 < argc) {
            app_config_path = argv[++i];
        } else if (arg == "--log" && i + 1 < argc) {
            log_path = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  -c, --config <path>    Livox SDK config file (default: mid360_config.json)\n"
                      << "  --app-config <path>    App config file for filters (default: ./config/app_config.json)\n"
                      << "  --log <path>           Log file path (default: log_headless.txt)\n"
                      << "  -h, --help             Show this help\n";
            return 0;
        }
    }

    // 使用 log_path 作为日志基础名称（日志将存放到 log/<日期>/<log_path>.log）
    // 移除 .txt 扩展名作为基础名称
    std::string log_base_name = "linger_headless";
    if (log_path != "log_headless.txt") {
        // 如果用户指定了自定义路径，提取文件名（不含扩展名）
        std::filesystem::path p(log_path);
        log_base_name = p.stem().string();
    }
    Logger::instance().init(log_base_name);
    LOG_INFO("[Headless] starting. Config: {}, AppConfig: {}", config_path, app_config_path);

    // 加载应用配置
    Linger::AppConfig appCfg;
    if (!Linger::LoadAppConfig(app_config_path, appCfg)) {
        LOG_WARN("[Headless] Failed to load app config from {}, using defaults", app_config_path);
    } else {
        LOG_INFO("[Headless] Loaded app config from {}", app_config_path);
    }

    // 注册信号，方便 Ctrl+C 优雅退出
    std::signal(SIGINT, HandleSignal);
#if defined(_WIN32)
    std::signal(SIGBREAK, HandleSignal);
#endif

    PointCloudProcessor processor;

    // 按配置文件参数挂接第三阶段预处理管线（Sensor 前置 + World 后置）
    auto dist_filter = std::make_shared<Linger::DistanceFilter>(appCfg.distance_min, appCfg.distance_max);
    dist_filter->setEnabled(appCfg.distance_enabled);
    processor.addFilter(dist_filter, false);

    auto crop_filter = std::make_shared<Linger::CropBoxFilter>(
        appCfg.roi_x_min, appCfg.roi_x_max,
        appCfg.roi_y_min, appCfg.roi_y_max,
        appCfg.roi_z_min, appCfg.roi_z_max);
    crop_filter->setEnabled(appCfg.roi_enabled);
    processor.addFilter(crop_filter, true);

    auto sea_filter = std::make_shared<Linger::SeaSurfaceFilter>(
        appCfg.sea_level_z,
        appCfg.sea_margin);
    sea_filter->setEnabled(appCfg.sea_enabled);
    processor.addFilter(sea_filter, true);

    auto stat_filter = std::make_shared<Linger::StatisticalOutlierFilter>(
        appCfg.outlier_mean_k,
        appCfg.outlier_stddev_mul);
    stat_filter->setEnabled(appCfg.outlier_enabled);
    processor.addFilter(stat_filter, true);

    auto voxel_filter = std::make_shared<Linger::VoxelFilter>(appCfg.voxel_leaf_size);
    voxel_filter->setEnabled(appCfg.voxel_enabled);
    processor.addFilter(voxel_filter, true);

    processor.start();

    DeviceManager dev;
    dev.onFrameWithPose = [&processor](PointCloudPtr pc, const Pose& pose) {
        processor.enqueue(pc, pose);
    };
    dev.onError = [](const std::string& msg) {
        LOG_ERROR("[Device] {}", msg);
    };
    dev.onInfo = [](const std::string& msg) {
        LOG_INFO("[Device] {}", msg);
    };

    if (!dev.start(config_path)) {
        LOG_ERROR("[Headless] failed to start DeviceManager with config: {}", config_path);
        processor.stop();
        return 1;
    }

    dev.startAcquisition();
    LOG_INFO("[Headless] running. Press Ctrl+C to exit.");

    while (g_running.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    LOG_INFO("[Headless] shutting down...");
    dev.stopAcquisition();
    dev.stopReplay();
    dev.stopRecording();
    dev.stop();
    processor.stop();
    LOG_INFO("[Headless] exit complete.");
    return 0;
}
