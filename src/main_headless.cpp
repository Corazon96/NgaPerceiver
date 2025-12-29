#include <atomic>
#include <csignal>
#include <chrono>
#include <thread>
#include <string>
#include <iostream>

#include "core/logger.h"
#include "core/app_config.h"
#include "core/app_init.h"
#include "acquisition/device_manager.h"
#include "core/processor.h"
#include "algorithms/docking_algorithm.h"
#include "communication/docking_publisher.h"

// 简单的退出标志，响应 Ctrl+C/CTRL_BREAK
static std::atomic<bool> g_running{true};

static void HandleSignal(int /*signum*/)
{
    g_running.store(false);
}

int main(int argc, char** argv)
{
    std::string config_path;  // Livox 配置（空表示自动搜索）
    std::string app_config_path = "./config/app_config.json";
    std::string replay_file;  // 回放文件路径（空表示实时采集）

    // 解析命令行参数
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
            config_path = argv[++i];
        } else if (arg == "--app-config" && i + 1 < argc) {
            app_config_path = argv[++i];
        } else if ((arg == "-r" || arg == "--replay") && i + 1 < argc) {
            replay_file = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            std::cout << "Usage: " << argv[0] << " [options]\n"
                      << "Options:\n"
                      << "  -c, --config <path>    Livox SDK config file (default: auto-search)\n"
                      << "  --app-config <path>    App config file for filters (default: ./config/app_config.json)\n"
                      << "  -r, --replay <path>    Replay .lgv file instead of live acquisition\n"
                      << "  -h, --help             Show this help\n";
            return 0;
        }
    }

    // 初始化日志系统（日志将存放到 log/<日期>/linger_headless.log）
    Logger::instance().init("linger_headless");

    // 搜索 Livox 配置文件
    config_path = Linger::FindLivoxConfig(config_path);
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

    // 创建核心组件
    PointCloudProcessor processor;
    Linger::DockingAlgorithm docking;
    Linger::DockingPublisher publisher;

    // 使用共享初始化模块设置滤波器、靠泊算法和 UDP 发布器
    Linger::SetupFiltersFromConfig(processor, appCfg);
    Linger::SetupDockingFromConfig(docking, appCfg);
    Linger::SetupPublisherFromConfig(publisher, appCfg);

    // 连接回调（Headless 模式无渲染器）
    Linger::ConnectDockingCallbacks(docking, publisher, nullptr);
    Linger::SetupMapUpdateCallback(processor, docking, nullptr);

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

    // 回放模式：读取 .lgv 文件
    if (!replay_file.empty()) {
        LOG_INFO("[Headless] Replay mode: {}", replay_file);
        
        // 回放模式不需要启动 Livox SDK
        if (!dev.startReplay(replay_file)) {
            LOG_ERROR("[Headless] Failed to start replay: {}", replay_file);
            processor.stop();
            return 1;
        }
        
        LOG_INFO("[Headless] Replaying... Press Ctrl+C to exit.");
        
        // 等待回放结束或用户中断
        while (g_running.load() && dev.isReplaying()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        
        LOG_INFO("[Headless] Replay finished or interrupted.");
    } else {
        // 实时采集模式
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
    }

    LOG_INFO("[Headless] shutting down...");
    dev.stopAcquisition();
    dev.stopReplay();
    dev.stopRecording();
    dev.stop();
    publisher.stop();
    processor.stop();
    LOG_INFO("[Headless] exit complete.");
    return 0;
}
