#pragma once

/**
 * @file app_init.h
 * @brief 应用程序公共初始化模块
 * 
 * 提供 main.cpp 和 main_headless.cpp 共享的初始化逻辑，
 * 包括滤波器管线设置、靠泊算法配置、UDP 发布器初始化等。
 */

#include <string>
#include <memory>
#include <functional>

#include "core/app_config.h"
#include "core/processor.h"
#include "algorithms/docking_algorithm.h"
#include "communication/docking_publisher.h"

namespace Linger {

/**
 * @brief 初始化上下文，包含所有共享组件的引用
 */
struct AppContext {
    PointCloudProcessor& processor;
    DockingAlgorithm& docking;
    DockingPublisher& publisher;
    AppConfig& config;
    std::string config_path;  // 用于保存配置的路径
};

/**
 * @brief 根据配置初始化滤波器管线
 * @param processor 点云处理器引用
 * @param config 应用配置
 */
void SetupFiltersFromConfig(PointCloudProcessor& processor, const AppConfig& config);

/**
 * @brief 根据配置初始化靠泊检测算法
 * @param docking 靠泊算法引用
 * @param config 应用配置
 */
void SetupDockingFromConfig(DockingAlgorithm& docking, const AppConfig& config);

/**
 * @brief 根据配置初始化 UDP 发布器
 * @param publisher UDP 发布器引用
 * @param config 应用配置
 * @return 是否启用并启动成功
 */
bool SetupPublisherFromConfig(DockingPublisher& publisher, const AppConfig& config);

/**
 * @brief 连接靠泊算法状态回调到发布器
 * @param docking 靠泊算法引用
 * @param publisher UDP 发布器引用
 * @param renderer_callback 可选的渲染器更新回调（用于 GUI 模式）
 */
void ConnectDockingCallbacks(
    DockingAlgorithm& docking,
    DockingPublisher& publisher,
    std::function<void(const DockingState&)> renderer_callback = nullptr);

/**
 * @brief 设置处理器的地图更新回调，执行靠泊检测
 * @param processor 点云处理器引用
 * @param docking 靠泊算法引用
 * @param renderer_callback 可选的渲染器提交回调（用于 GUI 模式）
 */
void SetupMapUpdateCallback(
    PointCloudProcessor& processor,
    DockingAlgorithm& docking,
    std::function<void(const std::vector<PointCloudPtr>&, size_t)> renderer_callback = nullptr);

/**
 * @brief 搜索 Livox 配置文件
 * @param cmd_config 命令行指定的配置路径（可为空）
 * @return 找到的配置文件路径
 */
std::string FindLivoxConfig(const std::string& cmd_config);

/**
 * @brief 创建用于持久化配置的保存函数
 * @param config_path 配置文件路径
 * @param config 配置引用
 * @return 保存函数
 */
std::function<void()> MakeConfigSaver(const std::string& config_path, AppConfig& config);

} // namespace Linger
