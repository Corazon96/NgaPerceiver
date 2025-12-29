#include <iostream>
#include <memory>
#include <chrono>
#include "core/logger.h"
#include "core/app_config.h"
#include "core/app_init.h"

#ifdef _WIN32
#include <windows.h>
#endif

#include <QApplication>
#include <QMessageBox>

#include "acquisition/device_manager.h"
#include "core/processor.h"
#include "visualization/renderer.h"
#include "ui/point_cloud_wgt.h"
#include "core/pipeline_setup.h"
#include "core/config.h"
#include "algorithms/docking_algorithm.h"
#include "communication/docking_publisher.h"

#ifdef ENABLE_TRANSLATIONS
#include "core/translation_manager.h"
#endif

// 全局配置实例及路径（用于持久化）
static Linger::AppConfig g_appConfig;
static std::string g_appConfigPath = "./config/app_config.json";

int main(int argc, char **argv)
{
#ifdef _WIN32
	// 确保控制台使用 UTF-8，以便程序输出的中文能正确显示
	SetConsoleOutputCP(CP_UTF8);
	SetConsoleCP(CP_UTF8);
#endif

	QApplication app(argc, argv);
	
#ifdef ENABLE_TRANSLATIONS
	// 初始化翻译管理器
	TranslationManager::instance().initialize(&app);
#endif

	PointCloudWgt w;
	w.show();

	// 初始化日志系统（日志将存放到 log/<日期>/linger_perceiver.log）
    Logger::instance().init("linger_perceiver");

	// 析构顺序：dev -> proc -> w（保证数据流安全停止）
	auto rend = w.getRenderer();
	PointCloudProcessor proc;
	DeviceManager dev;
	Linger::DockingAlgorithm docking;
	Linger::DockingPublisher publisher;

	// 配置 processor
	proc.start();
	
	// 设置滤波管线（将具体的滤波器创建与 UI 连接逻辑分离到 pipeline_setup.cpp 中）
	SetupFilterPipeline(proc, w);

	// 加载应用配置（支持 --app-config 命令行参数）
	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if ((arg == "--app-config") && i + 1 < argc) {
			g_appConfigPath = argv[++i];
		}
	}
	if (!Linger::LoadAppConfig(g_appConfigPath, g_appConfig)) {
		LOG_WARN("Failed to load app config from {}, using defaults", g_appConfigPath);
		// 使用默认值（AppConfig 构造函数已设定）
	} else {
		LOG_INFO("Loaded app config from {}", g_appConfigPath);
	}
	
	// 将配置值应用到 UI
	w.setFilterValues(
		g_appConfig.distance_min, g_appConfig.distance_max, g_appConfig.distance_enabled,
		g_appConfig.roi_x_min, g_appConfig.roi_x_max, g_appConfig.roi_y_min, g_appConfig.roi_y_max, g_appConfig.roi_z_min, g_appConfig.roi_z_max, g_appConfig.roi_enabled,
		g_appConfig.sea_level_z, g_appConfig.sea_margin, g_appConfig.sea_enabled,
		g_appConfig.outlier_mean_k, g_appConfig.outlier_stddev_mul, g_appConfig.outlier_enabled,
		g_appConfig.voxel_leaf_size, g_appConfig.voxel_enabled
	);
	
	// 将 Docking 配置应用到 UI
	w.setDockingValues(
		g_appConfig.nr_sector_x_min, g_appConfig.nr_sector_x_max,
		g_appConfig.nr_sector_y_min, g_appConfig.nr_sector_y_max,
		g_appConfig.nr_sector_z_min, g_appConfig.nr_sector_z_max,
		g_appConfig.nr_percentile, g_appConfig.nr_enabled,
		g_appConfig.edge_x_min, g_appConfig.edge_x_max,
		g_appConfig.edge_y_min, g_appConfig.edge_y_max,
		g_appConfig.edge_z_min, g_appConfig.edge_z_max,
		g_appConfig.edge_ransac_dist, g_appConfig.edge_enabled
	);

	// 触发滤波器参数初始化（发送信号以更新滤波器）
	emit w.minDistChanged(g_appConfig.distance_min);
	emit w.maxDistChanged(g_appConfig.distance_max);
	emit w.distEnabledChanged(g_appConfig.distance_enabled);
	emit w.seaLevelChanged(g_appConfig.sea_level_z);
	emit w.seaMarginChanged(g_appConfig.sea_margin);
	emit w.seaEnabledChanged(g_appConfig.sea_enabled);
	emit w.outlierParamsChanged(g_appConfig.outlier_mean_k, g_appConfig.outlier_stddev_mul);
	emit w.outlierEnabledChanged(g_appConfig.outlier_enabled);
	emit w.voxelSizeChanged(g_appConfig.voxel_leaf_size);
	emit w.voxelEnabledChanged(g_appConfig.voxel_enabled);
	// 密度滤波器初始化
	emit w.densityParamsChanged(g_appConfig.density_voxel_size, g_appConfig.density_min_points);
	emit w.densityEnabledChanged(g_appConfig.density_enabled);
	// 运动滤波器初始化
	emit w.motionParamsChanged(g_appConfig.motion_cell_size, g_appConfig.motion_threshold);
	emit w.motionOutputChanged(g_appConfig.motion_output_static);
	emit w.motionEnabledChanged(g_appConfig.motion_enabled);

	// 使用共享模块初始化靠泊算法和 UDP 发布器
	Linger::SetupDockingFromConfig(docking, g_appConfig);
	Linger::SetupPublisherFromConfig(publisher, g_appConfig);

	// 设置 Docking 状态更新回调（发送 UDP + 更新可视化）
	Linger::ConnectDockingCallbacks(docking, publisher, [rend](const Linger::DockingState& state) {
		if (rend) {
			rend->updateDocking(state);
		}
	});

	// 连接 UI 信号以持久化配置变更
	auto saveConfig = [](){ Linger::SaveAppConfig(g_appConfigPath, g_appConfig); };
	
	QObject::connect(&w, &PointCloudWgt::minDistChanged, [saveConfig](double val){
		g_appConfig.distance_min = static_cast<float>(val);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::maxDistChanged, [saveConfig](double val){
		g_appConfig.distance_max = static_cast<float>(val);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::distEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.distance_enabled = enabled;
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::roiBoundsChanged, [saveConfig](double xmin, double xmax, double ymin, double ymax, double zmin, double zmax){
		g_appConfig.roi_x_min = static_cast<float>(xmin);
		g_appConfig.roi_x_max = static_cast<float>(xmax);
		g_appConfig.roi_y_min = static_cast<float>(ymin);
		g_appConfig.roi_y_max = static_cast<float>(ymax);
		g_appConfig.roi_z_min = static_cast<float>(zmin);
		g_appConfig.roi_z_max = static_cast<float>(zmax);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::roiEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.roi_enabled = enabled;
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::seaLevelChanged, [saveConfig](double z){
		g_appConfig.sea_level_z = static_cast<float>(z);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::seaMarginChanged, [saveConfig](double margin){
		g_appConfig.sea_margin = static_cast<float>(margin);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::seaEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.sea_enabled = enabled;
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::outlierParamsChanged, [saveConfig](int k, double stddev){
		g_appConfig.outlier_mean_k = k;
		g_appConfig.outlier_stddev_mul = static_cast<float>(stddev);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::outlierEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.outlier_enabled = enabled;
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::voxelSizeChanged, [saveConfig](double size){
		g_appConfig.voxel_leaf_size = static_cast<float>(size);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::voxelEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.voxel_enabled = enabled;
		saveConfig();
	});

	// 密度滤波器参数持久化
	QObject::connect(&w, &PointCloudWgt::densityParamsChanged, [saveConfig](double voxel, int minpts){
		g_appConfig.density_voxel_size = static_cast<float>(voxel);
		g_appConfig.density_min_points = minpts;
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::densityEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.density_enabled = enabled;
		saveConfig();
	});

	// 运动滤波器参数持久化
	QObject::connect(&w, &PointCloudWgt::motionParamsChanged, [saveConfig](double cell, double thresh){
		g_appConfig.motion_cell_size = static_cast<float>(cell);
		g_appConfig.motion_threshold = static_cast<float>(thresh);
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::motionOutputChanged, [saveConfig](bool output_static){
		g_appConfig.motion_output_static = output_static;
		saveConfig();
	});
	QObject::connect(&w, &PointCloudWgt::motionEnabledChanged, [saveConfig](bool enabled){
		g_appConfig.motion_enabled = enabled;
		saveConfig();
	});

	// 靠泊检测参数连接 - 最近区域检测
	QObject::connect(&w, &PointCloudWgt::nearestSectorChanged, [&docking, rend, saveConfig](double xmin, double xmax, double ymin, double ymax, double zmin, double zmax){
		auto cfg = docking.getNearestConfig();
		cfg.sector_x_min = static_cast<float>(xmin);
		cfg.sector_x_max = static_cast<float>(xmax);
		cfg.sector_y_min = static_cast<float>(ymin);
		cfg.sector_y_max = static_cast<float>(ymax);
		cfg.sector_z_min = static_cast<float>(zmin);
		cfg.sector_z_max = static_cast<float>(zmax);
		docking.setNearestConfig(cfg);
		// 持久化
		g_appConfig.nr_sector_x_min = static_cast<float>(xmin);
		g_appConfig.nr_sector_x_max = static_cast<float>(xmax);
		g_appConfig.nr_sector_y_min = static_cast<float>(ymin);
		g_appConfig.nr_sector_y_max = static_cast<float>(ymax);
		g_appConfig.nr_sector_z_min = static_cast<float>(zmin);
		g_appConfig.nr_sector_z_max = static_cast<float>(zmax);
		saveConfig();
		// 更新扇区可视化
		if (rend) {
			rend->setNearestSector(
				static_cast<float>(xmin), static_cast<float>(xmax),
				static_cast<float>(ymin), static_cast<float>(ymax),
				static_cast<float>(zmin), static_cast<float>(zmax),
				cfg.enabled);
		}
		LOG_INFO("[Docking] Nearest sector: X[{:.1f},{:.1f}] Y[{:.1f},{:.1f}] Z[{:.1f},{:.1f}]", xmin, xmax, ymin, ymax, zmin, zmax);
	});
	QObject::connect(&w, &PointCloudWgt::nearestPercentileChanged, [&docking, saveConfig](double pct){
		auto cfg = docking.getNearestConfig();
		cfg.percentile = static_cast<float>(pct);
		docking.setNearestConfig(cfg);
		g_appConfig.nr_percentile = static_cast<float>(pct);
		saveConfig();
		LOG_INFO("[Docking] Nearest percentile: {:.1f}%", pct);
	});
	QObject::connect(&w, &PointCloudWgt::nearestEnabledChanged, [&docking, rend, saveConfig](bool enabled){
		auto cfg = docking.getNearestConfig();
		cfg.enabled = enabled;
		docking.setNearestConfig(cfg);
		g_appConfig.nr_enabled = enabled;
		saveConfig();
		// 更新可视化
		if (rend) {
			rend->setNearestSector(
				cfg.sector_x_min, cfg.sector_x_max,
				cfg.sector_y_min, cfg.sector_y_max,
				cfg.sector_z_min, cfg.sector_z_max,
				enabled);
		}
		LOG_INFO("[Docking] Nearest region detection: {}", enabled ? "enabled" : "disabled");
	});
	
	// 靠泊检测参数连接 - 码头边缘检测
	QObject::connect(&w, &PointCloudWgt::edgeSectorChanged, [&docking, rend, saveConfig](double xmin, double xmax, double ymin, double ymax, double zmin, double zmax){
		auto cfg = docking.getEdgeConfig();
		cfg.sector_x_min = static_cast<float>(xmin);
		cfg.sector_x_max = static_cast<float>(xmax);
		cfg.sector_y_min = static_cast<float>(ymin);
		cfg.sector_y_max = static_cast<float>(ymax);
		cfg.edge_z_min = static_cast<float>(zmin);
		cfg.edge_z_max = static_cast<float>(zmax);
		docking.setEdgeConfig(cfg);
		g_appConfig.edge_x_min = static_cast<float>(xmin);
		g_appConfig.edge_x_max = static_cast<float>(xmax);
		g_appConfig.edge_y_min = static_cast<float>(ymin);
		g_appConfig.edge_y_max = static_cast<float>(ymax);
		g_appConfig.edge_z_min = static_cast<float>(zmin);
		g_appConfig.edge_z_max = static_cast<float>(zmax);
		saveConfig();
		// 更新边缘检测区域可视化
		if (rend) {
			rend->setEdgeRegion(
				static_cast<float>(xmin), static_cast<float>(xmax),
				static_cast<float>(ymin), static_cast<float>(ymax),
				static_cast<float>(zmin), static_cast<float>(zmax),
				cfg.enabled);
		}
		LOG_INFO("[Docking] Edge sector: X[{:.1f}, {:.1f}], Y[{:.1f}, {:.1f}], Z[{:.2f}, {:.2f}]",
		         xmin, xmax, ymin, ymax, zmin, zmax);
	});
	QObject::connect(&w, &PointCloudWgt::edgeRansacDistChanged, [&docking, saveConfig](double dist){
		auto cfg = docking.getEdgeConfig();
		cfg.ransac_distance_threshold = static_cast<float>(dist);
		docking.setEdgeConfig(cfg);
		g_appConfig.edge_ransac_dist = static_cast<float>(dist);
		saveConfig();
		LOG_INFO("[Docking] Edge RANSAC threshold: {:.3f}m", dist);
	});
	QObject::connect(&w, &PointCloudWgt::edgeEnabledChanged, [&docking, rend, saveConfig](bool enabled){
		auto cfg = docking.getEdgeConfig();
		cfg.enabled = enabled;
		docking.setEdgeConfig(cfg);
		g_appConfig.edge_enabled = enabled;
		saveConfig();
		// 更新可视化
		if (rend) {
			auto edgeCfg = docking.getEdgeConfig();
			rend->setEdgeRegion(
				edgeCfg.sector_x_min, edgeCfg.sector_x_max,
				edgeCfg.sector_y_min, edgeCfg.sector_y_max,
				edgeCfg.edge_z_min, edgeCfg.edge_z_max,
				enabled);
		}
		LOG_INFO("[Docking] Dock edge detection: {}", enabled ? "enabled" : "disabled");
	});

	// 触发初始化信号，确保渲染器接收到配置参数（在信号连接之后）
	emit w.nearestSectorChanged(g_appConfig.nr_sector_x_min, g_appConfig.nr_sector_x_max,
	                            g_appConfig.nr_sector_y_min, g_appConfig.nr_sector_y_max,
	                            g_appConfig.nr_sector_z_min, g_appConfig.nr_sector_z_max);
	emit w.nearestPercentileChanged(g_appConfig.nr_percentile);
	emit w.nearestEnabledChanged(g_appConfig.nr_enabled);
	emit w.edgeSectorChanged(g_appConfig.edge_x_min, g_appConfig.edge_x_max,
	                         g_appConfig.edge_y_min, g_appConfig.edge_y_max,
	                         g_appConfig.edge_z_min, g_appConfig.edge_z_max);
	emit w.edgeRansacDistChanged(g_appConfig.edge_ransac_dist);
	emit w.edgeEnabledChanged(g_appConfig.edge_enabled);

	// 链式回调：设备 -> 处理器 -> 渲染器
	dev.onFrameWithPose = [&](PointCloudPtr pc, const Pose& pose)
	{ proc.enqueue(pc, pose); };
	
	proc.setMapUpdateCallback([rend, &docking](const std::vector<PointCloudPtr>& pc, size_t filtered_count)
							  {
		// 检查渲染器是否存在
		if (rend) {
			// 使用 submitMap 原子发布累积快照到 Renderer
			rend->submitMap(pc, filtered_count);
		}
		
		// 对累积的点云执行靠泊检测（直接使用多点云块接口，避免合并拷贝）
		if (!pc.empty()) {
			// 获取当前时间戳
			auto now_ns = static_cast<uint64_t>(
				std::chrono::duration_cast<std::chrono::nanoseconds>(
					std::chrono::system_clock::now().time_since_epoch()).count());
			
			// 执行靠泊检测（最近区域 + 边缘检测）
			auto state = docking.processMultiple(pc, now_ns);
			
			// 更新可视化
			if (rend) {
				rend->updateDocking(state);
			}
		}
	});

	// ROI 线框可视化（调试用，默认跟随 ROI 页面参数/开关）
	double roi_xmin = LingerConfig::ROI_X_MIN_DEFAULT;
	double roi_xmax = LingerConfig::ROI_X_MAX_DEFAULT;
	double roi_ymin = LingerConfig::ROI_Y_MIN_DEFAULT;
	double roi_ymax = LingerConfig::ROI_Y_MAX_DEFAULT;
	double roi_zmin = LingerConfig::ROI_Z_MIN_DEFAULT;
	double roi_zmax = LingerConfig::ROI_Z_MAX_DEFAULT;
	bool roi_enabled = false;
	if (rend) {
		rend->setRoiBox(static_cast<float>(roi_xmin), static_cast<float>(roi_xmax),
					 static_cast<float>(roi_ymin), static_cast<float>(roi_ymax),
					 static_cast<float>(roi_zmin), static_cast<float>(roi_zmax), roi_enabled);
	}

	QObject::connect(&w, &PointCloudWgt::roiBoundsChanged, [rend, &roi_xmin, &roi_xmax, &roi_ymin, &roi_ymax, &roi_zmin, &roi_zmax, &roi_enabled](double xmin, double xmax, double ymin, double ymax, double zmin, double zmax){
		roi_xmin = xmin; roi_xmax = xmax;
		roi_ymin = ymin; roi_ymax = ymax;
		roi_zmin = zmin; roi_zmax = zmax;
		if (rend) {
			rend->setRoiBox(static_cast<float>(roi_xmin), static_cast<float>(roi_xmax),
					 static_cast<float>(roi_ymin), static_cast<float>(roi_ymax),
					 static_cast<float>(roi_zmin), static_cast<float>(roi_zmax), roi_enabled);
		}
	});
	QObject::connect(&w, &PointCloudWgt::roiEnabledChanged, [rend, &roi_xmin, &roi_xmax, &roi_ymin, &roi_ymax, &roi_zmin, &roi_zmax, &roi_enabled](bool enabled){
		roi_enabled = enabled;
		if (rend) {
			rend->setRoiBox(static_cast<float>(roi_xmin), static_cast<float>(roi_xmax),
					 static_cast<float>(roi_ymin), static_cast<float>(roi_ymax),
					 static_cast<float>(roi_zmin), static_cast<float>(roi_zmax), roi_enabled);
		}
	});

	// 监听 UI retention 改变并设置到 processor（单位 ms -> s）
	QObject::connect(&w, &PointCloudWgt::retentionChanged, [&proc, &dev, rend](int ms)
					 {
		double s = static_cast<double>(ms) /1000.0;
		proc.setRetentionSeconds(s);
		dev.setRetentionTime(ms); // 同步设置回放预加载时间
		//立刻触发一次渲染更新（读取最新快照并提交给 renderer）
		std::vector<PointCloudPtr> snap = proc.takeLatestSnapshot();
		if (!snap.empty() && rend) rend->submitMap(snap); });

	// 触发 ROI Filter 初始化信号（在信号连接之后）
	emit w.roiBoundsChanged(g_appConfig.roi_x_min, g_appConfig.roi_x_max, g_appConfig.roi_y_min, g_appConfig.roi_y_max, g_appConfig.roi_z_min, g_appConfig.roi_z_max);
	emit w.roiEnabledChanged(g_appConfig.roi_enabled);

	// 连接设备管理器信号
	QObject::connect(&w, &PointCloudWgt::connectDeviceRequested, [&dev](uint32_t handle){
		dev.connectDevice(handle);
	});
	QObject::connect(&w, &PointCloudWgt::disconnectDeviceRequested, [&dev](){
		dev.disconnectDevice();
	});
	QObject::connect(&w, &PointCloudWgt::startAcquisitionRequested, [&dev](){
		dev.startAcquisition();
	});
	QObject::connect(&w, &PointCloudWgt::stopAcquisitionRequested, [&dev](){
		dev.stopAcquisition();
	});

	// 连接录制/回放信号
	QObject::connect(&w, &PointCloudWgt::startRecordingRequested, [&dev](const std::string& path){
		dev.startRecording(path);
	});
	QObject::connect(&w, &PointCloudWgt::stopRecordingRequested, [&dev](){
		dev.stopRecording();
	});
	
	// 连接录制结束回调（异步）
	dev.onRecordingFinished = [&w]() {
		emit w.recordingFinishedSignal();
	};

	// 连接错误回调
	dev.onError = [&w](const std::string& msg) {
		// 确保在 UI 线程显示错误
		QMetaObject::invokeMethod(&w, "onDeviceError", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(msg)));
	};

	// 连接信息回调
	dev.onInfo = [&w](const std::string& msg) {
		// 确保在 UI 线程显示信息
		QMetaObject::invokeMethod(&w, "onDeviceInfo", Qt::QueuedConnection, Q_ARG(QString, QString::fromStdString(msg)));
	};

	QObject::connect(&w, &PointCloudWgt::startReplayRequested, [&dev](const std::string& path){
		dev.startReplay(path);
	});
	QObject::connect(&w, &PointCloudWgt::stopReplayRequested, [&dev](){
		dev.stopReplay();
	});
	QObject::connect(&w, &PointCloudWgt::replaySpeedChanged, [&dev](float speed){
		dev.setReplaySpeed(speed);
	});
	QObject::connect(&w, &PointCloudWgt::replayPauseRequested, [&dev](bool paused){
		if (paused) dev.pauseReplay();
		else dev.resumeReplay();
	});
	QObject::connect(&w, &PointCloudWgt::replaySeekRequested, [&dev](double progress){
		dev.seekReplay(progress);
	});
	
	// 连接回放进度回调 (从 Worker 线程 -> UI 线程)
	dev.onReplayProgress = [&w](uint64_t cur, uint64_t total) {
		emit w.replayProgressUpdated(cur, total);
	};

	// 连接回放结束回调
	dev.onReplayFinished = [&w]() {
		// 使用 invokeMethod 调用槽函数，确保在主线程执行
		QMetaObject::invokeMethod(&w, "onReplayFinished", Qt::QueuedConnection);
	};

	// 连接录制丢帧回调
	dev.onRecordingDrop = [&w]() {
		emit w.recordingDroppedSignal();
	};

	// 连接设备管理器回调到 UI
	dev.onDeviceListUpdated = [&w](const std::vector<DeviceInfo>& devices){
		// 使用 QMetaObject::invokeMethod 安全地在主线程更新 UI
		QMetaObject::invokeMethod(&w, [&w, devices](){
			w.updateDeviceList(devices);
		});
	};

	// 解析命令行参数并搜索 Livox 配置文件
	// 解析命令行参数获取 Livox 配置路径
	std::string config_path;
	for (int i = 1; i < argc; ++i) {
		std::string arg = argv[i];
		if ((arg == "-c" || arg == "--config") && i + 1 < argc) {
			config_path = argv[++i];
			break;
		}
	}
	// 使用共享模块搜索 Livox 配置文件
	config_path = Linger::FindLivoxConfig(config_path);

	// 启动设备采集（处理器已启动）
	if (!dev.start(config_path))
	{
		QMessageBox::critical(&w, "Startup Error", 
			QString("Failed to start DeviceManager.\nConfig file not found or invalid:\n%1").arg(QString::fromStdString(config_path)));
	}

	int ret = app.exec();
	
	// 停止 UDP 发布器
	publisher.stop();
	
	// 由于调整了对象声明顺序，现在可以依赖 RAII 自动析构，无需手动清理
	return ret;
}
