#pragma once

#include <memory>
#include <QWidget>
#include <QTimer>
#include <QTime>

#include <QVTKOpenGLNativeWidget.h>
#include "acquisition/device_manager.h"

class QVTKOpenGLNativeWidget;
class CameraInteractor;
class Renderer;
class QPushButton;
class QComboBox;
class QSlider;
class QLabel;
class QGroupBox;

/// Qt Designer 生成的 UI 类前向声明
namespace Ui
{
	class PointCloudWgt;
}

class PointCloudWgt : public QWidget
{
	Q_OBJECT

public:
	explicit PointCloudWgt(QWidget *parent = nullptr);
	~PointCloudWgt();

	/// 返回 VTK Widget（调用者可用 qobject_cast 转换为 QVTKOpenGLNativeWidget）
	QWidget *getVtkWidget() const { return vtkWidget_; }

	/// 获取 Renderer 实例
	std::shared_ptr<Renderer> getRenderer() const { return renderer_; }

	/// 设置滤波器 UI 初始值（从配置加载后调用）
	void setFilterValues(double dist_min, double dist_max, bool dist_enabled,
						 double roi_xmin, double roi_xmax, double roi_ymin, double roi_ymax, double roi_zmin, double roi_zmax, bool roi_enabled,
						 double sea_z, double sea_margin, bool sea_enabled,
						 int outlier_k, double outlier_stddev, bool outlier_enabled,
						 double voxel_size, bool voxel_enabled);
	/// 设置靠泊检测 UI 初始值（从配置加载后调用）
	void setDockingValues(double nr_xmin, double nr_xmax, double nr_ymin, double nr_ymax, double nr_zmin, double nr_zmax,
	                      double nr_percentile, bool nr_enabled,
	                      double edge_xmin, double edge_xmax, double edge_ymin, double edge_ymax,
	                      double edge_zmin, double edge_zmax, double edge_ransac, bool edge_enabled);
signals:
	// === 滤波器参数变化信号 ===
	void retentionChanged(int ms);        ///< 帧积分时长变化（毫秒）
	void pointSizeChanged(int size);      ///< 点大小变化（像素）
	void minDistChanged(double val);      ///< 最小距离变化（米）
	void maxDistChanged(double val);      ///< 最大距离变化（米）
	void voxelSizeChanged(double val);    ///< 体素大小变化（米）
	void voxelEnabledChanged(bool enabled);
	void distEnabledChanged(bool enabled);
	void roiBoundsChanged(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
	void roiEnabledChanged(bool enabled);
	void seaLevelChanged(double z);
	void seaMarginChanged(double margin);
	void seaEnabledChanged(bool enabled);
	void outlierParamsChanged(int mean_k, double stddev_mul);
	void outlierEnabledChanged(bool enabled);
	
	// 密度滤波器信号
	void densityParamsChanged(double voxel_size, int min_points);
	void densityEnabledChanged(bool enabled);
	
	// 运动滤波器信号
	void motionParamsChanged(double cell_size, double threshold);
	void motionOutputChanged(bool output_static);
	void motionEnabledChanged(bool enabled);
	
	// 靠泊检测信号 - 最近区域检测
	void nearestSectorChanged(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
	void nearestPercentileChanged(double pct);
	void nearestEnabledChanged(bool enabled);
	
	// 靠泊检测信号 - 码头边缘检测
	void edgeSectorChanged(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
	void edgeRansacDistChanged(double dist);
	void edgeEnabledChanged(bool enabled);

	void connectDeviceRequested(uint32_t handle);
	void disconnectDeviceRequested();
	void startAcquisitionRequested();
	void stopAcquisitionRequested();

	void startRecordingRequested(const std::string& path);
	void stopRecordingRequested();
	void startReplayRequested(const std::string& path);
	void stopReplayRequested();
	
	void replaySpeedChanged(float speed);
	void replayPauseRequested(bool paused);
	void replaySeekRequested(double progress);

signals:
	void replayProgressUpdated(uint64_t current_ns, uint64_t total_ns);
	void replayFinishedSignal();
	void recordingDroppedSignal();
	void recordingFinishedSignal();

public slots:
	void updateDeviceList(const std::vector<DeviceInfo>& devices);
	void onReplayProgress(uint64_t current_ns, uint64_t total_ns);
	void onReplayFinished();
	void onRecordingDropped();
	void onRecordingFinished();
	void onDeviceError(const QString& msg);
	void onDeviceInfo(const QString& msg);

private slots:
	void updateRecordTime();
#ifdef ENABLE_TRANSLATIONS
	void setupLanguageSelector();
	void retranslateCustomWidgets();
#endif

private:
	std::unique_ptr<Ui::PointCloudWgt> ui;

	QVTKOpenGLNativeWidget *vtkWidget_ = nullptr;
	std::unique_ptr<CameraInteractor> cameraInteractor_;
	std::shared_ptr<Renderer> renderer_;

	QTimer* recordTimer_ = nullptr;
	QTime recordStartTime_;

	// 回放控制 UI
	QWidget* replayControlWidget_ = nullptr;
	QPushButton* btnPlayPause_ = nullptr;
	QComboBox* comboSpeed_ = nullptr;
	QSlider* sliderProgress_ = nullptr;
	QLabel* lblTime_ = nullptr;
	bool isSliderPressed_ = false;
	
	std::string currentReplayPath_;
	bool isReplayFinished_ = false;
	
	bool isDeviceConnected_ = false;
	bool isAcquiring_ = false;
	std::vector<DeviceInfo> cachedDevices_; // 缓存设备列表，用于语言切换时刷新

#ifdef ENABLE_TRANSLATIONS
	QComboBox* comboLanguage_ = nullptr;
	QGroupBox* langGroup_ = nullptr;
	QLabel* lblLang_ = nullptr;
#endif
};
