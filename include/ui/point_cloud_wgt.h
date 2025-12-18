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

/** @brief 生成�?UI 类的前向声明 */
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

	/** @brief 返回 QWidget* 以保持类型安全，调用者可以使�?qobject_cast 转换�?QVTKOpenGLNativeWidget�?*/
	QWidget *getVtkWidget() const { return vtkWidget_; }

	/** @brief 获取 Renderer 实例 */
	std::shared_ptr<Renderer> getRenderer() const { return renderer_; }

	/** @brief 设置滤波�?UI 初始值（从配置加载后调用�?*/
	void setFilterValues(double dist_min, double dist_max, bool dist_enabled,
						 double roi_xmin, double roi_xmax, double roi_ymin, double roi_ymax, double roi_zmin, double roi_zmax, bool roi_enabled,
						 double sea_z, double sea_margin, bool sea_enabled,
						 int outlier_k, double outlier_stddev, bool outlier_enabled,
						 double voxel_size, bool voxel_enabled);

signals:
	/** @brief 当用户在 UI 中改变帧积分时长（单位毫秒）时发�?*/
	void retentionChanged(int ms);
	/** @brief 当用户在 UI 中改变点大小（单位像素）时发�?*/
	void pointSizeChanged(int size);
	/** @brief 当用户在 UI 中改变最小距离（单位米）时发�?*/
	void minDistChanged(double val);
	/** @brief 当用户在 UI 中改变最大距离（单位米）时发�?*/
	void maxDistChanged(double val);
	/** @brief 当用户在 UI 中改变体素大小（单位米）时发�?*/
	void voxelSizeChanged(double val);
	/** @brief 当用户在 UI 中改变体素滤波器是否启用时发�?*/
	void voxelEnabledChanged(bool enabled);
	void distEnabledChanged(bool enabled);
	void roiBoundsChanged(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
	void roiEnabledChanged(bool enabled);
	void seaLevelChanged(double z);
	void seaMarginChanged(double margin);
	void seaEnabledChanged(bool enabled);
	void outlierParamsChanged(int mean_k, double stddev_mul);
	void outlierEnabledChanged(bool enabled);
	
	// 靠泊检测信�?- 最近区域检�?
	void nearestSectorChanged(double x_min, double x_max, double y_min, double y_max, double z_min, double z_max);
	void nearestPercentileChanged(double pct);
	void nearestEnabledChanged(bool enabled);
	
	// 靠泊检测信�?- 码头边缘检�?
	void edgeZRangeChanged(double z_min, double z_max);
	void edgeXMaxChanged(double x_max);
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
};
