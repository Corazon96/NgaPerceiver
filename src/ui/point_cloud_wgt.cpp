#include "ui/point_cloud_wgt.h"
#include "ui_point_cloud_wgt.h"

#include <QKeyEvent>
#include <QVTKOpenGLNativeWidget.h>
#include <QStackedWidget>
#include <vector>
#include <QFileDialog>
#include <QMessageBox>
#include <QDateTime>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QLabel>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>

#include "visualization/camera_interactor.h"
#include "visualization/renderer.h"

PointCloudWgt::PointCloudWgt(QWidget *parent)
	: QWidget(parent), ui(std::make_unique<Ui::PointCloudWgt>())
{
	ui->setupUi(this);

	vtkWidget_ = ui->vtk_wgt_;

	// 初始化渲染器
	renderer_ = std::make_shared<Renderer>();
	if (vtkWidget_)
	{
		renderer_->init(vtkWidget_);
		// 确保 widget 能接收焦点并处理按键（在此处设置一次）
		vtkWidget_->setFocusPolicy(Qt::StrongFocus);
		vtkWidget_->setFocus();
		// 附加相机交互器，使用 unique_ptr 管理
		cameraInteractor_ = std::make_unique<CameraInteractor>(vtkWidget_, nullptr);
	}

	// --- 动态构建回放控制条 ---
	// 1. 创建容器和布局
	QWidget* leftContainer = new QWidget(this);
	QVBoxLayout* leftLayout = new QVBoxLayout(leftContainer);
	leftLayout->setContentsMargins(0, 0, 0, 0);
	
	// 2. 从原布局中移�?vtk_wgt_ 并添加到新布局
	ui->horizontalLayout->removeWidget(vtkWidget_);
	leftLayout->addWidget(vtkWidget_, 1); // stretch = 1

	// 3. 创建回放控制�?
	replayControlWidget_ = new QWidget(this);
	QHBoxLayout* replayLayout = new QHBoxLayout(replayControlWidget_);
	replayLayout->setContentsMargins(4, 4, 4, 4);
	
	btnPlayPause_ = new QPushButton("Pause", replayControlWidget_);
	btnPlayPause_->setCheckable(true); // Checked = Paused
	
	QLabel* lblSpeed = new QLabel("Speed:", replayControlWidget_);
	comboSpeed_ = new QComboBox(replayControlWidget_);
	comboSpeed_->addItem("0.5x", 0.5f);
	comboSpeed_->addItem("1.0x", 1.0f);
	comboSpeed_->addItem("2.0x", 2.0f);
	comboSpeed_->addItem("4.0x", 4.0f);
	comboSpeed_->addItem("8.0x", 8.0f);
	comboSpeed_->setCurrentIndex(1); // Default 1.0x

	lblTime_ = new QLabel("00:00 / 00:00", replayControlWidget_);
	
	sliderProgress_ = new QSlider(Qt::Horizontal, replayControlWidget_);
	sliderProgress_->setRange(0, 1000);

	replayLayout->addWidget(btnPlayPause_);
	replayLayout->addWidget(lblSpeed);
	replayLayout->addWidget(comboSpeed_);
	replayLayout->addWidget(lblTime_);
	replayLayout->addWidget(sliderProgress_, 1); // Expand slider

	// 4. 添加到左侧布局并默认隐�?
	leftLayout->addWidget(replayControlWidget_);
	replayControlWidget_->setVisible(false);

	// 5. 将容器放回主布局
	ui->horizontalLayout->insertWidget(0, leftContainer, 3); // stretch = 3 (保持原比�?

	// --- 连接回放控制信号 ---
	connect(btnPlayPause_, &QPushButton::toggled, this, [this](bool checked) {
		if (isReplayFinished_) {
			// 如果回放已结束，点击按钮（此�?checked 变为 false，因为之前被设为 true/Paused�?
			// 我们希望重新开始播放：跳转到开头并继续
			isReplayFinished_ = false;
			emit replaySeekRequested(0.0);
			emit replayPauseRequested(false);
			btnPlayPause_->setText("Pause");
			// 注意：此�?checked 已经�?false 了，所以不需要再 setChecked(false)
			return;
		}
		btnPlayPause_->setText(checked ? "Resume" : "Pause");
		emit replayPauseRequested(checked);
	});

	connect(comboSpeed_, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
		float speed = comboSpeed_->itemData(index).toFloat();
		emit replaySpeedChanged(speed);
	});
	
	connect(sliderProgress_, &QSlider::sliderPressed, this, [this]() {
		isSliderPressed_ = true;
	});
	
	// 拖动时实时更�?(Scrubbing)
	connect(sliderProgress_, &QSlider::valueChanged, this, [this](int value) {
		if (isSliderPressed_) {
			double progress = static_cast<double>(value) / 1000.0;
			emit replaySeekRequested(progress);
		}
	});

	connect(sliderProgress_, &QSlider::sliderReleased, this, [this]() {
		isSliderPressed_ = false;
		// 释放时再发送一次，确保最终位置准�?
		double progress = static_cast<double>(sliderProgress_->value()) / 1000.0;
		emit replaySeekRequested(progress);
	});
	
	// 点击跳转 (Slider 默认不支持点击跳转，这里简单处理拖动结�?
	// 如果需要点击跳转，需要重�?QSlider 或使�?eventFilter，这里暂只支持拖动释放跳�?

	// 内部信号连接，用于跨线程更新 UI
	qRegisterMetaType<uint64_t>("uint64_t");
	connect(this, &PointCloudWgt::replayProgressUpdated, this, &PointCloudWgt::onReplayProgress, Qt::QueuedConnection);
	connect(this, &PointCloudWgt::replayFinishedSignal, this, &PointCloudWgt::onReplayFinished, Qt::QueuedConnection);
	connect(this, &PointCloudWgt::recordingDroppedSignal, this, &PointCloudWgt::onRecordingDropped, Qt::QueuedConnection);
	connect(this, &PointCloudWgt::recordingFinishedSignal, this, &PointCloudWgt::onRecordingFinished, Qt::QueuedConnection);


	/** @brief 连接 retention 下拉框到信号 */
	if (ui->retentionCombo)
	{
		// 使用常量数组初始化选项，确�?UI 显示与逻辑一�?
		ui->retentionCombo->clear();
		for (int ms : kRetentionValues) {
			ui->retentionCombo->addItem(QString("%1").arg(ms));
		}
		// 默认选中第一�?
		if (ui->retentionCombo->count() > 0) {
			ui->retentionCombo->setCurrentIndex(0);
		}

		QObject::connect(ui->retentionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx)
						 {
			if (idx >= 0 && idx < static_cast<int>(kRetentionValues.size())) {
				emit retentionChanged(kRetentionValues[idx]);
			}
		});
	}

	/** @brief 连接 sizeCombo 下拉框到信号 */
	if (ui->sizeCombo)
	{
		QObject::connect(ui->sizeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx)
						 {
			// 索引 0 -> 1px, 1 -> 2px, ...
			int size = idx + 1;
			if (renderer_) renderer_->setPointSize(size);
			emit pointSizeChanged(size); });
	}

	// 连接 Min Dist SpinBox
	if (ui->minDistSpinBox)
	{
		QObject::connect(ui->minDistSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double val)
						 { emit minDistChanged(val); });
	}

	// 连接 Max Dist SpinBox
	if (ui->maxDistSpinBox)
	{
		QObject::connect(ui->maxDistSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double val)
						 { emit maxDistChanged(val); });
	}

	// 连接 Distance Enable CheckBox
	if (ui->distEnableCheck)
	{
		QObject::connect(ui->distEnableCheck, &QCheckBox::toggled, [this](bool checked)
						 { emit distEnabledChanged(checked); });
	}

	// 连接 Filter Type ComboBox 切换 StackedWidget
	if (ui->filterTypeCombo && ui->filterStack)
	{
		QObject::connect(ui->filterTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx)
						 {
			ui->filterStack->setCurrentIndex(idx);
		});
	}

	// 连接 Voxel Size SpinBox
	if (ui->voxelSizeSpinBox)
	{
		QObject::connect(ui->voxelSizeSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double val)
						 { emit voxelSizeChanged(val); });
	}

	// 连接 Voxel Enable CheckBox
	if (ui->voxelEnableCheck)
	{
		QObject::connect(ui->voxelEnableCheck, &QCheckBox::toggled, [this](bool checked)
						 { emit voxelEnabledChanged(checked); });
	}

	// ROI 参数与开�?
	if (ui->roiXMinSpin && ui->roiXMaxSpin && ui->roiYMinSpin && ui->roiYMaxSpin && ui->roiZMinSpin && ui->roiZMaxSpin)
	{
		auto emitRoi = [this]() {
			emit roiBoundsChanged(
				ui->roiXMinSpin->value(), ui->roiXMaxSpin->value(),
				ui->roiYMinSpin->value(), ui->roiYMaxSpin->value(),
				ui->roiZMinSpin->value(), ui->roiZMaxSpin->value());
		};
		QObject::connect(ui->roiXMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitRoi](double){ emitRoi(); });
		QObject::connect(ui->roiXMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitRoi](double){ emitRoi(); });
		QObject::connect(ui->roiYMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitRoi](double){ emitRoi(); });
		QObject::connect(ui->roiYMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitRoi](double){ emitRoi(); });
		QObject::connect(ui->roiZMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitRoi](double){ emitRoi(); });
		QObject::connect(ui->roiZMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitRoi](double){ emitRoi(); });
	}
	if (ui->roiEnableCheck)
	{
		QObject::connect(ui->roiEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit roiEnabledChanged(checked); });
	}

	// 海面滤除参数与开�?
	if (ui->seaLevelSpin)
	{
		QObject::connect(ui->seaLevelSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double v){ emit seaLevelChanged(v); });
	}
	if (ui->seaMarginSpin)
	{
		QObject::connect(ui->seaMarginSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double v){ emit seaMarginChanged(v); });
	}
	if (ui->seaEnableCheck)
	{
		QObject::connect(ui->seaEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit seaEnabledChanged(checked); });
	}

	// 统计离群点参数与开关
	if (ui->outlierMeanKSpin && ui->outlierStddevSpin)
	{
		auto emitOutlier = [this]() {
			emit outlierParamsChanged(ui->outlierMeanKSpin->value(), ui->outlierStddevSpin->value());
		};
		QObject::connect(ui->outlierMeanKSpin, QOverload<int>::of(&QSpinBox::valueChanged), [emitOutlier](int){ emitOutlier(); });
		QObject::connect(ui->outlierStddevSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitOutlier](double){ emitOutlier(); });
	}
	if (ui->outlierEnableCheck)
	{
		QObject::connect(ui->outlierEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit outlierEnabledChanged(checked); });
	}

	// 密度滤波器参数与开关
	if (ui->densityVoxelSpin && ui->densityMinPtsSpin)
	{
		auto emitDensity = [this]() {
			emit densityParamsChanged(ui->densityVoxelSpin->value(), ui->densityMinPtsSpin->value());
		};
		QObject::connect(ui->densityVoxelSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitDensity](double){ emitDensity(); });
		QObject::connect(ui->densityMinPtsSpin, QOverload<int>::of(&QSpinBox::valueChanged), [emitDensity](int){ emitDensity(); });
	}
	if (ui->densityEnableCheck)
	{
		QObject::connect(ui->densityEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit densityEnabledChanged(checked); });
	}

	// 运动滤波器参数与开关
	if (ui->motionCellSpin && ui->motionThreshSpin)
	{
		auto emitMotion = [this]() {
			emit motionParamsChanged(ui->motionCellSpin->value(), ui->motionThreshSpin->value());
		};
		QObject::connect(ui->motionCellSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitMotion](double){ emitMotion(); });
		QObject::connect(ui->motionThreshSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitMotion](double){ emitMotion(); });
	}
	if (ui->motionOutputCombo)
	{
		QObject::connect(ui->motionOutputCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx){
			emit motionOutputChanged(idx == 0);  // 0=Static, 1=Dynamic
		});
	}
	if (ui->motionEnableCheck)
	{
		QObject::connect(ui->motionEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit motionEnabledChanged(checked); });
	}

	// 最近区域检测参数与开关
	if (ui->nrXMinSpin && ui->nrXMaxSpin && ui->nrYMinSpin && ui->nrYMaxSpin && ui->nrZMinSpin && ui->nrZMaxSpin)
	{
		auto emitNearestSector = [this]() {
			emit nearestSectorChanged(
				ui->nrXMinSpin->value(), ui->nrXMaxSpin->value(),
				ui->nrYMinSpin->value(), ui->nrYMaxSpin->value(),
				ui->nrZMinSpin->value(), ui->nrZMaxSpin->value());
		};
		QObject::connect(ui->nrXMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitNearestSector](double){ emitNearestSector(); });
		QObject::connect(ui->nrXMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitNearestSector](double){ emitNearestSector(); });
		QObject::connect(ui->nrYMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitNearestSector](double){ emitNearestSector(); });
		QObject::connect(ui->nrYMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitNearestSector](double){ emitNearestSector(); });
		QObject::connect(ui->nrZMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitNearestSector](double){ emitNearestSector(); });
		QObject::connect(ui->nrZMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitNearestSector](double){ emitNearestSector(); });
	}
	if (ui->nrPercentileSpin)
	{
		QObject::connect(ui->nrPercentileSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double v){ emit nearestPercentileChanged(v); });
	}
	if (ui->nearestEnableCheck)
	{
		QObject::connect(ui->nearestEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit nearestEnabledChanged(checked); });
	}

	// 码头边缘检测参数与开�?
	if (ui->edgeXMinSpin && ui->edgeXMaxSpin && ui->edgeYMinSpin && ui->edgeYMaxSpin && ui->edgeZMinSpin && ui->edgeZMaxSpin)
	{
		auto emitEdgeSector = [this]() {
			emit edgeSectorChanged(ui->edgeXMinSpin->value(), ui->edgeXMaxSpin->value(),
				ui->edgeYMinSpin->value(), ui->edgeYMaxSpin->value(),
				ui->edgeZMinSpin->value(), ui->edgeZMaxSpin->value());
		};
		QObject::connect(ui->edgeXMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitEdgeSector](double){ emitEdgeSector(); });
		QObject::connect(ui->edgeXMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitEdgeSector](double){ emitEdgeSector(); });
		QObject::connect(ui->edgeYMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitEdgeSector](double){ emitEdgeSector(); });
		QObject::connect(ui->edgeYMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitEdgeSector](double){ emitEdgeSector(); });
		QObject::connect(ui->edgeZMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitEdgeSector](double){ emitEdgeSector(); });
		QObject::connect(ui->edgeZMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [emitEdgeSector](double){ emitEdgeSector(); });
	}
	if (ui->edgeRansacDistSpin)
	{
		QObject::connect(ui->edgeRansacDistSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), [this](double v){ emit edgeRansacDistChanged(v); });
	}
	if (ui->edgeEnableCheck)
	{
		QObject::connect(ui->edgeEnableCheck, &QCheckBox::toggled, [this](bool checked){ emit edgeEnabledChanged(checked); });
	}

	// 连接 Docking Type ComboBox 切换 StackedWidget
	if (ui->dockingTypeCombo && ui->dockingStack)
	{
		QObject::connect(ui->dockingTypeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), [this](int idx)
					 {
			ui->dockingStack->setCurrentIndex(idx);
		});
	}

	// 连接设备管理按钮
	if (ui->btnConnect) {
		connect(ui->btnConnect, &QPushButton::clicked, this, [this](){
			auto items = ui->deviceListWidget->selectedItems();
			if (!items.empty()) {
				uint32_t handle = items[0]->data(Qt::UserRole).toUInt();
				emit connectDeviceRequested(handle);
			}
		});
	}
	if (ui->btnDisconnect) connect(ui->btnDisconnect, &QPushButton::clicked, this, &PointCloudWgt::disconnectDeviceRequested);
	if (ui->btnStart) connect(ui->btnStart, &QPushButton::clicked, this, &PointCloudWgt::startAcquisitionRequested);
	if (ui->btnStop) connect(ui->btnStop, &QPushButton::clicked, this, &PointCloudWgt::stopAcquisitionRequested);

	// Connect Record/Replay Buttons
	recordTimer_ = new QTimer(this);
	connect(recordTimer_, &QTimer::timeout, this, &PointCloudWgt::updateRecordTime);

	if (ui->btnRecord) {
		connect(ui->btnRecord, &QPushButton::toggled, this, [this](bool checked){
			if (checked) {
				// 检查是否正在采�?
				if (!isAcquiring_) {
					QMessageBox::warning(this, "Recording Error", "Cannot start recording: No device is acquiring data.\nPlease start acquisition first.");
					// 阻止信号以避免递归调用
					{
						const QSignalBlocker blocker(ui->btnRecord);
						ui->btnRecord->setChecked(false);
					}
					return;
				}

				QString filename = QString("LingerData_%1.lgv").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
				QString path = QFileDialog::getSaveFileName(this, "Save Recording", filename, "LingerPerceiver Data (*.lgv)");
				if (!path.isEmpty()) {
					emit startRecordingRequested(path.toStdString());
					
					// 更新 UI 状�?
					ui->btnRecord->setText("Stop Recording");
					// 设置红色背景警示正在录制
					ui->btnRecord->setStyleSheet("QPushButton { background-color: #ff4444; color: white; border: 1px solid #cc0000; border-radius: 4px; padding: 4px; } QPushButton:hover { background-color: #ff6666; }");
					
					recordStartTime_ = QTime::currentTime();
					recordTimer_->start(1000); // 每秒更新一�?
					updateRecordTime(); // 立即更新一�?
				} else {
					ui->btnRecord->setChecked(false); // Cancelled
				}
			} else {
				emit stopRecordingRequested();
				
				// 更新 UI 状态为正在保存
				ui->btnRecord->setText("Saving...");
				ui->btnRecord->setEnabled(false); // 禁止重复点击
				recordTimer_->stop();
			}
		});
	}

	if (ui->btnReplay) {
		connect(ui->btnReplay, &QPushButton::toggled, this, [this](bool checked){
			if (checked) {
				// 检查是否有设备连接
				if (isDeviceConnected_) {
					QMessageBox::warning(this, "Replay Error", "Cannot start replay: A device is currently connected.\nPlease disconnect the device first.");
					// 阻止信号以避免递归调用
					{
						const QSignalBlocker blocker(ui->btnReplay);
						ui->btnReplay->setChecked(false);
					}
					return;
				}

				QString path = QFileDialog::getOpenFileName(this, "Open Recording", "", "LingerPerceiver Data (*.lgv)");
				if (!path.isEmpty()) {
					currentReplayPath_ = path.toStdString();
					emit startReplayRequested(currentReplayPath_);
					// 显示控制条，重置状�?
					replayControlWidget_->setVisible(true);
					btnPlayPause_->setChecked(false);
					btnPlayPause_->setText("Pause");
					comboSpeed_->setCurrentIndex(1); // 1.0x
					isReplayFinished_ = false;
				} else {
					ui->btnReplay->setChecked(false); // Cancelled
				}
			} else {
				emit stopReplayRequested();
				// 隐藏控制�?
				replayControlWidget_->setVisible(false);
			}
		});
	}
}

PointCloudWgt::~PointCloudWgt() = default;

void PointCloudWgt::updateDeviceList(const std::vector<DeviceInfo>& devices)
{
	ui->deviceListWidget->clear();
	isDeviceConnected_ = false;
	isAcquiring_ = false;

	for (const auto& dev : devices) {
		if (dev.is_connected) {
			isDeviceConnected_ = true;
			if (dev.is_acquiring) {
				isAcquiring_ = true;
			}
		}

		QString status;
		if (dev.is_connected) {
			status = dev.is_acquiring ? "Acquiring" : "Connected";
		} else {
			status = "Disconnected";
		}
		QString label = QString("%1 (%2) - %3").arg(QString::fromStdString(dev.ip))
											   .arg(QString::fromStdString(dev.sn))
											   .arg(status);
		QListWidgetItem* item = new QListWidgetItem(label);
		item->setData(Qt::UserRole, dev.handle);
		ui->deviceListWidget->addItem(item);
		
		if (dev.is_connected) {
			item->setSelected(true);
		}
	}
}

void PointCloudWgt::onReplayProgress(uint64_t current_ns, uint64_t total_ns)
{
	if (total_ns == 0) return;
	
	// 更新时间标签
	auto formatTime = [](uint64_t ns) {
		int secs = static_cast<int>(ns / 1000000000ULL);
		int mins = secs / 60;
		secs = secs % 60;
		return QString("%1:%2").arg(mins, 2, 10, QChar('0')).arg(secs, 2, 10, QChar('0'));
	};
	
	lblTime_->setText(QString("%1 / %2").arg(formatTime(current_ns)).arg(formatTime(total_ns)));
	
	// 更新滑块 (如果用户没有正在拖动)
	if (!isSliderPressed_) {
		int val = static_cast<int>((static_cast<double>(current_ns) / total_ns) * 1000.0);
		sliderProgress_->setValue(val);
	}
}

void PointCloudWgt::updateRecordTime()
{
	int secs = recordStartTime_.secsTo(QTime::currentTime());
	// 格式化为 HH:mm:ss
	int hours = secs / 3600;
	int minutes = (secs % 3600) / 60;
	int seconds = secs % 60;
	
	QString timeStr = QString("%1:%2:%3")
		.arg(hours, 2, 10, QChar('0'))
		.arg(minutes, 2, 10, QChar('0'))
		.arg(seconds, 2, 10, QChar('0'));
		
	ui->label_record->setText(QString("Recording: %1").arg(timeStr));
}

void PointCloudWgt::onReplayFinished()
{
	isReplayFinished_ = true;
	// 设置为暂停状态，并更改文本为 Replay
	// 阻止信号，防止触�?toggled 导致自动重播
	{
		const QSignalBlocker blocker(btnPlayPause_);
		btnPlayPause_->setChecked(true); // Checked = Paused
	}
	btnPlayPause_->setText("Replay");
}

void PointCloudWgt::onRecordingDropped()
{
	// 临时更改窗口标题以警告用�?
	// 也可以考虑使用 QToolTip::showText 或更新状态栏
	static int dropCount = 0;
	dropCount++;
	this->setWindowTitle(QString("LingerPerceiver - WARNING: RECORDING DROPPED FRAMES (%1)").arg(dropCount));
	
	// 3秒后恢复标题（如果不再丢帧）
	QTimer::singleShot(3000, this, [this](){
		this->setWindowTitle("LingerPerceiver");
	});
}

void PointCloudWgt::onRecordingFinished()
{
	ui->btnRecord->setText("Record");
	ui->btnRecord->setStyleSheet("");
	ui->btnRecord->setEnabled(true);
	ui->label_record->setText("Data Recording");
	
	// 确保按钮状态为未选中
	const QSignalBlocker blocker(ui->btnRecord);
	ui->btnRecord->setChecked(false);
}

void PointCloudWgt::onDeviceError(const QString& msg)
{
	QMessageBox::critical(this, "Device Error", msg);

	// 发生错误时，尝试重置相关 UI 状�?
	
	// 如果录制按钮被按下（例如启动录制失败），重置�?
	if (ui->btnRecord->isChecked() || !ui->btnRecord->isEnabled()) {
		const QSignalBlocker blocker(ui->btnRecord);
		ui->btnRecord->setChecked(false);
		ui->btnRecord->setText("Record");
		ui->btnRecord->setStyleSheet("");
		ui->btnRecord->setEnabled(true);
		if (recordTimer_->isActive()) recordTimer_->stop();
		ui->label_record->setText("Data Recording");
	}

	// 如果回放按钮被按下（例如启动回放失败），重置�?
	if (ui->btnReplay->isChecked()) {
		const QSignalBlocker blocker(ui->btnReplay);
		ui->btnReplay->setChecked(false);
		replayControlWidget_->setVisible(false);
	}
}

void PointCloudWgt::onDeviceInfo(const QString& msg)
{
	QMessageBox::information(this, "Device Info", msg);
}

void PointCloudWgt::setFilterValues(double dist_min, double dist_max, bool dist_enabled,
									double roi_xmin, double roi_xmax, double roi_ymin, double roi_ymax, double roi_zmin, double roi_zmax, bool roi_enabled,
									double sea_z, double sea_margin, bool sea_enabled,
									int outlier_k, double outlier_stddev, bool outlier_enabled,
									double voxel_size, bool voxel_enabled)
{
	// 使用 QSignalBlocker 防止设置值时触发信号
	if (ui->minDistSpinBox) {
		const QSignalBlocker blocker(ui->minDistSpinBox);
		ui->minDistSpinBox->setValue(dist_min);
	}
	if (ui->maxDistSpinBox) {
		const QSignalBlocker blocker(ui->maxDistSpinBox);
		ui->maxDistSpinBox->setValue(dist_max);
	}
	if (ui->distEnableCheck) {
		const QSignalBlocker blocker(ui->distEnableCheck);
		ui->distEnableCheck->setChecked(dist_enabled);
	}

	if (ui->roiXMinSpin) { const QSignalBlocker b(ui->roiXMinSpin); ui->roiXMinSpin->setValue(roi_xmin); }
	if (ui->roiXMaxSpin) { const QSignalBlocker b(ui->roiXMaxSpin); ui->roiXMaxSpin->setValue(roi_xmax); }
	if (ui->roiYMinSpin) { const QSignalBlocker b(ui->roiYMinSpin); ui->roiYMinSpin->setValue(roi_ymin); }
	if (ui->roiYMaxSpin) { const QSignalBlocker b(ui->roiYMaxSpin); ui->roiYMaxSpin->setValue(roi_ymax); }
	if (ui->roiZMinSpin) { const QSignalBlocker b(ui->roiZMinSpin); ui->roiZMinSpin->setValue(roi_zmin); }
	if (ui->roiZMaxSpin) { const QSignalBlocker b(ui->roiZMaxSpin); ui->roiZMaxSpin->setValue(roi_zmax); }
	if (ui->roiEnableCheck) { const QSignalBlocker b(ui->roiEnableCheck); ui->roiEnableCheck->setChecked(roi_enabled); }

	if (ui->seaLevelSpin) { const QSignalBlocker b(ui->seaLevelSpin); ui->seaLevelSpin->setValue(sea_z); }
	if (ui->seaMarginSpin) { const QSignalBlocker b(ui->seaMarginSpin); ui->seaMarginSpin->setValue(sea_margin); }
	if (ui->seaEnableCheck) { const QSignalBlocker b(ui->seaEnableCheck); ui->seaEnableCheck->setChecked(sea_enabled); }

	if (ui->outlierMeanKSpin) { const QSignalBlocker b(ui->outlierMeanKSpin); ui->outlierMeanKSpin->setValue(outlier_k); }
	if (ui->outlierStddevSpin) { const QSignalBlocker b(ui->outlierStddevSpin); ui->outlierStddevSpin->setValue(outlier_stddev); }
	if (ui->outlierEnableCheck) { const QSignalBlocker b(ui->outlierEnableCheck); ui->outlierEnableCheck->setChecked(outlier_enabled); }

	if (ui->voxelSizeSpinBox) { const QSignalBlocker b(ui->voxelSizeSpinBox); ui->voxelSizeSpinBox->setValue(voxel_size); }
	if (ui->voxelEnableCheck) { const QSignalBlocker b(ui->voxelEnableCheck); ui->voxelEnableCheck->setChecked(voxel_enabled); }
}

void PointCloudWgt::setDockingValues(double nr_xmin, double nr_xmax, double nr_ymin, double nr_ymax, double nr_zmin, double nr_zmax,
                                     double nr_percentile, bool nr_enabled,
                                     double edge_xmin, double edge_xmax, double edge_ymin, double edge_ymax,
                                     double edge_zmin, double edge_zmax, double edge_ransac, bool edge_enabled)
{
	// Nearest Region
	if (ui->nrXMinSpin) { const QSignalBlocker b(ui->nrXMinSpin); ui->nrXMinSpin->setValue(nr_xmin); }
	if (ui->nrXMaxSpin) { const QSignalBlocker b(ui->nrXMaxSpin); ui->nrXMaxSpin->setValue(nr_xmax); }
	if (ui->nrYMinSpin) { const QSignalBlocker b(ui->nrYMinSpin); ui->nrYMinSpin->setValue(nr_ymin); }
	if (ui->nrYMaxSpin) { const QSignalBlocker b(ui->nrYMaxSpin); ui->nrYMaxSpin->setValue(nr_ymax); }
	if (ui->nrZMinSpin) { const QSignalBlocker b(ui->nrZMinSpin); ui->nrZMinSpin->setValue(nr_zmin); }
	if (ui->nrZMaxSpin) { const QSignalBlocker b(ui->nrZMaxSpin); ui->nrZMaxSpin->setValue(nr_zmax); }
	if (ui->nrPercentileSpin) { const QSignalBlocker b(ui->nrPercentileSpin); ui->nrPercentileSpin->setValue(nr_percentile); }
	// nearestEnableCheck = nr_enabled
	if (ui->nearestEnableCheck) { const QSignalBlocker b(ui->nearestEnableCheck); ui->nearestEnableCheck->setChecked(nr_enabled); }
	
	// Dock Edge
	if (ui->edgeXMinSpin) { const QSignalBlocker b(ui->edgeXMinSpin); ui->edgeXMinSpin->setValue(edge_xmin); }
	if (ui->edgeXMaxSpin) { const QSignalBlocker b(ui->edgeXMaxSpin); ui->edgeXMaxSpin->setValue(edge_xmax); }
	if (ui->edgeYMinSpin) { const QSignalBlocker b(ui->edgeYMinSpin); ui->edgeYMinSpin->setValue(edge_ymin); }
	if (ui->edgeYMaxSpin) { const QSignalBlocker b(ui->edgeYMaxSpin); ui->edgeYMaxSpin->setValue(edge_ymax); }
	if (ui->edgeZMinSpin) { const QSignalBlocker b(ui->edgeZMinSpin); ui->edgeZMinSpin->setValue(edge_zmin); }
	if (ui->edgeZMaxSpin) { const QSignalBlocker b(ui->edgeZMaxSpin); ui->edgeZMaxSpin->setValue(edge_zmax); }
	if (ui->edgeRansacDistSpin) { const QSignalBlocker b(ui->edgeRansacDistSpin); ui->edgeRansacDistSpin->setValue(edge_ransac); }
	// edgeEnableCheck = edge_enabled
	if (ui->edgeEnableCheck) { const QSignalBlocker b(ui->edgeEnableCheck); ui->edgeEnableCheck->setChecked(edge_enabled); }
}
