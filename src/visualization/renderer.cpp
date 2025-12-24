#define _USE_MATH_DEFINES
#include <cmath>

#include "visualization/renderer.h"
#include "visualization/color_map.h"
#include "core/config.h"
#include "core/logger.h"

#include <vector>
#include <mutex>
#include <atomic>
#include <memory>
#include <algorithm>
#include <iostream>
#include <thread>

#include <QMetaObject>
#include <QCoreApplication>
#include <QVTKOpenGLNativeWidget.h>

// VTK 输出窗口控制（禁用警告弹窗）
#include <vtkOutputWindow.h>
#include <vtkFileOutputWindow.h>

// VTK 渲染头文件
#include <vtkNew.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderer.h>
#include <vtkProperty.h>
#include <vtkUnsignedCharArray.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkIdTypeArray.h>
#include <vtkCamera.h>

// 额外 VTK头，用于网格、同心圆与标�?
#include <vtkCellArray.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkVectorText.h>
#include <vtkFollower.h>
#include <vtkAxesActor.h>
#include <vtkOrientationMarkerWidget.h>

// 新增用于角注�?文本与色条的头文�?
#include <vtkTextActor.h>
#include <vtkTextProperty.h>
#include <vtkScalarBarActor.h>
#include <vtkLookupTable.h>
#include <vtkOutlineSource.h>

/** @brief 自适应并行策略 */
static const size_t PARALLEL_THRESHOLD = LingerConfig::RENDERER_PARALLEL_THRESHOLD;

struct Renderer::Impl
{
	// �?Processor 发布的最新地图快�?
	std::vector<PointCloudPtr> latestMapSnapshot;
	std::mutex snapshotMutex;

	// 指向 QVTKOpenGLNativeWidget
	QWidget *widget = nullptr;

	// VTK 对象
	vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow;
	vtkSmartPointer<vtkRenderer> renderer;
	vtkSmartPointer<vtkPolyData> polyData;
	vtkSmartPointer<vtkPoints> vtkPoints;
	vtkSmartPointer<vtkPolyDataMapper> mapper;
	vtkSmartPointer<vtkActor> actor;

	// ROI 可视�?
	vtkSmartPointer<vtkOutlineSource> roiOutline;
	vtkSmartPointer<vtkPolyDataMapper> roiMapper;
	vtkSmartPointer<vtkActor> roiActor;

	// 靠泊检测可视化
	// 最近区域扇区（蓝色线框�?
	vtkSmartPointer<vtkOutlineSource> nearestSectorOutline;
	vtkSmartPointer<vtkPolyDataMapper> nearestSectorMapper;
	vtkSmartPointer<vtkActor> nearestSectorActor;
	bool nearestSectorVisible = true;
	
	// 码头边缘检测区域（黄色线框�?
	vtkSmartPointer<vtkOutlineSource> edgeRegionOutline;
	vtkSmartPointer<vtkPolyDataMapper> edgeRegionMapper;
	vtkSmartPointer<vtkActor> edgeRegionActor;
	bool edgeRegionVisible = false;
	
	// 边缘直线可视化（绿色�?
	vtkSmartPointer<vtkPolyData> edgeLinePoly;
	vtkSmartPointer<vtkPolyDataMapper> edgeLineMapper;
	vtkSmartPointer<vtkActor> edgeLineActor;
	
	// 最近区域距离指示（蓝色圆弧�?
	vtkSmartPointer<vtkPolyData> nearestArcPoly;
	vtkSmartPointer<vtkPolyDataMapper> nearestArcMapper;
	vtkSmartPointer<vtkActor> nearestArcActor;
	
	// 靠泊信息文本
	vtkSmartPointer<vtkTextActor> dockingInfoActor;
	
	// 状态缓�?
	Linger::DockingState pendingDockingState;
	bool hasPendingDockingState = false;
	std::mutex dockingStateMutex;

	// 网格与同心环
	vtkSmartPointer<vtkPolyData> gridPoly;
	vtkSmartPointer<vtkActor> gridActor;
	vtkSmartPointer<vtkPolyData> ringsPoly;
	vtkSmartPointer<vtkActor> ringsActor;
	std::vector<vtkSmartPointer<vtkFollower>> ringLabels;
	// 角落坐标轴和文本、色�?
	vtkSmartPointer<vtkAxesActor> axesActor;
	vtkSmartPointer<vtkOrientationMarkerWidget> axesWidget;
	vtkSmartPointer<vtkTextActor> infoTextActor;
	vtkSmartPointer<vtkScalarBarActor> scalarBar;
	vtkSmartPointer<vtkLookupTable> scalarLUT;
	// 独立的标题文本，用于控制与色条之间的间距
	vtkSmartPointer<vtkTextActor> scalarTitleActor;
	// 独立�?FPS 文本
	vtkSmartPointer<vtkTextActor> fpsTextActor;

	// 渲染工作线程相关
	std::thread renderWorker;
	std::atomic<bool> workerRunning{false};
	std::condition_variable workerCv;
	std::mutex workerMutex;
	std::vector<PointCloudPtr> pendingMap; // 待处理的地图
	size_t pendingFilteredCount = 0;
	bool hasPendingMap = false;

	// 预处理好的渲染数据包
	struct RenderData {
		vtkSmartPointer<::vtkPoints> points;
		vtkSmartPointer<vtkCellArray> verts;
		vtkSmartPointer<vtkUnsignedCharArray> colors;
		vtkSmartPointer<vtkFloatArray> coordsArray;
		vtkSmartPointer<vtkIdTypeArray> vertsArray;
		size_t pointCount = 0;
		size_t filteredCount = 0;

		// 初始�?重置数据结构
		void initialize() {
			if (!points) points = vtkSmartPointer<::vtkPoints>::New();
			if (!verts) verts = vtkSmartPointer<vtkCellArray>::New();
			if (!colors) {
				colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
				colors->SetNumberOfComponents(3);
			}
			if (!coordsArray) {
				coordsArray = vtkSmartPointer<vtkFloatArray>::New();
				coordsArray->SetNumberOfComponents(3);
				points->SetData(coordsArray);
			}
			if (!vertsArray) {
				vertsArray = vtkSmartPointer<vtkIdTypeArray>::New();
			}
		}
	};
	
    // 对象池：保存不再使用�?RenderData 以便复用
	std::vector<RenderData> bufferPool;
	std::mutex poolMutex;

	// 双缓冲：后台线程准备数据
	RenderData nextFrameData;
	bool hasNextFrame = false;
	std::mutex nextFrameMutex;

	// 双缓冲：当前正在渲染的数据（持有引用以防被回收）
	RenderData currentRenderData;

	// FPS 计算相关
	std::chrono::steady_clock::time_point lastFrameTime;
	int frameCount = 0;
	double currentFPS = 0.0;

	// FPS 控制相关
	// 将最小间隔设置为 25ms (相当�?40FPS 的上�?�?
	// 上游 Processor �?30ms (33FPS) 生产�?
	// 下游 Renderer �?25ms 为阈值进行限流�?
	// 因为 33ms > 25ms，正常的 30FPS 帧都能顺利通过�?
	// 只有当出现异常的高频抖动 (<25ms) 时才会丢帧�?
	// 这样既避免了"拍频干扰"导致的掉帧，又防止了过高 FPS�?
	const double minFrameIntervalMs = LingerConfig::RENDERER_MIN_FRAME_INTERVAL_MS; 
	std::chrono::steady_clock::time_point lastSubmitTime;
	std::chrono::steady_clock::time_point renderScheduleTime; // 记录上次调度渲染的时间，用于检测卡�?

	// 获取一个可用的缓冲区（从池中取或新建）
	RenderData getFreeBuffer() {
		std::lock_guard<std::mutex> lock(poolMutex);
		if (!bufferPool.empty()) {
			RenderData data = bufferPool.back();
			bufferPool.pop_back();
			return data;
		}
		RenderData newData;
		newData.initialize();
		return newData;
	}

	// 回收缓冲�?
	void recycleBuffer(RenderData& data) {
		if (!data.points) return; // 空数据不回收
		std::lock_guard<std::mutex> lock(poolMutex);
		bufferPool.push_back(data);
	}

	// 防止渲染事件堆积的标�?
	std::atomic<bool> renderScheduled{false};

	// 渲染一帧（�?UI 线程调用�?
	void renderFrame()
	{
		// 使用 RAII 确保 renderScheduled 最终被重置
		struct ScheduleGuard {
			std::atomic<bool>& flag;
			ScheduleGuard(std::atomic<bool>& f) : flag(f) {}
			~ScheduleGuard() { flag = false; }
		} guard(renderScheduled);

		try {
			// LOG_DEBUG("[Renderer] renderFrame start");
			auto startRender = std::chrono::steady_clock::now();

			RenderData newData;
			bool shouldUpdate = false;
			{
				std::lock_guard<std::mutex> lock(nextFrameMutex);
				if (hasNextFrame) {
					newData = nextFrameData;
					// nextFrameData 已被移走，重置标�?
					hasNextFrame = false;
					// 注意：这里不清除 nextFrameData 的内容，只是标记为已取走
					// 真正的对象所有权已经转移�?newData
					shouldUpdate = true;
				}
			}

			if (shouldUpdate)
			{
				if (newData.points) {
					// 1. 回收上一帧的数据到池�?
					recycleBuffer(currentRenderData);

					// 2. 应用新数�?
					currentRenderData = newData;

					polyData->SetPoints(currentRenderData.points);
					polyData->SetVerts(currentRenderData.verts);
					polyData->GetPointData()->SetScalars(currentRenderData.colors);
					polyData->Modified();

					// 执行渲染
					renderWindow->Render();

					// 计算渲染耗时�?FPS
					auto endRender = std::chrono::steady_clock::now();

					frameCount++;
					std::chrono::duration<double> elapsed = endRender - lastFrameTime;
					if (elapsed.count() >= 1.0) // 每秒更新一�?FPS
					{
						currentFPS = static_cast<double>(frameCount) / elapsed.count();
						frameCount = 0;
						lastFrameTime = endRender;
					}

					// 更新左下角信息文�?(仅点�?
					if (infoTextActor)
					{
						size_t totalPts = currentRenderData.pointCount;
						size_t filteredPts = currentRenderData.filteredCount;
						double ratio = 0.0;
						if (totalPts + filteredPts > 0) {
							ratio = static_cast<double>(filteredPts) / static_cast<double>(totalPts + filteredPts) * 100.0;
						}
						char buf[256];
						std::snprintf(buf, sizeof(buf), "Points Num:%zu Filtered Points Num:%zu (%.1f%%)", totalPts, filteredPts, ratio);
						infoTextActor->SetInput(buf);
					}

					// 更新右上�?FPS 文本
					if (fpsTextActor)
					{
						char buf[256];
						double frameTime = (currentFPS > 0.001) ? (1000.0 / currentFPS) : 0.0;
						std::snprintf(buf, sizeof(buf), "FPS: %.1f  FrameTime: %.1f ms", currentFPS, frameTime);
						fpsTextActor->SetInput(buf);
					}
				}
			}
		} catch (const std::exception& e) {
			LOG_ERROR("[Renderer] Exception in renderFrame: {}", e.what());
		} catch (...) {
			LOG_ERROR("[Renderer] Unknown exception in renderFrame");
		}
		
		// renderScheduled 将由 guard 的析构函数重�?
	}
};

Renderer::Renderer() : pimpl(new Impl())
{
	// 禁用 VTK 警告弹窗（重定向到日志或完全静默）
	vtkOutputWindow::SetInstance(nullptr);  // 禁用所有 VTK 输出窗口
	vtkObject::GlobalWarningDisplayOff();   // 禁用全局警告显示

	// 启动渲染工作线程
	pimpl->workerRunning = true;
	pimpl->renderWorker = std::thread([this]() {
		while (pimpl->workerRunning)
		{
			std::vector<PointCloudPtr> currentMap;
			size_t currentFilteredCount = 0;
			{
				std::unique_lock<std::mutex> lock(pimpl->workerMutex);
				pimpl->workerCv.wait(lock, [this]() {
					return !pimpl->workerRunning || pimpl->hasPendingMap;
				});

				if (!pimpl->workerRunning) break;

				if (pimpl->hasPendingMap)
				{
					currentMap = std::move(pimpl->pendingMap);
					currentFilteredCount = pimpl->pendingFilteredCount;
					pimpl->hasPendingMap = false;
				}
			}

			if (currentMap.empty()) continue;

			// --- 执行繁重�?VTK 数据准备 ---
			// 使用缓冲池复用内存，避免频繁 new/delete
			Impl::RenderData data = pimpl->getFreeBuffer();
			
			size_t total_n = 0;
			for(const auto& pc : currentMap) total_n += pc->points.size();

			data.pointCount = total_n;
			data.filteredCount = currentFilteredCount;

			if (total_n > 0)
			{
				// 1. 准备 Points (复用 FloatArray)
				data.coordsArray->SetNumberOfTuples(static_cast<vtkIdType>(total_n));
				float* coordsBase = data.coordsArray->WritePointer(0, static_cast<vtkIdType>(total_n * 3));
				
				// 2. 准备 Colors (复用 UnsignedCharArray)
				data.colors->SetNumberOfTuples(static_cast<vtkIdType>(total_n));
				unsigned char* colorsBase = data.colors->WritePointer(0, static_cast<vtkIdType>(total_n * 3));

				// 3. 准备 Verts (复用 IdTypeArray)
				data.vertsArray->SetNumberOfValues(static_cast<vtkIdType>(total_n + 1));
				vtkIdType* idsBase = data.vertsArray->WritePointer(0, static_cast<vtkIdType>(total_n + 1));
				*idsBase = static_cast<vtkIdType>(total_n); // 第一个元素是点数
				vtkIdType* idsData = idsBase + 1; // 实际索引数据开始位�?
				
				if (total_n < PARALLEL_THRESHOLD)
				{
					// --- 单线程路�?---
					float* cPtr = coordsBase;
					unsigned char* colPtr = colorsBase;
					vtkIdType* iPtr = idsData;
					vtkIdType currentIdx = 0;

					for (const auto& pc : currentMap) {
						const auto* srcPtr = pc->points.data();
						size_t n = pc->points.size();
						for (size_t j = 0; j < n; ++j) {
							const auto& pt = srcPtr[j];
							*cPtr++ = pt.x; *cPtr++ = pt.y; *cPtr++ = pt.z;
							Linger::Color c = Linger::IntensityToRGB(pt.intensity);
							*colPtr++ = c.r; *colPtr++ = c.g; *colPtr++ = c.b;
							*iPtr++ = currentIdx++;
						}
					}
				}
				else
				{
					// --- OpenMP 并行路径 ---
					std::vector<size_t> offsets;
					offsets.reserve(currentMap.size());
					size_t current_offset = 0;
					for(const auto& pc : currentMap) {
						offsets.push_back(current_offset);
						current_offset += pc->points.size();
					}

					#pragma omp parallel for
					for (int i = 0; i < static_cast<int>(currentMap.size()); ++i) {
						const auto& pc = currentMap[i];
						size_t n = pc->points.size();
						if (n == 0) continue;

						size_t offset = offsets[i];
						float* cPtr = coordsBase + offset * 3;
						unsigned char* colPtr = colorsBase + offset * 3;
						vtkIdType* iPtr = idsData + offset;
						
						const auto* srcPtr = pc->points.data();
						
						for (size_t j = 0; j < n; ++j) {
							const auto& pt = srcPtr[j];
							*cPtr++ = pt.x; *cPtr++ = pt.y; *cPtr++ = pt.z;
							Linger::Color c = Linger::IntensityToRGB(pt.intensity);
							*colPtr++ = c.r; *colPtr++ = c.g; *colPtr++ = c.b;
							*iPtr++ = static_cast<vtkIdType>(offset + j);
						}
					}
				}
				
				data.verts->SetCells(1, data.vertsArray);
			}
			else
			{
				data.coordsArray->SetNumberOfTuples(0);
				data.colors->SetNumberOfTuples(0);
				data.vertsArray->SetNumberOfValues(0);
				data.verts->SetCells(0, data.vertsArray);
			}

			// 将准备好的数据存�?nextFrameData
			{
				std::lock_guard<std::mutex> lock(pimpl->nextFrameMutex);
				if (pimpl->hasNextFrame) {
					pimpl->recycleBuffer(pimpl->nextFrameData);
				}
				pimpl->nextFrameData = data;
				pimpl->hasNextFrame = true;
			}

			// 通知 UI 线程
			bool invoked = QMetaObject::invokeMethod(pimpl->widget, [this]()
									  {
				pimpl->renderFrame();
			}, Qt::QueuedConnection);

			if (!invoked)
			{
				LOG_ERROR("[Renderer] Failed to invoke renderFrame on UI thread. Resetting schedule.");
				pimpl->renderScheduled = false;
			}
		}
	});
}

Renderer::~Renderer()
{
	// 停止工作线程
	pimpl->workerRunning = false;
	pimpl->workerCv.notify_all();
	if (pimpl->renderWorker.joinable())
	{
		pimpl->renderWorker.join();
	}
	delete pimpl;
}

void Renderer::init(QWidget *parent)
{
	pimpl->widget = parent;

	auto vtkWidget = qobject_cast<QVTKOpenGLNativeWidget *>(pimpl->widget);
	if (!vtkWidget && pimpl->widget)
	{
		LOG_ERROR("Renderer::init: parent is not a QVTKOpenGLNativeWidget!");
		return;
	}

	if (vtkWidget)
	{
		// 创建渲染窗口并附加到 widget
		pimpl->renderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
		vtkWidget->setRenderWindow(pimpl->renderWindow);

		// 设置渲染器与 VTK 管线
		pimpl->renderer = vtkSmartPointer<vtkRenderer>::New();
		pimpl->renderWindow->AddRenderer(pimpl->renderer);

		pimpl->polyData = vtkSmartPointer<vtkPolyData>::New();
		pimpl->vtkPoints = vtkSmartPointer<vtkPoints>::New();
		pimpl->polyData->SetPoints(pimpl->vtkPoints);

		pimpl->mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pimpl->actor = vtkSmartPointer<vtkActor>::New();

		// 这需�?PolyData 中包�?Verts 拓扑结构
		pimpl->mapper->SetInputData(pimpl->polyData);
		
		pimpl->mapper->SetColorModeToDefault();
		pimpl->mapper->SetScalarVisibility(true);

		pimpl->actor->SetMapper(pimpl->mapper);
		pimpl->actor->GetProperty()->SetPointSize(1);
		pimpl->renderer->AddActor(pimpl->actor);
		pimpl->renderer->ResetCamera();

		// ROI 线框（默认隐藏）
		pimpl->roiOutline = vtkSmartPointer<vtkOutlineSource>::New();
		double roiBounds[6] = {
			static_cast<double>(LingerConfig::ROI_X_MIN_DEFAULT), static_cast<double>(LingerConfig::ROI_X_MAX_DEFAULT),
			static_cast<double>(LingerConfig::ROI_Y_MIN_DEFAULT), static_cast<double>(LingerConfig::ROI_Y_MAX_DEFAULT),
			static_cast<double>(LingerConfig::ROI_Z_MIN_DEFAULT), static_cast<double>(LingerConfig::ROI_Z_MAX_DEFAULT)};
		pimpl->roiOutline->SetBounds(roiBounds);
		pimpl->roiMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pimpl->roiMapper->SetInputConnection(pimpl->roiOutline->GetOutputPort());
		pimpl->roiActor = vtkSmartPointer<vtkActor>::New();
		pimpl->roiActor->SetMapper(pimpl->roiMapper);
		pimpl->roiActor->GetProperty()->SetColor(1.0, 0.6, 0.0); // 橙色线框
		pimpl->roiActor->GetProperty()->SetLineWidth(2.0);
		pimpl->roiActor->GetProperty()->SetOpacity(0.8);
		pimpl->roiActor->SetVisibility(false);
		pimpl->roiActor->PickableOff();
		pimpl->renderer->AddActor(pimpl->roiActor);

		// ====================================================================
		// 靠泊检测可视化初始�?
		// ====================================================================
		
		// 最近区域扇区（蓝色线框�?
		pimpl->nearestSectorOutline = vtkSmartPointer<vtkOutlineSource>::New();
		pimpl->nearestSectorOutline->SetBounds(0, 50, -15, 15, -1, 10);
		pimpl->nearestSectorMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pimpl->nearestSectorMapper->SetInputConnection(pimpl->nearestSectorOutline->GetOutputPort());
		pimpl->nearestSectorActor = vtkSmartPointer<vtkActor>::New();
		pimpl->nearestSectorActor->SetMapper(pimpl->nearestSectorMapper);
		pimpl->nearestSectorActor->GetProperty()->SetColor(0.2, 0.6, 1.0);  // 蓝色
		pimpl->nearestSectorActor->GetProperty()->SetLineWidth(2.0);
		pimpl->nearestSectorActor->GetProperty()->SetOpacity(0.7);
		pimpl->nearestSectorActor->SetVisibility(true);
		pimpl->nearestSectorActor->PickableOff();
		pimpl->renderer->AddActor(pimpl->nearestSectorActor);
		
		// 码头边缘检测区域（黄色线框�?
		pimpl->edgeRegionOutline = vtkSmartPointer<vtkOutlineSource>::New();
		pimpl->edgeRegionOutline->SetBounds(0, 30, -15, 15, 0.3, 1.5);
		pimpl->edgeRegionMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pimpl->edgeRegionMapper->SetInputConnection(pimpl->edgeRegionOutline->GetOutputPort());
		pimpl->edgeRegionActor = vtkSmartPointer<vtkActor>::New();
		pimpl->edgeRegionActor->SetMapper(pimpl->edgeRegionMapper);
		pimpl->edgeRegionActor->GetProperty()->SetColor(1.0, 1.0, 0.0);  // 黄色
		pimpl->edgeRegionActor->GetProperty()->SetLineWidth(2.0);
		pimpl->edgeRegionActor->GetProperty()->SetOpacity(0.7);
		pimpl->edgeRegionActor->SetVisibility(false);  // 默认隐藏
		pimpl->edgeRegionActor->PickableOff();
		pimpl->renderer->AddActor(pimpl->edgeRegionActor);
		
		// 边缘直线可视化（绿色粗线�?
		pimpl->edgeLinePoly = vtkSmartPointer<vtkPolyData>::New();
		pimpl->edgeLineMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pimpl->edgeLineMapper->SetInputData(pimpl->edgeLinePoly);
		pimpl->edgeLineActor = vtkSmartPointer<vtkActor>::New();
		pimpl->edgeLineActor->SetMapper(pimpl->edgeLineMapper);
		pimpl->edgeLineActor->GetProperty()->SetColor(0.0, 1.0, 0.0);  // 绿色
		pimpl->edgeLineActor->GetProperty()->SetLineWidth(4.0);
		pimpl->edgeLineActor->SetVisibility(false);
		pimpl->edgeLineActor->PickableOff();
		pimpl->renderer->AddActor(pimpl->edgeLineActor);
		
		// 最近区域距离指示（蓝色圆弧�?
		pimpl->nearestArcPoly = vtkSmartPointer<vtkPolyData>::New();
		pimpl->nearestArcMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		pimpl->nearestArcMapper->SetInputData(pimpl->nearestArcPoly);
		pimpl->nearestArcActor = vtkSmartPointer<vtkActor>::New();
		pimpl->nearestArcActor->SetMapper(pimpl->nearestArcMapper);
		pimpl->nearestArcActor->GetProperty()->SetColor(0.2, 0.8, 1.0);  // 浅蓝
		pimpl->nearestArcActor->GetProperty()->SetLineWidth(3.0);
		pimpl->nearestArcActor->SetVisibility(false);
		pimpl->nearestArcActor->PickableOff();
		pimpl->renderer->AddActor(pimpl->nearestArcActor);

		// 靠泊信息文本（左下方�?
		pimpl->dockingInfoActor = vtkSmartPointer<vtkTextActor>::New();
		pimpl->dockingInfoActor->SetInput("Docking: --");
		pimpl->dockingInfoActor->GetTextProperty()->SetFontSize(16);
		pimpl->dockingInfoActor->GetTextProperty()->SetColor(0.0, 1.0, 0.3);
		pimpl->dockingInfoActor->GetTextProperty()->SetFontFamilyToCourier();
		pimpl->dockingInfoActor->GetTextProperty()->BoldOn();
		pimpl->dockingInfoActor->SetDisplayPosition(10, 80);  // 左下方偏�?
		pimpl->dockingInfoActor->SetVisibility(false);
		pimpl->renderer->AddActor2D(pimpl->dockingInfoActor);

		// 创建世界坐标网格与坐标轴（红/绿）
		pimpl->gridPoly = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPoints> gridPts = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> gridLines = vtkSmartPointer<vtkCellArray>::New();

		// 轴线：X (�? �?-100..100, Y (�? �?-100..100
		// 优化：仅绘制网格线，不绘制中心轴线（由彩色轴线替代）
		// vtkIdType id0 = gridPts->InsertNextPoint(-100.0, 0.0, 0.0);
		// vtkIdType id1 = gridPts->InsertNextPoint(100.0, 0.0, 0.0);
		// vtkSmartPointer<vtkCellArray> axesLines = vtkSmartPointer<vtkCellArray>::New();
		// axesLines->InsertNextCell(2);
		// axesLines->InsertCellPoint(id0);
		// axesLines->InsertCellPoint(id1);

		// vtkIdType id2 = gridPts->InsertNextPoint(0.0, -100.0, 0.0);
		// vtkIdType id3 = gridPts->InsertNextPoint(0.0, 100.0, 0.0);
		// axesLines->InsertNextCell(2);
		// axesLines->InsertCellPoint(id2);
		// axesLines->InsertCellPoint(id3);

		// pimpl->gridPoly->SetPoints(gridPts);
		// pimpl->gridPoly->SetLines(axesLines);
		
		// 重新构建纯网格（不含轴线�?
		// 假设我们想要一个简单的网格背景，或者直接跳�?gridPoly 的轴线部�?
		// 这里简单起见，我们只保留彩色轴线，不再重复绘制白色轴线
		// 如果需要绘制其他网格线（如�?0m一条），可以在这里添加循环
		
		// 为了保持代码结构，我们创建一个空�?gridPoly 或者仅包含非轴线的网格
		// 这里演示添加�?10m 的辅助网格线（灰色）
		// vtkSmartPointer<vtkCellArray> gridLines = vtkSmartPointer<vtkCellArray>::New(); // 移除重复定义
		for (int i = -100; i <= 100; i += 10) {
			if (i == 0) continue; // 跳过轴线
			// 平行�?Y 轴的�?
			vtkIdType p1 = gridPts->InsertNextPoint(i, -100, 0);
			vtkIdType p2 = gridPts->InsertNextPoint(i, 100, 0);
			vtkSmartPointer<vtkIdList> line1 = vtkSmartPointer<vtkIdList>::New();
			line1->InsertNextId(p1); line1->InsertNextId(p2);
			gridLines->InsertNextCell(line1);

			// 平行�?X 轴的�?
			vtkIdType p3 = gridPts->InsertNextPoint(-100, i, 0);
			vtkIdType p4 = gridPts->InsertNextPoint(100, i, 0);
			vtkSmartPointer<vtkIdList> line2 = vtkSmartPointer<vtkIdList>::New();
			line2->InsertNextId(p3); line2->InsertNextId(p4);
			gridLines->InsertNextCell(line2);
		}
		pimpl->gridPoly->SetPoints(gridPts);
		pimpl->gridPoly->SetLines(gridLines);

		vtkSmartPointer<vtkPolyDataMapper> gridMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		gridMapper->SetInputData(pimpl->gridPoly);
		pimpl->gridActor = vtkSmartPointer<vtkActor>::New();
		pimpl->gridActor->SetMapper(gridMapper);
		pimpl->gridActor->GetProperty()->SetLineWidth(1.0);
		pimpl->gridActor->GetProperty()->SetColor(0.3, 0.3, 0.3); // 深灰色网�?
		pimpl->renderer->AddActor(pimpl->gridActor);

		// 我们通过创建两个 actor 来分别给轴着色：创建有色副本
		// 需要重新定义轴线的端点，因�?gridPts 现在包含了网格点
		vtkSmartPointer<vtkPoints> axisPts = vtkSmartPointer<vtkPoints>::New();
		axisPts->InsertNextPoint(-100, 0, 0);
		axisPts->InsertNextPoint(100, 0, 0);
		axisPts->InsertNextPoint(0, -100, 0);
		axisPts->InsertNextPoint(0, 100, 0);

		vtkSmartPointer<vtkPolyData> xPoly = vtkSmartPointer<vtkPolyData>::New();
		xPoly->SetPoints(axisPts);
		// 使用第一�?cell 创建 X轴线�?
		vtkSmartPointer<vtkCellArray> xLine = vtkSmartPointer<vtkCellArray>::New();
		xLine->InsertNextCell(2);
		xLine->InsertCellPoint(0);
		xLine->InsertCellPoint(1);
		xPoly->SetLines(xLine);
		vtkSmartPointer<vtkPolyDataMapper> xMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		xMapper->SetInputData(xPoly);
		vtkSmartPointer<vtkActor> xActor = vtkSmartPointer<vtkActor>::New();
		xActor->SetMapper(xMapper);
		xActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // 红色
		xActor->GetProperty()->SetLineWidth(2.0); // 加粗主轴
		pimpl->renderer->AddActor(xActor);

		vtkSmartPointer<vtkPolyData> yPoly = vtkSmartPointer<vtkPolyData>::New();
		yPoly->SetPoints(axisPts);
		vtkSmartPointer<vtkCellArray> yLine = vtkSmartPointer<vtkCellArray>::New();
		yLine->InsertNextCell(2);
		yLine->InsertCellPoint(2);
		yLine->InsertCellPoint(3);
		yPoly->SetLines(yLine);
		vtkSmartPointer<vtkPolyDataMapper> yMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		yMapper->SetInputData(yPoly);
		vtkSmartPointer<vtkActor> yActor = vtkSmartPointer<vtkActor>::New();
		yActor->SetMapper(yMapper);
		yActor->GetProperty()->SetColor(0.0, 1.0, 0.0); // 绿色
		yActor->GetProperty()->SetLineWidth(2.0); // 加粗主轴
		pimpl->renderer->AddActor(yActor);

		// --- 创建每隔20m同心环，直到100m ---
		pimpl->ringsPoly = vtkSmartPointer<vtkPolyData>::New();
		vtkSmartPointer<vtkPoints> ringPts = vtkSmartPointer<vtkPoints>::New();
		vtkSmartPointer<vtkCellArray> ringLines = vtkSmartPointer<vtkCellArray>::New();
		const int segments = 180;
		for (int r = 10; r <= 100; r += 20)
		{
			vtkIdType startId = ringPts->GetNumberOfPoints();
			for (int s = 0; s <= segments; ++s)
			{
				double ang = 2.0 * 3.14159265358979323846 * (static_cast<double>(s) / static_cast<double>(segments));
				double x = r * std::cos(ang);
				double y = r * std::sin(ang);
				ringPts->InsertNextPoint(x, y, 0.0);
			}
			// 为该环创建折�?cell
			vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
			for (int s = 0; s <= segments; ++s)
				idList->InsertNextId(startId + s);
			ringLines->InsertNextCell(idList);

			// �?(r,0,0)处创�?D文本标签
			vtkSmartPointer<vtkVectorText> txt = vtkSmartPointer<vtkVectorText>::New();
			std::string lbl = std::to_string(r) + "m";
			txt->SetText(lbl.c_str());
			vtkSmartPointer<vtkPolyDataMapper> txtMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
			txtMapper->SetInputConnection(txt->GetOutputPort());
			vtkSmartPointer<vtkFollower> txtActor = vtkSmartPointer<vtkFollower>::New();
			txtActor->SetMapper(txtMapper);
			txtActor->SetScale(0.5, 0.5, 0.5);
			txtActor->SetPosition(static_cast<double>(r), 0.0, 0.5);
			txtActor->GetProperty()->SetColor(1.0, 1.0, 1.0);
			pimpl->renderer->AddActor(txtActor);
			pimpl->ringLabels.push_back(txtActor);
		}
		pimpl->ringsPoly->SetPoints(ringPts);
		pimpl->ringsPoly->SetLines(ringLines);
		vtkSmartPointer<vtkPolyDataMapper> ringsMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
		ringsMapper->SetInputData(pimpl->ringsPoly);
		pimpl->ringsActor = vtkSmartPointer<vtkActor>::New();
		pimpl->ringsActor->SetMapper(ringsMapper);
		pimpl->ringsActor->GetProperty()->SetColor(0.8, 0.8, 0.8);
		pimpl->ringsActor->GetProperty()->SetLineWidth(1.0);
		pimpl->renderer->AddActor(pimpl->ringsActor);

		pimpl->renderer->ResetCamera();

		// 设置初始摄像机视角（倾斜俯视�?
		{
			auto cam = pimpl->renderer->GetActiveCamera();
			if (cam)
			{
				// 摄像机放置在 Y轴负方向并稍微抬高，指向世界原点
				double camX = 0.0;
				double camY = -40.0;
				double camZ = 20.0;
				cam->SetPosition(camX, camY, camZ);
				cam->SetFocalPoint(0.0, 0.0, 0.0);
				cam->SetViewUp(0.0, 0.0, 1.0);
				cam->ParallelProjectionOff();
				pimpl->renderer->ResetCameraClippingRange();
			}
		}

		// 如果创建了文本跟随器，设置其相机
		for (auto &lbl : pimpl->ringLabels)
		{
			if (lbl)
				lbl->SetCamera(pimpl->renderer->GetActiveCamera());
		}

		// 左下角坐标系
		pimpl->axesActor = vtkSmartPointer<vtkAxesActor>::New();
		// 调整轴标签大�?
		pimpl->axesActor->SetTotalLength(1.0, 1.0, 1.0);
		pimpl->axesActor->SetShaftTypeToCylinder();
		pimpl->axesActor->SetAxisLabels(true);
		// 只有在有 interactor 时启�?widget
		if (pimpl->renderWindow && pimpl->renderWindow->GetInteractor())
		{
			pimpl->axesWidget = vtkSmartPointer<vtkOrientationMarkerWidget>::New();
			pimpl->axesWidget->SetOrientationMarker(pimpl->axesActor);
			pimpl->axesWidget->SetInteractor(pimpl->renderWindow->GetInteractor());
			pimpl->axesWidget->SetViewport(0.02, 0.02, 0.18, 0.18); // 左下角小窗口
			pimpl->axesWidget->SetEnabled(1);
			pimpl->axesWidget->InteractiveOff();
		}

		// --- 新增:右下角反射率色彩�?(scalar bar) ---
		pimpl->scalarLUT = vtkSmartPointer<vtkLookupTable>::New();
		pimpl->scalarLUT->SetNumberOfTableValues(256);
		pimpl->scalarLUT->Build();
		// 将我们自定义�?intensityToRGB 用于填充 LUT
		for (int i = 0; i < 256; ++i)
		{
			Linger::Color c = Linger::IntensityToRGB(static_cast<float>(i) / 255.0f);
			pimpl->scalarLUT->SetTableValue(i, c.r / 255.0, c.g / 255.0, c.b / 255.0, 1.0);
		}
		pimpl->scalarBar = vtkSmartPointer<vtkScalarBarActor>::New();
		pimpl->scalarBar->SetLookupTable(pimpl->scalarLUT);
		pimpl->scalarBar->SetNumberOfLabels(6);
		pimpl->scalarBar->SetOrientationToVertical();
		// 使用归一化视口位置放在右下角
		pimpl->scalarBar->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
		pimpl->scalarBar->SetPosition(0.92, 0.08);
		pimpl->scalarBar->SetWidth(0.03);
		pimpl->scalarBar->SetHeight(0.35);
		pimpl->renderer->AddActor(pimpl->scalarBar);

		// 如果支持 vtkTextActor，用单独的文�?actor 放置标题于色条上方以增加间距控制
		pimpl->scalarTitleActor = vtkSmartPointer<vtkTextActor>::New();
		pimpl->scalarTitleActor->SetInput("Reflectivity");
		pimpl->scalarTitleActor->GetTextProperty()->SetFontSize(20);
		pimpl->scalarTitleActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0);
		pimpl->scalarTitleActor->GetTextProperty()->SetItalic(1);
		pimpl->scalarTitleActor->GetTextProperty()->SetJustificationToCentered();
		// 使用归一化视口坐标定位到色条上方（以给定�?margin 控制间距�?
		pimpl->scalarTitleActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
		const double barX = 0.92;	// �?scalarBar X
		const double barY = 0.08;	// scalarBar Y
		const double barH = 0.35;	// scalarBar 高度
		const double margin = 0.01; // 标题与色条的垂直间距
		// 将标题放在色条中心上�?
		const double titleX = barX + (pimpl->scalarBar->GetWidth() / 2.0);
		const double titleY = barY + barH + margin;
		pimpl->scalarTitleActor->SetPosition(titleX, titleY);
		pimpl->renderer->AddActor2D(pimpl->scalarTitleActor);

		// --- 恢复: 左下角点数文�?(白色) ---
		pimpl->infoTextActor = vtkSmartPointer<vtkTextActor>::New();
		pimpl->infoTextActor->GetTextProperty()->SetFontSize(20);
		pimpl->infoTextActor->GetTextProperty()->SetColor(1.0, 1.0, 1.0); // 白色
		pimpl->infoTextActor->SetInput("Points Num:0 Filtered Points Num:0 (0%)");
		pimpl->infoTextActor->SetDisplayPosition(10, 10);
		pimpl->renderer->AddActor2D(pimpl->infoTextActor);

		// --- 新增: 右上�?FPS 文本 (绿色) ---
		pimpl->fpsTextActor = vtkSmartPointer<vtkTextActor>::New();
		pimpl->fpsTextActor->GetTextProperty()->SetFontSize(20);
		pimpl->fpsTextActor->GetTextProperty()->SetColor(0.0, 1.0, 0.0); // 绿色
		pimpl->fpsTextActor->SetInput("FPS: 0.0  FrameTime: 0.0 ms");
		
		// 设置对齐方式为右上角
		pimpl->fpsTextActor->GetTextProperty()->SetJustificationToRight();
		pimpl->fpsTextActor->GetTextProperty()->SetVerticalJustificationToTop();
		
		// 使用归一化视口坐标定位到右上�?
		pimpl->fpsTextActor->GetPositionCoordinate()->SetCoordinateSystemToNormalizedViewport();
		pimpl->fpsTextActor->SetPosition(0.99, 0.99);
		
		pimpl->renderer->AddActor2D(pimpl->fpsTextActor);

		for (auto &lbl : pimpl->ringLabels)
		{
			if (lbl)
				lbl->SetCamera(pimpl->renderer->GetActiveCamera());
		}
	}
}


void Renderer::submitMap(const std::vector<PointCloudPtr>& map, size_t filtered_count)
{
	{
		std::lock_guard<std::mutex> lock(pimpl->snapshotMutex);
		pimpl->latestMapSnapshot = map;
	}

	// 使用 QMetaObject::invokeMethod �?UI 线程触发渲染更新
	if (pimpl->widget)
	{
		// FPS 控制：限制提交频�?
		auto now = std::chrono::steady_clock::now();
		std::chrono::duration<double, std::milli> diff = now - pimpl->lastSubmitTime;
		if (diff.count() < pimpl->minFrameIntervalMs)
		{
			return;
		}
		pimpl->lastSubmitTime = now;

		// 检查是否已有渲染任务在队列中或正在执行
		bool expected = false;
		if (pimpl->renderScheduled.compare_exchange_strong(expected, true))
		{
			pimpl->renderScheduleTime = std::chrono::steady_clock::now();
			// 将任务提交给工作线程
			{
				std::lock_guard<std::mutex> lock(pimpl->workerMutex);
				pimpl->pendingMap = map;
				pimpl->pendingFilteredCount = filtered_count;
				pimpl->hasPendingMap = true;
			}
			pimpl->workerCv.notify_one();
		}
		else {
			// 检测是否卡�?(超过 1�?
			auto now = std::chrono::steady_clock::now();
			if (std::chrono::duration_cast<std::chrono::milliseconds>(now - pimpl->renderScheduleTime).count() > 1000)
			{
				LOG_WARN("[Renderer] Warning: Render pipeline stuck for >1s. Forcing reset.");
				pimpl->renderScheduled = false;
			}
		}
	}
}


std::vector<PointCloudPtr> Renderer::takeLatestMapSnapshot()
{
	std::lock_guard<std::mutex> lock(pimpl->snapshotMutex);
	return pimpl->latestMapSnapshot;
}

void Renderer::setPointSize(int size)
{
	if (pimpl && pimpl->actor)
	{
		pimpl->actor->GetProperty()->SetPointSize(static_cast<float>(size));
		if (pimpl->renderWindow)
		{
			pimpl->renderWindow->Render();
		}
	}
}

void Renderer::setRoiBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, bool visible)
{
	if (!pimpl || !pimpl->roiOutline || !pimpl->roiActor) return;

	double bounds[6] = {
		static_cast<double>(x_min), static_cast<double>(x_max),
		static_cast<double>(y_min), static_cast<double>(y_max),
		static_cast<double>(z_min), static_cast<double>(z_max)};
	pimpl->roiOutline->SetBounds(bounds);
	pimpl->roiOutline->Update();
	pimpl->roiActor->SetVisibility(visible ? 1 : 0);

	if (pimpl->renderWindow) {
		pimpl->renderWindow->Render();
	}
}

void Renderer::setNearestSector(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, bool visible)
{
	if (!pimpl || !pimpl->nearestSectorOutline || !pimpl->nearestSectorActor) return;

	QMetaObject::invokeMethod(static_cast<QWidget*>(pimpl->widget), [this, x_min, x_max, y_min, y_max, z_min, z_max, visible]() {
		if (!pimpl || !pimpl->nearestSectorOutline) return;
		
		double bounds[6] = {x_min, x_max, y_min, y_max, z_min, z_max};
		pimpl->nearestSectorOutline->SetBounds(bounds);
		pimpl->nearestSectorOutline->Update();
		pimpl->nearestSectorActor->SetVisibility(visible ? 1 : 0);
		pimpl->nearestSectorVisible = visible;
		
		if (pimpl->renderWindow) pimpl->renderWindow->Render();
	}, Qt::QueuedConnection);
}

void Renderer::setEdgeRegion(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, bool visible)
{
	if (!pimpl || !pimpl->edgeRegionOutline || !pimpl->edgeRegionActor) return;

	QMetaObject::invokeMethod(static_cast<QWidget*>(pimpl->widget), [this, x_min, x_max, y_min, y_max, z_min, z_max, visible]() {
		if (!pimpl || !pimpl->edgeRegionOutline) return;
		
		double bounds[6] = {x_min, x_max, y_min, y_max, z_min, z_max};
		pimpl->edgeRegionOutline->SetBounds(bounds);
		pimpl->edgeRegionOutline->Update();
		pimpl->edgeRegionActor->SetVisibility(visible ? 1 : 0);
		pimpl->edgeRegionVisible = visible;
		
		if (pimpl->renderWindow) pimpl->renderWindow->Render();
	}, Qt::QueuedConnection);
}

void Renderer::setDockingVisible(bool nearest_visible, bool edge_visible)
{
	if (!pimpl) return;
	
	QMetaObject::invokeMethod(static_cast<QWidget*>(pimpl->widget), [this, nearest_visible, edge_visible]() {
		if (!pimpl) return;
		
		// 控制扇区可视�?
		if (pimpl->nearestSectorActor) {
			pimpl->nearestSectorActor->SetVisibility(nearest_visible && pimpl->nearestSectorVisible ? 1 : 0);
		}
		if (pimpl->edgeRegionActor) {
			pimpl->edgeRegionActor->SetVisibility(edge_visible && pimpl->edgeRegionVisible ? 1 : 0);
		}
		
		// 控制结果可视�?
		if (!nearest_visible && pimpl->nearestArcActor) {
			pimpl->nearestArcActor->SetVisibility(false);
		}
		if (!edge_visible && pimpl->edgeLineActor) {
			pimpl->edgeLineActor->SetVisibility(false);
		}
		
		// 文本控制
		if (!nearest_visible && !edge_visible && pimpl->dockingInfoActor) {
			pimpl->dockingInfoActor->SetVisibility(false);
		}
		
		if (pimpl->renderWindow) pimpl->renderWindow->Render();
	}, Qt::QueuedConnection);
}

void Renderer::updateDocking(const Linger::DockingState& state)
{
	if (!pimpl) return;

	// 将状态存入待处理队列
	{
		std::lock_guard<std::mutex> lock(pimpl->dockingStateMutex);
		pimpl->pendingDockingState = state;
		pimpl->hasPendingDockingState = true;
	}

	// 通过 Qt 事件循环�?UI 线程更新
	QMetaObject::invokeMethod(static_cast<QWidget*>(pimpl->widget), [this, state]() {
		if (!pimpl || !pimpl->dockingInfoActor) return;

		// ===== 最近区域距离可视化 =====
		if (state.nearest.valid && pimpl->nearestArcActor && pimpl->nearestArcPoly) {
			// 绘制距离圆弧
			vtkSmartPointer<vtkPoints> arcPoints = vtkSmartPointer<vtkPoints>::New();
			vtkSmartPointer<vtkCellArray> arcLines = vtkSmartPointer<vtkCellArray>::New();
			
			float radius = state.nearest.distance_m;
			float z_draw = 0.2f;
			int num_segments = 60;
			float arc_start = -M_PI / 3;  // -60 �?
			float arc_end = M_PI / 3;     // +60 �?
			
			for (int i = 0; i <= num_segments; ++i) {
				float angle = arc_start + (arc_end - arc_start) * i / num_segments;
				float x = radius * std::cos(angle);
				float y = radius * std::sin(angle);
				arcPoints->InsertNextPoint(x, y, z_draw);
			}
			
			// 连接圆弧�?
			for (int i = 0; i < num_segments; ++i) {
				vtkIdType ids[2] = {i, i + 1};
				arcLines->InsertNextCell(2, ids);
			}
			
			// 添加指向最近点的指示线
			vtkIdType center_id = arcPoints->InsertNextPoint(0, 0, z_draw);
			vtkIdType nearest_id = arcPoints->InsertNextPoint(state.nearest.nearest_x, state.nearest.nearest_y, z_draw);
			vtkIdType line_ids[2] = {center_id, nearest_id};
			arcLines->InsertNextCell(2, line_ids);
			
			pimpl->nearestArcPoly->SetPoints(arcPoints);
			pimpl->nearestArcPoly->SetLines(arcLines);
			pimpl->nearestArcPoly->Modified();
			
			// 根据置信度设置颜�?
			if (state.nearest.confidence >= 50) {
				pimpl->nearestArcActor->GetProperty()->SetColor(0.2, 0.8, 1.0);  // 浅蓝
			} else {
				pimpl->nearestArcActor->GetProperty()->SetColor(1.0, 1.0, 0.0);  // 黄色
			}
			pimpl->nearestArcActor->SetVisibility(pimpl->nearestSectorVisible ? 1 : 0);
		} else if (pimpl->nearestArcActor) {
			pimpl->nearestArcActor->SetVisibility(false);
		}
		
		// ===== 码头边缘可视�?=====
		if (state.edge.valid && pimpl->edgeLineActor && pimpl->edgeLinePoly) {
			vtkSmartPointer<vtkPoints> linePoints = vtkSmartPointer<vtkPoints>::New();
			vtkSmartPointer<vtkCellArray> lineCells = vtkSmartPointer<vtkCellArray>::New();

			float z_draw = 0.5f;
			linePoints->InsertNextPoint(state.edge.edge_line.x1, state.edge.edge_line.y1, z_draw);
			linePoints->InsertNextPoint(state.edge.edge_line.x2, state.edge.edge_line.y2, z_draw);

			vtkIdType ids[2] = {0, 1};
			lineCells->InsertNextCell(2, ids);

			pimpl->edgeLinePoly->SetPoints(linePoints);
			pimpl->edgeLinePoly->SetLines(lineCells);
			pimpl->edgeLinePoly->Modified();

			if (state.edge.confidence >= 50) {
				pimpl->edgeLineActor->GetProperty()->SetColor(0.0, 1.0, 0.0);  // 绿色
			} else {
				pimpl->edgeLineActor->GetProperty()->SetColor(1.0, 1.0, 0.0);  // 黄色
			}
			pimpl->edgeLineActor->SetVisibility(pimpl->edgeRegionVisible ? 1 : 0);
		} else if (pimpl->edgeLineActor) {
			pimpl->edgeLineActor->SetVisibility(false);
		}

		// ===== 更新文本信息 =====
		char info_buf[512];
		
		if (state.nearest.valid && state.edge.valid) {
			// 两者都有效：显示融合结果和各自的评分
			snprintf(info_buf, sizeof(info_buf), 
				"Final: %.2fm | Nearest: %.2fm (cont:%.2f, conf:%d%%) | Edge: %.2fm/%.1f° (geo:%.2f, conf:%d%%)",
				state.final_distance_m,
				state.nearest.distance_m, state.nearest.continuity_score, state.nearest.confidence,
				state.edge.distance_m, state.edge.angle_deg, state.edge.geometry_score, state.edge.confidence);
		} else if (state.nearest.valid) {
			// 仅最近区域有效
			snprintf(info_buf, sizeof(info_buf), 
				"Nearest: %.2fm  (cont:%.2f, used:%zu/%zu, conf:%d%%)",
				state.nearest.distance_m, state.nearest.continuity_score,
				state.nearest.used_points, state.nearest.point_count, state.nearest.confidence);
		} else if (state.edge.valid) {
			// 仅边缘检测有效
			snprintf(info_buf, sizeof(info_buf), 
				"Edge: %.2fm / %.1f°  (geo:%.2f, inliers:%zu/%zu, conf:%d%%)",
				state.edge.distance_m, state.edge.angle_deg, state.edge.geometry_score,
				state.edge.inlier_count, state.edge.total_points, state.edge.confidence);
		} else {
			snprintf(info_buf, sizeof(info_buf), "Docking: No detection");
		}
		
		pimpl->dockingInfoActor->SetInput(info_buf);
		
		if (state.status == Linger::DockingStatus::NORMAL) {
			pimpl->dockingInfoActor->GetTextProperty()->SetColor(0.0, 1.0, 0.3);
		} else if (state.status == Linger::DockingStatus::LOW_CONFIDENCE) {
			pimpl->dockingInfoActor->GetTextProperty()->SetColor(1.0, 1.0, 0.0);
		} else {
			pimpl->dockingInfoActor->GetTextProperty()->SetColor(1.0, 0.5, 0.0);
		}
		pimpl->dockingInfoActor->SetVisibility(true);

		if (pimpl->renderWindow) {
			pimpl->renderWindow->Render();
		}
	}, Qt::QueuedConnection);
}
