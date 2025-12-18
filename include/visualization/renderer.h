#pragma once

#include "core/common.h"
#include "algorithms/docking_types.h"

class QWidget;

class Renderer
{
public:
	Renderer();
	~Renderer();

	void init(QWidget *parent = nullptr);

	/** @brief 原子发布累积地图快照（由 Processor 使用） */
	void submitMap(const std::vector<PointCloudPtr>& map, size_t filtered_count = 0);

	/** @brief 原子读取最新的地图快照 */
	std::vector<PointCloudPtr> takeLatestMapSnapshot();

	/** @brief 设置点云渲染大小 */
	void setPointSize(int size);

	/** @brief 显示/更新 ROI 线框（用于调试，可关闭） */
	void setRoiBox(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, bool visible);

	//==========================================================================
	// 靠泊检测可视化
	//==========================================================================
	
	/** @brief 更新靠泊检测可视化 */
	void updateDocking(const Linger::DockingState& state);

	/** @brief 设置最近区域检测扇区可视化 */
	void setNearestSector(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, bool visible);
	
	/** @brief 设置码头边缘检测区域可视化（XY 扇区 + Z 切片） */
	void setEdgeRegion(float x_min, float x_max, float y_min, float y_max, float z_min, float z_max, bool visible);

	/** @brief 设置靠泊检测整体可视化开关 */
	void setDockingVisible(bool nearest_visible, bool edge_visible);

	// 兼容旧接口
	void updateShoreline(const Linger::DockingState& state) { updateDocking(state); }
	void setShorelineVisible(bool visible) { setDockingVisible(visible, visible); }

private:
	struct Impl;
	Impl *pimpl;
};

