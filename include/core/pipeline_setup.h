#pragma once

class PointCloudProcessor;
class PointCloudWgt;

/**
 * @brief 配置点云处理管线
 * 创建并添加滤波器，同时连接 UI 信号与滤波器参数
 * 
 * @param proc 处理器实例
 * @param w UI 窗口实例
 */
void SetupFilterPipeline(PointCloudProcessor& proc, PointCloudWgt& w);
