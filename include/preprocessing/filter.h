#pragma once

#include "core/common.h"
#include <string>
#include <memory>
#include <mutex>

namespace Linger {

/**
 * @brief 抽象滤波基类
 * 所有具体的滤波器都应继承此类并实现 filter 方法
 */
class Filter {
public:
    virtual ~Filter() = default;

    /**
     * @brief 执行滤波操作
     * @param input 输入点云
     * @param output 输出点云 (可以是同一个对象)
     */
    virtual void filter(const PointCloudPtr& input, PointCloudPtr& output) = 0;

    /**
     * @brief 获取滤波器名称
     */
    virtual std::string name() const = 0;

    /**
     * @brief 获取最近一次过滤掉的点数
     */
    virtual size_t getLastFilteredCount() const { return 0; }
};

using FilterPtr = std::shared_ptr<Filter>;

} // namespace Linger
