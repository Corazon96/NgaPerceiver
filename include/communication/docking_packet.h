#pragma once

#include <cstdint>

namespace Linger {

/**
 * @brief 靠泊检测数据包（UDP 通信协议）
 * 
 * 设计说明：
 * - 感知系统输出雷达坐标系下的原始距离/角度
 * - 坐标转换（雷达→船体）由控制系统负责
 * - 小端字节序，1字节对齐
 * 
 * 字段布局（共 20 字节）：
 * | 偏移 | 长度 | 字段名        | 说明                              |
 * |------|------|---------------|-----------------------------------|
 * | 0    | 8    | timestamp_ns  | 时间戳（纳秒）                    |
 * | 8    | 1    | sensor_id     | 雷达编号 (1=前, 2=左舷, 3=右舷)   |
 * | 9    | 1    | status        | 状态码                            |
 * | 10   | 1    | confidence    | 置信度 0-100                      |
 * | 11   | 1    | reserved      | 保留                              |
 * | 12   | 4    | distance_m    | 检测距离（米）                    |
 * | 16   | 4    | angle_deg     | 边缘角度（度），无效时为0         |
 */
#pragma pack(push, 1)
struct DockingPacket {
    uint64_t timestamp_ns;   ///< 时间戳（纳秒）
    uint8_t  sensor_id;      ///< 雷达编号 (1=前, 2=左舷, 3=右舷)
    uint8_t  status;         ///< 状态: 0=未检测, 1=正常, 2=低置信度
    uint8_t  confidence;     ///< 置信度 0-100
    uint8_t  reserved;       ///< 保留（对齐）
    float    distance_m;     ///< 雷达检测距离（米，雷达坐标系）
    float    angle_deg;      ///< 边缘角度（度），无边缘检测时为0
};
#pragma pack(pop)

static_assert(sizeof(DockingPacket) == 20, "DockingPacket must be 20 bytes");

/**
 * @brief 状态码定义
 */
namespace DockingPacketStatus {
    constexpr uint8_t NOT_DETECTED = 0;    ///< 未检测到目标
    constexpr uint8_t NORMAL = 1;          ///< 正常检测
    constexpr uint8_t LOW_CONFIDENCE = 2;  ///< 低置信度警告
}

/**
 * @brief 传感器ID定义
 */
namespace SensorId {
    constexpr uint8_t FRONT = 1;           ///< 前向雷达
    constexpr uint8_t PORT = 2;            ///< 左舷雷达
    constexpr uint8_t STARBOARD = 3;       ///< 右舷雷达
}

} // namespace Linger
