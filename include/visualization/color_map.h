#pragma once

#include <cstdint>

namespace Linger {

struct Color {
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

/**
 * @brief 将 intensity [0..1] 映射到伪彩色 RGB
 * 映射关系：深蓝 -> 蓝 -> 青 ->绿 -> 黄 -> 红
 * @param intensity 强度值，范围 [0, 1]
 * @return Color RGB 颜色结构体
 */
Color IntensityToRGB(float intensity);

} // namespace Linger
