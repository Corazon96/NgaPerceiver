#include "visualization/color_map.h"
#include <cmath>
#include <algorithm>
#include <vector>

namespace Linger {

namespace {
/**
 * @brief 内部辅助函数：计算单个颜色的原始逻辑
 */
Color ComputeColor(float intensity)
{
    double x = std::isnan(static_cast<double>(intensity)) ? 0.0 : static_cast<double>(intensity);
    x = std::clamp(x, 0.0, 1.0);
    struct Stop
    {
        double t;
        int r;
        int g;
        int b;
    };
    static const Stop stops[] = {
        {0.0, 48, 0, 96},
        {0.20, 0, 0, 255},
        {0.35, 0, 255, 255},
        {0.50, 0, 255, 0},
        {0.75, 255, 255, 0},
        {1.00, 255, 0, 0}};
    const int nStops = static_cast<int>(sizeof(stops) / sizeof(stops[0]));
    if (x <= stops[0].t)
    {
        return {
            static_cast<uint8_t>(stops[0].r),
            static_cast<uint8_t>(stops[0].g),
            static_cast<uint8_t>(stops[0].b)
        };
    }
    for (int i = 0; i < nStops - 1; ++i)
    {
        if (x <= stops[i + 1].t)
        {
            double t0 = stops[i].t;
            double t1 = stops[i + 1].t;
            double a = (t1 > t0) ? ((x - t0) / (t1 - t0)) : 0.0;
            double r = (1.0 - a) * stops[i].r + a * stops[i + 1].r;
            double g = (1.0 - a) * stops[i].g + a * stops[i + 1].g;
            double b = (1.0 - a) * stops[i].b + a * stops[i + 1].b;
            return {
                static_cast<uint8_t>(std::lround(r)),
                static_cast<uint8_t>(std::lround(g)),
                static_cast<uint8_t>(std::lround(b))
            };
        }
    }
    return {
        static_cast<uint8_t>(stops[nStops - 1].r),
        static_cast<uint8_t>(stops[nStops - 1].g),
        static_cast<uint8_t>(stops[nStops - 1].b)
    };
}
}

Color IntensityToRGB(float intensity)
{
    // 使用 1024 大小的查找表，平衡精度与内存
    static const int kLutSize = 1024;
    static const std::vector<Color> lut = []() {
        std::vector<Color> table(kLutSize);
        for (int i = 0; i < kLutSize; ++i) {
            float v = static_cast<float>(i) / (kLutSize - 1);
            table[i] = ComputeColor(v);
        }
        return table;
    }();

    float x = std::isnan(intensity) ? 0.0f : intensity;
    x = std::clamp(x, 0.0f, 1.0f);
    
    // 映射到 [0, kLutSize - 1]
    int index = static_cast<int>(x * (kLutSize - 1));
    return lut[index];
}

} // namespace Linger
