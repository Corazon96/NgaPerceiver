#include "algorithms/docking_algorithm.h"
#include "core/logger.h"

#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>

namespace Linger {

// 常量定义
static constexpr float PI = 3.14159265358979323846f;
static constexpr float RAD_TO_DEG = 180.0f / PI;

DockingAlgorithm::DockingAlgorithm() = default;
DockingAlgorithm::~DockingAlgorithm() = default;

void DockingAlgorithm::setConfig(const DockingConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
}

DockingConfig DockingAlgorithm::getConfig() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
}

void DockingAlgorithm::setNearestConfig(const NearestRegionConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_.nearest = config;
}

void DockingAlgorithm::setEdgeConfig(const DockEdgeConfig& config)
{
    std::lock_guard<std::mutex> lock(mutex_);
    config_.edge = config;
}

DockingState DockingAlgorithm::getLastState() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return last_state_;
}

void DockingAlgorithm::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    last_state_ = DockingState{};
    smoothed_nearest_dist_ = 0.0f;
    smoothed_edge_dist_ = 0.0f;
    smoothed_edge_angle_ = 0.0f;
    has_previous_nearest_ = false;
    has_previous_edge_ = false;
}

DockingState DockingAlgorithm::process(const PointCloudPtr& cloud, uint64_t timestamp_ns)
{
    DockingState state;
    state.timestamp_ns = timestamp_ns;
    state.status = DockingStatus::NOT_DETECTED;

    if (!cloud || cloud->empty()) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = state;
        if (onStateUpdated) onStateUpdated(state);
        return state;
    }

    DockingConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_;
    }

    // 1. 最近区域距离检�?
    if (cfg.nearest.enabled) {
        state.nearest = detectNearestRegion(cloud);
    }

    // 2. 码头边缘检�?
    if (cfg.edge.enabled) {
        state.edge = detectDockEdge(cloud);
    }

    // 3. 综合结果
    if (state.nearest.valid || state.edge.valid) {
        state.status = DockingStatus::NORMAL;
        
        // 优先使用边缘检测（如果有效且启用）
        if (state.edge.valid) {
            state.final_distance_m = state.edge.distance_m;
            state.final_angle_deg = state.edge.angle_deg;
        } else if (state.nearest.valid) {
            state.final_distance_m = state.nearest.distance_m;
            state.final_angle_deg = 0.0f;  // 最近区域无角度
        }
        
        // 低置信度检�?
        uint8_t conf = state.edge.valid ? state.edge.confidence : state.nearest.confidence;
        if (conf < 50) {
            state.status = DockingStatus::LOW_CONFIDENCE;
        }
    }

    // 4. 更新状�?
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = state;
    }

    if (onStateUpdated) {
        onStateUpdated(state);
    }

    return state;
}

DockingState DockingAlgorithm::processMultiple(const std::vector<PointCloudPtr>& clouds, uint64_t timestamp_ns)
{
    DockingState state;
    state.timestamp_ns = timestamp_ns;
    state.status = DockingStatus::NOT_DETECTED;

    // 检查是否有有效点云
    size_t total_points = 0;
    for (const auto& cloud : clouds) {
        if (cloud) total_points += cloud->points.size();
    }

    if (total_points == 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = state;
        if (onStateUpdated) onStateUpdated(state);
        return state;
    }

    DockingConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_;
    }

    // 1. 最近区域距离检测（直接遍历多个点云块）
    if (cfg.nearest.enabled) {
        state.nearest = detectNearestRegionMultiple(clouds);
    }

    // 2. 码头边缘检测（直接遍历多个点云块）
    if (cfg.edge.enabled) {
        state.edge = detectDockEdgeMultiple(clouds);
    }

    // 3. 综合结果
    if (state.nearest.valid || state.edge.valid) {
        state.status = DockingStatus::NORMAL;
        
        // 优先使用边缘检测（如果有效且启用）
        if (state.edge.valid) {
            state.final_distance_m = state.edge.distance_m;
            state.final_angle_deg = state.edge.angle_deg;
        } else if (state.nearest.valid) {
            state.final_distance_m = state.nearest.distance_m;
            state.final_angle_deg = 0.0f;
        }
        
        // 低置信度检�?
        uint8_t conf = state.edge.valid ? state.edge.confidence : state.nearest.confidence;
        if (conf < 50) {
            state.status = DockingStatus::LOW_CONFIDENCE;
        }
    }

    // 4. 更新状�?
    {
        std::lock_guard<std::mutex> lock(mutex_);
        last_state_ = state;
    }

    if (onStateUpdated) {
        onStateUpdated(state);
    }

    return state;
}

//=============================================================================
// 最近区域距离检�?
//=============================================================================

NearestRegionResult DockingAlgorithm::detectNearestRegion(const PointCloudPtr& cloud)
{
    NearestRegionResult result;
    
    NearestRegionConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_.nearest;
    }

    // 1. 扇区过滤 + 计算距离
    std::vector<float> distances;
    std::vector<std::pair<float, float>> positions;  // (x, y)
    distances.reserve(cloud->points.size());
    positions.reserve(cloud->points.size());

    for (const auto& pt : cloud->points) {
        // 扇区过滤
        if (pt.x < cfg.sector_x_min || pt.x > cfg.sector_x_max) continue;
        if (pt.y < cfg.sector_y_min || pt.y > cfg.sector_y_max) continue;
        if (pt.z < cfg.sector_z_min || pt.z > cfg.sector_z_max) continue;

        // 计算到原点的水平距离（XY平面�?
        float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
        distances.push_back(dist);
        positions.emplace_back(pt.x, pt.y);
    }

    result.point_count = distances.size();

    // 2. 检查点�?
    if (distances.size() < cfg.min_points) {
        LOG_DEBUG("[Docking/Nearest] Insufficient points: {} < {}", distances.size(), cfg.min_points);
        return result;
    }

    // 3. 计算最�?percentile 的点
    std::vector<size_t> indices(distances.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    // 按距离排序索�?
    std::sort(indices.begin(), indices.end(), [&distances](size_t a, size_t b) {
        return distances[a] < distances[b];
    });

    // 取最近的 N% �?
    size_t use_count = static_cast<size_t>(distances.size() * cfg.percentile / 100.0f);
    use_count = std::max(use_count, static_cast<size_t>(1));
    use_count = std::min(use_count, distances.size());
    result.used_points = use_count;

    // 4. 计算平均距离和位�?
    float sum_dist = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    for (size_t i = 0; i < use_count; ++i) {
        size_t idx = indices[i];
        sum_dist += distances[idx];
        sum_x += positions[idx].first;
        sum_y += positions[idx].second;
    }
    
    result.distance_m = sum_dist / use_count;
    result.nearest_x = sum_x / use_count;
    result.nearest_y = sum_y / use_count;

    // 5. 应用平滑
    if (cfg.enable_smoothing) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_previous_nearest_) {
            result.distance_m = cfg.smoothing_alpha * result.distance_m + 
                               (1.0f - cfg.smoothing_alpha) * smoothed_nearest_dist_;
        }
        smoothed_nearest_dist_ = result.distance_m;
        has_previous_nearest_ = true;
    }

    // 6. 计算置信�?
    // 基于点数和分�?
    float point_ratio = static_cast<float>(distances.size()) / static_cast<float>(cfg.min_points);
    point_ratio = std::min(point_ratio, 5.0f);  // cap at 5x
    result.confidence = static_cast<uint8_t>(std::min(100.0f, point_ratio * 20.0f));

    result.valid = true;
    
    LOG_DEBUG("[Docking/Nearest] Distance: {:.2f}m, Points: {}/{}, Conf: {}%",
              result.distance_m, result.used_points, result.point_count, result.confidence);

    return result;
}

NearestRegionResult DockingAlgorithm::detectNearestRegionMultiple(const std::vector<PointCloudPtr>& clouds)
{
    NearestRegionResult result;
    
    NearestRegionConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_.nearest;
    }

    // 计算总点数用于预分配
    size_t total_points = 0;
    for (const auto& cloud : clouds) {
        if (cloud) total_points += cloud->points.size();
    }

    // 1. 扇区过滤 + 计算距离（直接遍历多个点云块�?
    std::vector<float> distances;
    std::vector<std::pair<float, float>> positions;
    distances.reserve(total_points);
    positions.reserve(total_points);

    for (const auto& cloud : clouds) {
        if (!cloud) continue;
        for (const auto& pt : cloud->points) {
            if (pt.x < cfg.sector_x_min || pt.x > cfg.sector_x_max) continue;
            if (pt.y < cfg.sector_y_min || pt.y > cfg.sector_y_max) continue;
            if (pt.z < cfg.sector_z_min || pt.z > cfg.sector_z_max) continue;

            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y);
            distances.push_back(dist);
            positions.emplace_back(pt.x, pt.y);
        }
    }

    result.point_count = distances.size();

    if (distances.size() < cfg.min_points) {
        LOG_DEBUG("[Docking/Nearest] Insufficient points: {} < {}", distances.size(), cfg.min_points);
        return result;
    }

    // 3. 计算最�?percentile 的点
    std::vector<size_t> indices(distances.size());
    std::iota(indices.begin(), indices.end(), 0);
    std::sort(indices.begin(), indices.end(), [&distances](size_t a, size_t b) {
        return distances[a] < distances[b];
    });

    size_t use_count = static_cast<size_t>(distances.size() * cfg.percentile / 100.0f);
    use_count = std::max(use_count, static_cast<size_t>(1));
    use_count = std::min(use_count, distances.size());
    result.used_points = use_count;

    float sum_dist = 0.0f;
    float sum_x = 0.0f, sum_y = 0.0f;
    for (size_t i = 0; i < use_count; ++i) {
        size_t idx = indices[i];
        sum_dist += distances[idx];
        sum_x += positions[idx].first;
        sum_y += positions[idx].second;
    }
    
    result.distance_m = sum_dist / use_count;
    result.nearest_x = sum_x / use_count;
    result.nearest_y = sum_y / use_count;

    if (cfg.enable_smoothing) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_previous_nearest_) {
            result.distance_m = cfg.smoothing_alpha * result.distance_m + 
                               (1.0f - cfg.smoothing_alpha) * smoothed_nearest_dist_;
        }
        smoothed_nearest_dist_ = result.distance_m;
        has_previous_nearest_ = true;
    }

    float point_ratio = static_cast<float>(distances.size()) / static_cast<float>(cfg.min_points);
    point_ratio = std::min(point_ratio, 5.0f);
    result.confidence = static_cast<uint8_t>(std::min(100.0f, point_ratio * 20.0f));

    result.valid = true;
    
    LOG_DEBUG("[Docking/Nearest] Distance: {:.2f}m, Points: {}/{}, Conf: {}%",
              result.distance_m, result.used_points, result.point_count, result.confidence);

    return result;
}

//=============================================================================
// 码头边缘检�?
//=============================================================================

DockEdgeResult DockingAlgorithm::detectDockEdge(const PointCloudPtr& cloud)
{
    DockEdgeResult result;
    
    DockEdgeConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_.edge;
    }

    // 1. 扇区 + Z轴切片过滤，投影�?XY 平面
    std::vector<std::pair<float, float>> points_2d;
    points_2d.reserve(cloud->points.size());

    for (const auto& pt : cloud->points) {
        // XY 扇区过滤
        if (pt.x < cfg.sector_x_min || pt.x > cfg.sector_x_max) continue;
        if (pt.y < cfg.sector_y_min || pt.y > cfg.sector_y_max) continue;
        
        // Z 轴切片（码头边缘高度�?
        if (pt.z < cfg.edge_z_min || pt.z > cfg.edge_z_max) continue;

        points_2d.emplace_back(pt.x, pt.y);
    }

    result.total_points = points_2d.size();

    // 2. 检查点�?
    if (points_2d.size() < cfg.min_points) {
        LOG_DEBUG("[Docking/Edge] Insufficient points in Z slice: {} < {}", 
                  points_2d.size(), cfg.min_points);
        return result;
    }

    // 3. RANSAC 直线拟合
    Line2D line;
    std::vector<size_t> inliers;
    
    if (!fitLineRansac(points_2d, cfg, line, inliers)) {
        LOG_DEBUG("[Docking/Edge] RANSAC fitting failed");
        return result;
    }

    // 4. 最小二乘精�?
    refineLine(points_2d, inliers, line);

    // 5. 计算内点平均误差
    float total_error = 0.0f;
    for (size_t idx : inliers) {
        float dist = std::abs(pointToLineDistance(points_2d[idx].first, points_2d[idx].second, line));
        total_error += dist;
    }
    result.mean_error = inliers.empty() ? 0.0f : total_error / inliers.size();
    result.inlier_count = inliers.size();

    // 6. 计算直线端点（用于可视化�?
    computeLineEndpoints(points_2d, inliers, line);
    result.edge_line = line;

    // 7. 计算距离和角�?
    // 原点到直线的距离
    result.distance_m = std::abs(line.c);  // 因为 |a|²+|b|²=1，所�?|c| 就是原点到直线的距离
    
    // 直线�?X 轴的夹角
    // 直线方向向量�?(-b, a)
    // �?X 轴夹�?= atan2(a, -b)
    result.angle_deg = std::atan2(line.a, -line.b) * RAD_TO_DEG;

    // 8. 应用平滑
    if (cfg.enable_smoothing) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_previous_edge_) {
            result.distance_m = cfg.smoothing_alpha * result.distance_m + 
                               (1.0f - cfg.smoothing_alpha) * smoothed_edge_dist_;
            result.angle_deg = cfg.smoothing_alpha * result.angle_deg + 
                              (1.0f - cfg.smoothing_alpha) * smoothed_edge_angle_;
        }
        smoothed_edge_dist_ = result.distance_m;
        smoothed_edge_angle_ = result.angle_deg;
        has_previous_edge_ = true;
    }

    // 9. 计算置信�?
    float inlier_ratio = static_cast<float>(inliers.size()) / points_2d.size();
    float error_score = std::max(0.0f, 1.0f - result.mean_error / cfg.ransac_distance_threshold);
    result.confidence = static_cast<uint8_t>(std::min(100.0f, (inlier_ratio * 50.0f + error_score * 50.0f)));

    result.valid = true;
    
    LOG_DEBUG("[Docking/Edge] Distance: {:.2f}m, Angle: {:.1f}°, Inliers: {}/{}, Conf: {}%",
              result.distance_m, result.angle_deg, result.inlier_count, result.total_points, result.confidence);

    return result;
}

DockEdgeResult DockingAlgorithm::detectDockEdgeMultiple(const std::vector<PointCloudPtr>& clouds)
{
    DockEdgeResult result;
    
    DockEdgeConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_.edge;
    }

    // 计算总点数用于预分配
    size_t total_points = 0;
    for (const auto& cloud : clouds) {
        if (cloud) total_points += cloud->points.size();
    }

    // 1. 扇区 + Z轴切片过滤（直接遍历多个点云块）
    std::vector<std::pair<float, float>> points_2d;
    points_2d.reserve(total_points);

    for (const auto& cloud : clouds) {
        if (!cloud) continue;
        for (const auto& pt : cloud->points) {
            if (pt.x < cfg.sector_x_min || pt.x > cfg.sector_x_max) continue;
            if (pt.y < cfg.sector_y_min || pt.y > cfg.sector_y_max) continue;
            if (pt.z < cfg.edge_z_min || pt.z > cfg.edge_z_max) continue;

            points_2d.emplace_back(pt.x, pt.y);
        }
    }

    result.total_points = points_2d.size();

    if (points_2d.size() < cfg.min_points) {
        LOG_DEBUG("[Docking/Edge] Insufficient points in Z slice: {} < {}", 
                  points_2d.size(), cfg.min_points);
        return result;
    }

    // RANSAC 直线拟合
    Line2D line;
    std::vector<size_t> inliers;
    
    if (!fitLineRansac(points_2d, cfg, line, inliers)) {
        LOG_DEBUG("[Docking/Edge] RANSAC fitting failed");
        return result;
    }

    refineLine(points_2d, inliers, line);

    float total_error = 0.0f;
    for (size_t idx : inliers) {
        float dist = std::abs(pointToLineDistance(points_2d[idx].first, points_2d[idx].second, line));
        total_error += dist;
    }
    result.mean_error = inliers.empty() ? 0.0f : total_error / inliers.size();
    result.inlier_count = inliers.size();

    computeLineEndpoints(points_2d, inliers, line);
    result.edge_line = line;

    result.distance_m = std::abs(line.c);
    result.angle_deg = std::atan2(line.a, -line.b) * RAD_TO_DEG;

    if (cfg.enable_smoothing) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_previous_edge_) {
            result.distance_m = cfg.smoothing_alpha * result.distance_m + 
                               (1.0f - cfg.smoothing_alpha) * smoothed_edge_dist_;
            result.angle_deg = cfg.smoothing_alpha * result.angle_deg + 
                              (1.0f - cfg.smoothing_alpha) * smoothed_edge_angle_;
        }
        smoothed_edge_dist_ = result.distance_m;
        smoothed_edge_angle_ = result.angle_deg;
        has_previous_edge_ = true;
    }

    float inlier_ratio = static_cast<float>(inliers.size()) / points_2d.size();
    float error_score = std::max(0.0f, 1.0f - result.mean_error / cfg.ransac_distance_threshold);
    result.confidence = static_cast<uint8_t>(std::min(100.0f, (inlier_ratio * 50.0f + error_score * 50.0f)));

    result.valid = true;
    
    LOG_DEBUG("[Docking/Edge] Distance: {:.2f}m, Angle: {:.1f}°, Inliers: {}/{}, Conf: {}%",
              result.distance_m, result.angle_deg, result.inlier_count, result.total_points, result.confidence);

    return result;
}

bool DockingAlgorithm::fitLineRansac(const std::vector<std::pair<float, float>>& points,
                                      const DockEdgeConfig& cfg,
                                      Line2D& line,
                                      std::vector<size_t>& inliers)
{
    if (points.size() < 2) return false;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    Line2D best_line;
    std::vector<size_t> best_inliers;
    size_t best_count = 0;

    for (int iter = 0; iter < cfg.ransac_max_iterations; ++iter) {
        // 随机选择两个不同的点
        size_t idx1 = dist(gen);
        size_t idx2 = dist(gen);
        while (idx2 == idx1) {
            idx2 = dist(gen);
        }

        float x1 = points[idx1].first;
        float y1 = points[idx1].second;
        float x2 = points[idx2].first;
        float y2 = points[idx2].second;

        // 计算直线参数 Ax + By + C = 0
        float dx = x2 - x1;
        float dy = y2 - y1;
        
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6f) continue;

        Line2D candidate;
        candidate.a = dy / len;
        candidate.b = -dx / len;
        candidate.c = -(candidate.a * x1 + candidate.b * y1);

        // 计算内点
        std::vector<size_t> current_inliers;
        for (size_t i = 0; i < points.size(); ++i) {
            float d = std::abs(pointToLineDistance(points[i].first, points[i].second, candidate));
            if (d < cfg.ransac_distance_threshold) {
                current_inliers.push_back(i);
            }
        }

        if (current_inliers.size() > best_count) {
            best_count = current_inliers.size();
            best_line = candidate;
            best_inliers = std::move(current_inliers);
        }

        // 提前终止
        float inlier_ratio = static_cast<float>(best_count) / points.size();
        if (inlier_ratio > 0.8f) {
            break;
        }
    }

    // 检查内点比�?
    float inlier_ratio = static_cast<float>(best_count) / points.size();
    if (inlier_ratio < cfg.ransac_min_inlier_ratio) {
        return false;
    }

    line = best_line;
    inliers = std::move(best_inliers);
    return true;
}

void DockingAlgorithm::refineLine(const std::vector<std::pair<float, float>>& points,
                                   const std::vector<size_t>& inliers,
                                   Line2D& line)
{
    if (inliers.size() < 2) return;

    // PCA 最小二乘精�?
    float mean_x = 0.0f, mean_y = 0.0f;
    for (size_t idx : inliers) {
        mean_x += points[idx].first;
        mean_y += points[idx].second;
    }
    mean_x /= inliers.size();
    mean_y /= inliers.size();

    float sxx = 0.0f, syy = 0.0f, sxy = 0.0f;
    for (size_t idx : inliers) {
        float dx = points[idx].first - mean_x;
        float dy = points[idx].second - mean_y;
        sxx += dx * dx;
        syy += dy * dy;
        sxy += dx * dy;
    }

    float trace = sxx + syy;
    float det = sxx * syy - sxy * sxy;
    float discriminant = std::sqrt(std::max(0.0f, trace * trace / 4.0f - det));
    
    float nx, ny;
    if (std::abs(sxy) > 1e-6f) {
        float eigen_val = trace / 2.0f - discriminant;
        nx = sxy;
        ny = eigen_val - sxx;
    } else {
        if (sxx < syy) {
            nx = 1.0f;
            ny = 0.0f;
        } else {
            nx = 0.0f;
            ny = 1.0f;
        }
    }

    float len = std::sqrt(nx * nx + ny * ny);
    if (len > 1e-6f) {
        nx /= len;
        ny /= len;
    }

    line.a = nx;
    line.b = ny;
    line.c = -(nx * mean_x + ny * mean_y);
}

float DockingAlgorithm::pointToLineDistance(float x, float y, const Line2D& line)
{
    return line.a * x + line.b * y + line.c;
}

void DockingAlgorithm::computeLineEndpoints(const std::vector<std::pair<float, float>>& points,
                                             const std::vector<size_t>& inliers,
                                             Line2D& line)
{
    if (inliers.empty()) return;

    // 投影所有内点到直线上，找最小和最�?t
    // 直线参数�? P = P0 + t * d
    // 其中 d = (-line.b, line.a) 是直线方�?
    
    float dx = -line.b;
    float dy = line.a;
    
    // 找直线上的一�?P0
    float p0_x, p0_y;
    if (std::abs(line.a) > std::abs(line.b)) {
        p0_x = -line.c / line.a;
        p0_y = 0.0f;
    } else {
        p0_x = 0.0f;
        p0_y = -line.c / line.b;
    }
    
    float t_min = std::numeric_limits<float>::max();
    float t_max = std::numeric_limits<float>::lowest();
    
    for (size_t idx : inliers) {
        float px = points[idx].first;
        float py = points[idx].second;
        
        // 计算投影 t
        float t = (px - p0_x) * dx + (py - p0_y) * dy;
        t_min = std::min(t_min, t);
        t_max = std::max(t_max, t);
    }
    
    // 计算端点
    line.x1 = p0_x + t_min * dx;
    line.y1 = p0_y + t_min * dy;
    line.x2 = p0_x + t_max * dx;
    line.y2 = p0_y + t_max * dy;
}

NearestRegionConfig DockingAlgorithm::getNearestConfig() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.nearest;
}

DockEdgeConfig DockingAlgorithm::getEdgeConfig() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return config_.edge;
}

} // namespace Linger
