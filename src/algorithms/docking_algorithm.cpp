#include "algorithms/docking_algorithm.h"
#include "core/logger.h"

#include <cmath>
#include <random>
#include <algorithm>
#include <numeric>
#include <map>
#include <tuple>

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

float DockingAlgorithm::getStabilityScore() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return filter_.stability_score;
}

int DockingAlgorithm::getConsecutiveValidFrames() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return filter_.consecutive_valid_frames;
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
    filter_.reset();
    previous_cloud_.reset();
    previous_timestamp_ns_ = 0;
    last_valid_edge_line_ = Line2D{};
    has_valid_edge_history_ = false;
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

    // 直接使用原始点云（预处理功能已重构为独立 Filter: DensityFilter, MotionFilter）
    // 如需预处理，请在 Processor 管线中配置对应 Filter
    std::vector<PointCloudPtr> working_clouds = clouds;

    // 1. 最近区域距离检测
    if (cfg.nearest.enabled) {
        state.nearest = detectNearestRegion(working_clouds);
    }

    // 2. 码头边缘检测
    if (cfg.edge.enabled) {
        state.edge = detectDockEdge(working_clouds);
    }

    // 3. 交叉验证与融合
    crossValidate(state);

    // 4. 时序滤波
    if (cfg.temporal.enabled) {
        updateTemporalFilter(state);
    }

    // 5. 更新状态
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
// 最近区域距离检测
//=============================================================================
NearestRegionResult DockingAlgorithm::detectNearestRegion(const std::vector<PointCloudPtr>& clouds)
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

    // 1. 扇区过滤 + 计算距离（直接遍历多个点云块）
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

    return computeNearestStatistics(distances, positions, cfg);
}

NearestRegionResult DockingAlgorithm::computeNearestStatistics(
    const std::vector<float>& distances,
    const std::vector<std::pair<float, float>>& positions,
    const NearestRegionConfig& cfg) {
    
    NearestRegionResult result;
    result.valid = false;
    result.point_count = distances.size();

    // 2. 检查点数
    if (distances.size() < cfg.min_points) {
        LOG_DEBUG("[Docking/Nearest] Insufficient points: {} < {}", distances.size(), cfg.min_points);
        return result;
    }

    // 3. 计算最近 percentile 的点
    std::vector<size_t> indices(distances.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    // 按距离排序索引
    std::sort(indices.begin(), indices.end(), [&distances](size_t a, size_t b) {
        return distances[a] < distances[b];
    });

    // 取最近的 N% 点
    size_t use_count = static_cast<size_t>(distances.size() * cfg.percentile / 100.0f);
    use_count = std::max(use_count, static_cast<size_t>(1));
    use_count = std::min(use_count, distances.size());
    result.used_points = use_count;

    // 4. 计算平均距离和位置
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

    // 5. 连续性检查
    if (cfg.check_continuity && use_count > 1) {
        // 计算所选点的标准差
        float variance = 0.0f;
        for (size_t i = 0; i < use_count; ++i) {
            size_t idx = indices[i];
            float diff = distances[idx] - result.distance_m;
            variance += diff * diff;
        }
        float std_dev = std::sqrt(variance / use_count);
        
        // 连续性评分：标准差越小越好
        result.continuity_score = std::max(0.0f, 1.0f - std_dev / cfg.continuity_threshold);
        
        if (std_dev > cfg.continuity_threshold) {
            LOG_DEBUG("[Docking/Nearest] Poor continuity: std_dev={:.2f} > {}", std_dev, cfg.continuity_threshold);
        }
    } else {
        result.continuity_score = 1.0f;
    }

    // 6. 应用平滑
    if (cfg.enable_smoothing) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (has_previous_nearest_) {
            result.distance_m = cfg.smoothing_alpha * result.distance_m + 
                               (1.0f - cfg.smoothing_alpha) * smoothed_nearest_dist_;
        }
        smoothed_nearest_dist_ = result.distance_m;
        has_previous_nearest_ = true;
    }

    // 7. 计算置信度
    // 基于点数和分布
    float point_ratio = static_cast<float>(distances.size()) / static_cast<float>(cfg.min_points);
    point_ratio = std::min(point_ratio, 5.0f);  // cap at 5x
    
    float base_conf = std::min(100.0f, point_ratio * 20.0f);
    // 结合连续性评分
    result.confidence = static_cast<uint8_t>(base_conf * (0.5f + 0.5f * result.continuity_score));

    result.valid = true;
    
    LOG_DEBUG("[Docking/Nearest] Dist: {:.2f}m, Cont: {:.2f}, Conf: {}%",
              result.distance_m, result.continuity_score, result.confidence);

    return result;
}

//=============================================================================
// 码头边缘检测
//=============================================================================
DockEdgeResult DockingAlgorithm::detectDockEdge(const std::vector<PointCloudPtr>& clouds)
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
    
    // Layer 2: 多假设 RANSAC - 获取多条候选直线
    std::vector<LineHypothesis> hypotheses = fitLineRansacMulti(points_2d, cfg, 3);
    
    if (hypotheses.empty()) {
        LOG_DEBUG("[Docking/Edge] RANSAC fitting failed - no valid hypotheses");
        return result;
    }
    
    // 使用最佳候选（已按综合评分排序）
    const auto& best = hypotheses[0];
    line = best.line;
    inliers = best.inliers;
    
    LOG_DEBUG("[Docking/Edge] Selected best hypothesis: inlier_ratio={:.2f}, geom={:.2f}, hist={:.2f}, total={:.2f}",
              best.inlier_ratio, best.geometry_score, best.history_score, best.total_score);
    
    // 存储所有候选供 Layer 3 使用（通过 result）
    result.alternative_lines.clear();
    for (size_t i = 1; i < hypotheses.size(); ++i) {
        result.alternative_lines.push_back(hypotheses[i].line);
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

    // 几何约束检查
    float edge_len = std::sqrt(std::pow(line.x1 - line.x2, 2) + std::pow(line.y1 - line.y2, 2));
    float angle_diff = std::abs(result.angle_deg - cfg.expected_angle_deg);
    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    angle_diff = std::abs(angle_diff);

    bool length_ok = edge_len >= cfg.min_edge_length;
    bool angle_ok = angle_diff <= cfg.angle_tolerance_deg;

    result.geometry_score = 1.0f;
    if (!length_ok) result.geometry_score *= 0.5f;
    if (!angle_ok) result.geometry_score *= 0.5f;

    if (result.geometry_score < 0.5f) {
        LOG_DEBUG("[Docking/Edge] Poor geometry: len={:.1f}m, angle={:.1f}deg", edge_len, result.angle_deg);
    }

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
    
    // 综合评分：内点比例(40%) + 误差(40%) + 几何(20%)
    float base_conf = (inlier_ratio * 40.0f + error_score * 40.0f + result.geometry_score * 20.0f);
    result.confidence = static_cast<uint8_t>(std::min(100.0f, base_conf));

    result.valid = true;
    
    LOG_DEBUG("[Docking/Edge] Dist: {:.2f}m, Ang: {:.1f}, Geo: {:.2f}, Conf: {}%",
              result.distance_m, result.angle_deg, result.geometry_score, result.confidence);

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

    // 检查内点比例
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

    // PCA 最小二乘精化
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

    // 投影所有内点到直线上，找最小和最大t
    // 直线参数： P = P0 + t * d
    // 其中 d = (-line.b, line.a) 是直线方向
    
    float dx = -line.b;
    float dy = line.a;
    
    // 找直线上的一点P0
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

void DockingAlgorithm::crossValidate(DockingState& state)
{
    // 如果两个检测都有效，进行交叉验证
    if (state.nearest.valid && state.edge.valid) {
        float dist_diff = std::abs(state.nearest.distance_m - state.edge.distance_m);
        
        // 差异过大，可能检测到了不同的物体
        if (dist_diff > 1.0f) {
            LOG_WARN("[Docking] Inconsistent detection: Nearest={:.2f}m, Edge={:.2f}m, Diff={:.2f}m",
                     state.nearest.distance_m, state.edge.distance_m, dist_diff);
            
            // Layer 3 增强：尝试使用备选边缘候选
            bool found_better_match = false;
            if (!state.edge.alternative_lines.empty()) {
                LOG_DEBUG("[Docking] Trying {} alternative edge lines", state.edge.alternative_lines.size());
                
                for (const auto& alt_line : state.edge.alternative_lines) {
                    float alt_dist = std::abs(alt_line.c);
                    float alt_diff = std::abs(state.nearest.distance_m - alt_dist);
                    
                    if (alt_diff < 0.5f) { // 备选直线与 Nearest 一致性更好
                        LOG_INFO("[Docking] Found better alternative: dist={:.2f}m, diff={:.2f}m", 
                                 alt_dist, alt_diff);
                        
                        // 使用备选直线
                        state.edge.edge_line = alt_line;
                        state.edge.distance_m = alt_dist;
                        state.edge.angle_deg = std::atan2(alt_line.a, -alt_line.b) * RAD_TO_DEG;
                        
                        found_better_match = true;
                        dist_diff = alt_diff;
                        break;
                    }
                }
            }
            
            if (!found_better_match) {
                // 没有找到更好的备选，按原逻辑处理
                // 信任置信度更高的那个
                if (state.edge.confidence > state.nearest.confidence + 20) {
                    state.status = DockingStatus::NORMAL;
                    state.final_distance_m = state.edge.distance_m;
                    state.final_angle_deg = state.edge.angle_deg;
                } else if (state.nearest.confidence > state.edge.confidence + 20) {
                    state.status = DockingStatus::NORMAL;
                    state.final_distance_m = state.nearest.distance_m;
                    state.final_angle_deg = 0.0f;
                } else {
                    // 都不确定，报低置信度
                    state.status = DockingStatus::LOW_CONFIDENCE;
                    // 保守策略：取较近的距离（避免撞击）
                    state.final_distance_m = std::min(state.nearest.distance_m, state.edge.distance_m);
                    state.final_angle_deg = state.edge.angle_deg;
                }
                return;
            }
        }
        
        // 一致性良好（原始或找到备选后）
        state.status = DockingStatus::NORMAL;
        
        // 边缘检测通常更精确（基于拟合），给予更高权重
        float w_edge = 0.7f;
        float w_nearest = 0.3f;
        
        // 根据置信度动态调整权重
        float total_conf = state.edge.confidence + state.nearest.confidence;
        if (total_conf > 0) {
            w_edge = static_cast<float>(state.edge.confidence) / total_conf;
            w_nearest = 1.0f - w_edge;
        }
        
        state.final_distance_m = w_edge * state.edge.distance_m + w_nearest * state.nearest.distance_m;
        state.final_angle_deg = state.edge.angle_deg;
    } 
    // 只有边缘检测有效
    else if (state.edge.valid) {
        state.status = (state.edge.confidence >= 50) ? DockingStatus::NORMAL : DockingStatus::LOW_CONFIDENCE;
        state.final_distance_m = state.edge.distance_m;
        state.final_angle_deg = state.edge.angle_deg;
    }
    // 只有最近区域有效
    else if (state.nearest.valid) {
        state.status = (state.nearest.confidence >= 50) ? DockingStatus::NORMAL : DockingStatus::LOW_CONFIDENCE;
        state.final_distance_m = state.nearest.distance_m;
        state.final_angle_deg = 0.0f;
    }
    // 都无效
    else {
        state.status = DockingStatus::NOT_DETECTED;
    }
}

void DockingAlgorithm::updateTemporalFilter(DockingState& state)
{
    if (!state.is_valid()) {
        // 更新稳定性评分：连续无效
        filter_.consecutive_invalid_frames++;
        filter_.consecutive_valid_frames = 0;
        filter_.consecutive_jump_frames = 0;
        
        // 稳定性评分衰减
        filter_.stability_score *= 0.9f;
        
        // 连续多次无效后重置滤波器
        if (filter_.consecutive_invalid_frames > 20) {
            LOG_WARN("[Docking/Temporal] Consecutive invalid frames: {}, resetting filter", 
                     filter_.consecutive_invalid_frames);
            filter_.reset();
        }
        return;
    }

    DockingConfig cfg;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cfg = config_;
    }
    
    float dt = 0.1f; // 默认 10Hz
    if (filter_.initialized && state.timestamp_ns > filter_.last_update_ns) {
        dt = (state.timestamp_ns - filter_.last_update_ns) * 1e-9f;
        dt = std::min(dt, 1.0f); // 防止时间跳变过大
    }
    
    // ========== 初始化 ==========
    if (!filter_.initialized) {
        filter_.predicted_dist = state.final_distance_m;
        filter_.predicted_velocity = 0.0f;
        filter_.dist_error_cov = 1.0f;
        
        filter_.predicted_angle = state.final_angle_deg;
        filter_.predicted_angle_rate = 0.0f;
        filter_.angle_error_cov = 10.0f; // 角度初始不确定性较大
        
        filter_.consecutive_valid_frames = 1;
        filter_.consecutive_invalid_frames = 0;
        filter_.consecutive_jump_frames = 0;
        filter_.stability_score = 0.0f;
        
        filter_.initialized = true;
        filter_.last_update_ns = state.timestamp_ns;
        
        LOG_DEBUG("[Docking/Temporal] Initialized: dist={:.2f}m, angle={:.1f}°", 
                  state.final_distance_m, state.final_angle_deg);
        return;
    }
    
    // ========== 预测步骤 ==========
    // 距离预测
    filter_.predicted_dist += filter_.predicted_velocity * dt;
    filter_.dist_error_cov += cfg.temporal.process_noise * dt;
    
    // 角度预测
    filter_.predicted_angle += filter_.predicted_angle_rate * dt;
    filter_.angle_error_cov += (cfg.temporal.process_noise * 5.0f) * dt; // 角度噪声稍大
    
    // ========== 异常值检测 (Gating) ==========
    float dist_innovation = state.final_distance_m - filter_.predicted_dist;
    float angle_innovation = state.final_angle_deg - filter_.predicted_angle;
    
    // 处理角度循环（-180° 到 180°）
    while (angle_innovation > 180.0f) angle_innovation -= 360.0f;
    while (angle_innovation < -180.0f) angle_innovation += 360.0f;
    
    bool dist_jump = std::abs(dist_innovation) > cfg.temporal.max_jump_m;
    bool angle_jump = std::abs(angle_innovation) > 30.0f; // 角度跳变阈值 30°
    
    if (dist_jump || angle_jump) {
        filter_.consecutive_jump_frames++;
        
        LOG_WARN("[Docking/Temporal] Jump detected: dist_innov={:.2f}m (jump={}), angle_innov={:.1f}° (jump={})",
                 dist_innovation, dist_jump, angle_innovation, angle_jump);
        
        // 如果连续多次跳变，可能是真实运动，重新初始化
        if (filter_.consecutive_jump_frames > 3) {
            LOG_WARN("[Docking/Temporal] Consecutive jumps: {}, reinitializing", 
                     filter_.consecutive_jump_frames);
            filter_.initialized = false;
            updateTemporalFilter(state); // 递归调用进行重新初始化
            return;
        }
        
        // 否则忽略测量，保持预测值
        state.status = DockingStatus::LOW_CONFIDENCE;
        state.final_distance_m = filter_.predicted_dist;
        state.final_angle_deg = filter_.predicted_angle;
        
        // 稳定性评分降低
        filter_.stability_score *= 0.8f;
        filter_.consecutive_valid_frames = 0;
        
        filter_.last_update_ns = state.timestamp_ns;
        return;
    }
    
    // 没有跳变，重置跳变计数
    filter_.consecutive_jump_frames = 0;
    
    // ========== 更新步骤 (Kalman Gain) ==========
    
    // 根据置信度调整测量噪声（高置信度 -> 低噪声 -> 更信任测量）
    float confidence_factor = 1.0f;
    if (state.edge.valid && state.nearest.valid) {
        // 两个检测都有效时，取平均置信度
        confidence_factor = (state.edge.confidence + state.nearest.confidence) / 200.0f;
    } else if (state.edge.valid) {
        confidence_factor = state.edge.confidence / 100.0f;
    } else if (state.nearest.valid) {
        confidence_factor = state.nearest.confidence / 100.0f;
    }
    confidence_factor = std::max(0.1f, std::min(confidence_factor, 1.0f));
    
    // 距离更新
    float dist_meas_noise = cfg.temporal.measurement_noise / confidence_factor;
    float dist_S = filter_.dist_error_cov + dist_meas_noise;
    float dist_K = filter_.dist_error_cov / dist_S;
    
    filter_.predicted_dist += dist_K * dist_innovation;
    filter_.predicted_velocity += dist_K * (dist_innovation / dt);
    filter_.dist_error_cov = (1.0f - dist_K) * filter_.dist_error_cov;
    
    // 角度更新（仅当边缘检测有效时）
    if (state.edge.valid) {
        float angle_meas_noise = 5.0f / confidence_factor; // 角度测量噪声 ~5°
        float angle_S = filter_.angle_error_cov + angle_meas_noise;
        float angle_K = filter_.angle_error_cov / angle_S;
        
        filter_.predicted_angle += angle_K * angle_innovation;
        filter_.predicted_angle_rate += angle_K * (angle_innovation / dt);
        filter_.angle_error_cov = (1.0f - angle_K) * filter_.angle_error_cov;
        
        // 角度归一化到 [-180, 180]
        while (filter_.predicted_angle > 180.0f) filter_.predicted_angle -= 360.0f;
        while (filter_.predicted_angle < -180.0f) filter_.predicted_angle += 360.0f;
    }
    
    filter_.last_update_ns = state.timestamp_ns;
    
    // ========== 输出滤波后的值 ==========
    state.final_distance_m = filter_.predicted_dist;
    if (state.edge.valid) {
        state.final_angle_deg = filter_.predicted_angle;
    }
    
    // ========== 速度/角速度限制 ==========
    if (std::abs(filter_.predicted_velocity) > cfg.temporal.velocity_limit) {
        filter_.predicted_velocity = (filter_.predicted_velocity > 0 ? 1.0f : -1.0f) * cfg.temporal.velocity_limit;
    }
    
    if (std::abs(filter_.predicted_angle_rate) > 20.0f) { // 角速度限制 ±20°/s
        filter_.predicted_angle_rate = (filter_.predicted_angle_rate > 0 ? 1.0f : -1.0f) * 20.0f;
    }
    
    // ========== 稳定性评分更新 ==========
    filter_.consecutive_valid_frames++;
    filter_.consecutive_invalid_frames = 0;
    
    // 指数平滑稳定性评分
    float target_stability = 0.0f;
    if (filter_.consecutive_valid_frames > 20) {
        target_stability = 1.0f; // 连续20帧以上，完全稳定
    } else if (filter_.consecutive_valid_frames > 10) {
        target_stability = 0.8f; // 10-20帧，高稳定
    } else if (filter_.consecutive_valid_frames > 5) {
        target_stability = 0.5f; // 5-10帧，中等稳定
    } else {
        target_stability = 0.2f; // 少于5帧，低稳定
    }
    
    filter_.stability_score = 0.9f * filter_.stability_score + 0.1f * target_stability;
    
    LOG_DEBUG("[Docking/Temporal] Filtered: dist={:.2f}m (innov={:.3f}), angle={:.1f}° (innov={:.2f}), "
              "stability={:.2f}, valid_frames={}", 
              filter_.predicted_dist, dist_innovation, filter_.predicted_angle, angle_innovation,
              filter_.stability_score, filter_.consecutive_valid_frames);
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

//=============================================================================
// Layer 1: 预处理（动静分离、密度过滤、平面检测）
// 注意：预处理功能已重构为独立的 Filter 类：
//   - DensityFilter (preprocessing/density_filter.h): 密度过滤
//   - MotionFilter (preprocessing/motion_filter.h): 动静分离
// 以下实现保留用于兼容性，建议在 Processor 管线中使用上述 Filter
//=============================================================================

DockingAlgorithm::PreprocessResult DockingAlgorithm::preprocessClouds(const std::vector<PointCloudPtr>& clouds)
{
    PreprocessResult result;
    result.static_cloud = std::make_shared<PointCloud>();
    result.dynamic_cloud = std::make_shared<PointCloud>();
    
    // 1. 合并输入点云（预处理阶段需要完整视图）
    PointCloudPtr merged = std::make_shared<PointCloud>();
    for (const auto& cloud : clouds) {
        if (cloud && !cloud->points.empty()) {
            *merged += *cloud;
        }
    }
    
    if (merged->points.empty()) {
        return result;
    }
    
    // 2. 密度过滤（去除稀疏噪声）
    PointCloudPtr dense_cloud = densityFilter(merged, 0.3f, 3);
    result.low_density_filtered = merged->points.size() - dense_cloud->points.size();
    
    // 3. 动静分离
    separateStaticDynamic(dense_cloud, result);
    
    // 4. 检测码头平面
    if (result.static_cloud && !result.static_cloud->points.empty()) {
        detectDockSurface(result.static_cloud, result);
    }
    
    return result;
}

PointCloudPtr DockingAlgorithm::densityFilter(const PointCloudPtr& cloud, float voxel_size, int min_points_per_voxel)
{
    if (!cloud || cloud->points.empty()) {
        return std::make_shared<PointCloud>();
    }
    
    // 使用简单的体素网格统计
    struct VoxelKey {
        int x, y, z;
        
        bool operator<(const VoxelKey& other) const {
            if (x != other.x) return x < other.x;
            if (y != other.y) return y < other.y;
            return z < other.z;
        }
    };
    
    std::map<VoxelKey, std::vector<size_t>> voxel_map;
    
    // 分配点到体素
    for (size_t i = 0; i < cloud->points.size(); ++i) {
        const auto& pt = cloud->points[i];
        VoxelKey key{
            static_cast<int>(std::floor(pt.x / voxel_size)),
            static_cast<int>(std::floor(pt.y / voxel_size)),
            static_cast<int>(std::floor(pt.z / voxel_size))
        };
        voxel_map[key].push_back(i);
    }
    
    // 保留密度足够的体素中的点
    PointCloudPtr filtered = std::make_shared<PointCloud>();
    filtered->points.reserve(cloud->points.size());
    
    for (const auto& [key, indices] : voxel_map) {
        if (static_cast<int>(indices.size()) >= min_points_per_voxel) {
            for (size_t idx : indices) {
                filtered->points.push_back(cloud->points[idx]);
            }
        }
    }
    
    filtered->width = filtered->points.size();
    filtered->height = 1;
    filtered->is_dense = false;
    
    return filtered;
}

void DockingAlgorithm::separateStaticDynamic(const PointCloudPtr& cloud, PreprocessResult& result)
{
    if (!cloud || cloud->points.empty()) {
        return;
    }
    
    // 简化版：基于速度阈值和历史帧差分
    // 如果没有历史帧，全部视为静态
    if (!previous_cloud_ || previous_cloud_->points.empty()) {
        result.static_cloud = cloud;
        result.static_point_count = cloud->points.size();
        result.dynamic_point_count = 0;
        
        // 保存当前帧供下次使用
        previous_cloud_ = cloud;
        return;
    }
    
    // 使用简单的空间网格匹配检测运动
    const float cell_size = 0.5f;  // 0.5米网格
    const float motion_threshold = 0.3f;  // 运动阈值（米）
    
    // 构建历史帧的空间索引
    std::map<std::tuple<int, int, int>, std::vector<Point>> history_grid;
    for (const auto& pt : previous_cloud_->points) {
        int gx = static_cast<int>(std::floor(pt.x / cell_size));
        int gy = static_cast<int>(std::floor(pt.y / cell_size));
        int gz = static_cast<int>(std::floor(pt.z / cell_size));
        history_grid[{gx, gy, gz}].push_back(pt);
    }
    
    result.static_cloud->points.reserve(cloud->points.size());
    result.dynamic_cloud->points.reserve(cloud->points.size() / 10);  // 预计动态点较少
    
    // 对当前帧的每个点，检查是否与历史帧匹配
    for (const auto& pt : cloud->points) {
        int gx = static_cast<int>(std::floor(pt.x / cell_size));
        int gy = static_cast<int>(std::floor(pt.y / cell_size));
        int gz = static_cast<int>(std::floor(pt.z / cell_size));
        
        bool is_static = false;
        
        // 搜索邻近网格
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    auto it = history_grid.find({gx + dx, gy + dy, gz + dz});
                    if (it != history_grid.end()) {
                        // 检查是否有近似匹配的点
                        for (const auto& hist_pt : it->second) {
                            float dist = std::sqrt(
                                std::pow(pt.x - hist_pt.x, 2) +
                                std::pow(pt.y - hist_pt.y, 2) +
                                std::pow(pt.z - hist_pt.z, 2)
                            );
                            if (dist < motion_threshold) {
                                is_static = true;
                                break;
                            }
                        }
                    }
                    if (is_static) break;
                }
                if (is_static) break;
            }
            if (is_static) break;
        }
        
        if (is_static) {
            result.static_cloud->points.push_back(pt);
        } else {
            result.dynamic_cloud->points.push_back(pt);
        }
    }
    
    result.static_cloud->width = result.static_cloud->points.size();
    result.static_cloud->height = 1;
    result.static_cloud->is_dense = false;
    
    result.dynamic_cloud->width = result.dynamic_cloud->points.size();
    result.dynamic_cloud->height = 1;
    result.dynamic_cloud->is_dense = false;
    
    result.static_point_count = result.static_cloud->points.size();
    result.dynamic_point_count = result.dynamic_cloud->points.size();
    
    // 保存当前帧
    previous_cloud_ = cloud;
}

void DockingAlgorithm::detectDockSurface(const PointCloudPtr& cloud, PreprocessResult& result)
{
    if (!cloud || cloud->points.empty() || cloud->points.size() < 100) {
        return;
    }
    
    // 简化版平面检测：寻找水平密集区域
    // 统计 Z 轴直方图，找最密集的高度层
    const float z_bin_size = 0.2f;  // 20cm 高度分层
    const float horizontal_tolerance = 0.5f;  // 水平容差
    
    std::map<int, std::vector<Point>> z_layers;
    
    for (const auto& pt : cloud->points) {
        // 只考虑前方和侧面的点（X > 0）
        if (pt.x < 1.0f) continue;
        
        int z_bin = static_cast<int>(std::floor(pt.z / z_bin_size));
        z_layers[z_bin].push_back(pt);
    }
    
    // 找点数最多的层
    int best_layer = 0;
    size_t max_points = 0;
    for (const auto& [layer, points] : z_layers) {
        if (points.size() > max_points) {
            max_points = points.size();
            best_layer = layer;
        }
    }
    
    // 至少需要 100 个点才认为是有效平面
    if (max_points < 100) {
        return;
    }
    
    // 计算该层的平均高度和距离
    const auto& layer_points = z_layers[best_layer];
    float sum_z = 0.0f;
    float sum_dist = 0.0f;
    
    for (const auto& pt : layer_points) {
        sum_z += pt.z;
        sum_dist += std::sqrt(pt.x * pt.x + pt.y * pt.y);
    }
    
    float avg_z = sum_z / layer_points.size();
    float avg_dist = sum_dist / layer_points.size();
    
    // 检查该层是否足够水平（标准差小）
    float variance_z = 0.0f;
    for (const auto& pt : layer_points) {
        float diff = pt.z - avg_z;
        variance_z += diff * diff;
    }
    float std_dev_z = std::sqrt(variance_z / layer_points.size());
    
    // 如果高度标准差小于阈值，认为是水平面
    if (std_dev_z < horizontal_tolerance) {
        result.dock_surface_detected = true;
        result.dock_plane_height = avg_z;
        result.dock_plane_distance = avg_dist;
        
        LOG_DEBUG("[Docking/Preprocess] Dock surface detected: height={:.2f}m, dist={:.2f}m, points={}, std_dev={:.3f}",
                  avg_z, avg_dist, max_points, std_dev_z);
    }
}

//=============================================================================
// Layer 2: 多假设边缘检测
//=============================================================================

std::vector<DockingAlgorithm::LineHypothesis> DockingAlgorithm::fitLineRansacMulti(
    const std::vector<std::pair<float, float>>& points,
    const DockEdgeConfig& cfg,
    int max_hypotheses)
{
    if (points.size() < 2) {
        return {};
    }

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<size_t> dist(0, points.size() - 1);

    std::vector<LineHypothesis> all_candidates;
    all_candidates.reserve(cfg.ransac_max_iterations);

    // RANSAC 迭代：收集所有可能的候选
    for (int iter = 0; iter < cfg.ransac_max_iterations; ++iter) {
        // 随机选择两个不同的点
        size_t idx1 = dist(gen);
        size_t idx2 = dist(gen);
        int attempts = 0;
        while (idx2 == idx1 && attempts++ < 10) {
            idx2 = dist(gen);
        }
        if (idx2 == idx1) continue;

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
        std::vector<size_t> inliers;
        float total_error = 0.0f;
        for (size_t i = 0; i < points.size(); ++i) {
            float d = std::abs(pointToLineDistance(points[i].first, points[i].second, candidate));
            if (d < cfg.ransac_distance_threshold) {
                inliers.push_back(i);
                total_error += d;
            }
        }

        if (inliers.size() < 10) continue;  // 至少需要10个内点

        // 创建候选
        LineHypothesis hyp;
        hyp.line = candidate;
        hyp.inliers = std::move(inliers);
        hyp.inlier_ratio = static_cast<float>(hyp.inliers.size()) / points.size();
        hyp.mean_error = total_error / hyp.inliers.size();

        // 检查是否与已有候选足够不同
        bool is_distinct = true;
        for (const auto& existing : all_candidates) {
            if (!areLinesDistinct(hyp.line, existing.line)) {
                // 如果相似，只保留内点更多的
                if (hyp.inliers.size() <= existing.inliers.size()) {
                    is_distinct = false;
                    break;
                }
            }
        }

        if (is_distinct) {
            all_candidates.push_back(std::move(hyp));
        }

        // 提前终止：如果找到非常好的解
        if (!all_candidates.empty() && all_candidates.back().inlier_ratio > 0.8f) {
            break;
        }
    }

    if (all_candidates.empty()) {
        return {};
    }

    // 按内点数量排序，保留 Top-N
    std::sort(all_candidates.begin(), all_candidates.end(),
              [](const LineHypothesis& a, const LineHypothesis& b) {
                  return a.inliers.size() > b.inliers.size();
              });

    size_t keep_count = std::min(static_cast<size_t>(max_hypotheses), all_candidates.size());
    std::vector<LineHypothesis> hypotheses(all_candidates.begin(), all_candidates.begin() + keep_count);

    // 精化每条候选直线
    for (auto& hyp : hypotheses) {
        refineLine(points, hyp.inliers, hyp.line);
        
        // 重新计算误差（精化后可能改变）
        float total_error = 0.0f;
        for (size_t idx : hyp.inliers) {
            total_error += std::abs(pointToLineDistance(points[idx].first, points[idx].second, hyp.line));
        }
        hyp.mean_error = total_error / hyp.inliers.size();
        
        // 计算端点
        computeLineEndpoints(points, hyp.inliers, hyp.line);
        
        // 评估几何约束
        evaluateGeometry(hyp, cfg);
        
        // 评估历史一致性
        evaluateHistoryConsistency(hyp);
        
        // 综合评分：内点比例(40%) + 几何(30%) + 历史一致性(30%)
        float error_score = std::max(0.0f, 1.0f - hyp.mean_error / cfg.ransac_distance_threshold);
        hyp.total_score = hyp.inlier_ratio * 40.0f + 
                         hyp.geometry_score * 30.0f + 
                         hyp.history_score * 30.0f;
    }

    // 按综合评分重新排序
    std::sort(hypotheses.begin(), hypotheses.end(),
              [](const LineHypothesis& a, const LineHypothesis& b) {
                  return a.total_score > b.total_score;
              });

    // 检查最佳候选是否达到最低要求
    if (hypotheses[0].inlier_ratio < cfg.ransac_min_inlier_ratio) {
        LOG_DEBUG("[Docking/Edge] Best hypothesis inlier_ratio {:.2f} < min {:.2f}",
                  hypotheses[0].inlier_ratio, cfg.ransac_min_inlier_ratio);
        return {};
    }

    // 更新历史最佳边缘
    if (hypotheses[0].total_score > 50.0f) {  // 综合评分阈值
        last_valid_edge_line_ = hypotheses[0].line;
        has_valid_edge_history_ = true;
    }

    return hypotheses;
}

void DockingAlgorithm::evaluateGeometry(LineHypothesis& hypothesis, const DockEdgeConfig& cfg)
{
    const auto& line = hypothesis.line;
    
    // 计算边缘长度
    float edge_len = std::sqrt(
        std::pow(line.x1 - line.x2, 2) + 
        std::pow(line.y1 - line.y2, 2)
    );
    
    // 计算角度
    float angle_deg = std::atan2(line.a, -line.b) * RAD_TO_DEG;
    float angle_diff = std::abs(angle_deg - cfg.expected_angle_deg);
    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    angle_diff = std::abs(angle_diff);
    
    // 长度评分：0-1
    float length_score = std::min(1.0f, edge_len / cfg.min_edge_length);
    
    // 角度评分：0-1
    float angle_score = std::max(0.0f, 1.0f - angle_diff / cfg.angle_tolerance_deg);
    
    // 综合几何评分
    hypothesis.geometry_score = (length_score * 0.6f + angle_score * 0.4f) * 100.0f;
    
    LOG_DEBUG("[Docking/Edge] Geometry: len={:.2f}m (score={:.2f}), angle={:.1f}° (diff={:.1f}°, score={:.2f})",
              edge_len, length_score, angle_deg, angle_diff, angle_score);
}

void DockingAlgorithm::evaluateHistoryConsistency(LineHypothesis& hypothesis)
{
    if (!has_valid_edge_history_) {
        hypothesis.history_score = 50.0f;  // 无历史数据时给中性分
        return;
    }
    
    const auto& curr = hypothesis.line;
    const auto& hist = last_valid_edge_line_;
    
    //=========================================================================
    // 1. 距离一致性：当前直线到原点距离与历史的差异
    //=========================================================================
    float curr_dist = std::abs(curr.c);
    float hist_dist = std::abs(hist.c);
    float dist_diff = std::abs(curr_dist - hist_dist);
    
    // 考虑时序滤波的速度预测，允许更大的合理变化范围
    float expected_change = std::abs(filter_.predicted_velocity) * 0.1f; // 假设100ms帧间隔
    float adjusted_dist_diff = std::max(0.0f, dist_diff - expected_change);
    
    // 距离一致性评分：调整后差异小于0.5m为满分
    float dist_score = std::max(0.0f, 1.0f - adjusted_dist_diff / 0.5f);
    
    //=========================================================================
    // 2. 角度一致性：直线方向与历史的差异
    //=========================================================================
    float curr_angle = std::atan2(curr.a, -curr.b) * RAD_TO_DEG;
    float hist_angle = std::atan2(hist.a, -hist.b) * RAD_TO_DEG;
    float angle_diff = curr_angle - hist_angle;
    // 归一化到 [-180, 180]
    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    angle_diff = std::abs(angle_diff);
    
    // 角度一致性评分：差异小于10°为满分
    float angle_score = std::max(0.0f, 1.0f - angle_diff / 10.0f);
    
    //=========================================================================
    // 3. 端点位置一致性：直线端点与历史端点的重叠程度
    //=========================================================================
    float endpoint_score = 1.0f;
    
    // 计算端点中心的偏移
    float curr_cx = (curr.x1 + curr.x2) * 0.5f;
    float curr_cy = (curr.y1 + curr.y2) * 0.5f;
    float hist_cx = (hist.x1 + hist.x2) * 0.5f;
    float hist_cy = (hist.y1 + hist.y2) * 0.5f;
    float center_diff = std::sqrt((curr_cx - hist_cx) * (curr_cx - hist_cx) + 
                                   (curr_cy - hist_cy) * (curr_cy - hist_cy));
    
    // 端点中心偏移小于2m为满分
    endpoint_score = std::max(0.0f, 1.0f - center_diff / 2.0f);
    
    //=========================================================================
    // 4. 长度一致性：边缘长度与历史的差异
    //=========================================================================
    float curr_len = std::sqrt((curr.x2 - curr.x1) * (curr.x2 - curr.x1) + 
                                (curr.y2 - curr.y1) * (curr.y2 - curr.y1));
    float hist_len = std::sqrt((hist.x2 - hist.x1) * (hist.x2 - hist.x1) + 
                                (hist.y2 - hist.y1) * (hist.y2 - hist.y1));
    float len_diff = std::abs(curr_len - hist_len);
    
    // 长度差异小于1m为满分
    float length_score = std::max(0.0f, 1.0f - len_diff / 1.0f);
    
    //=========================================================================
    // 5. 时序置信度加权：根据连续有效帧数调整权重
    //=========================================================================
    float temporal_weight = std::min(1.0f, filter_.consecutive_valid_frames / 10.0f);
    
    //=========================================================================
    // 综合历史一致性评分（加权组合）
    //=========================================================================
    // 基础评分：距离(40%) + 角度(30%) + 端点(20%) + 长度(10%)
    float base_score = dist_score * 0.4f + angle_score * 0.3f + 
                       endpoint_score * 0.2f + length_score * 0.1f;
    
    // 时序加权：历史越长越可信
    // 当无历史时 temporal_weight=0，base_score 权重降低
    hypothesis.history_score = (base_score * (0.5f + 0.5f * temporal_weight)) * 100.0f;
    
    LOG_DEBUG("[Docking/Edge] History: dist_diff={:.3f}m(adj={:.3f},s={:.2f}), angle_diff={:.1f}°(s={:.2f}), "
              "endpoint={:.2f}m(s={:.2f}), len_diff={:.2f}m(s={:.2f}), temporal_w={:.2f}, final={:.1f}",
              dist_diff, adjusted_dist_diff, dist_score, 
              angle_diff, angle_score, 
              center_diff, endpoint_score,
              len_diff, length_score,
              temporal_weight, hypothesis.history_score);
}

bool DockingAlgorithm::areLinesDistinct(const Line2D& line1, const Line2D& line2) const
{
    // 距离差异
    float dist_diff = std::abs(std::abs(line1.c) - std::abs(line2.c));
    
    // 角度差异
    float angle1 = std::atan2(line1.a, -line1.b) * RAD_TO_DEG;
    float angle2 = std::atan2(line2.a, -line2.b) * RAD_TO_DEG;
    float angle_diff = std::abs(angle1 - angle2);
    while (angle_diff > 180.0f) angle_diff -= 360.0f;
    angle_diff = std::abs(angle_diff);
    
    // 如果距离差 > 0.5m 或 角度差 > 10°，认为是不同的直线
    return (dist_diff > 0.5f) || (angle_diff > 10.0f);
}

} // namespace Linger
