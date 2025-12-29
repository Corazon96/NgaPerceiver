#include "core/app_config.h"
#include "core/logger.h"

#include <fstream>
#include <sstream>
#include <filesystem>

// rapidjson 头文件位于 3rdParty/rapidjson 目录
#include "document.h"
#include "prettywriter.h"
#include "stringbuffer.h"
#include "filereadstream.h"
#include "filewritestream.h"

namespace Linger {

std::string GetDefaultAppConfigPath()
{
    return "config/app_config.json";
}

bool LoadAppConfig(const std::string& path, AppConfig& cfg)
{
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        LOG_WARN("[AppConfig] Cannot open config file: {}, using defaults.", path);
        return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    std::string content = buffer.str();
    ifs.close();

    rapidjson::Document doc;
    if (doc.Parse(content.c_str()).HasParseError()) {
        LOG_ERROR("[AppConfig] JSON parse error in {} at offset {}", path, doc.GetErrorOffset());
        return false;
    }

    // filters
    if (doc.HasMember("filters") && doc["filters"].IsObject()) {
        const auto& filters = doc["filters"];

        // distance
        if (filters.HasMember("distance") && filters["distance"].IsObject()) {
            const auto& d = filters["distance"];
            if (d.HasMember("min_m") && d["min_m"].IsNumber())
                cfg.distance_min = static_cast<float>(d["min_m"].GetDouble());
            if (d.HasMember("max_m") && d["max_m"].IsNumber())
                cfg.distance_max = static_cast<float>(d["max_m"].GetDouble());
            if (d.HasMember("enabled") && d["enabled"].IsBool())
                cfg.distance_enabled = d["enabled"].GetBool();
        }

        // roi
        if (filters.HasMember("roi") && filters["roi"].IsObject()) {
            const auto& r = filters["roi"];
            if (r.HasMember("x_min") && r["x_min"].IsNumber())
                cfg.roi_x_min = static_cast<float>(r["x_min"].GetDouble());
            if (r.HasMember("x_max") && r["x_max"].IsNumber())
                cfg.roi_x_max = static_cast<float>(r["x_max"].GetDouble());
            if (r.HasMember("y_min") && r["y_min"].IsNumber())
                cfg.roi_y_min = static_cast<float>(r["y_min"].GetDouble());
            if (r.HasMember("y_max") && r["y_max"].IsNumber())
                cfg.roi_y_max = static_cast<float>(r["y_max"].GetDouble());
            if (r.HasMember("z_min") && r["z_min"].IsNumber())
                cfg.roi_z_min = static_cast<float>(r["z_min"].GetDouble());
            if (r.HasMember("z_max") && r["z_max"].IsNumber())
                cfg.roi_z_max = static_cast<float>(r["z_max"].GetDouble());
            if (r.HasMember("enabled") && r["enabled"].IsBool())
                cfg.roi_enabled = r["enabled"].GetBool();
        }

        // sea_filter
        if (filters.HasMember("sea_filter") && filters["sea_filter"].IsObject()) {
            const auto& s = filters["sea_filter"];
            if (s.HasMember("sea_level_z") && s["sea_level_z"].IsNumber())
                cfg.sea_level_z = static_cast<float>(s["sea_level_z"].GetDouble());
            if (s.HasMember("margin_m") && s["margin_m"].IsNumber())
                cfg.sea_margin = static_cast<float>(s["margin_m"].GetDouble());
            if (s.HasMember("enabled") && s["enabled"].IsBool())
                cfg.sea_enabled = s["enabled"].GetBool();
        }

        // statistical_outlier
        if (filters.HasMember("statistical_outlier") && filters["statistical_outlier"].IsObject()) {
            const auto& o = filters["statistical_outlier"];
            if (o.HasMember("mean_k") && o["mean_k"].IsInt())
                cfg.outlier_mean_k = o["mean_k"].GetInt();
            if (o.HasMember("stddev_mul") && o["stddev_mul"].IsNumber())
                cfg.outlier_stddev_mul = static_cast<float>(o["stddev_mul"].GetDouble());
            if (o.HasMember("enabled") && o["enabled"].IsBool())
                cfg.outlier_enabled = o["enabled"].GetBool();
        }

        // voxel
        if (filters.HasMember("voxel") && filters["voxel"].IsObject()) {
            const auto& v = filters["voxel"];
            if (v.HasMember("leaf_size") && v["leaf_size"].IsNumber())
                cfg.voxel_leaf_size = static_cast<float>(v["leaf_size"].GetDouble());
            if (v.HasMember("enabled") && v["enabled"].IsBool())
                cfg.voxel_enabled = v["enabled"].GetBool();
        }

        // density
        if (filters.HasMember("density") && filters["density"].IsObject()) {
            const auto& d = filters["density"];
            if (d.HasMember("voxel_size") && d["voxel_size"].IsNumber())
                cfg.density_voxel_size = static_cast<float>(d["voxel_size"].GetDouble());
            if (d.HasMember("min_points") && d["min_points"].IsInt())
                cfg.density_min_points = d["min_points"].GetInt();
            if (d.HasMember("enabled") && d["enabled"].IsBool())
                cfg.density_enabled = d["enabled"].GetBool();
        }

        // motion
        if (filters.HasMember("motion") && filters["motion"].IsObject()) {
            const auto& m = filters["motion"];
            if (m.HasMember("cell_size") && m["cell_size"].IsNumber())
                cfg.motion_cell_size = static_cast<float>(m["cell_size"].GetDouble());
            if (m.HasMember("threshold") && m["threshold"].IsNumber())
                cfg.motion_threshold = static_cast<float>(m["threshold"].GetDouble());
            if (m.HasMember("output_static") && m["output_static"].IsBool())
                cfg.motion_output_static = m["output_static"].GetBool();
            if (m.HasMember("enabled") && m["enabled"].IsBool())
                cfg.motion_enabled = m["enabled"].GetBool();
        }
    }

    // docking
    if (doc.HasMember("docking") && doc["docking"].IsObject()) {
        const auto& docking = doc["docking"];

        // nearest_region
        if (docking.HasMember("nearest_region") && docking["nearest_region"].IsObject()) {
            const auto& nr = docking["nearest_region"];
            if (nr.HasMember("sector_x_min") && nr["sector_x_min"].IsNumber())
                cfg.nr_sector_x_min = static_cast<float>(nr["sector_x_min"].GetDouble());
            if (nr.HasMember("sector_x_max") && nr["sector_x_max"].IsNumber())
                cfg.nr_sector_x_max = static_cast<float>(nr["sector_x_max"].GetDouble());
            if (nr.HasMember("sector_y_min") && nr["sector_y_min"].IsNumber())
                cfg.nr_sector_y_min = static_cast<float>(nr["sector_y_min"].GetDouble());
            if (nr.HasMember("sector_y_max") && nr["sector_y_max"].IsNumber())
                cfg.nr_sector_y_max = static_cast<float>(nr["sector_y_max"].GetDouble());
            if (nr.HasMember("sector_z_min") && nr["sector_z_min"].IsNumber())
                cfg.nr_sector_z_min = static_cast<float>(nr["sector_z_min"].GetDouble());
            if (nr.HasMember("sector_z_max") && nr["sector_z_max"].IsNumber())
                cfg.nr_sector_z_max = static_cast<float>(nr["sector_z_max"].GetDouble());
            if (nr.HasMember("percentile") && nr["percentile"].IsNumber())
                cfg.nr_percentile = static_cast<float>(nr["percentile"].GetDouble());
            if (nr.HasMember("enabled") && nr["enabled"].IsBool())
                cfg.nr_enabled = nr["enabled"].GetBool();
        }

        // dock_edge
        if (docking.HasMember("dock_edge") && docking["dock_edge"].IsObject()) {
            const auto& edge = docking["dock_edge"];
            if (edge.HasMember("x_min") && edge["x_min"].IsNumber())
                cfg.edge_x_min = static_cast<float>(edge["x_min"].GetDouble());
            if (edge.HasMember("x_max") && edge["x_max"].IsNumber())
                cfg.edge_x_max = static_cast<float>(edge["x_max"].GetDouble());
            if (edge.HasMember("y_min") && edge["y_min"].IsNumber())
                cfg.edge_y_min = static_cast<float>(edge["y_min"].GetDouble());
            if (edge.HasMember("y_max") && edge["y_max"].IsNumber())
                cfg.edge_y_max = static_cast<float>(edge["y_max"].GetDouble());
            if (edge.HasMember("z_min") && edge["z_min"].IsNumber())
                cfg.edge_z_min = static_cast<float>(edge["z_min"].GetDouble());
            if (edge.HasMember("z_max") && edge["z_max"].IsNumber())
                cfg.edge_z_max = static_cast<float>(edge["z_max"].GetDouble());
            if (edge.HasMember("ransac_dist") && edge["ransac_dist"].IsNumber())
                cfg.edge_ransac_dist = static_cast<float>(edge["ransac_dist"].GetDouble());
            if (edge.HasMember("enabled") && edge["enabled"].IsBool())
                cfg.edge_enabled = edge["enabled"].GetBool();
        }

        // temporal
        if (docking.HasMember("temporal") && docking["temporal"].IsObject()) {
            const auto& t = docking["temporal"];
            if (t.HasMember("max_jump_m") && t["max_jump_m"].IsNumber())
                cfg.temporal_max_jump = static_cast<float>(t["max_jump_m"].GetDouble());
            if (t.HasMember("enabled") && t["enabled"].IsBool())
                cfg.temporal_enabled = t["enabled"].GetBool();
        }
    }

    // udp
    if (doc.HasMember("udp") && doc["udp"].IsObject()) {
        const auto& udp = doc["udp"];
        if (udp.HasMember("target_ip") && udp["target_ip"].IsString())
            cfg.udp_target_ip = udp["target_ip"].GetString();
        if (udp.HasMember("target_port") && udp["target_port"].IsUint())
            cfg.udp_target_port = static_cast<uint16_t>(udp["target_port"].GetUint());
        if (udp.HasMember("sensor_id") && udp["sensor_id"].IsUint())
            cfg.udp_sensor_id = static_cast<uint8_t>(udp["sensor_id"].GetUint());
        if (udp.HasMember("enabled") && udp["enabled"].IsBool())
            cfg.udp_enabled = udp["enabled"].GetBool();
    }

    LOG_INFO("[AppConfig] Loaded config from {}", path);
    return true;
}

bool SaveAppConfig(const std::string& path, const AppConfig& cfg)
{
    // 确保目录存在
    std::filesystem::path p(path);
    if (p.has_parent_path()) {
        std::error_code ec;
        std::filesystem::create_directories(p.parent_path(), ec);
    }

    rapidjson::Document doc;
    doc.SetObject();
    auto& alloc = doc.GetAllocator();

    // filters
    {
        rapidjson::Value filters(rapidjson::kObjectType);

        // distance
        {
            rapidjson::Value d(rapidjson::kObjectType);
            d.AddMember("min_m", cfg.distance_min, alloc);
            d.AddMember("max_m", cfg.distance_max, alloc);
            d.AddMember("enabled", cfg.distance_enabled, alloc);
            filters.AddMember("distance", d, alloc);
        }

        // roi
        {
            rapidjson::Value r(rapidjson::kObjectType);
            r.AddMember("x_min", cfg.roi_x_min, alloc);
            r.AddMember("x_max", cfg.roi_x_max, alloc);
            r.AddMember("y_min", cfg.roi_y_min, alloc);
            r.AddMember("y_max", cfg.roi_y_max, alloc);
            r.AddMember("z_min", cfg.roi_z_min, alloc);
            r.AddMember("z_max", cfg.roi_z_max, alloc);
            r.AddMember("enabled", cfg.roi_enabled, alloc);
            filters.AddMember("roi", r, alloc);
        }

        // sea_filter
        {
            rapidjson::Value s(rapidjson::kObjectType);
            s.AddMember("sea_level_z", cfg.sea_level_z, alloc);
            s.AddMember("margin_m", cfg.sea_margin, alloc);
            s.AddMember("enabled", cfg.sea_enabled, alloc);
            filters.AddMember("sea_filter", s, alloc);
        }

        // statistical_outlier
        {
            rapidjson::Value o(rapidjson::kObjectType);
            o.AddMember("mean_k", cfg.outlier_mean_k, alloc);
            o.AddMember("stddev_mul", cfg.outlier_stddev_mul, alloc);
            o.AddMember("enabled", cfg.outlier_enabled, alloc);
            filters.AddMember("statistical_outlier", o, alloc);
        }

        // voxel
        {
            rapidjson::Value v(rapidjson::kObjectType);
            v.AddMember("leaf_size", cfg.voxel_leaf_size, alloc);
            v.AddMember("enabled", cfg.voxel_enabled, alloc);
            filters.AddMember("voxel", v, alloc);
        }

        // density
        {
            rapidjson::Value d(rapidjson::kObjectType);
            d.AddMember("voxel_size", cfg.density_voxel_size, alloc);
            d.AddMember("min_points", cfg.density_min_points, alloc);
            d.AddMember("enabled", cfg.density_enabled, alloc);
            filters.AddMember("density", d, alloc);
        }

        // motion
        {
            rapidjson::Value m(rapidjson::kObjectType);
            m.AddMember("cell_size", cfg.motion_cell_size, alloc);
            m.AddMember("threshold", cfg.motion_threshold, alloc);
            m.AddMember("output_static", cfg.motion_output_static, alloc);
            m.AddMember("enabled", cfg.motion_enabled, alloc);
            filters.AddMember("motion", m, alloc);
        }

        doc.AddMember("filters", filters, alloc);
    }

    // docking
    {
        rapidjson::Value docking(rapidjson::kObjectType);

        // nearest_region
        {
            rapidjson::Value nr(rapidjson::kObjectType);
            nr.AddMember("sector_x_min", cfg.nr_sector_x_min, alloc);
            nr.AddMember("sector_x_max", cfg.nr_sector_x_max, alloc);
            nr.AddMember("sector_y_min", cfg.nr_sector_y_min, alloc);
            nr.AddMember("sector_y_max", cfg.nr_sector_y_max, alloc);
            nr.AddMember("sector_z_min", cfg.nr_sector_z_min, alloc);
            nr.AddMember("sector_z_max", cfg.nr_sector_z_max, alloc);
            nr.AddMember("percentile", cfg.nr_percentile, alloc);
            nr.AddMember("enabled", cfg.nr_enabled, alloc);
            docking.AddMember("nearest_region", nr, alloc);
        }

        // dock_edge
        {
            rapidjson::Value edge(rapidjson::kObjectType);
            edge.AddMember("x_min", cfg.edge_x_min, alloc);
            edge.AddMember("x_max", cfg.edge_x_max, alloc);
            edge.AddMember("y_min", cfg.edge_y_min, alloc);
            edge.AddMember("y_max", cfg.edge_y_max, alloc);
            edge.AddMember("z_min", cfg.edge_z_min, alloc);
            edge.AddMember("z_max", cfg.edge_z_max, alloc);
            edge.AddMember("ransac_dist", cfg.edge_ransac_dist, alloc);
            edge.AddMember("enabled", cfg.edge_enabled, alloc);
            docking.AddMember("dock_edge", edge, alloc);
        }

        // temporal
        {
            rapidjson::Value t(rapidjson::kObjectType);
            t.AddMember("max_jump_m", cfg.temporal_max_jump, alloc);
            t.AddMember("enabled", cfg.temporal_enabled, alloc);
            docking.AddMember("temporal", t, alloc);
        }

        doc.AddMember("docking", docking, alloc);
    }

    // udp
    {
        rapidjson::Value udp(rapidjson::kObjectType);
        udp.AddMember("target_ip", rapidjson::Value(cfg.udp_target_ip.c_str(), alloc), alloc);
        udp.AddMember("target_port", cfg.udp_target_port, alloc);
        udp.AddMember("sensor_id", cfg.udp_sensor_id, alloc);
        udp.AddMember("enabled", cfg.udp_enabled, alloc);
        doc.AddMember("udp", udp, alloc);
    }

    // 写入文件
    rapidjson::StringBuffer sb;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
    writer.SetMaxDecimalPlaces(2);  // 限制浮点数精度为 2 位小数
    doc.Accept(writer);

    std::ofstream ofs(path);
    if (!ofs.is_open()) {
        LOG_ERROR("[AppConfig] Cannot write config file: {}", path);
        return false;
    }
    ofs << sb.GetString();
    ofs.close();

    LOG_INFO("[AppConfig] Saved config to {}", path);
    return true;
}

} // namespace Linger
