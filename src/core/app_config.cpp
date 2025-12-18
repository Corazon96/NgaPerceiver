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

    // logging
    if (doc.HasMember("logging") && doc["logging"].IsObject()) {
        const auto& logging = doc["logging"];
        if (logging.HasMember("log_file") && logging["log_file"].IsString())
            cfg.log_file = logging["log_file"].GetString();
        if (logging.HasMember("log_level") && logging["log_level"].IsString())
            cfg.log_level = logging["log_level"].GetString();
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

    // logging
    {
        rapidjson::Value logging(rapidjson::kObjectType);
        logging.AddMember("log_file", rapidjson::Value(cfg.log_file.c_str(), alloc), alloc);
        logging.AddMember("log_level", rapidjson::Value(cfg.log_level.c_str(), alloc), alloc);
        doc.AddMember("logging", logging, alloc);
    }

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

        doc.AddMember("filters", filters, alloc);
    }

    // 写入文件
    rapidjson::StringBuffer sb;
    rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(sb);
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
