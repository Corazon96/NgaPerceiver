#pragma once

/**
 * @file logger.h
 * @brief 日志系统封装
 * 基于 spdlog 实现，支持控制台输出和按日期滚动的日志文件。
 */

#include <spdlog/spdlog.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>

#include <memory>
#include <string>
#include <filesystem>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <vector>
#include <atomic>

// --- 辅助宏定义 ---

/** 
 * @brief 线程安全的限流日志宏实现 (内部使用)
 * 使用 std::atomic 确保多线程环境下的正确性
 * 注意：为简化实现，使用 int64_t 存储纳秒时间戳
 */
#define LOG_THROTTLED_IMPL(level, interval_ms, ...) \
    do { \
        static std::atomic<int64_t> last_log_time_ns_##__LINE__{0}; \
        auto now = std::chrono::steady_clock::now(); \
        int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count(); \
        int64_t last_ns = last_log_time_ns_##__LINE__.load(std::memory_order_relaxed); \
        int64_t interval_ns = static_cast<int64_t>(interval_ms) * 1000000LL; \
        if (now_ns - last_ns >= interval_ns) { \
            if (last_log_time_ns_##__LINE__.compare_exchange_weak(last_ns, now_ns, std::memory_order_relaxed)) { \
                spdlog::level(__VA_ARGS__); \
            } \
        } \
    } while(0)

// --- 公开日志宏 ---

#define LOG_TRACE(...)    spdlog::trace(__VA_ARGS__)
#define LOG_DEBUG(...)    spdlog::debug(__VA_ARGS__)
#define LOG_INFO(...)     spdlog::info(__VA_ARGS__)
#define LOG_WARN(...)     spdlog::warn(__VA_ARGS__)
#define LOG_ERROR(...)    spdlog::error(__VA_ARGS__)
#define LOG_CRITICAL(...) spdlog::critical(__VA_ARGS__)

// 限流日志宏：每隔 interval_ms 毫秒最多打印一次（线程安全）
#define LOG_TRACE_THROTTLED(interval_ms, ...) LOG_THROTTLED_IMPL(trace, interval_ms, __VA_ARGS__)
#define LOG_DEBUG_THROTTLED(interval_ms, ...) LOG_THROTTLED_IMPL(debug, interval_ms, __VA_ARGS__)
#define LOG_INFO_THROTTLED(interval_ms, ...)  LOG_THROTTLED_IMPL(info, interval_ms, __VA_ARGS__)
#define LOG_WARN_THROTTLED(interval_ms, ...)  LOG_THROTTLED_IMPL(warn, interval_ms, __VA_ARGS__)
#define LOG_ERROR_THROTTLED(interval_ms, ...) LOG_THROTTLED_IMPL(error, interval_ms, __VA_ARGS__)

/**
 * @brief 日志管理器单例
 */
class Logger {
public:
    // 日志滚动配置
    static constexpr size_t MAX_FILE_SIZE = 5 * 1024 * 1024;  // 单个日志文件最大 5MB
    static constexpr size_t MAX_FILES = 5;                     // 每天最多保留 5 个滚动文件

    /** @brief 获取单例实例 */
    static Logger& instance() {
        static Logger instance;
        return instance;
    }

    // 禁用拷贝和赋值
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    /**
     * @brief 初始化日志系统
     * @param base_name 日志文件基础名称（不含路径和扩展名）
     * @param level 日志等级 (trace, debug, info, warn, error, critical)
     * 
     * 日志将自动存放到 log/<日期>/<base_name>.log
     */
    void init(const std::string& base_name = "linger_perceiver", const std::string& level = "info") {
        try {
            // 获取当前日期字符串
            std::string date_str = getCurrentDateString();
            
            // 构建日志目录: log/<日期>/
            std::filesystem::path log_dir = std::filesystem::path("log") / date_str;
            std::filesystem::create_directories(log_dir);
            
            // 构建完整日志文件路径
            std::filesystem::path log_path = log_dir / (base_name + ".log");

            // 创建控制台和滚动文件双输出
            auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            auto file_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
                log_path.string(), MAX_FILE_SIZE, MAX_FILES);

            std::vector<spdlog::sink_ptr> sinks{console_sink, file_sink};
            auto logger = std::make_shared<spdlog::logger>("linger", sinks.begin(), sinks.end());

            // 设置格式: [时间] [等级] 消息
            logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%l%$] %v");

            // 设置等级
            setLogLevel(logger, level);

            // 设置为默认 logger
            spdlog::set_default_logger(logger);
            spdlog::flush_on(spdlog::level::warn); // warn 及以上立即刷新
            
            // 记录日志初始化信息
            spdlog::info("Logger initialized. Log path: {}", log_path.string());
        } catch (const spdlog::spdlog_ex& ex) {
            // 初始化失败时回退到控制台
            spdlog::error("Logger init failed: {}", ex.what());
        }
    }

    /**
     * @brief 关闭日志系统（程序退出时调用）
     */
    void shutdown() {
        spdlog::shutdown();
    }

private:
    Logger() {
        // 默认初始化控制台输出，防止未调用 init 时崩溃
        auto console = spdlog::stdout_color_mt("console");
        spdlog::set_default_logger(console);
    }

    /** @brief 获取当前日期字符串 (YYYY-MM-DD) */
    std::string getCurrentDateString() {
        auto now = std::chrono::system_clock::now();
        auto in_time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d");
        return ss.str();
    }

    /** @brief 设置日志等级 */
    void setLogLevel(std::shared_ptr<spdlog::logger> logger, const std::string& level) {
        if (level == "trace") logger->set_level(spdlog::level::trace);
        else if (level == "debug") logger->set_level(spdlog::level::debug);
        else if (level == "info") logger->set_level(spdlog::level::info);
        else if (level == "warn") logger->set_level(spdlog::level::warn);
        else if (level == "error") logger->set_level(spdlog::level::err);
        else if (level == "critical") logger->set_level(spdlog::level::critical);
        else logger->set_level(spdlog::level::info);
    }
};
