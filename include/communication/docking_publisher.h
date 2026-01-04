#pragma once

#include "communication/docking_packet.h"
#include "algorithms/docking_types.h"
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <chrono>
#include <queue>
#include <condition_variable>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>  // For inet_pton on Windows
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

namespace Linger {

/**
 * @brief UDP 配置
 */
struct UdpConfig {
    std::string target_ip = "127.0.0.1";  ///< 目标IP
    uint16_t target_port = 5000;          ///< 目标端口
    uint8_t sensor_id = SensorId::FRONT;  ///< 传感器ID
    bool enabled = false;                  ///< 是否启用
};

/**
 * @brief 靠泊数据 UDP 发布器
 * 
 * 功能：
 * - 将 DockingState 转换为 DockingPacket 并通过 UDP 发送
 * - 异步发送，不阻塞调用线程
 * - 支持超时检测（长时间无数据时发送 status=0）
 * 
 * 使用方式：
 * @code
 * DockingPublisher pub;
 * pub.setConfig(config);
 * pub.start();
 * // ... 在检测回调中 ...
 * pub.publish(dockingState);
 * // ... 退出时 ...
 * pub.stop();
 * @endcode
 */
class DockingPublisher {
public:
    DockingPublisher();
    ~DockingPublisher();

    // 禁止拷贝
    DockingPublisher(const DockingPublisher&) = delete;
    DockingPublisher& operator=(const DockingPublisher&) = delete;

    /**
     * @brief 设置配置
     */
    void setConfig(const UdpConfig& config);
    UdpConfig getConfig() const;

    /**
     * @brief 启动发布器
     * @return 成功返回 true
     */
    bool start();

    /**
     * @brief 停止发布器
     */
    void stop();

    /**
     * @brief 发布靠泊状态
     * @param state 靠泊检测结果
     * 
     * 线程安全，可从任意线程调用
     */
    void publish(const DockingState& state);

    /**
     * @brief 检查是否正在运行
     */
    bool isRunning() const { return running_.load(); }

    /**
     * @brief 获取已发送包数量
     */
    uint64_t getSentCount() const { return sent_count_.load(); }

    /**
     * @brief 获取发送失败数量
     */
    uint64_t getErrorCount() const { return error_count_.load(); }

    /**
     * @brief 重置统计计数器
     */
    void resetStatistics() {
        sent_count_.store(0);
        error_count_.store(0);
    }

    /**
     * @brief 获取发送成功率（百分比）
     * @return 成功率 0.0-100.0，无发送时返回 0.0
     */
    float getSuccessRate() const {
        uint64_t total = sent_count_.load() + error_count_.load();
        if (total == 0) return 0.0f;
        return (sent_count_.load() * 100.0f) / total;
    }

private:
    /**
     * @brief 初始化 Socket
     */
    bool initSocket();

    /**
     * @brief 关闭 Socket
     */
    void closeSocket();

    /**
     * @brief 发送数据包
     */
    bool sendPacket(const DockingPacket& packet);

    /**
     * @brief 将 DockingState 转换为 DockingPacket
     */
    DockingPacket stateToPacket(const DockingState& state);

    /**
     * @brief 发送线程工作函数
     */
    void senderThreadFunc();

private:
    UdpConfig config_;
    mutable std::mutex config_mutex_;

    std::atomic<bool> running_{false};
    std::atomic<uint64_t> sent_count_{0};
    std::atomic<uint64_t> error_count_{0};

#ifdef _WIN32
    SOCKET socket_ = INVALID_SOCKET;
#else
    int socket_ = -1;
#endif
    sockaddr_in target_addr_{};
    std::mutex socket_mutex_;

    // 异步发送线程
    std::thread sender_worker_;
    std::queue<DockingPacket> packet_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    static constexpr size_t MAX_QUEUE_SIZE = 100;  ///< 队列最大长度

    // WinSock 初始化标志
    static std::atomic<int> wsa_init_count_;
};

} // namespace Linger
