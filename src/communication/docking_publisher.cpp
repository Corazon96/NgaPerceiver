#include "communication/docking_publisher.h"
#include "core/logger.h"

#include <cstring>

namespace Linger {

// 静态成员初始化
std::atomic<int> DockingPublisher::wsa_init_count_{0};

DockingPublisher::DockingPublisher() = default;

DockingPublisher::~DockingPublisher()
{
    stop();
}

void DockingPublisher::setConfig(const UdpConfig& config)
{
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = config;
}

UdpConfig DockingPublisher::getConfig() const
{
    std::lock_guard<std::mutex> lock(config_mutex_);
    return config_;
}

bool DockingPublisher::start()
{
    if (running_.load()) {
        return true;
    }

    UdpConfig cfg;
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        cfg = config_;
    }

    if (!cfg.enabled) {
        LOG_INFO("[DockingPublisher] Disabled by config");
        return true;
    }

    if (!initSocket()) {
        return false;
    }

    running_.store(true);
    LOG_INFO("[DockingPublisher] Started, target={}:{}, sensor_id={}",
             cfg.target_ip, cfg.target_port, cfg.sensor_id);
    return true;
}

void DockingPublisher::stop()
{
    if (!running_.load()) {
        return;
    }

    running_.store(false);
    closeSocket();
    LOG_INFO("[DockingPublisher] Stopped, sent={}, errors={}",
             sent_count_.load(), error_count_.load());
}

void DockingPublisher::publish(const DockingState& state)
{
    if (!running_.load()) {
        return;
    }

    DockingPacket packet = stateToPacket(state);
    
    if (sendPacket(packet)) {
        sent_count_.fetch_add(1);
    } else {
        error_count_.fetch_add(1);
    }
}

bool DockingPublisher::initSocket()
{
#ifdef _WIN32
    // WinSock 初始化（引用计数）
    if (wsa_init_count_.fetch_add(1) == 0) {
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            LOG_ERROR("[DockingPublisher] WSAStartup failed: {}", result);
            wsa_init_count_.fetch_sub(1);
            return false;
        }
    }
#endif

    std::lock_guard<std::mutex> lock(socket_mutex_);

    // 创建 UDP socket
    socket_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
#ifdef _WIN32
    if (socket_ == INVALID_SOCKET) {
        LOG_ERROR("[DockingPublisher] socket() failed: {}", WSAGetLastError());
        return false;
    }
#else
    if (socket_ < 0) {
        LOG_ERROR("[DockingPublisher] socket() failed: {}", errno);
        return false;
    }
#endif

    // 设置目标地址
    UdpConfig cfg;
    {
        std::lock_guard<std::mutex> cfg_lock(config_mutex_);
        cfg = config_;
    }

    std::memset(&target_addr_, 0, sizeof(target_addr_));
    target_addr_.sin_family = AF_INET;
    target_addr_.sin_port = htons(cfg.target_port);
    
    // 统一使用 inet_pton() (Windows Vista+ 支持)
    if (inet_pton(AF_INET, cfg.target_ip.c_str(), &target_addr_.sin_addr) <= 0) {
        LOG_ERROR("[DockingPublisher] Invalid IP address: {}", cfg.target_ip);
#ifdef _WIN32
        closesocket(socket_);
        socket_ = INVALID_SOCKET;
#else
        close(socket_);
        socket_ = -1;
#endif
        return false;
    }

    return true;
}

void DockingPublisher::closeSocket()
{
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_ != INVALID_SOCKET) {
        closesocket(socket_);
        socket_ = INVALID_SOCKET;
    }
    
    // WinSock 清理（引用计数）
    if (wsa_init_count_.fetch_sub(1) == 1) {
        WSACleanup();
    }
#else
    if (socket_ >= 0) {
        close(socket_);
        socket_ = -1;
    }
#endif
}

bool DockingPublisher::sendPacket(const DockingPacket& packet)
{
    std::lock_guard<std::mutex> lock(socket_mutex_);

#ifdef _WIN32
    if (socket_ == INVALID_SOCKET) {
        return false;
    }
#else
    if (socket_ < 0) {
        return false;
    }
#endif

    int sent = sendto(
        socket_,
        reinterpret_cast<const char*>(&packet),
        sizeof(packet),
        0,
        reinterpret_cast<sockaddr*>(&target_addr_),
        sizeof(target_addr_)
    );

    if (sent != sizeof(packet)) {
#ifdef _WIN32
        LOG_WARN("[DockingPublisher] sendto failed: {}", WSAGetLastError());
#else
        LOG_WARN("[DockingPublisher] sendto failed: {}", errno);
#endif
        return false;
    }

    return true;
}

DockingPacket DockingPublisher::stateToPacket(const DockingState& state)
{
    DockingPacket packet{};
    
    packet.timestamp_ns = state.timestamp_ns;
    
    {
        std::lock_guard<std::mutex> lock(config_mutex_);
        packet.sensor_id = config_.sensor_id;
    }

    // 状态映射
    if (state.status == DockingStatus::NORMAL) {
        packet.status = DockingPacketStatus::NORMAL;
    } else if (state.status == DockingStatus::LOW_CONFIDENCE) {
        packet.status = DockingPacketStatus::LOW_CONFIDENCE;
    } else {
        packet.status = DockingPacketStatus::NOT_DETECTED;
    }
    
    // 置信度：优先使用 edge（更精确），否则使用 nearest
    // 未检测时置信度为0
    if (packet.status == DockingPacketStatus::NOT_DETECTED) {
        packet.confidence = 0;
    } else if (state.edge.valid) {
        packet.confidence = state.edge.confidence;
    } else if (state.nearest.valid) {
        packet.confidence = state.nearest.confidence;
    } else {
        packet.confidence = 0;
    }

    packet.reserved = 0;

    // 使用融合后的最终距离
    packet.distance_m = state.final_distance_m;
    
    // 如果边缘检测有效，使用边缘角度
    if (state.edge.valid) {
        packet.angle_deg = state.edge.angle_deg;
    } else {
        packet.angle_deg = 0.0f;
    }

    return packet;
}

} // namespace Linger
