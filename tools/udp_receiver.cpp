/**
 * @file udp_receiver.cpp
 * @brief DockingPacket UDP receiver test tool
 * 
 * Build (Windows MSVC):
 *   cl /EHsc /std:c++17 udp_receiver.cpp /link ws2_32.lib
 * 
 * Build (MinGW):
 *   g++ -std=c++17 udp_receiver.cpp -o udp_receiver.exe -lws2_32
 * 
 * Usage:
 *   udp_receiver.exe [port]
 *   Default port: 5000
 */

#include <iostream>
#include <iomanip>
#include <cstdint>
#include <cstring>
#include <chrono>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#define SOCKET int
#define INVALID_SOCKET -1
#define SOCKET_ERROR -1
#define closesocket close
#endif

// ============================================================================
// Protocol definition (must match docking_packet.h)
// ============================================================================

#pragma pack(push, 1)
struct DockingPacket {
    uint64_t timestamp_ns;
    uint8_t  sensor_id;
    uint8_t  status;
    uint8_t  confidence;
    uint8_t  reserved;
    float    distance_m;
    float    angle_deg;
};
#pragma pack(pop)

static_assert(sizeof(DockingPacket) == 20, "DockingPacket must be 20 bytes");

// ============================================================================
// Helper functions
// ============================================================================

const char* statusToString(uint8_t status) {
    switch (status) {
        case 0: return "NOT_DETECTED";
        case 1: return "NORMAL";
        case 2: return "LOW_CONFIDENCE";
        default: return "UNKNOWN";
    }
}

const char* sensorIdToString(uint8_t id) {
    switch (id) {
        case 1: return "FRONT";
        case 2: return "PORT";
        case 3: return "STARBOARD";
        default: return "UNKNOWN";
    }
}

void printPacket(const DockingPacket& pkt, const char* sender_ip, uint16_t sender_port) {
    // Convert timestamp to seconds
    double ts_sec = pkt.timestamp_ns / 1e9;
    
    std::cout << "------------------------------------------------------------\n";
    std::cout << "From: " << sender_ip << ":" << sender_port << "\n";
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "  Timestamp:  " << ts_sec << " s\n";
    std::cout << "  Sensor ID:  " << (int)pkt.sensor_id 
              << " (" << sensorIdToString(pkt.sensor_id) << ")\n";
    std::cout << "  Status:     " << (int)pkt.status 
              << " (" << statusToString(pkt.status) << ")\n";
    std::cout << "  Confidence: " << (int)pkt.confidence << "%\n";
    std::cout << std::setprecision(2);
    std::cout << "  Distance:   " << pkt.distance_m << " m\n";
    std::cout << "  Angle:      " << pkt.angle_deg << " deg\n";
    std::cout << std::flush;
}

void printHex(const uint8_t* data, size_t len) {
    std::cout << "  Raw (" << len << " bytes): ";
    for (size_t i = 0; i < len; ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') 
                  << (int)data[i] << " ";
    }
    std::cout << std::dec << "\n";
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char* argv[]) {
    uint16_t port = 5000;
    
    if (argc > 1) {
        port = static_cast<uint16_t>(std::stoi(argv[1]));
    }

    std::cout << "========================================\n";
    std::cout << "  DockingPacket UDP Receiver\n";
    std::cout << "  Listening on port: " << port << "\n";
    std::cout << "  Expected packet size: " << sizeof(DockingPacket) << " bytes\n";
    std::cout << "  Press Ctrl+C to exit\n";
    std::cout << "========================================\n\n";

#ifdef _WIN32
    // Initialize WinSock
    WSADATA wsaData;
    int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
    if (result != 0) {
        std::cerr << "WSAStartup failed: " << result << std::endl;
        return 1;
    }
#endif

    // Create UDP socket
    SOCKET sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        std::cerr << "socket() failed\n";
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    // Bind to port
    sockaddr_in local_addr{};
    local_addr.sin_family = AF_INET;
    local_addr.sin_addr.s_addr = INADDR_ANY;
    local_addr.sin_port = htons(port);

    if (bind(sock, reinterpret_cast<sockaddr*>(&local_addr), sizeof(local_addr)) == SOCKET_ERROR) {
        std::cerr << "bind() failed\n";
        closesocket(sock);
#ifdef _WIN32
        WSACleanup();
#endif
        return 1;
    }

    std::cout << "Waiting for packets...\n\n";

    // Receive loop
    uint8_t buffer[256];
    sockaddr_in sender_addr{};
    int sender_len = sizeof(sender_addr);

    uint64_t packet_count = 0;

    while (true) {
#ifdef _WIN32
        int recv_len = recvfrom(sock, reinterpret_cast<char*>(buffer), sizeof(buffer), 0,
                                reinterpret_cast<sockaddr*>(&sender_addr), &sender_len);
#else
        socklen_t sender_len_socklen = sender_len;
        int recv_len = recvfrom(sock, buffer, sizeof(buffer), 0,
                                reinterpret_cast<sockaddr*>(&sender_addr), &sender_len_socklen);
#endif

        if (recv_len == SOCKET_ERROR) {
            std::cerr << "recvfrom() failed\n";
            continue;
        }

        packet_count++;
        
        char sender_ip[INET_ADDRSTRLEN];
#ifdef _WIN32
        inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, sizeof(sender_ip));
#else
        inet_ntop(AF_INET, &sender_addr.sin_addr, sender_ip, sizeof(sender_ip));
#endif
        uint16_t sender_port = ntohs(sender_addr.sin_port);

        std::cout << "[#" << packet_count << "] ";
        
        if (recv_len == sizeof(DockingPacket)) {
            DockingPacket pkt;
            std::memcpy(&pkt, buffer, sizeof(pkt));
            printPacket(pkt, sender_ip, sender_port);
        } else {
            std::cout << "Unexpected packet size: " << recv_len << " bytes\n";
            printHex(buffer, recv_len);
        }
    }

    closesocket(sock);
#ifdef _WIN32
    WSACleanup();
#endif
    return 0;
}
