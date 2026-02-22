#pragma once

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#undef WIN32_LEAN_AND_MEAN
#else
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <sys/socket.h>
#endif

#include "utils/logging/signals/ISignalLogger.h"
#include <string>
#include <atomic>

class UDPSignalLogger : public ISignalLogger
{
public:
    UDPSignalLogger(const std::string &host, int port);
    ~UDPSignalLogger() override;

    void WriteBoolean(std::string_view signalID, bool value, uint64_t timestamp) override;
    void WriteDouble(std::string_view signalID, double value, std::string_view units, uint64_t timestamp) override;
    void WriteInteger(std::string_view signalID, int64_t value, std::string_view units, uint64_t timestamp) override;
    void WriteString(std::string_view signalID, const std::string &value, uint64_t timestamp) override;
    void WriteDoubleArray(std::string_view signalID, const std::vector<double> &value, std::string_view units, uint64_t timestamp) override;

    void WritePose2d(std::string_view signalID, const frc::Pose2d &value, uint64_t timestamp) override;
    void WritePose3d(std::string_view signalID, const frc::Pose3d &value, uint64_t timestamp) override;
    void WriteChassisSpeeds(std::string_view signalID, const frc::ChassisSpeeds &value, uint64_t timestamp) override;
    void WriteSwerveModuleState(std::string_view signalID, const frc::SwerveModuleState &value, uint64_t timestamp) override;
    void WriteGamePadState(std::string_view signalID, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs, uint64_t timestamp) override;
    void Start() override;
    void Stop() override;

private:
    void SendData(const std::string &message);
    std::string FormatMessage(std::string_view signalID, std::string_view type,
                              std::string_view value, std::string_view units, uint64_t timestamp);

    std::string m_host;
    int m_port;
    std::atomic<bool> m_isRunning;
#ifdef _WIN32
    SOCKET m_socket;
#else
    int m_socket;
#endif
    sockaddr_in m_serverAddr;
};