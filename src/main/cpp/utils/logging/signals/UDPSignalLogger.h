#pragma once

#ifdef _WIN32
#define _HAS_STD_BYTE 0
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
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
    UDPSignalLogger(const std::string& ipAddress, int port);
    ~UDPSignalLogger() override;

    void WriteBoolean(std::string signalID, bool value, units::time::second_t timestamp) override;
    void WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t timestamp) override;
    void WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t timestamp) override;
    void WriteString(std::string signalID, const std::string &value, units::time::second_t timestamp) override;

    void Start() override;
    void Stop() override;

private:
    void SendData(const std::string& message);
    std::string FormatMessage(std::string signalID, std::string type,
                              std::string value, std::string_view units, units::time::second_t timestamp);

#ifdef _WIN32
    SOCKET m_socket;
#else
    int m_socket;
#endif
    sockaddr_in m_serverAddr;
    std::string m_ipAddress;
    int m_port;
    std::atomic<bool> m_isRunning;
};