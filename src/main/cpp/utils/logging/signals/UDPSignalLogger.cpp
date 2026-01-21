#include "utils/logging/signals/UDPSignalLogger.h"
#include <sstream>
#include <iostream>
#ifdef _WIN32
// Windows-specific includes are in the header
#else
#include <cerrno>
#endif
#include <string>

UDPSignalLogger::UDPSignalLogger(const std::string &ipAddress, int port)
    : m_ipAddress(ipAddress), m_port(port), m_isRunning(false)
#ifdef _WIN32
      ,
      m_socket(INVALID_SOCKET)
#else
      ,
      m_socket(-1)
#endif
{
#ifdef _WIN32
    // Initialize Winsock
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0)
    {
        std::cerr << "WSAStartup failed" << std::endl;
        return;
    }
#endif

    // Create UDP socket
    m_socket = socket(AF_INET, SOCK_DGRAM, 0);
#ifdef _WIN32
    if (m_socket == INVALID_SOCKET)
#else
    if (m_socket < 0)
#endif
    {
        std::cerr << "UDP Socket creation failed" << std::endl;
        return;
    }

    // Configure server address
    memset(&m_serverAddr, 0, sizeof(m_serverAddr));
    m_serverAddr.sin_family = AF_INET;
    m_serverAddr.sin_port = htons(m_port);

    // Convert IP address from string to binary form
#ifdef _WIN32
    if (inet_pton(AF_INET, m_ipAddress.c_str(), &m_serverAddr.sin_addr) <= 0)
#else
    if (inet_pton(AF_INET, m_ipAddress.c_str(), &m_serverAddr.sin_addr) <= 0)
#endif
    {
        std::cerr << "Invalid address / Address not supported: " << m_ipAddress << std::endl;
#ifdef _WIN32
        closesocket(m_socket);
        m_socket = INVALID_SOCKET;
#else
        close(m_socket);
        m_socket = -1;
#endif
        return;
    }

    std::cout << "UDP Logger initialized for " << m_ipAddress << ":" << m_port << std::endl;
}

UDPSignalLogger::~UDPSignalLogger()
{
    Stop();
#ifdef _WIN32
    if (m_socket != INVALID_SOCKET)
    {
        closesocket(m_socket);
    }
    WSACleanup();
#else
    if (m_socket >= 0)
    {
        close(m_socket);
    }
#endif
}

void UDPSignalLogger::Start()
{
    m_isRunning = true;
}

void UDPSignalLogger::Stop()
{
    m_isRunning = false;
}

std::string UDPSignalLogger::FormatMessage(std::string signalID, std::string type,
                                           std::string value, std::string_view units, units::time::second_t timestamp)
{
    std::ostringstream oss;
    oss << timestamp.value() << "," << signalID << "," << type << "," << value << "," << units;
    return oss.str();
}

void UDPSignalLogger::SendData(const std::string& message)
{
#ifdef _WIN32
    if (!m_isRunning || m_socket == INVALID_SOCKET)
#else
    if (!m_isRunning || m_socket < 0)
#endif
    {
        return;
    }

#ifdef _WIN32
    int bytesSent = sendto(m_socket, message.c_str(), (int)message.length(), 0,
                           (struct sockaddr *)&m_serverAddr, sizeof(m_serverAddr));
    if (bytesSent == SOCKET_ERROR)
    {
        std::cerr << "sendto failed: " << WSAGetLastError() << std::endl;
    }
#else
    ssize_t bytesSent = sendto(m_socket, message.c_str(), message.length(), 0,
                               (struct sockaddr *)&m_serverAddr, sizeof(m_serverAddr));
    if (bytesSent < 0)
    {
        std::cerr << "sendto failed: " << strerror(errno) << std::endl;
    }
#endif
}

void UDPSignalLogger::WriteBoolean(std::string signalID, bool value, units::time::second_t timestamp)
{
    std::string message = FormatMessage(signalID, "bool", value ? "true" : "false", "bool", timestamp);
    SendData(message);
}

void UDPSignalLogger::WriteDouble(std::string signalID, double value, std::string_view units, units::time::second_t timestamp)
{
    std::ostringstream oss;
    oss << value;
    std::string message = FormatMessage(signalID, "double", oss.str(), units, timestamp);
    SendData(message);
}

void UDPSignalLogger::WriteInteger(std::string signalID, int64_t value, std::string_view units, units::time::second_t timestamp)
{
    std::ostringstream oss;
    oss << value;
    std::string message = FormatMessage(signalID, "int64", oss.str(), units, timestamp);
    SendData(message);
}

void UDPSignalLogger::WriteString(std::string signalID, const std::string &value, units::time::second_t timestamp)
{
    std::string message = FormatMessage(signalID, "string", value, "string", timestamp);
    SendData(message);
}