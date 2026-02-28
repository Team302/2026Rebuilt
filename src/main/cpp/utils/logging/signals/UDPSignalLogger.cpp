#include "utils/logging/signals/UDPSignalLogger.h"
#include <cinttypes>
#include <cstdio>
#include <iostream>
#include <cstring>
#ifdef _WIN32
// Windows-specific includes are in the header
#include <ws2tcpip.h>
#else
#include <cerrno>
#include <netdb.h>
#endif

UDPSignalLogger::UDPSignalLogger(const std::string &host, int port)
    : m_host(host), m_port(port), m_isRunning(false)
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

    // Configure server address (support DNS or IP)
    memset(&m_serverAddr, 0, sizeof(m_serverAddr));
    m_serverAddr.sin_family = AF_INET;
    m_serverAddr.sin_port = htons(m_port);

    struct addrinfo hints = {};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;
    struct addrinfo *result = nullptr;
    int res = getaddrinfo(m_host.c_str(), nullptr, &hints, &result);
    if (res != 0 || !result || !result->ai_addr)
    {
        std::cerr << "!!! Failed to resolve host: " << m_host << "!!!" << std::endl;
#ifdef _WIN32
        closesocket(m_socket);
        m_socket = INVALID_SOCKET;
#else
        close(m_socket);
        m_socket = -1;
#endif
        return;
    }
    // Copy resolved address
    m_serverAddr.sin_addr = ((struct sockaddr_in *)result->ai_addr)->sin_addr;
    freeaddrinfo(result);

    std::cout << "UDP Logger initialized for " << m_host << ":" << m_port << std::endl;
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

int UDPSignalLogger::FormatMessage(char *buf, int bufSize, const char *signalID, const char *type,
                                   const char *value, std::string_view units, uint64_t timestamp)
{
    return snprintf(buf, bufSize, "%llu,%s,%s,%s,%.*s",
                    (unsigned long long)timestamp, signalID, type, value,
                    (int)units.size(), units.data());
}

void UDPSignalLogger::SendData(const char *buf, int len)
{
#ifdef _WIN32
    if (!m_isRunning || m_socket == INVALID_SOCKET)
#else
    if (!m_isRunning || m_socket < 0)
#endif
    {
        return;
    }

    // len is expected to be the snprintf return value; if it's non-positive or
    // indicates truncation (>= k_bufSize), treat the message as invalid and drop it.
    if (len <= 0 || len >= k_bufSize)
        return;

#ifdef _WIN32
    int bytesSent = sendto(m_socket, buf, len, 0,
                           (struct sockaddr *)&m_serverAddr, sizeof(m_serverAddr));
    if (bytesSent == SOCKET_ERROR)
    {
        std::cerr << "sendto failed: " << WSAGetLastError() << std::endl;
    }
#else
    ssize_t bytesSent = sendto(m_socket, buf, len, 0,
                               (struct sockaddr *)&m_serverAddr, sizeof(m_serverAddr));
    if (bytesSent < 0)
    {
        std::cerr << "sendto failed: " << strerror(errno) << std::endl;
    }
#endif
}

void UDPSignalLogger::WriteBoolean(std::string signalID, bool value, uint64_t timestamp)
{
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "bool", value ? "true" : "false", "bool", timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteDouble(std::string signalID, double value, std::string_view units, uint64_t timestamp)
{
    char valBuf[32];
    snprintf(valBuf, sizeof(valBuf), "%.6g", value);
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "double", valBuf, units, timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteInteger(std::string signalID, int64_t value, std::string_view units, uint64_t timestamp)
{
    char valBuf[32];
    snprintf(valBuf, sizeof(valBuf), "%lld", (long long)value);
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "int64", valBuf, units, timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteString(std::string signalID, const std::string &value, uint64_t timestamp)
{
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "string", value.c_str(), "string", timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteDoubleArray(std::string signalID, const std::vector<double> &value, std::string_view units, uint64_t timestamp)
{
    char valBuf[256];
    valBuf[0] = '\0';
    int pos = 0;
    int remaining = (int)sizeof(valBuf);
    for (size_t i = 0; i < value.size() && remaining > 1; ++i)
    {
        int written = snprintf(valBuf + pos, remaining, i ? ";%.6g" : "%.6g", value[i]);
        if (written < 0 || written >= remaining)
        {
            // Discard any partially written token to avoid truncated final element
            if (pos >= 0 && pos < static_cast<int>(sizeof(valBuf)))
            {
                valBuf[pos] = '\0';
            }
            break;
        }
        pos += written;
        remaining -= written;
    }
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "double_array", valBuf, units, timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WritePose2d(std::string signalID, const frc::Pose2d &value, uint64_t timestamp)
{
    char valBuf[96];
    snprintf(valBuf, sizeof(valBuf), "%.6g;%.6g;%.6g",
             value.X().value(), value.Y().value(), value.Rotation().Radians().value());
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "pose2d", valBuf, "X_m;Y_m;Rot_rad", timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WritePose3d(std::string signalID, const frc::Pose3d &value, uint64_t timestamp)
{
    char valBuf[160];
    snprintf(valBuf, sizeof(valBuf), "%.6g;%.6g;%.6g;%.6g;%.6g;%.6g;%.6g",
             value.X().value(), value.Y().value(), value.Z().value(),
             value.Rotation().GetQuaternion().W(),
             value.Rotation().GetQuaternion().X(),
             value.Rotation().GetQuaternion().Y(),
             value.Rotation().GetQuaternion().Z());
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "pose3d", valBuf, "X_m;Y_m;Z_m;QW;QX;QY;QZ", timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteChassisSpeeds(std::string signalID, const frc::ChassisSpeeds &value, uint64_t timestamp)
{
    char valBuf[96];
    snprintf(valBuf, sizeof(valBuf), "%.6g;%.6g;%.6g",
             value.vx.value(), value.vy.value(), value.omega.value());
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "chassis_speeds", valBuf, "Vx_mps;Vy_mps;Omega_radps", timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteSwerveModuleState(std::string signalID, const frc::SwerveModuleState &value, uint64_t timestamp)
{
    char valBuf[64];
    snprintf(valBuf, sizeof(valBuf), "%.6g;%.6g", value.speed.value(), value.angle.Radians().value());
    char buf[k_bufSize];
    int len = FormatMessage(buf, k_bufSize, signalID.c_str(), "swerve_module_state", valBuf, "Speed_mps;Angle_rad", timestamp);
    SendData(buf, len);
}

void UDPSignalLogger::WriteGamePadState(std::string signalID, const std::array<double, 6> axes, const std::array<bool, 10> buttons, const std::array<int, 1> povs, uint64_t timestamp)
{
    char valBuf[128];
    char buf[k_bufSize];
    int pos, remaining, written, len;

    // Log axes
    pos = 0;
    remaining = (int)sizeof(valBuf);
    for (size_t i = 0; i < axes.size() && remaining > 1; ++i)
    {
        written = snprintf(valBuf + pos, remaining, i ? ";%.6g" : "%.6g", axes[i]);
        if (written < 0 || written >= remaining)
            break;
        pos += written;
        remaining -= written;
    }
    std::string axesID = signalID + "/axes";
    len = FormatMessage(buf, k_bufSize, axesID.c_str(), "float_array", valBuf, "", timestamp);
    SendData(buf, len);

    // Log buttons
    pos = 0;
    remaining = (int)sizeof(valBuf);
    for (size_t i = 0; i < buttons.size() && remaining > 1; ++i)
    {
        written = snprintf(valBuf + pos, remaining, i ? ";%d" : "%d", buttons[i] ? 1 : 0);
        if (written < 0 || written >= remaining)
            break;
        pos += written;
        remaining -= written;
    }
    std::string buttonsID = signalID + "/buttons";
    len = FormatMessage(buf, k_bufSize, buttonsID.c_str(), "bool_array", valBuf, "", timestamp);
    SendData(buf, len);

    // Log POVs
    pos = 0;
    remaining = (int)sizeof(valBuf);
    for (size_t i = 0; i < povs.size() && remaining > 1; ++i)
    {
        written = snprintf(valBuf + pos, remaining, i ? ";%d" : "%d", povs[i]);
        if (written < 0 || written >= remaining)
            break;
        pos += written;
        remaining -= written;
    }
    std::string povsID = signalID + "/povs";
    len = FormatMessage(buf, k_bufSize, povsID.c_str(), "int_array", valBuf, "", timestamp);
    SendData(buf, len);
}