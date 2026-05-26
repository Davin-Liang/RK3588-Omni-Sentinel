#include "sentinel_lslidarer.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <climits>
#include <fcntl.h>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>

// ============================================================================
// SerialPort
// ============================================================================

SerialPort::SerialPort() : fd_(-1) {}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudRate) {
    // 阻塞模式：不用 O_NONBLOCK，不用 poll，和 cat 读串口一样
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
        std::fprintf(stderr, "[SerialPort] Failed to open %s: %s\n",
                     port.c_str(), std::strerror(errno));
        return false;
    }

    // ttyACM* (CDC-ACM USB 虚拟串口) 的波特率是虚拟值，tcsetattr 可能破坏
    // CDC 协议握手。对 ACM 设备直接跳过所有 termios 配置，仅使用内核默认设置。
    char resolved[PATH_MAX];
    bool isAcm = false;
    if (realpath(port.c_str(), resolved)) {
        std::string realPort(resolved);
        isAcm = (realPort.find("ACM") != std::string::npos) ||
                (realPort.find("acm") != std::string::npos);
    } else {
        isAcm = (port.find("ACM") != std::string::npos);
    }

    struct termios tio;
    tcgetattr(fd_, &tio);
    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD | CS8;
    tio.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tio.c_cc[VTIME] = 5;
    tio.c_cc[VMIN]  = 0;

    speed_t speed;
    switch (baudRate) {
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 500000: speed = B500000; break;
    case 921600: speed = B921600; break;
    default:
        std::fprintf(stderr, "[SerialPort] Unsupported baud rate: %d\n", baudRate);
        close();
        return false;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tcflush(fd_, TCIOFLUSH);
    tcsetattr(fd_, TCSANOW, &tio);

    std::fprintf(stderr, "[SerialPort] Opened %s fd=%d baud=%d (%s)\n",
                 port.c_str(), fd_, baudRate, isAcm ? "ACM" : "UART");
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

int SerialPort::read_packet(uint8_t* buffer, int& packetLen) {
    if (fd_ < 0) return -1;

    uint8_t chunk[512];
    ssize_t n = ::read(fd_, chunk, sizeof(chunk));
    if (n <= 0) return (n < 0 && errno != EINTR) ? -1 : 0;

    // 在内存中扫描 0xA5 0x5A 包头
    // N10Plus: 包长固定 108 字节，CRC 校验在 check_packet_validity_ 中完成
    for (int i = 0; i < static_cast<int>(n) - 3; ++i) {
        if (chunk[i] != 0xA5 || chunk[i + 1] != 0x5A) continue;

        // N10Plus 固定包长 108，不解析长度字段
        int pktLen = LidarConfig::kPacketLength;

        if (i + pktLen > static_cast<int>(n)) return 0;

        std::memcpy(buffer, chunk + i, pktLen);
        packetLen = pktLen;
        return pktLen;
    }

    return 0;
}
