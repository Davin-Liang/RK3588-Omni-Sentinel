#include "voice_warning.h"

#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

constexpr const char* kVoiceDevice = "/dev/ttyUSB0";

constexpr std::uint8_t kDangerFrame[5] = {
    0xAA,
    0x55,
    0x08,
    0x1E,
    0xFB
};

bool configureSerial(int fd)
{
    struct termios tty {};

    if (tcgetattr(fd, &tty) != 0) {
        std::fprintf(
            stderr,
            "[Voice] tcgetattr failed: %s\n",
            std::strerror(errno)
        );
        return false;
    }

    cfmakeraw(&tty);

    // 9600波特率
    if (cfsetispeed(&tty, B9600) != 0 ||
        cfsetospeed(&tty, B9600) != 0) {
        std::fprintf(
            stderr,
            "[Voice] set baud rate failed: %s\n",
            std::strerror(errno)
        );
        return false;
    }

    // 8位数据位
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;

    // 无校验
    tty.c_cflag &= ~PARENB;

    // 1位停止位
    tty.c_cflag &= ~CSTOPB;

#ifdef CRTSCTS
    // 关闭硬件流控
    tty.c_cflag &= ~CRTSCTS;
#endif

    // 关闭软件流控
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_cflag |= CLOCAL | CREAD;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        std::fprintf(
            stderr,
            "[Voice] tcsetattr failed: %s\n",
            std::strerror(errno)
        );
        return false;
    }

    tcflush(fd, TCIOFLUSH);
    return true;
}

int openVoiceDevice()
{
    const int fd = open(
        kVoiceDevice,
        O_RDWR | O_NOCTTY | O_CLOEXEC
    );

    if (fd < 0) {
        std::fprintf(
            stderr,
            "[Voice] open %s failed: %s\n",
            kVoiceDevice,
            std::strerror(errno)
        );
        return -1;
    }

    if (!configureSerial(fd)) {
        close(fd);
        return -1;
    }

    std::printf(
        "[Voice] initialized: %s, 9600 8N1\n",
        kVoiceDevice
    );

    return fd;
}

bool sendFrame(int fd)
{
    std::size_t sent = 0;

    while (sent < sizeof(kDangerFrame)) {
        const ssize_t result = write(
            fd,
            kDangerFrame + sent,
            sizeof(kDangerFrame) - sent
        );

        if (result > 0) {
            sent += static_cast<std::size_t>(result);
            continue;
        }

        if (result < 0 && errno == EINTR) {
            continue;
        }

        std::fprintf(
            stderr,
            "[Voice] write failed: %s\n",
            std::strerror(errno)
        );
        return false;
    }

    return tcdrain(fd) == 0;
}

}  // namespace

namespace voice {

bool playDangerWarning()
{
    // 第一次调用时打开，之后一直复用。
    // 不创建任何独立线程。
    static int fd = -1;

    if (fd < 0) {
        fd = openVoiceDevice();

        if (fd < 0) {
            return false;
        }
    }

    if (!sendFrame(fd)) {
        std::fprintf(
            stderr,
            "[Voice] danger warning send failed\n"
        );

        close(fd);
        fd = -1;
        return false;
    }

    std::printf(
        "[Voice] sent: AA 55 08 1E FB\n"
    );

    return true;
}

}  // namespace voice
