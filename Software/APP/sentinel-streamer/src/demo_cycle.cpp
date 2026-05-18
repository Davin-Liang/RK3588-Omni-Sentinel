/**
 * @brief Demo — 反复启停推流/录像，每次录像不同文件名
 */

#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <csignal>
#include <string>
#include <execinfo.h>
#include <unistd.h>

static void crashHandler(int sig, siginfo_t* info, void* /*ucontext*/)
{
    fprintf(stderr, "\n[SIGSEGV] si_addr=%p si_code=%d\n", info->si_addr, info->si_code);
    void* bt[32];
    int n = backtrace(bt, 32);
    backtrace_symbols_fd(bt, n, STDERR_FILENO);
    _exit(1);
}

#include "sentinel-visioner.h"
#include "sentinel_streamer.h"

static std::atomic<bool> gRunning(true);

static void signalHandler(int /*sig*/)
{
    gRunning.store(false);
}

int main(int argc, char* argv[])
{
    const char* device   = (argc > 1) ? argv[1] : "/dev/video11";
    const char* rtspUrl  = (argc > 2) ? argv[2] : "rtsp://127.0.0.1:8554/live/cam0";
    int cycles           = (argc > 3) ? atoi(argv[3]) : 3;

    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    struct sigaction sa;
    sa.sa_sigaction = crashHandler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = SA_SIGINFO;
    sigaction(SIGSEGV, &sa, nullptr);

    // ---- 初始化 ----
    SentinelVisioner visioner;
    std::string devStr(device);
    if (!visioner.add_camera(devStr, 1920, 1080, 8, 0)) {
        fprintf(stderr, "[DemoCycle] visioner add_camera failed\n");
        return 1;
    }
    if (!visioner.camera_stream_ctrl(0, true)) {
        fprintf(stderr, "[DemoCycle] visioner camera_stream_ctrl failed\n");
        return 1;
    }

    SentinelStreamer streamer;
    if (!streamer.add_camera(0, &visioner)) {
        fprintf(stderr, "[DemoCycle] streamer add_camera failed\n");
        visioner.camera_stream_ctrl(0, false);
        return 1;
    }

    // ---- 循环启停 ----
    for (int i = 1; i <= cycles && gRunning.load(); ++i) {
        std::string mp4Path = "/tmp/stream_record_" + std::to_string(i) + ".mp4";

        fprintf(stderr, "[DemoCycle] ====== Cycle %d/%d: start ======\n", i, cycles);

        // 启动推流
        if (!streamer.start_stream(0, rtspUrl)) {
            fprintf(stderr, "[DemoCycle] start_stream failed at cycle %d\n", i);
            break;
        }

        // 启动录像
        if (!streamer.start_record(0, mp4Path.c_str(), RecordResolution::RES_1080P)) {
            fprintf(stderr, "[DemoCycle] start_record failed at cycle %d\n", i);
            streamer.stop_stream(0);
            break;
        }

        // 录制 8 秒
        for (int s = 0; s < 8 && gRunning.load(); ++s) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }

        // 停止录像
        fprintf(stderr, "[DemoCycle] stopping record...\n");
        streamer.stop_record(0);

        // 停止推流
        fprintf(stderr, "[DemoCycle] stopping stream...\n");
        streamer.stop_stream(0);

        fprintf(stderr, "[DemoCycle] ====== Cycle %d/%d: done → %s ======\n",
                i, cycles, mp4Path.c_str());

        if (i < cycles) {
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    }

    // ---- 清理 ----
    streamer.remove_camera(0);
    visioner.camera_stream_ctrl(0, false);

    fprintf(stderr, "[DemoCycle] All done.\n");
    return 0;
}
