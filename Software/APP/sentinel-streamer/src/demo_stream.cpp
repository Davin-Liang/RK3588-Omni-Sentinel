/**
 * @brief Demo — SentinelStreamer 推流与录像使用示例
 */

#include <cstdio>
#include <cstdlib>
#include <thread>
#include <chrono>
#include <csignal>

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
    const char* mp4Path  = (argc > 3) ? argv[3] : "/tmp/stream_record.mp4";
    int runSeconds       = (argc > 4) ? atoi(argv[4]) : 30;

    signal(SIGINT,  signalHandler);
    signal(SIGTERM, signalHandler);

    fprintf(stderr, "[Demo] device=%s rtsp=%s mp4=%s duration=%ds\n",
            device, rtspUrl, mp4Path, runSeconds);

    // ---- 1. SentinelVisioner ----
    SentinelVisioner visioner;
    std::string devStr(device);
    if (!visioner.add_camera(devStr, 1920, 1080, 8, 0)) {
        fprintf(stderr, "[Demo] visioner add_camera failed\n");
        return 1;
    }
    if (!visioner.camera_stream_ctrl(0, true)) {
        fprintf(stderr, "[Demo] visioner camera_stream_ctrl failed\n");
        return 1;
    }

    // ---- 2. SentinelStreamer ----
    SentinelStreamer streamer;
    if (!streamer.add_camera(0, &visioner)) {
        fprintf(stderr, "[Demo] streamer add_camera failed\n");
        visioner.camera_stream_ctrl(0, false);
        return 1;
    }

    // ---- 3. 720p RTSP 推流 ----
    if (!streamer.start_stream(0, rtspUrl)) {
        fprintf(stderr, "[Demo] start_stream failed\n");
        streamer.remove_camera(0);
        visioner.camera_stream_ctrl(0, false);
        return 1;
    }

    // ---- 4. 720p MP4 录像 ----
    if (!streamer.start_record(0, mp4Path, RecordResolution::RES_1080P)) {
        fprintf(stderr, "[Demo] start_record failed\n");
        streamer.stop_stream(0);
        streamer.remove_camera(0);
        visioner.camera_stream_ctrl(0, false);
        return 1;
    }

    fprintf(stderr, "[Demo] Stream + Record running %ds...\n", runSeconds);
    while (gRunning.load() && runSeconds > 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        --runSeconds;
    }

    fprintf(stderr, "[Demo] Stopping record...\n");
    streamer.stop_record(0);

    fprintf(stderr, "[Demo] Stopping stream...\n");
    streamer.stop_stream(0);

    streamer.remove_camera(0);
    visioner.camera_stream_ctrl(0, false);

    fprintf(stderr, "[Demo] Done.\n");
    return 0;
}
