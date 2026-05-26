#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include "sentinel-visioner.h"

std::atomic<bool> g_is_running(true);

void npu_osd_consumer_thread(SentinelVisioner* visioner, int camNum) {
    std::cout << "[NPU Thread] Started for Camera " << camNum << " - Waiting for data..." << std::endl;

    int total_frame_count = 0;
    int fps_frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (g_is_running) {
        NpuPreview task = visioner->wait_get_preview(camNum);

        if (task.npuImage != nullptr) {
            total_frame_count++;
            fps_frame_count++;

            if (total_frame_count % 30 == 0) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - start_time).count();

                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t current_sys_us = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
                uint64_t latency_ms = (current_sys_us - task.npuImage->timestampUs) / 1000;

                double fps = fps_frame_count * 1000.0 / elapsed_ms;

                std::cout << "\033[1;32m[NPU Pipeline Cam " << camNum << "]\033[0m "
                          << "Total: " << total_frame_count << " frames | "
                          << "FPS: " << std::fixed << std::setprecision(2) << fps
                          << " | Latency: " << latency_ms << " ms" << std::endl;

                fps_frame_count = 0;
                start_time = current_time;
            }

            visioner->release_preview(camNum, &task);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[NPU Thread] Exited cleanly." << std::endl;
}

void stream_consumer_thread(SentinelVisioner* visioner, int camNum) {
    std::cout << "[Stream Thread] Started for Camera " << camNum << " - Waiting for data..."
              << std::endl;

    int total_stream_frames = 0;

    while (g_is_running) {
        DmaBuffer_t* origImage = visioner->wait_get_orig_copy_buffer(camNum);

        if (origImage != nullptr) {
            total_stream_frames++;

            if (total_stream_frames % 30 == 0) {
                std::cout << "\033[1;34m[Stream Pipeline Cam " << camNum << "]\033[0m "
                          << "Successfully processed " << total_stream_frames
                          << " frames. Latest TS: " << origImage->timestampUs << " us" << std::endl;
            }

            visioner->release_orig_copy_buffer(camNum, origImage);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[Stream Thread] Exited cleanly." << std::endl;
}

int main(int argc, char* argv[]) {
    SentinelVisioner visioner;

    std::string devName = (argc > 1) ? argv[1] : "/dev/video0";
    int width           = (argc > 2) ? atoi(argv[2]) : 640;
    int height          = (argc > 3) ? atoi(argv[3]) : 480;
    int runSeconds      = (argc > 4) ? atoi(argv[4]) : 30;
    int camNum = 0;

    // 以 USB 摄像头类型注册
    if (!visioner.add_camera(devName, width, height, 8, camNum, CameraType::USB_CAM)) {
        std::cerr << "Failed to add USB camera!" << std::endl;
        return -1;
    }

    if (!visioner.camera_stream_ctrl(camNum, true)) {
        std::cerr << "Failed to start camera stream!" << std::endl;
        return -1;
    }

    std::thread npu_thread(npu_osd_consumer_thread, &visioner, camNum);
    std::thread stream_thread(stream_consumer_thread, &visioner, camNum);

    std::cout << "System running... Press Ctrl+C to stop (or wait " << runSeconds << "s)."
              << std::endl;
    for (int i = 0; i < runSeconds; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "\nShutting down..." << std::endl;

    visioner.camera_stream_ctrl(camNum, false);
    g_is_running = false;

    NpuPreview dummy_task = {nullptr, nullptr};
    visioner.release_preview(camNum, &dummy_task);
    visioner.release_orig_copy_buffer(camNum, nullptr);

    if (npu_thread.joinable()) npu_thread.join();
    if (stream_thread.joinable()) stream_thread.join();

    std::cout << "System successfully shut down." << std::endl;
    return 0;
}
