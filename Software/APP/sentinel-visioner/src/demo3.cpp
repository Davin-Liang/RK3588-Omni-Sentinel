#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include <vector>
#include "sentinel-visioner.h"

std::atomic<bool> g_is_running(true);

void npu_osd_consumer_thread(SentinelVisioner* visioner, int camNum, const char* label) {
    std::cout << "[NPU Thread] Started for Camera " << camNum << " (" << label
              << ") - Waiting for data..." << std::endl;

    int total_frame_count = 0;
    int fps_frame_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    while (g_is_running) {
        DmaBuffer_t* npuBuf = visioner->wait_get_npu(camNum);

        if (npuBuf != nullptr) {
            total_frame_count++;
            fps_frame_count++;

            if (total_frame_count % 30 == 0) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    current_time - start_time).count();

                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t current_sys_us = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
                uint64_t latency_ms = (current_sys_us - npuBuf->timestampUs) / 1000;

                double fps = fps_frame_count * 1000.0 / elapsed_ms;

                std::cout << "\033[1;32m[NPU Cam " << camNum << " " << label << "]\033[0m "
                          << "Total: " << total_frame_count << " frames | "
                          << "FPS: " << std::fixed << std::setprecision(2) << fps
                          << " | Latency: " << latency_ms << " ms" << std::endl;

                fps_frame_count = 0;
                start_time = current_time;
            }

            visioner->release_npu(camNum, npuBuf);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[NPU Thread] Camera " << camNum << " exited cleanly." << std::endl;
}

void stream_consumer_thread(SentinelVisioner* visioner, int camNum, const char* label) {
    std::cout << "[Stream Thread] Started for Camera " << camNum << " (" << label
              << ") - Waiting for data..." << std::endl;

    int total_stream_frames = 0;

    while (g_is_running) {
        DmaBuffer_t* origImage = visioner->wait_get_orig_copy_buffer(camNum);

        if (origImage != nullptr) {
            total_stream_frames++;

            if (total_stream_frames % 30 == 0) {
                std::cout << "\033[1;34m[Stream Cam " << camNum << " " << label << "]\033[0m "
                          << "Processed " << total_stream_frames
                          << " frames. Latest TS: " << origImage->timestampUs << " us" << std::endl;
            }

            visioner->release_orig_copy_buffer(camNum, origImage);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[Stream Thread] Camera " << camNum << " exited cleanly." << std::endl;
}

int main(int argc, char* argv[]) {
    SentinelVisioner visioner;

    // ISP 相机参数
    std::string ispDev    = "/dev/video11";
    int ispWidth          = 1920;
    int ispHeight         = 1080;
    int ispCamNum         = 0;

    // USB 相机参数
    std::string usbDev    = "/dev/video21";
    int usbWidth          = 640;
    int usbHeight         = 480;
    int usbCamNum         = 1;

    int runSeconds = 30;
    bool enableVisualEis = false;

    // 支持命令行覆盖
    if (argc > 1) ispDev    = argv[1];
    if (argc > 2) usbDev    = argv[2];
    if (argc > 3) runSeconds = atoi(argv[3]);
    if (argc > 4) enableVisualEis = (atoi(argv[4]) != 0);

    std::cout << "========================================" << std::endl;
    std::cout << "Dual Camera Test" << std::endl;
    std::cout << "  ISP Cam : " << ispDev << " " << ispWidth << "x" << ispHeight
              << " (camNum=" << ispCamNum << ")" << std::endl;
    std::cout << "  USB Cam : " << usbDev << " " << usbWidth << "x" << usbHeight
              << " (camNum=" << usbCamNum << ")" << std::endl;
    std::cout << "  Runtime : " << runSeconds << "s" << std::endl;
    std::cout << "  VisualEIS: " << (enableVisualEis ? "ON" : "OFF") << std::endl;
    std::cout << "========================================" << std::endl;

    // 1. 添加 ISP 相机
    if (!visioner.add_camera(ispDev, ispWidth, ispHeight, 8, ispCamNum, CameraType::ISP_CAM)) {
        std::cerr << "Failed to add ISP camera!" << std::endl;
        return -1;
    }

    // 2. 添加 USB 相机
    if (!visioner.add_camera(usbDev, usbWidth, usbHeight, 8, usbCamNum, CameraType::USB_CAM)) {
        std::cerr << "Failed to add USB camera!" << std::endl;
        return -1;
    }

    // 可选：两路相机分别启用视觉为主 EIS。
    // 注意两个相机必须各自维护独立 VisionEisStabilizer，不能共用轨迹状态。
    if (enableVisualEis) {
        VisionEisConfig ispCfg;
        ispCfg.camId = ispCamNum;
        ispCfg.inputWidth = ispWidth;
        ispCfg.inputHeight = ispHeight;
        ispCfg.processWidth = 640;
        ispCfg.processHeight = 360;
        ispCfg.maxOffsetPixel = 80;
        ispCfg.enableImuAdaptiveAlpha = false;
        visioner.set_visual_eis_config(ispCamNum, ispCfg);
        visioner.enable_visual_eis(ispCamNum, true);

        VisionEisConfig usbCfg;
        usbCfg.camId = usbCamNum;
        usbCfg.inputWidth = usbWidth;
        usbCfg.inputHeight = usbHeight;
        usbCfg.processWidth = 480;
        usbCfg.processHeight = 360;
        usbCfg.maxOffsetPixel = 60;
        usbCfg.enableImuAdaptiveAlpha = false;
        visioner.set_visual_eis_config(usbCamNum, usbCfg);
        visioner.enable_visual_eis(usbCamNum, true);
    }

    // 3. 开启两路视频流
    if (!visioner.camera_stream_ctrl(ispCamNum, true)) {
        std::cerr << "Failed to start ISP camera stream!" << std::endl;
        return -1;
    }
    if (!visioner.camera_stream_ctrl(usbCamNum, true)) {
        std::cerr << "Failed to start USB camera stream!" << std::endl;
        visioner.camera_stream_ctrl(ispCamNum, false);
        return -1;
    }

    // 4. 启动消费者线程（每路两个：NPU + Stream，共 4 个线程）
    std::vector<std::thread> consumerThreads;

    consumerThreads.emplace_back(npu_osd_consumer_thread, &visioner, ispCamNum, "ISP");
    consumerThreads.emplace_back(stream_consumer_thread, &visioner, ispCamNum, "ISP");
    consumerThreads.emplace_back(npu_osd_consumer_thread, &visioner, usbCamNum, "USB");
    consumerThreads.emplace_back(stream_consumer_thread, &visioner, usbCamNum, "USB");

    std::cout << "\nAll cameras and consumers started." << std::endl;
    std::cout << "Running " << runSeconds << " seconds..."
              << " Press Ctrl+C to stop earlier." << std::endl;

    // 5. 主线程等待
    for (int i = 0; i < runSeconds; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 6. 关闭流程：先停捕获线程，再通知消费者退出
    std::cout << "\nShutting down..." << std::endl;

    visioner.camera_stream_ctrl(ispCamNum, false);
    visioner.camera_stream_ctrl(usbCamNum, false);

    g_is_running = false;

    // 唤醒所有可能阻塞在队列上的消费者线程
    for (int camNum : {ispCamNum, usbCamNum}) {
        visioner.release_npu(camNum, nullptr);
        visioner.release_preview(camNum, nullptr);
        visioner.release_orig_copy_buffer(camNum, nullptr);
    }

    // 等待所有消费者线程退出
    for (auto& t : consumerThreads) {
        if (t.joinable()) t.join();
    }

    std::cout << "Dual-camera test finished successfully." << std::endl;
    return 0;
}
