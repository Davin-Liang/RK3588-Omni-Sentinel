#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <atomic>
#include "sentinel-visioner.h"

// 全局运行标志位，用于优雅退出所有消费者线程
std::atomic<bool> g_is_running(true);

// ============================================================================
// 消费者线程 1：负责 NPU 推理 (消费 npuTaskQueue)
// ============================================================================
void npu_consumer_thread(SentinelVisioner* visioner, int camNum) {
    std::cout << "[NPU Thread] Started for Camera " << camNum << " - Waiting for data..." << std::endl;

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
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t current_sys_us = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
                uint64_t latency_ms = (current_sys_us - npuBuf->timestampUs) / 1000;

                double fps = fps_frame_count * 1000.0 / elapsed_ms;

                std::cout << "\033[1;32m[NPU Pipeline Cam " << camNum << "]\033[0m "
                          << "Total: " << total_frame_count << " frames | "
                          << "FPS: " << std::fixed << std::setprecision(2) << fps
                          << " | Latency: " << latency_ms << " ms" << std::endl;

                fps_frame_count = 0;
                start_time = current_time;
            }

            // 模拟 NPU 推理
            // auto results = yolo_infer(npuBuf->dmaFd);

            visioner->release_npu(camNum, npuBuf);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[NPU Thread] Exited cleanly." << std::endl;
}

// ============================================================================
// 消费者线程 2：负责预览显示 (消费 previewTaskQueue)
// ============================================================================
void preview_consumer_thread(SentinelVisioner* visioner, int camNum) {
    std::cout << "[Preview Thread] Started for Camera " << camNum << " - Waiting for data..." << std::endl;

    int frame_count = 0;
    while (g_is_running) {
        DmaBuffer_t* previewBuf = visioner->try_get_preview(camNum, 200);

        if (previewBuf != nullptr) {
            frame_count++;
            // 模拟 QT 界面使用 1080P RGB888 图像渲染
            // qt_render(previewBuf->virtAddr);
            visioner->release_preview(camNum, previewBuf);
        }
    }
    std::cout << "[Preview Thread] Exited cleanly. (" << frame_count << " frames)" << std::endl;
}

// ============================================================================
// 消费者线程 3：负责原始 1080P 图像推流或录像 (消费 processTaskQueue)
// ============================================================================
void stream_consumer_thread(SentinelVisioner* visioner, int camNum) {
    std::cout << "[Stream Thread] Started for Camera " << camNum << " - Waiting for data..." << std::endl;

    int total_stream_frames = 0;

    while (g_is_running) {
        // 1. 阻塞等待：获取纯净的 1080P NV12 原始拷贝
        DmaBuffer_t* origImage = visioner->wait_get_orig_copy_buffer(camNum);

        if (origImage != nullptr) {
            total_stream_frames++;

            // --- 每隔 30 帧打印一次心跳，证明推流队列正在稳定消费 ---
            if (total_stream_frames % 30 == 0) {
                // 使用蓝色字体打印推流线程状态
                std::cout << "\033[1;34m[Stream Pipeline Cam " << camNum << "]\033[0m "
                          << "Successfully processed " << total_stream_frames 
                          << " frames. Latest TS: " << origImage->timestampUs << " us" << std::endl;
            }

            // 2. 模拟将其送给 MPP 硬件编码器进行推流或保存录像
            // mpp_encode_push(origImage->dmaFd, origImage->timestampUs);

            // 3. 【极度重要】：推流编码完成后，归还原始大图内存块
            visioner->release_orig_copy_buffer(camNum, origImage);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[Stream Thread] Exited cleanly." << std::endl;
}

// ============================================================================
// 主函数入口
// ============================================================================
int main(int argc, char* argv[]) {
    SentinelVisioner visioner;

    std::string devName = (argc > 1) ? argv[1] : "/dev/video11";
    int runSeconds      = (argc > 2) ? atoi(argv[2]) : 60;
    bool enableVisualEis = (argc > 3) ? (atoi(argv[3]) != 0) : false;
    int camNum = 0;

    // 1. 注册并添加摄像头
    if (!visioner.add_camera(devName, 1920, 1080, 8, camNum)) {
        std::cerr << "Failed to add camera!" << std::endl;
        return -1;
    }

    // 可选：开启“视觉为主 + IMU辅助”EIS。
    // 当前 demo1 不直接依赖 ICM45686，因此没有设置 IMU 回调；视觉 EIS 会独立运行，
    // 后续在完整系统中可通过 set_imu_assist_callback() 接入 ICM45686 的 gyro_rms/vibration_level。
    if (enableVisualEis) {
        VisionEisConfig eisCfg;
        eisCfg.camId = camNum;
        eisCfg.inputWidth = 1920;
        eisCfg.inputHeight = 1080;
        eisCfg.processWidth = 640;
        eisCfg.processHeight = 360;
        eisCfg.maxOffsetPixel = 80;
        eisCfg.alphaLowVibration = 0.30f;
        eisCfg.alphaMidVibration = 0.20f;
        eisCfg.alphaHighVibration = 0.12f;
        eisCfg.enableImuAdaptiveAlpha = false;
        visioner.set_visual_eis_config(camNum, eisCfg);
        visioner.enable_visual_eis(camNum, true);
    }

    // 2. 开启视频流
    if (!visioner.camera_stream_ctrl(camNum, true)) {
        std::cerr << "Failed to start camera stream!" << std::endl;
        return -1;
    }

    // 3. 拉起下游的消费者线程（NPU + 预览 + 推流）
    std::thread npu_thread(npu_consumer_thread, &visioner, camNum);
    std::thread preview_thread(preview_consumer_thread, &visioner, camNum);
    std::thread stream_thread(stream_consumer_thread, &visioner, camNum);

    // 主线程保持运行
    std::cout << "System running... Press Ctrl+C to stop (or wait " << runSeconds
              << "s). VisualEIS=" << (enableVisualEis ? "ON" : "OFF") << std::endl;
    for (int i = 0; i < runSeconds; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 4. 优雅关闭系统
    std::cout << "\nShutting down..." << std::endl;

    // a. 停止底层的 V4L2 采集和 RGA 捕获线程
    visioner.camera_stream_ctrl(camNum, false);

    // b. 通知用户态的消费者线程退出
    g_is_running = false;

    // c. 释放空指针（触发队列条件变量唤醒）
    visioner.release_npu(camNum, nullptr);
    visioner.release_preview(camNum, nullptr);
    visioner.release_orig_copy_buffer(camNum, nullptr);

    // d. 回收线程
    if (npu_thread.joinable()) npu_thread.join();
    if (preview_thread.joinable()) preview_thread.join();
    if (stream_thread.joinable()) stream_thread.join();

    std::cout << "System successfully shut down." << std::endl;
    return 0;
}