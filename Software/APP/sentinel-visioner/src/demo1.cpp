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
// 消费者线程 1：负责 NPU 推理 与 OSD 画框 (消费 npuTaskQueue)
// ============================================================================
void npu_osd_consumer_thread(SentinelVisioner* visioner, int camNum) {
    std::cout << "[NPU Thread] Started for Camera " << camNum << " - Waiting for data..." << std::endl;

    int total_frame_count = 0;
    int fps_frame_count = 0;
    auto start_time = std::chrono::steady_clock::now(); 

    while (g_is_running) {
        // 1. 阻塞等待：获取 NPU 专用小图和 OSD 720P 底图
        NpuOSD task = visioner->wait_get_npuOSD(camNum);

        if (task.npuImage != nullptr) {
            total_frame_count++;
            fps_frame_count++;
            
            // --- 每隔 30 帧 (约 1 秒) 打印一次心跳和详细状态 ---
            if (total_frame_count % 30 == 0) {
                auto current_time = std::chrono::steady_clock::now();
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

                // 测算端到端延迟 (当前系统时间 - 底层驱动打上的硬件时间戳)
                struct timespec ts;
                clock_gettime(CLOCK_MONOTONIC, &ts);
                uint64_t current_sys_us = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
                uint64_t latency_ms = (current_sys_us - task.npuImage->timestampUs) / 1000;

                double fps = fps_frame_count * 1000.0 / elapsed_ms;
                
                // 使用绿色字体打印 NPU 线程状态
                std::cout << "\033[1;32m[NPU Pipeline Cam " << camNum << "]\033[0m " 
                          << "Total: " << total_frame_count << " frames | "
                          << "FPS: " << std::fixed << std::setprecision(2) << fps 
                          << " | Latency: " << latency_ms << " ms" << std::endl;

                fps_frame_count = 0;
                start_time = current_time;
            }

            // 2. 模拟 NPU 推理 (使用 task.npuImage)
            // auto results = yolo_infer(task.npuImage->dmaFd);

            // 3. OSD 绘制
            if (task.osdImage != nullptr) {
                // 模拟根据 NPU 结果在 720P 图像上画框
                // imfill(task.osdImage, ...);
            }

            // 4. 【极度重要】：用完之后释放结构体中的所有 DMA 内存
            visioner->release_npuOSD(camNum, &task);
        } else {
            // 如果拿到 nullptr，说明可能是由于唤醒或退出，稍微休眠防止 CPU 空转
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }
    std::cout << "[NPU Thread] Exited cleanly." << std::endl;
}

// ============================================================================
// 消费者线程 2：负责原始 1080P 图像推流或录像 (消费 processTaskQueue)
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
int main() {
    SentinelVisioner visioner;

    std::string devName = "/dev/video11"; // 请替换为你实际的摄像头节点
    int camNum = 0;
    
    // 1. 注册并添加摄像头 (移除了 enableOsd 参数，恢复 5 个参数的调用)
    if (!visioner.add_camera(devName, 1920, 1080, 8, camNum)) {
        std::cerr << "Failed to add camera!" << std::endl;
        return -1;
    }

    // 2. 开启视频流（此时内部的捕获线程会启动，开始抓图并压入队列）
    if (!visioner.camera_stream_ctrl(camNum, true)) {
        std::cerr << "Failed to start camera stream!" << std::endl;
        return -1;
    }

    // 3. 拉起下游的两个消费者线程
    std::thread npu_thread(npu_osd_consumer_thread, &visioner, camNum);
    std::thread stream_thread(stream_consumer_thread, &visioner, camNum);

    // 主线程保持运行 60 秒以进行观察...
    std::cout << "System running... Press Ctrl+C to stop (or wait 60s)." << std::endl;
    for (int i = 0; i < 60; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // 4. 优雅关闭系统
    std::cout << "\nShutting down..." << std::endl;
    
    // a. 停止底层的 V4L2 采集和 RGA 捕获线程
    visioner.camera_stream_ctrl(camNum, false);
    
    // b. 通知用户态的消费者线程退出
    g_is_running = false; 

    // c. 唤醒可能卡在 wait_get_xxx 的队列
    NpuOSD dummy_task = {nullptr, nullptr};
    visioner.release_npuOSD(camNum, &dummy_task); 
    visioner.release_orig_copy_buffer(camNum, nullptr);

    // d. 回收线程
    if (npu_thread.joinable()) npu_thread.join();
    if (stream_thread.joinable()) stream_thread.join();

    std::cout << "System successfully shut down." << std::endl;
    return 0;
}