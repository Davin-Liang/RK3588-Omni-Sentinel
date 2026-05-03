#include <iostream>
#include <string>
#include <thread>
#include <chrono> // 新增：用于时间计算
#include <iomanip> // 新增：用于格式化输出
#include "sentinel-visioner.h"

// 假设这是你的 YOLO 推理业务线程
void consumer_thread_func(SentinelVisioner* visioner, int camNum) {
    std::cout << "Consumer thread started for Camera " << camNum << std::endl;
    bool is_running = true;

    // --- FPS 统计相关的变量 ---
    int frame_count = 0;
    // 使用 steady_clock 保证不受系统时间被修改的影响
    auto start_time = std::chrono::steady_clock::now(); 

    while (is_running) {
        // 1. 阻塞等待：从安全队列获取经过 RGA 硬件处理好的 RGB888 DMA 内存块
        DmaBuffer_t* readyBuffer = visioner->wait_get_rga_buffer(camNum);

        if (readyBuffer != nullptr) {
            
            // --- FPS 计算逻辑 ---
            frame_count++; // 收到一帧，计数器 +1
            
            auto current_time = std::chrono::steady_clock::now();
            // 计算距离上次打印经过了多少毫秒
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - start_time).count();

            // 如果经过的时间大于等于 1000 毫秒（1秒），则计算并打印 FPS
            if (elapsed_ms >= 1000) {
                // 为了严谨，考虑到可能不是精确的 1000ms 触发，用实际经过的时间来除
                double fps = frame_count * 1000.0 / elapsed_ms;
                
                std::cout << "[Camera " << camNum << "] " 
                          << "FPS: " << std::fixed << std::setprecision(2) << fps 
                          << " | 耗时: " << elapsed_ms << " ms | 获取帧数: " << frame_count 
                          << std::endl;

                // 重置计数器和时间基准，开始下一秒的统计
                frame_count = 0;
                start_time = current_time;
            }


            // 3. 【极度重要】：业务使用完毕后，必须将其归还给内存池！
            visioner->release_rga_buffer(camNum, readyBuffer);
        }
    }
}

int main() {
    SentinelVisioner visioner;

    std::string devName = "/dev/video11";
    int camNum = 0;
    
    // 1. 注册并添加摄像头
    if (!visioner.add_camera(devName, 1920, 1080, 8, camNum)) {
        std::cerr << "Failed to add camera!" << std::endl;
        return -1;
    }

    // 2. 开启视频流
    if (!visioner.camera_stream_ctrl(camNum, true)) {
        std::cerr << "Failed to start camera stream!" << std::endl;
        return -1;
    }

    // 3. 拉起下游的消费者线程
    std::thread consumer(consumer_thread_func, &visioner, camNum);

    // 主线程保持运行...
    std::this_thread::sleep_for(std::chrono::seconds(60));

    // 4. 关闭系统
    std::cout << "Shutting down..." << std::endl;
    
    visioner.camera_stream_ctrl(camNum, false);
    
    // 注意：在实际严谨的代码中，这里应该有一个机制去优雅地通知 consumer 线程退出 (比如 is_running = false)
    // 否则如果没数据来了，wait_get_rga_buffer 会一直阻塞导致 join() 卡死。
    // 但作为测试 Demo，这里保持你的原样。
    consumer.join();

    return 0;
}