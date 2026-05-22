#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <iomanip>
#include <atomic>
#include <csignal>
#include "NVMeDataManager.h"

// ============================================================
//  Benchmark 配置
// ============================================================
// YUV422: 每像素2字节
static constexpr int CAMERA_WIDTH     = 1920;
static constexpr int CAMERA_HEIGHT    = 1080;
static constexpr size_t CAMERA_FRAME_SIZE = CAMERA_WIDTH * CAMERA_HEIGHT * 2;  // YUV422: 4,147,200 bytes
static constexpr int LIDAR_POINT_COUNT   = 4096;
static constexpr size_t LIDAR_FRAME_SIZE = LIDAR_POINT_COUNT * 16;            // 65,536 bytes
static constexpr size_t IMU_FRAME_SIZE   = 24;                                // 6 floats

// 测试持续时间（秒）— 60秒方便观察CPU占用率
static constexpr int TEST_DURATION_SEC = 60;

// 传感器频率
static constexpr int CAMERA_INTERVAL_MS = 67;   // ~15 FPS
static constexpr int LIDAR_INTERVAL_MS  = 100;  // 10 Hz
static constexpr int IMU_INTERVAL_MS    = 10;   // 100 Hz

// 信号处理
static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// ============================================================
//  Benchmark 主函数
// ============================================================
void run_benchmark() {
    NVMeDataManager nvme_manager;

    std::cout << "NVMe 写入 Benchmark (CPU 优化版)" << std::endl;
    std::cout << "================================" << std::endl;
    std::cout << "测试时长: " << TEST_DURATION_SEC << "s" << std::endl;
    std::cout << "数据策略: malloc固定缓存，重复写入，无CPU生成" << std::endl;
    std::cout << "图像格式: YUV422" << std::endl;
    std::cout << "摄像头: " << CAMERA_WIDTH << "x" << CAMERA_HEIGHT
              << " (" << (CAMERA_FRAME_SIZE / 1024 / 1024.0) << " MB/帧)"
              << " @ " << (1000 / CAMERA_INTERVAL_MS) << " FPS" << std::endl;
    std::cout << "激光雷达: " << LIDAR_POINT_COUNT << " 点/帧 ("
              << (LIDAR_FRAME_SIZE / 1024.0) << " KB/帧)"
              << " @ " << (1000 / LIDAR_INTERVAL_MS) << " Hz" << std::endl;
    std::cout << "IMU: " << (IMU_FRAME_SIZE) << " bytes/帧"
              << " @ " << (1000 / IMU_INTERVAL_MS) << " Hz" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // 初始化 NVMe 管理器
    if (!nvme_manager.initialize()) {
        std::cerr << "NVMe 初始化失败！" << std::endl;
        return;
    }
    std::cout << "NVMe 初始化成功" << std::endl;

    // ============================================================
    //  预分配固定测试数据（malloc一次，不走CPU计算生成）
    // ============================================================
    // 摄像头：YUV422 固定灰度值（所有像素 Y=128, U=128, V=128）
    // posix_memalign 分配 512B 对齐内存，满足 O_DIRECT 要求
    uint8_t* camera_data = nullptr;
    if (posix_memalign((void**)&camera_data, 512, CAMERA_FRAME_SIZE) != 0) {
        std::cerr << "camera_data posix_memalign 失败" << std::endl;
        return;
    }
    std::memset(camera_data, 128, CAMERA_FRAME_SIZE);

    // 激光雷达：固定零值
    uint8_t* lidar_data = nullptr;
    if (posix_memalign((void**)&lidar_data, 512, LIDAR_FRAME_SIZE) != 0) {
        std::cerr << "lidar_data posix_memalign 失败" << std::endl;
        free(camera_data);
        return;
    }
    std::memset(lidar_data, 0, LIDAR_FRAME_SIZE);

    // IMU：固定零值
    uint8_t* imu_data = nullptr;
    if (posix_memalign((void**)&imu_data, 512, IMU_FRAME_SIZE) != 0) {
        std::cerr << "imu_data posix_memalign 失败" << std::endl;
        free(camera_data);
        free(lidar_data);
        return;
    }
    std::memset(imu_data, 0, IMU_FRAME_SIZE);

    std::cout << "测试数据预分配完毕 (malloc + memset，无CPU计算生成)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // ============================================================
    //  Step 2: 运行 Benchmark
    // ============================================================
    signal(SIGINT, signal_handler);

    // 统计
    int frame_count   = 0;
    int lidar_count   = 0;
    int imu_count     = 0;
    int frame_ok      = 0, frame_fail = 0;
    int lidar_ok      = 0, lidar_fail = 0;
    int imu_ok        = 0, imu_fail   = 0;

    // 计时累加（微秒）
    uint64_t total_camera_us = 0;
    uint64_t total_lidar_us  = 0;
    uint64_t total_imu_us    = 0;

    auto bench_start = std::chrono::steady_clock::now();
    auto last_camera = bench_start;
    auto last_lidar  = bench_start;
    auto last_imu    = bench_start;
    auto last_stat   = bench_start;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();

        // 检测是否达到测试时长
        auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
            now - bench_start).count();
        if (elapsed_s >= TEST_DURATION_SEC) break;

        // ---- 摄像头 ----
        auto cam_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_camera).count();
        if (cam_ms >= CAMERA_INTERVAL_MS) {
            auto t0 = std::chrono::steady_clock::now();
            uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

            bool ok = nvme_manager.write_video_frame_to_disk(
                camera_data, CAMERA_FRAME_SIZE, ts, true);

            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0).count();
            total_camera_us += dt;
            frame_count++;
            (ok ? frame_ok : frame_fail)++;

            if (frame_count <= 5 || frame_count % 150 == 0) {
                std::cout << "[摄像头] 帧 #" << frame_count
                          << " 耗时: " << std::fixed << std::setprecision(2)
                          << dt / 1000.0 << "ms"
                          << (ok ? "" : " FAIL") << std::endl;
            }
            last_camera = now;
        }

        // ---- 激光雷达 ----
        auto lid_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_lidar).count();
        if (lid_ms >= LIDAR_INTERVAL_MS) {
            auto t0 = std::chrono::steady_clock::now();
            uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

            bool ok = nvme_manager.write_lidar_points_to_disk(
                lidar_data, LIDAR_FRAME_SIZE, ts);

            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0).count();
            total_lidar_us += dt;
            lidar_count++;
            (ok ? lidar_ok : lidar_fail)++;

            if (lidar_count <= 5 || lidar_count % 100 == 0) {
                std::cout << "[激光雷达] 帧 #" << lidar_count
                          << " 耗时: " << std::fixed << std::setprecision(2)
                          << dt / 1000.0 << "ms"
                          << (ok ? "" : " FAIL") << std::endl;
            }
            last_lidar = now;
        }

        // ---- IMU ----
        auto imu_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_imu).count();
        if (imu_ms >= IMU_INTERVAL_MS) {
            auto t0 = std::chrono::steady_clock::now();
            uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

            bool ok = nvme_manager.write_imu_data_to_disk(
                imu_data, IMU_FRAME_SIZE, ts);

            auto dt = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0).count();
            total_imu_us += dt;
            imu_count++;
            (ok ? imu_ok : imu_fail)++;

            if (imu_count <= 10 || imu_count % 500 == 0) {
                std::cout << "[IMU] 数据 #" << imu_count
                          << " 耗时: " << std::fixed << std::setprecision(2)
                          << dt / 1000.0 << "ms"
                          << (ok ? "" : " FAIL") << std::endl;
            }
            last_imu = now;
        }

        // ---- 周期性统计（每5秒） ----
        auto stat_ms = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_stat).count();
        if (stat_ms >= 5) {
            auto qsize = nvme_manager.get_queue_size();
            std::cout << "\n[统计 +" << elapsed_s << "s]"
                      << " 队列=" << qsize
                      << " 摄像头=" << frame_count
                      << " 雷达=" << lidar_count
                      << " IMU=" << imu_count
                      << std::endl;
            std::cout << "----------------------------------------" << std::endl;
            last_stat = now;
        }

        // ============================================================
        //  CPU 优化：计算距下次传感器触发的最小时间，休眠到那时
        //  避免固定 1ms 轮询导致的大量无意义唤醒
        // ============================================================
        cam_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_camera).count();
        lid_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_lidar).count();
        imu_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_imu).count();

        auto next_cam = CAMERA_INTERVAL_MS - cam_ms;
        auto next_lid = LIDAR_INTERVAL_MS - lid_ms;
        auto next_imu = IMU_INTERVAL_MS - imu_ms;

        // 取最小值作为休眠时间，但保证至少1ms
        long sleep_ms = std::min(std::min(next_cam, next_lid), next_imu);
        if (sleep_ms < 1) sleep_ms = 1;

        std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
    }

    // ============================================================
    //  Step 3: 打印汇总报告
    // ============================================================
    auto bench_end  = std::chrono::steady_clock::now();
    double real_sec = std::chrono::duration_cast<std::chrono::duration<double>>(
        bench_end - bench_start).count();

    nvme_manager.shutdown();

    // 释放测试数据
    free(camera_data);
    free(lidar_data);
    free(imu_data);

    std::cout << "\n==========================================" << std::endl;
    std::cout << "  Benchmark 汇总" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "运行时长: " << std::fixed << std::setprecision(1) << real_sec << "s" << std::endl;

    auto print_summary = [&](const char* name, int count, int ok, int fail,
                             uint64_t total_us, double expect_hz) {
        double avg_ms = (count > 0) ? (total_us / 1000.0 / count) : 0;
        double actual_hz = (real_sec > 0) ? (count / real_sec) : 0;
        std::cout << "  " << name << ":"
                  << " " << count << " 帧"
                  << " | 成功=" << ok << " 失败=" << fail
                  << " | 平均耗时=" << avg_ms << "ms"
                  << " | 实际频率=" << std::setprecision(1) << actual_hz << "Hz"
                  << " (目标=" << expect_hz << "Hz)"
                  << std::endl;
    };

    print_summary("摄像头", frame_count, frame_ok, frame_fail,
                  total_camera_us, 1000.0 / CAMERA_INTERVAL_MS);
    print_summary("激光雷达", lidar_count, lidar_ok, lidar_fail,
                  total_lidar_us, 1000.0 / LIDAR_INTERVAL_MS);
    print_summary("IMU", imu_count, imu_ok, imu_fail,
                  total_imu_us, 1000.0 / IMU_INTERVAL_MS);

    auto qsize_final = nvme_manager.get_queue_size();
    std::cout << "最终队列深度: " << qsize_final << std::endl;
    std::cout << "==========================================" << std::endl;
}

int main() {
    try {
        run_benchmark();
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
