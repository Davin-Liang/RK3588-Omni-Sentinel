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

// ============================================================
//  export_trigger_video_clip 功能测试
//  模拟前/后双摄像头采集6色循环动态场景，验证回溯10s时间窗口导出
// ============================================================
void test_export_video_clip() {
    // 参数配置
    static constexpr int FRAME_WIDTH       = 1920;
    static constexpr int FRAME_HEIGHT      = 1080;
    static constexpr size_t FRAME_SIZE     = FRAME_WIDTH * FRAME_HEIGHT * 3;  // RGB888
    static constexpr int FPS               = 15;
    static constexpr double TIME_WINDOW    = 10.0;        // 回溯10秒
    static constexpr int TEST_DURATION_SEC = 35;          // 35s，保证前后各有充足帧
    static constexpr int COLOR_INTERVAL_SEC = 1;          // 每秒切换一种颜色
    static constexpr int NUM_COLORS          = 6;
    static constexpr int NUM_FRONT_COLORS    = 3;   // 前视: 红绿蓝
    static constexpr int NUM_REAR_COLORS     = 3;   // 后视: 黄黑紫

    // 六种纯色定义 (RGB)
    struct ColorInfo {
        const char* name;
        uint8_t r, g, b;
    };
    static const ColorInfo COLORS[NUM_COLORS] = {
        {"红", 255,   0,   0},
        {"绿",   0, 255,   0},
        {"蓝",   0,   0, 255},
        {"黄", 255, 255,   0},
        {"黑",   0,   0,   0},
        {"紫", 255,   0, 255},
    };

    std::cout << "\n==========================================" << std::endl;
    std::cout << "  export_trigger_video_clip 功能测试" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "图像格式: RGB888 " << FRAME_WIDTH << "x" << FRAME_HEIGHT
              << " (" << (FRAME_SIZE / 1024.0 / 1024.0) << " MB/帧)" << std::endl;
    std::cout << "帧率: " << FPS << " FPS | 时间窗口: 回溯 " << TIME_WINDOW << "s" << std::endl;
    std::cout << "测试时长: " << TEST_DURATION_SEC << "s" << std::endl;
    std::cout << "颜色策略: 每秒切换 | 前视(红→绿→蓝) | 后视(黄→黑→紫)" << std::endl;
    std::cout << "摄像头: 前视 + 后视 双路同时写入" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    // ---- Step 1: CPU 预生成6张纯色图片 ----
    std::cout << "Step 1/4: CPU 预生成6张纯色图片..." << std::endl;
    uint8_t* color_frames[NUM_COLORS];
    for (int i = 0; i < NUM_COLORS; i++) {
        if (posix_memalign((void**)&color_frames[i], 512, FRAME_SIZE) != 0) {
            std::cerr << "posix_memalign 失败 at index " << i << std::endl;
            for (int j = 0; j < i; j++) free(color_frames[j]);
            return;
        }
        for (size_t p = 0; p < FRAME_SIZE; p += 3) {
            color_frames[i][p + 0] = COLORS[i].r;
            color_frames[i][p + 1] = COLORS[i].g;
            color_frames[i][p + 2] = COLORS[i].b;
        }
        std::cout << "  [" << i << "] " << COLORS[i].name
                  << " RGB(" << (int)COLORS[i].r << ","
                  << (int)COLORS[i].g << "," << (int)COLORS[i].b << ")" << std::endl;
    }

    // ---- Step 2: 初始化 NVMe 并双路写入 ----
    NVMeDataManager nvme_manager;
    if (!nvme_manager.initialize()) {
        std::cerr << "NVMe 初始化失败！" << std::endl;
        for (int i = 0; i < NUM_COLORS; i++) free(color_frames[i]);
        return;
    }

    std::cout << "\nStep 2/4: 循环写入双路模拟视频帧 " << TEST_DURATION_SEC << "s..." << std::endl;

    // 记录每路的时间戳和对应颜色索引
    std::vector<uint64_t> front_ts, rear_ts;
    std::vector<int> front_color_idx, rear_color_idx;
    front_ts.reserve(FPS * TEST_DURATION_SEC);
    rear_ts.reserve(FPS * TEST_DURATION_SEC);
    front_color_idx.reserve(FPS * TEST_DURATION_SEC);
    rear_color_idx.reserve(FPS * TEST_DURATION_SEC);

    int frame_count = 0, fail_count = 0;
    auto test_start = std::chrono::steady_clock::now();
    auto last_frame = test_start;

    signal(SIGINT, signal_handler);
    g_running = true;

    while (g_running) {
        auto now = std::chrono::steady_clock::now();
        auto elapsed_s = std::chrono::duration_cast<std::chrono::seconds>(
            now - test_start).count();
        if (elapsed_s >= TEST_DURATION_SEC) break;

        auto frame_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_frame).count();
        if (frame_ms >= (1000 / FPS)) {
            // 根据时间确定当前颜色
            // 前视: 循环 红(0)→绿(1)→蓝(2)
            // 后视: 循环 黄(3)→黑(4)→紫(5)
            int front_color = (elapsed_s / COLOR_INTERVAL_SEC) % NUM_FRONT_COLORS;
            int rear_color  = NUM_FRONT_COLORS + (elapsed_s / COLOR_INTERVAL_SEC) % NUM_REAR_COLORS;

            uint64_t ts = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch()).count();

            auto t0 = std::chrono::steady_clock::now();

            // 前视摄像头（前3色: 红绿蓝）
            bool f_ok = nvme_manager.write_video_frame_to_disk(
                color_frames[front_color], FRAME_SIZE, ts, true);
            if (f_ok) {
                front_ts.push_back(ts);
                front_color_idx.push_back(front_color);
            }

            // 后视摄像头（后3色: 黄黑紫）
            bool r_ok = nvme_manager.write_video_frame_to_disk(
                color_frames[rear_color], FRAME_SIZE, ts, false);
            if (r_ok) {
                rear_ts.push_back(ts);
                rear_color_idx.push_back(rear_color);
            }

            auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now() - t0).count();
            frame_count++;
            if (!f_ok || !r_ok) fail_count++;

            // 前5帧 + 每75帧打印进度
            if (frame_count <= 5 || frame_count % 75 == 0) {
                std::cout << "[#" << frame_count << " t+" << elapsed_s << "s] "
                          << "前视=" << COLORS[front_color].name
                          << " 后视=" << COLORS[rear_color].name
                          << " ts=" << ts
                          << " " << std::fixed << std::setprecision(1)
                          << (dt_us / 1000.0) << "ms"
                          << (f_ok && r_ok ? "" : " FAIL") << std::endl;
            }

            last_frame = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::cout << "写入完毕: " << frame_count << " 帧"
              << " | 前视=" << front_ts.size()
              << " 后视=" << rear_ts.size()
              << " 失败=" << fail_count << std::endl;

    // ---- Step 3: 关闭写入线程 ----
    nvme_manager.shutdown();
    for (int i = 0; i < NUM_COLORS; i++) free(color_frames[i]);
    std::cout << "\nStep 3/4: NVMe 写入线程已关闭，准备导出..." << std::endl;

    // ---- Step 4: 导出两个摄像头的视频 ----
    // 选取靠后的触发点（倒数第 N 帧），确保前面有足够10s数据
    size_t trigger_offset = static_cast<size_t>(FPS * 2);  // 倒数 ~2s
    const std::string outputs[2] = {
        "/tmp/front_camera_clip.mp4",
        "/tmp/rear_camera_clip.mp4",
    };

    std::cout << "\nStep 4/4: 导出视频片段 (回溯 " << TIME_WINDOW << "s)..." << std::endl;

    for (int cam = 0; cam < 2; cam++) {
        const auto& ts_vec     = (cam == 0) ? front_ts  : rear_ts;
        const auto& color_vec  = (cam == 0) ? front_color_idx : rear_color_idx;
        const char* cam_name   = (cam == 0) ? "前视"   : "后视";

        if (ts_vec.size() < static_cast<size_t>(FPS * TIME_WINDOW)) {
            std::cerr << "  " << cam_name << ": 帧数不足，跳过 (需要"
                      << static_cast<int>(FPS * TIME_WINDOW) << "帧)" << std::endl;
            continue;
        }

        size_t t_idx = ts_vec.size() - trigger_offset;
        uint64_t trigger_ts = ts_vec[t_idx];

        std::cout << "  --- " << cam_name << "摄像头 ---" << std::endl;
        std::cout << "  触发时间戳: " << trigger_ts
                  << " (帧索引: " << t_idx << "/" << ts_vec.size() << ")" << std::endl;
        std::cout << "  触发时颜色: " << COLORS[color_vec[t_idx]].name << std::endl;

        auto t0 = std::chrono::steady_clock::now();
        // camera_id: 0=前视(VIDEO_FRONT), 1=后视(VIDEO_REAR), -1=全部
        bool ok = nvme_manager.export_trigger_video_clip(
            trigger_ts, outputs[cam], TIME_WINDOW, FPS,
            FRAME_WIDTH, FRAME_HEIGHT, cam);
        auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - t0).count();

        std::cout << "  " << cam_name << "导出: " << (ok ? "成功 ✓" : "失败 ✗")
                  << " | 耗时=" << dt << "ms"
                  << " | 文件=" << outputs[cam] << std::endl;
    }

    // ---- 结果汇总 ----
    std::cout << "\n==========================================" << std::endl;
    std::cout << "  测试完成" << std::endl;
    std::cout << "  前视视频: /tmp/front_camera_clip.mp4" << std::endl;
    std::cout << "  后视视频: /tmp/rear_camera_clip.mp4" << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "  验证方法:" << std::endl;
    std::cout << "  1. 前视视频应每1s切换: 红→绿→蓝 (仅3色循环)" << std::endl;
    std::cout << "  2. 后视视频应每1s切换: 黄→黑→紫 (仅3色循环)" << std::endl;
    std::cout << "  3. 如果颜色持续变化，说明时间窗口导出正确" << std::endl;
    std::cout << "  4. 如果全是一帧重复或颜色不对，说明匹配失败" << std::endl;
    std::cout << "==========================================" << std::endl;
}

int main(int argc, char* argv[]) {
    try {
        if (argc > 1 && std::string(argv[1]) == "bench") {
            run_benchmark();
        } else if (argc > 1 && std::string(argv[1]) == "export") {
            test_export_video_clip();
        } else {
            // 默认：运行 Benchmark
            run_benchmark();
        }
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
