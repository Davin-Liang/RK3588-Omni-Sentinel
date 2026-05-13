#include <iostream>
#include <chrono>
#include <thread>
#include <vector>
#include <random>
#include <cstring>
#include <iomanip>
#include "NVMeDataManager.h"

// 模拟生成摄像头图像数据
void generate_camera_frame(std::vector<uint8_t>& frame_data, int width = 1920, int height = 1080) {
    // 模拟 RGB888 图像数据
    size_t frame_size = width * height * 3; // RGB888: 3 bytes per pixel
    frame_data.resize(frame_size);

    // 使用随机数生成器模拟图像数据
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 255);

    for (size_t i = 0; i < frame_size; i++) {
        frame_data[i] = dis(gen);
    }
}

// 模拟生成激光雷达点云数据
void generate_lidar_points(std::vector<uint8_t>& points_data, int point_count = 4096) {
    // 模拟点云数据：每个点包含 x, y, z, intensity (4 float = 16 bytes)
    size_t point_size = point_count * 16;
    points_data.resize(point_size);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-50.0, 50.0); // -50 到 50 米的范围

    for (int i = 0; i < point_count; i++) {
        float x = dis(gen);
        float y = dis(gen);
        float z = dis(gen);
        float intensity = dis(gen) / 255.0f; // 0-1之间的强度值

        memcpy(&points_data[i * 16], &x, 4);
        memcpy(&points_data[i * 16 + 4], &y, 4);
        memcpy(&points_data[i * 16 + 8], &z, 4);
        memcpy(&points_data[i * 16 + 12], &intensity, 4);
    }
}

// 模拟生成 IMU 数据
void generate_imu_data(std::vector<uint8_t>& imu_data) {
    // 模拟 IMU 数据：acceleration + gyroscope (6 float = 24 bytes)
    imu_data.resize(24);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-1.0, 1.0); // -1 到 1 之间的值

    float ax = dis(gen), ay = dis(gen), az = dis(gen);
    float gx = dis(gen), gy = dis(gen), gz = dis(gen);

    memcpy(&imu_data[0], &ax, 4);
    memcpy(&imu_data[4], &ay, 4);
    memcpy(&imu_data[8], &az, 4);
    memcpy(&imu_data[12], &gx, 4);
    memcpy(&imu_data[16], &gy, 4);
    memcpy(&imu_data[20], &gz, 4);
}

// 测试函数
void run_nvme_test() {
    NVMeDataManager nvme_manager;

    // 初始化
    std::cout << "初始化 NVMe 数据管理器..." << std::endl;
    if (!nvme_manager.initialize()) {
        std::cerr << "初始化失败！" << std::endl;
        return;
    }
    std::cout << "初始化成功！" << std::endl;

    // 测试参数
    const int camera_interval_ms = 67;  // 15 FPS (1000/15 ≈ 67ms)
    const int lidar_interval_ms = 100; // 10 Hz (100ms)
    const int imu_interval_ms = 10;    // 100 Hz (10ms)

    // 数据存储
    std::vector<uint8_t> camera_frame;
    std::vector<uint8_t> lidar_points;
    std::vector<uint8_t> imu_data;

    // 统计变量
    int frame_count = 0;
    int lidar_count = 0;
    int imu_count = 0;
    auto total_start = std::chrono::steady_clock::now();

    // 生成初始数据
    generate_camera_frame(camera_frame);
    generate_lidar_points(lidar_points);
    generate_imu_data(imu_data);

    // 主循环
    auto last_camera_time = std::chrono::steady_clock::now();
    auto last_lidar_time = std::chrono::steady_clock::now();
    auto last_imu_time = std::chrono::steady_clock::now();

    std::cout << "\n开始测试数据写入..." << std::endl;
    std::cout << "摄像头帧间隔: " << camera_interval_ms << "ms (15 FPS)" << std::endl;
    std::cout << "激光雷达帧间隔: " << lidar_interval_ms << "ms (10 Hz)" << std::endl;
    std::cout << "IMU 数据间隔: " << imu_interval_ms << "ms (100 Hz)" << std::endl;
    std::cout << "----------------------------------------" << std::endl;

    while (true) {
        auto current_time = std::chrono::steady_clock::now();

        // 摄像头数据写入
        auto camera_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_camera_time).count();
        if (camera_elapsed >= camera_interval_ms) {
            auto write_start = std::chrono::steady_clock::now();

            // 生成新的摄像头帧
            generate_camera_frame(camera_frame);
            uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                current_time.time_since_epoch()).count();

            // 写入数据
            bool success = nvme_manager.write_video_frame_to_disk(
                camera_frame.data(), camera_frame.size(),
                timestamp, true); // true 表示前摄像头

            auto write_end = std::chrono::steady_clock::now();
            auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                write_end - write_start).count();

            if (success) {
                std::cout << "[摄像头] 帧 #" << ++frame_count
                          << " 写入耗时: " << std::fixed << std::setprecision(2)
                          << write_duration / 1000.0 << "ms" << std::endl;
            } else {
                std::cerr << "[摄像头] 写入失败！" << std::endl;
            }

            // 写入 10 帧后进行读取验证（只执行一次）
            if (frame_count == 10) {
                // 等待 writer 线程将数据写入内核页面缓存
                std::this_thread::sleep_for(std::chrono::milliseconds(100));

                std::cout << "\n=== 读取验证 ===" << std::endl;

                std::vector<uint8_t> read_buffer;
                // 以当前帧的时间戳为目标，搜索 ±10s 范围内的视频帧
                if (nvme_manager.read_video_frame_from_disk(timestamp, 10.0f, read_buffer)) {
                    std::cout << "[✓] 成功读取视频帧" << std::endl;
                    std::cout << "    - 数据大小: " << read_buffer.size() << " bytes" << std::endl;

                    // 验证数据完整性（打印前 16 字节的十六进制值）
                    std::cout << "    - 帧数据前 16 字节: ";
                    for (size_t i = 0; i < std::min(read_buffer.size(), size_t(16)); i++) {
                        std::cout << std::hex << std::setw(2) << std::setfill('0')
                                  << (int)read_buffer[i] << " ";
                    }
                    std::cout << std::dec << std::setfill(' ') << std::endl;
                } else {
                    std::cout << "[✗] 读取验证失败：未找到匹配时间戳的视频帧" << std::endl;
                }

                std::cout << "=== 读取验证结束 ===" << std::endl;
                std::cout << "----------------------------------------" << std::endl;
            }

            last_camera_time = current_time;
        }

        // 激光雷达数据写入
        auto lidar_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_lidar_time).count();
        if (lidar_elapsed >= lidar_interval_ms) {
            auto write_start = std::chrono::steady_clock::now();

            // 生成新的激光雷达数据
            generate_lidar_points(lidar_points);
            uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                current_time.time_since_epoch()).count();

            // 写入数据
            bool success = nvme_manager.write_lidar_points_to_disk(
                lidar_points.data(), lidar_points.size(),
                timestamp);

            auto write_end = std::chrono::steady_clock::now();
            auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                write_end - write_start).count();

            if (success) {
                std::cout << "[激光雷达] 帧 #" << ++lidar_count
                          << " 写入耗时: " << std::fixed << std::setprecision(2)
                          << write_duration / 1000.0 << "ms" << std::endl;
            } else {
                std::cerr << "[激光雷达] 写入失败！" << std::endl;
            }

            last_lidar_time = current_time;
        }

        // IMU 数据写入
        auto imu_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(current_time - last_imu_time).count();
        if (imu_elapsed >= imu_interval_ms) {
            auto write_start = std::chrono::steady_clock::now();

            // 生成新的 IMU 数据
            generate_imu_data(imu_data);
            uint64_t timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                current_time.time_since_epoch()).count();

            // 写入数据
            bool success = nvme_manager.write_imu_data_to_disk(
                imu_data.data(), imu_data.size(),
                timestamp);

            auto write_end = std::chrono::steady_clock::now();
            auto write_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                write_end - write_start).count();

            if (success) {
                std::cout << "[IMU] 数据 #" << ++imu_count
                          << " 写入耗时: " << std::fixed << std::setprecision(2)
                          << write_duration / 1000.0 << "ms" << std::endl;
            } else {
                std::cerr << "[IMU] 写入失败！" << std::endl;
            }

            last_imu_time = current_time;
        }

        // 每秒输出一次统计信息
        auto total_elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            current_time - total_start).count();
        if (total_elapsed > 0 && total_elapsed % 5 == 0) {
            auto queue_size = nvme_manager.get_queue_size();
            std::cout << "\n[统计] 运行时间: " << total_elapsed << "s, "
                      << "队列大小: " << queue_size << std::endl;
            std::cout << "----------------------------------------" << std::endl;
        }

        // 短暂休眠，避免 CPU 占用过高
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    // 清理（在实际使用中应该调用 shutdown）
    // nvme_manager.shutdown();
}

int main() {
    std::cout << "NVMe SSD 测试 Demo" << std::endl;
    std::cout << "===================" << std::endl;

    try {
        run_nvme_test();
    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}