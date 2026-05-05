/**
 * @brief SentinelLslidarer 功能验证 Demo
 *
 * 测试流程：
 *   1. 加载 N10Plus 默认配置，启动雷达
 *   2. 等待环形缓冲区积累足够帧数
 *   3. 以当前时间为相机时间戳，调用 get_closest_frame 查找最近一帧点云
 *   4. 打印帧时间戳、有效点数等统计信息
 *   5. 安全停止并退出
 */

#include "sentinel_lslidarer.h"

#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <signal.h>

static volatile bool gRunning = true;

static void signalHandler(int /*sig*/) {
    gRunning = false;
}

int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::printf("=== SentinelLslidarer Demo ===\n");

    // 1. 创建实例并加载默认配置
    SentinelLslidarer lidar;
    LidarConfig config;
    if (!lidar.load_config(config)) {
        std::fprintf(stderr, "Failed to load config.\n");
        return 1;
    }
    std::printf("Config loaded: serial=%s, baud=%d, ringBufferSize=%u\n",
                config.serialPort.c_str(), config.baudRate,
                config.ringBufferSize);

    // 2. 启动雷达
    if (!lidar.start()) {
        std::fprintf(stderr, "Failed to start lidar. Check serial port.\n");
        return 1;
    }
    std::printf("Lidar started. Waiting for frames...\n");

    // 3. 预分配帧缓冲区
    uint32_t maxPoints = lidar.max_points_per_frame();
    LidarFrame frame;
    frame.points = new LidarPoint[maxPoints];

    // 4. 主循环：周期性查询最近帧
    uint64_t lastFrameTs = 0;
    while (gRunning) {
        uint32_t available = lidar.available_frames();
        if (available > 0) {

            // 以当前单调时钟为假想相机时间戳
            struct timespec ts;
            clock_gettime(CLOCK_MONOTONIC, &ts);
            uint64_t cameraTsNs = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
                                + static_cast<uint64_t>(ts.tv_nsec);

            if (lidar.get_closest_frame(cameraTsNs, frame)) {
                // 仅在新帧到达时打印
                if (frame.timestampNs == lastFrameTs) {
                    usleep(10000);  // 10ms
                    continue;
                }
                lastFrameTs = frame.timestampNs;

                std::printf("[frame] ts=%lu ns, pts=%u, ring=%u, age=%lu us\n",
                            static_cast<unsigned long>(frame.timestampNs),
                            frame.pointsCount, available,
                            static_cast<unsigned long>(
                                (cameraTsNs - frame.timestampNs) / 1000));

                if (frame.pointsCount > 0) {
                    std::printf("  first: x=%.3f y=%.3f i=%.0f  last: x=%.3f y=%.3f i=%.0f\n",
                                static_cast<double>(frame.points[0].x),
                                static_cast<double>(frame.points[0].y),
                                static_cast<double>(frame.points[0].intensity),
                                static_cast<double>(frame.points[frame.pointsCount - 1].x),
                                static_cast<double>(frame.points[frame.pointsCount - 1].y),
                                static_cast<double>(frame.points[frame.pointsCount - 1].intensity));
                }
            } else {
                std::printf("[frame] get_closest_frame failed.\n");
            }
        }

        // 如果没有帧且超时，退出
        if (available == 0) {
            usleep(100000);  // 100ms
        }
    }

    // 5. 清理
    delete[] frame.points;
    lidar.stop();
    std::printf("Demo finished.\n");
    return 0;
}
