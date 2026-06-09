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

#include <cmath>
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

    std::printf("=== SentinelLslidarer Demo (build: %s %s) ===\n", __DATE__, __TIME__);

    // 1. 创建实例并加载配置（关闭角度屏蔽以查看全圈点云）
    SentinelLslidarer lidar;
    LidarConfig config;
    config.angleDisableMin = 0;
    config.angleDisableMax = 0;
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

                    // 打印后方 20° 的点（方位角 170°-190°）
                    int rearCount = 0;
                    constexpr double kDegToRad = M_PI / 180.0;
                    for (uint32_t i = 0; i < frame.pointsCount; ++i) {
                        double az = std::atan2(frame.points[i].y, frame.points[i].x) / kDegToRad;
                        if (az < -170.0 || az > 170.0) {  // 后方 20°: ±[170°, 180°]
                            if (rearCount < 5) {  // 最多打印 5 个
                                std::printf("  rear[%d]: az=%.1f° x=%.3f y=%.3f d=%.3f i=%.0f\n",
                                            i, az,
                                            static_cast<double>(frame.points[i].x),
                                            static_cast<double>(frame.points[i].y),
                                            std::sqrt(frame.points[i].x * frame.points[i].x +
                                                      frame.points[i].y * frame.points[i].y),
                                            static_cast<double>(frame.points[i].intensity));
                            }
                            ++rearCount;
                        }
                    }
                    std::printf("  rear 20° total: %d points\n", rearCount);
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
