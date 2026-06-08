/*
 * eis_demo.cpp - ICM45686防抖应用层接口测试Demo
 *
 * 基于当前已调通的 /dev/icm45686 字符设备方案，验证IMU读取、环形缓冲区、时间戳同步和EIS像素偏移计算
 *
 * 日期: 2026-06-08
 */

#include "imu_eis.hpp"
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/times.h>
#include <time.h>
#include <unistd.h>

#define DEFAULT_DEV_PATH        "/dev/icm45686"
#define DEFAULT_SAMPLE_HZ       100.0f
#define DEFAULT_FRAME_HZ        30.0f
#define DEFAULT_RUN_SECONDS     30
#define DEFAULT_FOCAL_X         1200.0f
#define DEFAULT_FOCAL_Y         1200.0f

/*
 * 重要：
 * 100Hz IMU的采样间隔约为10ms。
 * 如果halfWindowMs只有5ms，则[target-half, target+half]总窗口只有10ms，
 * 很容易只取到1条IMU样本，无法进行陀螺积分，导致success=0。
 * 因此防抖功能测试阶段默认使用20ms半窗口，总窗口40ms，通常可取到3~5条样本。
 */
#define DEFAULT_HALF_WINDOW_MS  20

static volatile int g_running = 1;

/**************************实现函数********************************************
 *函数原型:     static double read_process_cpu_seconds(void)
 *功    能:     读取当前进程累计CPU时间
 *输入参数:     无
 *输出参数:     CPU时间，单位秒
 ******************************************************************************/
static double read_process_cpu_seconds(void)
{
    struct tms t;
    long ticks = sysconf(_SC_CLK_TCK);

    if (ticks <= 0) {
        return 0.0;
    }

    times(&t);
    return (double)(t.tms_utime + t.tms_stime) / (double)ticks;
}

/**************************实现函数********************************************
 *函数原型:     static void print_usage(const char* prog)
 *功    能:     打印使用说明
 *输入参数:     prog - 程序名
 *输出参数:     无
 ******************************************************************************/
static void print_usage(const char* prog)
{
    printf("Usage: %s [device] [run_seconds] [focalX] [focalY] [sampleHz] [frameHz] [halfWindowMs]\n", prog);
    printf("Example:\n");
    printf("  %s\n", prog);
    printf("  %s /dev/icm45686 30 1200 1200 100 30 20\n", prog);
    printf("\n");
    printf("Test tips:\n");
    printf("  1. Static IMU: offset should be close to (0,0), success should increase.\n");
    printf("  2. Gently rotate IMU: offsetX/offsetY should change with gyro motion.\n");
    printf("  3. For 100Hz IMU, recommended halfWindowMs is 15~30ms.\n");
}

/**************************实现函数********************************************
 *函数原型:     int main(int argc, char* argv[])
 *功    能:     防抖接口测试Demo主函数
 *输入参数:     argc - 参数个数, argv - 参数数组
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int main(int argc, char* argv[])
{
    const char* devPath = DEFAULT_DEV_PATH;
    int runSeconds = DEFAULT_RUN_SECONDS;
    float focalX = DEFAULT_FOCAL_X;
    float focalY = DEFAULT_FOCAL_Y;
    float sampleHz = DEFAULT_SAMPLE_HZ;
    float frameHz = DEFAULT_FRAME_HZ;
    uint32_t halfWindowMs = DEFAULT_HALF_WINDOW_MS;

    Icm45686Reader reader(2048);
    EisStabilizer stabilizer;
    uint64_t framePeriodNs;
    uint64_t nextFrameNs;
    uint64_t startNs;
    uint64_t lastReportNs;
    uint64_t frames = 0;
    uint64_t successFrames = 0;
    uint64_t failedEisFrames = 0;
    uint64_t nonZeroOffsetFrames = 0;
    int32_t maxAbsOffsetX = 0;
    int32_t maxAbsOffsetY = 0;
    double maxCostMs = 0.0;
    double sumCostMs = 0.0;
    double lastCpuSeconds;
    uint64_t lastCpuTimeNs;

    if (argc > 1 && strcmp(argv[1], "-h") == 0) {
        print_usage(argv[0]);
        return 0;
    }

    if (argc > 1) devPath = argv[1];
    if (argc > 2) runSeconds = atoi(argv[2]);
    if (argc > 3) focalX = (float)atof(argv[3]);
    if (argc > 4) focalY = (float)atof(argv[4]);
    if (argc > 5) sampleHz = (float)atof(argv[5]);
    if (argc > 6) frameHz = (float)atof(argv[6]);
    if (argc > 7) halfWindowMs = (uint32_t)atoi(argv[7]);

    if (runSeconds <= 0 || focalX <= 0.0f || focalY <= 0.0f ||
        sampleHz <= 0.0f || frameHz <= 0.0f || halfWindowMs == 0) {
        print_usage(argv[0]);
        return -1;
    }

    printf("========================================\n");
    printf("ICM45686 EIS Demo\n");
    printf("  Device       : %s\n", devPath);
    printf("  Runtime      : %d s\n", runSeconds);
    printf("  Sample Rate  : %.2f Hz\n", sampleHz);
    printf("  Frame Rate   : %.2f Hz\n", frameHz);
    printf("  Focal        : %.2f / %.2f pixel\n", focalX, focalY);
    printf("  Half Window  : %u ms\n", halfWindowMs);
    printf("  Window Width : %u ms\n", halfWindowMs * 2);
    printf("========================================\n");
    printf("Test method:\n");
    printf("  - Keep IMU static first: success should increase, offset should be near zero.\n");
    printf("  - Then gently rotate/shake IMU: latest_offset and max_abs_offset should change.\n");
    printf("========================================\n");

    if (!reader.openDevice(devPath)) {
        printf("Failed to open IMU device: %s\n", devPath);
        return -1;
    }

    /* 默认配置：±2G，±250DPS */
    if (!reader.setAccelRange(0)) {
        printf("Warning: failed to set accel range, continue with driver default.\n");
    }

    if (!reader.setGyroRange(0)) {
        printf("Warning: failed to set gyro range, continue with driver default.\n");
    }

    if (!reader.start(sampleHz)) {
        printf("Failed to start IMU reader thread.\n");
        reader.closeDevice();
        return -1;
    }

    stabilizer.bindReader(&reader);
    stabilizer.setMaxOffset(200);

    printf("IMU reader started. Waiting for ring buffer warm-up...\n");

    /*
     * 等待环形缓冲区预热。
     * 预热时间至少覆盖2个窗口宽度，保证EIS计算窗口中有足够历史样本。
     */
    {
        useconds_t warmupUs = (useconds_t)(500000 + halfWindowMs * 4 * 1000);
        usleep(warmupUs);
    }

    framePeriodNs = (uint64_t)(1000000000.0 / frameHz);
    startNs = imu_get_time_ns();
    nextFrameNs = startNs;
    lastReportNs = startNs;
    lastCpuSeconds = read_process_cpu_seconds();
    lastCpuTimeNs = startNs;

    while (g_running) {
        uint64_t nowNs = imu_get_time_ns();
        uint64_t targetTimestampNs;
        int32_t offsetX = 0;
        int32_t offsetY = 0;
        bool ok;

        if (nowNs - startNs >= (uint64_t)runSeconds * 1000000000ULL) {
            break;
        }

        if (nowNs < nextFrameNs) {
            uint64_t sleepNs = nextFrameNs - nowNs;
            struct timespec ts;
            ts.tv_sec = sleepNs / 1000000000ULL;
            ts.tv_nsec = sleepNs % 1000000000ULL;
            nanosleep(&ts, NULL);
            continue;
        }

        /*
         * 关键修改：
         * 不能直接使用nowNs作为目标帧时间戳。
         * 因为窗口[target-halfWindow, target+halfWindow]的右半部分会落到未来，
         * 环形缓冲区中还没有未来IMU数据，容易导致样本不足。
         *
         * 这里将目标帧时间设置为当前时间往前halfWindowMs，
         * 使查询窗口完整落在历史数据区间：[now-2*halfWindow, now]。
         */
        nowNs = imu_get_time_ns();
        targetTimestampNs = nowNs - (uint64_t)halfWindowMs * 1000000ULL;

        ok = stabilizer.calculate_eis_offset(focalX, focalY,
                                             targetTimestampNs,
                                             halfWindowMs,
                                             offsetX, offsetY);
        frames++;
        if (ok) {
            successFrames++;
            sumCostMs += stabilizer.lastCostMs();
            if (stabilizer.lastCostMs() > maxCostMs) {
                maxCostMs = stabilizer.lastCostMs();
            }

            if (offsetX != 0 || offsetY != 0) {
                nonZeroOffsetFrames++;
            }

            if (abs(offsetX) > maxAbsOffsetX) {
                maxAbsOffsetX = abs(offsetX);
            }

            if (abs(offsetY) > maxAbsOffsetY) {
                maxAbsOffsetY = abs(offsetY);
            }
        } else {
            failedEisFrames++;
        }

        if (nowNs - lastReportNs >= 1000000000ULL) {
            ImuSample latest;
            double cpuNow = read_process_cpu_seconds();
            double cpuPercent = 0.0;
            double wallSec = (double)(nowNs - lastCpuTimeNs) / 1000000000.0;
            double avgCost = successFrames > 0 ? sumCostMs / (double)successFrames : 0.0;
            double successRate = frames > 0 ? (double)successFrames * 100.0 / (double)frames : 0.0;

            if (wallSec > 0.0) {
                cpuPercent = (cpuNow - lastCpuSeconds) / wallSec * 100.0;
            }

            printf("[EIS Demo] frames=%llu success=%llu failed_eis=%llu success_rate=%.2f%% "
                   "buffered=%zu total_imu=%llu failed_imu=%llu latest_offset=(%d,%d) "
                   "max_abs_offset=(%d,%d) nonzero=%llu avg_cost=%.3f ms max_cost=%.3f ms cpu=%.2f%%\n",
                   (unsigned long long)frames,
                   (unsigned long long)successFrames,
                   (unsigned long long)failedEisFrames,
                   successRate,
                   reader.bufferedSamples(),
                   (unsigned long long)reader.totalSamples(),
                   (unsigned long long)reader.failedReads(),
                   offsetX, offsetY,
                   maxAbsOffsetX, maxAbsOffsetY,
                   (unsigned long long)nonZeroOffsetFrames,
                   avgCost, maxCostMs, cpuPercent);

            if (reader.getLatestSample(latest)) {
                float accelNorm = sqrtf(latest.accelX * latest.accelX +
                                        latest.accelY * latest.accelY +
                                        latest.accelZ * latest.accelZ);
                float gyroNorm = sqrtf(latest.gyroX * latest.gyroX +
                                       latest.gyroY * latest.gyroY +
                                       latest.gyroZ * latest.gyroZ);

                printf("           latest_imu: accel=(%.2f %.2f %.2f | norm=%.2f) "
                       "gyro=(%.4f %.4f %.4f | norm=%.4f) temp=%.2f used_samples=%zu\n",
                       latest.accelX, latest.accelY, latest.accelZ, accelNorm,
                       latest.gyroX, latest.gyroY, latest.gyroZ, gyroNorm,
                       latest.temperature,
                       stabilizer.lastUsedSamples());
            }

            lastReportNs = nowNs;
            lastCpuSeconds = cpuNow;
            lastCpuTimeNs = nowNs;
        }

        nextFrameNs += framePeriodNs;
        if (nextFrameNs < nowNs) {
            nextFrameNs = nowNs + framePeriodNs;
        }
    }

    reader.stop();
    reader.closeDevice();

    printf("========================================\n");
    printf("Demo finished. frames=%llu success=%llu failed_eis=%llu success_rate=%.2f%% "
           "total_imu=%llu failed_imu=%llu max_abs_offset=(%d,%d)\n",
           (unsigned long long)frames,
           (unsigned long long)successFrames,
           (unsigned long long)failedEisFrames,
           frames > 0 ? (double)successFrames * 100.0 / (double)frames : 0.0,
           (unsigned long long)reader.totalSamples(),
           (unsigned long long)reader.failedReads(),
           maxAbsOffsetX, maxAbsOffsetY);
    printf("========================================\n");

    return 0;
}

