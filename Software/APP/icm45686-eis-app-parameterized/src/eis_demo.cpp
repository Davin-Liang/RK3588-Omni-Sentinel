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
#define DEFAULT_MAX_OFFSET      200

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
 *函数原型:     static const char* gyro_range_name(uint8_t range)
 *功    能:     将陀螺仪量程编号转换为可读字符串
 *输入参数:     range - 量程编号
 *输出参数:     量程字符串
 ******************************************************************************/
static const char* gyro_range_name(uint8_t range)
{
    switch (range) {
    case 0: return "±250DPS";
    case 1: return "±500DPS";
    case 2: return "±1000DPS";
    case 3: return "±2000DPS";
    default: return "unknown";
    }
}

/**************************实现函数********************************************
 *函数原型:     static const char* accel_range_name(uint8_t range)
 *功    能:     将加速度计量程编号转换为可读字符串
 *输入参数:     range - 量程编号
 *输出参数:     量程字符串
 ******************************************************************************/
static const char* accel_range_name(uint8_t range)
{
    switch (range) {
    case 0: return "±2G";
    case 1: return "±4G";
    case 2: return "±8G";
    case 3: return "±16G";
    default: return "unknown";
    }
}

/**************************实现函数********************************************
 *函数原型:     static void print_usage(const char* prog)
 *功    能:     打印使用说明
 *输入参数:     prog - 程序名
 *输出参数:     无
 ******************************************************************************/
static void print_usage(const char* prog)
{
    printf("Usage:\n");
    printf("  %s [device] [run_seconds] [focalX] [focalY] [sampleHz] [frameHz] [halfWindowMs]\n", prog);
    printf("     [gyroRange] [accelRange] [maxOffset] [timeOffsetMs]\n");
    printf("     [signX] [signY] [swapXY] [smoothingAlpha] [biasCalibMs]\n");
    printf("\n");
    printf("Examples:\n");
    printf("  %s\n", prog);
    printf("  %s /dev/icm45686 30 1200 1200 100 30 20\n", prog);
    printf("  %s /dev/icm45686 30 1200 1200 200 15 25 1 1 120 0 -1 1 0 0.30 2000\n", prog);
    printf("\n");
    printf("Parameter notes:\n");
    printf("  gyroRange     : 0=250DPS, 1=500DPS, 2=1000DPS, 3=2000DPS\n");
    printf("  accelRange    : 0=2G, 1=4G, 2=8G, 3=16G\n");
    printf("  maxOffset     : pixel clamp, should be smaller than image crop margin\n");
    printf("  timeOffsetMs  : camera timestamp correction, can be negative\n");
    printf("  signX/signY   : +1 or -1, used to correct installation direction\n");
    printf("  swapXY        : 0=false, 1=true, used when IMU X/Y axes are swapped\n");
    printf("  smoothingAlpha: 0 disables smoothing; 0.2~0.5 enables first-order smoothing\n");
    printf("  biasCalibMs   : 0 disables gyro bias calibration; e.g. 2000 means keep static for 2s\n");
    printf("\n");
    printf("Test tips:\n");
    printf("  1. Static IMU: success_rate should increase, offset should be close to (0,0).\n");
    printf("  2. Gently rotate IMU: offsetX/offsetY should change with gyro motion.\n");
    printf("  3. 30FPS camera: halfWindowMs usually 15~20ms; 15FPS camera: 20~30ms.\n");
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

    ImuConfig imuConfig;
    EisCameraConfig cameraConfig;

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

    imuConfig.sampleHz = DEFAULT_SAMPLE_HZ;
    imuConfig.gyroRange = 0;
    imuConfig.accelRange = 0;

    cameraConfig.camId = 0;
    cameraConfig.frameRate = DEFAULT_FRAME_HZ;
    cameraConfig.focalX = DEFAULT_FOCAL_X;
    cameraConfig.focalY = DEFAULT_FOCAL_Y;
    cameraConfig.halfWindowMs = DEFAULT_HALF_WINDOW_MS;
    cameraConfig.maxOffsetPixel = DEFAULT_MAX_OFFSET;
    cameraConfig.timeOffsetMs = 0.0f;
    cameraConfig.signX = -1.0f;
    cameraConfig.signY = 1.0f;
    cameraConfig.swapXY = false;
    cameraConfig.enableSmoothing = false;
    cameraConfig.smoothingAlpha = 0.25f;

    if (argc > 1 && strcmp(argv[1], "-h") == 0) {
        print_usage(argv[0]);
        return 0;
    }

    if (argc > 1) devPath = argv[1];
    if (argc > 2) runSeconds = atoi(argv[2]);
    if (argc > 3) cameraConfig.focalX = (float)atof(argv[3]);
    if (argc > 4) cameraConfig.focalY = (float)atof(argv[4]);
    if (argc > 5) imuConfig.sampleHz = (float)atof(argv[5]);
    if (argc > 6) cameraConfig.frameRate = (float)atof(argv[6]);
    if (argc > 7) cameraConfig.halfWindowMs = (uint32_t)atoi(argv[7]);
    if (argc > 8) imuConfig.gyroRange = (uint8_t)atoi(argv[8]);
    if (argc > 9) imuConfig.accelRange = (uint8_t)atoi(argv[9]);
    if (argc > 10) cameraConfig.maxOffsetPixel = (int32_t)atoi(argv[10]);
    if (argc > 11) cameraConfig.timeOffsetMs = (float)atof(argv[11]);
    if (argc > 12) cameraConfig.signX = (float)atof(argv[12]);
    if (argc > 13) cameraConfig.signY = (float)atof(argv[13]);
    if (argc > 14) cameraConfig.swapXY = atoi(argv[14]) != 0;
    if (argc > 15) {
        cameraConfig.smoothingAlpha = (float)atof(argv[15]);
        cameraConfig.enableSmoothing = cameraConfig.smoothingAlpha > 0.0f;
    }
    if (argc > 16) {
        imuConfig.biasCalibMs = (uint32_t)atoi(argv[16]);
        imuConfig.enableGyroBiasCalib = imuConfig.biasCalibMs > 0;
    }

    if (runSeconds <= 0 || cameraConfig.focalX <= 0.0f || cameraConfig.focalY <= 0.0f ||
        imuConfig.sampleHz <= 0.0f || cameraConfig.frameRate <= 0.0f ||
        cameraConfig.halfWindowMs == 0 || cameraConfig.maxOffsetPixel <= 0 ||
        imuConfig.gyroRange > 3 || imuConfig.accelRange > 3) {
        print_usage(argv[0]);
        return -1;
    }

    printf("========================================\n");
    printf("ICM45686 EIS Demo\n");
    printf("  Device       : %s\n", devPath);
    printf("  Runtime      : %d s\n", runSeconds);
    printf("  Sample Rate  : %.2f Hz\n", imuConfig.sampleHz);
    printf("  Frame Rate   : %.2f FPS\n", cameraConfig.frameRate);
    printf("  Focal        : %.2f / %.2f pixel\n", cameraConfig.focalX, cameraConfig.focalY);
    printf("  Half Window  : %u ms\n", cameraConfig.halfWindowMs);
    printf("  Window Width : %u ms\n", cameraConfig.halfWindowMs * 2);
    printf("  Gyro Range   : %u (%s)\n", imuConfig.gyroRange, gyro_range_name(imuConfig.gyroRange));
    printf("  Accel Range  : %u (%s)\n", imuConfig.accelRange, accel_range_name(imuConfig.accelRange));
    printf("  Max Offset   : %d px\n", cameraConfig.maxOffsetPixel);
    printf("  Time Offset  : %.2f ms\n", cameraConfig.timeOffsetMs);
    printf("  Axis Sign    : signX=%.1f signY=%.1f swapXY=%d\n",
           cameraConfig.signX, cameraConfig.signY, cameraConfig.swapXY ? 1 : 0);
    printf("  Smoothing    : %s alpha=%.2f\n",
           cameraConfig.enableSmoothing ? "enabled" : "disabled",
           cameraConfig.smoothingAlpha);
    printf("  Bias Calib   : %s %u ms\n",
           imuConfig.enableGyroBiasCalib ? "enabled" : "disabled",
           imuConfig.biasCalibMs);
    printf("========================================\n");
    printf("Test method:\n");
    printf("  - Keep IMU static first: success should increase, offset should be near zero.\n");
    printf("  - Then gently rotate/shake IMU: latest_offset and max_abs_offset should change.\n");
    printf("========================================\n");

    if (!reader.openDevice(devPath)) {
        printf("Failed to open IMU device: %s\n", devPath);
        return -1;
    }

    if (imuConfig.enableGyroBiasCalib) {
        printf("Keep IMU static. Gyro bias calibration starts for %u ms...\n",
               imuConfig.biasCalibMs);
    }

    if (!reader.configure(imuConfig)) {
        printf("Failed to configure IMU. Check range values and device state.\n");
        reader.closeDevice();
        return -1;
    }

    imuConfig = reader.config();
    printf("IMU configured. gyro_bias=(%.6f, %.6f, %.6f) rad/s\n",
           imuConfig.gyroBiasX, imuConfig.gyroBiasY, imuConfig.gyroBiasZ);

    if (!reader.start()) {
        printf("Failed to start IMU reader thread.\n");
        reader.closeDevice();
        return -1;
    }

    stabilizer.bindReader(&reader);

    printf("IMU reader started. Waiting for ring buffer warm-up...\n");

    /*
     * 等待环形缓冲区预热。
     * 预热时间至少覆盖2个窗口宽度，保证EIS计算窗口中有足够历史样本。
     */
    {
        useconds_t warmupUs = (useconds_t)(500000 + cameraConfig.halfWindowMs * 4 * 1000);
        usleep(warmupUs);
    }

    framePeriodNs = (uint64_t)(1000000000.0 / cameraConfig.frameRate);
    startNs = imu_get_time_ns();
    nextFrameNs = startNs;
    lastReportNs = startNs;
    lastCpuSeconds = read_process_cpu_seconds();
    lastCpuTimeNs = startNs;

    while (g_running) {
        uint64_t nowNs = imu_get_time_ns();
        uint64_t frameTimestampNs;
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
         * Demo阶段没有真实相机曝光中心时间戳。
         * 为了让查询窗口完整落在历史IMU数据内，这里用当前时间往前偏移halfWindowMs模拟帧时间戳。
         * 实际接入相机后，应传入真实frameExposureCenterTimestampNs。
         */
        nowNs = imu_get_time_ns();
        frameTimestampNs = nowNs - (uint64_t)cameraConfig.halfWindowMs * 1000000ULL;

        ok = stabilizer.calculate_eis_offset(cameraConfig, frameTimestampNs, offsetX, offsetY);
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
