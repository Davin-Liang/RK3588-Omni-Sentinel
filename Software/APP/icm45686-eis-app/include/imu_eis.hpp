/*
 * imu_eis.hpp - ICM45686防抖应用层接口头文件
 *
 * 基于当前已调通的 /dev/icm45686 字符设备方案实现IMU读取、环形缓冲区和EIS偏移计算接口
 *
 * 日期: 2026-06-08
 */

#ifndef __IMU_EIS_HPP__
#define __IMU_EIS_HPP__

#include <stdint.h>
#include <stddef.h>
#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif
#include "icm45686_user.h"
#ifdef __cplusplus
}
#endif

/* IMU单帧样本，时间戳单位为ns，基于CLOCK_MONOTONIC */
struct ImuSample {
    uint64_t timestampNs;  /* 采样时间戳，单位：ns */

    float accelX;          /* 加速度计X轴，单位：m/s² */
    float accelY;          /* 加速度计Y轴，单位：m/s² */
    float accelZ;          /* 加速度计Z轴，单位：m/s² */

    float gyroX;           /* 陀螺仪X轴，单位：rad/s */
    float gyroY;           /* 陀螺仪Y轴，单位：rad/s */
    float gyroZ;           /* 陀螺仪Z轴，单位：rad/s */

    float temperature;     /* 温度，单位：℃ */
};

/**************************实现类**********************************************
 *类    名:     ImuRingBuffer
 *功    能:     保存最近一段时间的IMU样本，供防抖算法按时间戳查询
 ******************************************************************************/
class ImuRingBuffer {
public:
    explicit ImuRingBuffer(size_t maxSamples = 512);

    void push(const ImuSample& sample);
    void clear();
    size_t size() const;

    bool latest(ImuSample& sample) const;
    bool getSamplesBetween(uint64_t startTimeNs,
                           uint64_t endTimeNs,
                           std::vector<ImuSample>& samples) const;

private:
    size_t maxSamples_;
    mutable std::mutex mutex_;
    std::deque<ImuSample> samples_;
};

/**************************实现类**********************************************
 *类    名:     Icm45686Reader
 *功    能:     封装/dev/icm45686访问，周期读取IMU数据并写入应用层环形缓冲区
 ******************************************************************************/
class Icm45686Reader {
public:
    explicit Icm45686Reader(size_t ringBufferSamples = 512);
    ~Icm45686Reader();

    bool openDevice(const std::string& devPath = "/dev/icm45686");
    void closeDevice();

    bool setAccelRange(uint8_t range);
    bool setGyroRange(uint8_t range);

    bool readSample(ImuSample& sample);

    bool start(float sampleHz = 100.0f);
    void stop();
    bool isRunning() const;

    bool getLatestSample(ImuSample& sample) const;
    bool getSamplesBetween(uint64_t startTimeNs,
                           uint64_t endTimeNs,
                           std::vector<ImuSample>& samples) const;

    size_t bufferedSamples() const;
    uint64_t totalSamples() const;
    uint64_t failedReads() const;

private:
    void readLoop();

private:
    int fd_;
    float sampleHz_;
    std::atomic<bool> running_;
    std::thread worker_;
    ImuRingBuffer ringBuffer_;
    std::atomic<uint64_t> totalSamples_;
    std::atomic<uint64_t> failedReads_;
};

/**************************实现类**********************************************
 *类    名:     EisStabilizer
 *功    能:     根据目标曝光时间附近的IMU数据，计算2D像素防抖补偿量
 ******************************************************************************/
class EisStabilizer {
public:
    EisStabilizer();

    bool bindReader(Icm45686Reader* reader);

    /**************************实现函数****************************************
     *函数原型:     bool calculate_eis_offset(float focalX, float focalY,
     *                                      uint64_t targetTimestampNs,
     *                                      uint32_t halfWindowMs,
     *                                      int32_t& offsetX, int32_t& offsetY)
     *功    能:     把目标时间窗口内的3D角速度积分结果转换为2D像素偏移
     *输入参数:     focalX - 水平焦距，单位：pixel
     *              focalY - 垂直焦距，单位：pixel
     *              targetTimestampNs - 当前帧中心曝光时间戳，单位：ns
     *              halfWindowMs - 时间窗口半径，单位：ms
     *输出参数:     offsetX - 水平方向补偿像素偏移量
     *              offsetY - 垂直方向补偿像素偏移量
     *返 回 值:     true成功，false失败
     *************************************************************************/
    bool calculate_eis_offset(float focalX,
                              float focalY,
                              uint64_t targetTimestampNs,
                              uint32_t halfWindowMs,
                              int32_t& offsetX,
                              int32_t& offsetY);

    double lastCostMs() const;
    size_t lastUsedSamples() const;

    void setAxisSign(float signX, float signY);
    void setMaxOffset(int32_t maxOffsetPixel);

private:
    bool integrateGyro(const std::vector<ImuSample>& samples,
                       float& thetaX,
                       float& thetaY,
                       float& thetaZ) const;

private:
    Icm45686Reader* reader_;
    double lastCostMs_;
    size_t lastUsedSamples_;
    float signX_;
    float signY_;
    int32_t maxOffsetPixel_;
};

uint64_t imu_get_time_ns();

#endif /* __IMU_EIS_HPP__ */
