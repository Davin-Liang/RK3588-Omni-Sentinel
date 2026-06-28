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
#include <map>
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

    float gyroX;           /* 陀螺仪X轴，单位：rad/s，已经扣除Reader中的gyro bias */
    float gyroY;           /* 陀螺仪Y轴，单位：rad/s，已经扣除Reader中的gyro bias */
    float gyroZ;           /* 陀螺仪Z轴，单位：rad/s，已经扣除Reader中的gyro bias */

    float temperature;     /* 温度，单位：℃ */
};

/*
 * IMU全局配置。
 * 这类参数影响IMU数据本身，通常在Reader启动前配置一次，不建议每帧动态修改。
 */
struct ImuConfig {
    float sampleHz;              /* 应用层读取频率，单位Hz，例如100/200/400 */
    uint8_t gyroRange;           /* 0/1/2/3 对应 ±250/±500/±1000/±2000DPS */
    uint8_t accelRange;          /* 0/1/2/3 对应 ±2G/±4G/±8G/±16G */

    bool enableGyroBiasCalib;    /* 是否启动时进行陀螺仪零偏标定 */
    uint32_t biasCalibMs;        /* 零偏标定时间，单位ms，标定期间IMU需要保持静止 */

    float gyroBiasX;             /* 陀螺仪X轴零偏，单位rad/s */
    float gyroBiasY;             /* 陀螺仪Y轴零偏，单位rad/s */
    float gyroBiasZ;             /* 陀螺仪Z轴零偏，单位rad/s */

    ImuConfig()
        : sampleHz(100.0f),
          gyroRange(0),
          accelRange(0),
          enableGyroBiasCalib(false),
          biasCalibMs(0),
          gyroBiasX(0.0f),
          gyroBiasY(0.0f),
          gyroBiasZ(0.0f)
    {
    }
};

/*
 * 单路相机EIS配置。
 * 这类参数和相机本身有关，双相机系统中建议每路相机各维护一份配置。
 */
struct EisCameraConfig {
    int camId;                   /* 摄像头编号，用于区分不同相机的平滑状态 */
    float frameRate;             /* 相机帧率，单位FPS，例如15/30 */
    float focalX;                /* 水平方向焦距，单位pixel */
    float focalY;                /* 垂直方向焦距，单位pixel */

    uint32_t halfWindowMs;       /* IMU积分半窗口，单位ms */
    int32_t maxOffsetPixel;      /* 最大像素补偿量，需小于视觉链路裁剪余量 */

    float timeOffsetMs;          /* 帧时间戳修正，单位ms，用于补偿相机和IMU时间偏差 */

    float signX;                 /* offsetX符号，+1或-1，用于适配安装方向 */
    float signY;                 /* offsetY符号，+1或-1，用于适配安装方向 */
    bool swapXY;                 /* 是否交换thetaX/thetaY到offset的映射 */

    bool enableSmoothing;        /* 是否对offset做一阶低通平滑 */
    float smoothingAlpha;        /* 平滑系数，范围(0,1]；越大响应越快，越小越平滑 */

    EisCameraConfig()
        : camId(0),
          frameRate(30.0f),
          focalX(1200.0f),
          focalY(1200.0f),
          halfWindowMs(20),
          maxOffsetPixel(200),
          timeOffsetMs(0.0f),
          signX(-1.0f),
          signY(1.0f),
          swapXY(false),
          enableSmoothing(false),
          smoothingAlpha(0.25f)
    {
    }
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

    /*
     * 按ImuConfig配置IMU量程、读取频率和零偏。
     * 该函数要求设备已经open，且通常应在start()之前调用。
     */
    bool configure(const ImuConfig& config);
    ImuConfig config() const;

    /*
     * 静置零偏标定：持续读取一段时间gyro均值，并写入当前配置。
     * 标定期间请保持IMU静止，否则会把真实运动误当成零偏扣掉。
     */
    bool calibrateGyroBias(uint32_t durationMs, float sampleHz = 100.0f);

    bool readSample(ImuSample& sample);

    bool start(float sampleHz);
    bool start();
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

    mutable std::mutex configMutex_;
    ImuConfig config_;
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
     *函数原型:     bool calculate_eis_offset(const EisCameraConfig& config,
     *                                      uint64_t frameTimestampNs,
     *                                      int32_t& offsetX, int32_t& offsetY)
     *功    能:     根据单路相机EIS配置计算像素补偿量
     *输入参数:     config - 单路相机防抖配置
     *              frameTimestampNs - 当前帧中心曝光时间戳，单位ns
     *输出参数:     offsetX - 水平方向补偿像素偏移量
     *              offsetY - 垂直方向补偿像素偏移量
     *返 回 值:     true成功，false失败
     *************************************************************************/
    bool calculate_eis_offset(const EisCameraConfig& config,
                              uint64_t frameTimestampNs,
                              int32_t& offsetX,
                              int32_t& offsetY);

    /*
     * 兼容旧接口：仍然支持直接传 focal/target/halfWindow。
     * 新项目建议使用 EisCameraConfig 版本，便于双相机分别配置参数。
     */
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
    void resetSmoothing(int camId = 0);

private:
    struct OffsetState {
        bool valid;
        float smoothX;
        float smoothY;

        OffsetState() : valid(false), smoothX(0.0f), smoothY(0.0f) {}
    };

    bool integrateGyro(const std::vector<ImuSample>& samples,
                       float& thetaX,
                       float& thetaY,
                       float& thetaZ) const;

    bool calculateWithResolvedParams(const EisCameraConfig& config,
                                     uint64_t targetTimestampNs,
                                     int32_t& offsetX,
                                     int32_t& offsetY);

private:
    Icm45686Reader* reader_;
    double lastCostMs_;
    size_t lastUsedSamples_;
    float signX_;
    float signY_;
    int32_t maxOffsetPixel_;

    std::map<int, OffsetState> offsetStates_;
};

uint64_t imu_get_time_ns();

#endif /* __IMU_EIS_HPP__ */
