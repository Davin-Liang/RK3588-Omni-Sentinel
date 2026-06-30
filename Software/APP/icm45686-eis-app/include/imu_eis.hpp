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

/* IMU辅助状态：给上层或视觉模块读取最近一段窗口内的震动状态。 */
struct ImuAssistState {
    uint64_t timestampNs;
    float accelX, accelY, accelZ;
    float gyroX, gyroY, gyroZ;
    float accelNorm;
    float gyroNorm;
    float gyroRms;
    int vibrationLevel; /* 0:低震动 1:中震动 2:高震动 */

    ImuAssistState()
        : timestampNs(0), accelX(0.0f), accelY(0.0f), accelZ(0.0f),
          gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f), accelNorm(0.0f),
          gyroNorm(0.0f), gyroRms(0.0f), vibrationLevel(0) {}
};

struct ImuEisCameraIntrinsics {
    int width;
    int height;
    float fx;
    float fy;
    float cx;
    float cy;

    ImuEisCameraIntrinsics()
        : width(1920), height(1080), fx(1000.0f), fy(1000.0f), cx(960.0f), cy(540.0f) {}
};

struct ImuEisCameraExtrinsic {
    /* R_C_B: body/mechanical frame B -> camera frame C, row-major 3x3 */
    float R_C_B[9];
    /* t_B: IMU center -> camera optical center, meter, expressed in B frame */
    float t_B[3];

    ImuEisCameraExtrinsic() {
        R_C_B[0]=1.0f; R_C_B[1]=0.0f; R_C_B[2]=0.0f;
        R_C_B[3]=0.0f; R_C_B[4]=1.0f; R_C_B[5]=0.0f;
        R_C_B[6]=0.0f; R_C_B[7]=0.0f; R_C_B[8]=1.0f;
        t_B[0]=0.0f; t_B[1]=0.0f; t_B[2]=0.0f;
    }
};

struct ImuOnlyEisConfig {
    ImuEisCameraIntrinsics intr;
    ImuEisCameraExtrinsic extr;

    /* R_B_imu_raw: IMU driver raw frame -> body/mechanical frame B, row-major 3x3.
     * User measured mapping:
     *   gyro_B.x = -gyro_raw.y
     *   gyro_B.y = -gyro_raw.x
     *   gyro_B.z =  gyro_raw.z
     */
    float R_B_imu_raw[9];

    int64_t timeOffsetNs;
    float smoothTauSec;
    float maxCompAngleRad;
    int32_t maxOffsetPixel;
    int32_t maxOffsetStepPixel;
    bool enableLeverArmCompensation;
    float nominalDepthMeter;
    bool debugLog;

    ImuOnlyEisConfig()
        : timeOffsetNs(0), smoothTauSec(0.25f), maxCompAngleRad(5.0f * 3.1415926535f / 180.0f),
          maxOffsetPixel(80), maxOffsetStepPixel(8), enableLeverArmCompensation(false),
          nominalDepthMeter(1.5f), debugLog(false) {
        R_B_imu_raw[0]=0.0f;  R_B_imu_raw[1]=-1.0f; R_B_imu_raw[2]=0.0f;
        R_B_imu_raw[3]=-1.0f; R_B_imu_raw[4]=0.0f;  R_B_imu_raw[5]=0.0f;
        R_B_imu_raw[6]=0.0f;  R_B_imu_raw[7]=0.0f;  R_B_imu_raw[8]=1.0f;
    }
};

struct ImuOnlyEisOutput {
    bool valid;
    float H[9];
    int32_t offsetX;
    int32_t offsetY;
    float rollRad;
    float gyroRaw[3];
    float gyroB[3];
    float gyroCam[3];
    float rawAngleB[3];
    float smoothAngleB[3];

    ImuOnlyEisOutput() : valid(false), offsetX(0), offsetY(0), rollRad(0.0f) {
        for (int i=0;i<9;++i) H[i] = (i % 4 == 0) ? 1.0f : 0.0f;
        for (int i=0;i<3;++i) {
            gyroRaw[i]=gyroB[i]=gyroCam[i]=rawAngleB[i]=smoothAngleB[i]=0.0f;
        }
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

    bool readSample(ImuSample& sample);

    bool start(float sampleHz = 100.0f);
    void stop();
    bool isRunning() const;

    bool getLatestSample(ImuSample& sample) const;
    bool getSamplesBetween(uint64_t startTimeNs,
                           uint64_t endTimeNs,
                           std::vector<ImuSample>& samples) const;

    bool getAssistState(ImuAssistState& state,
                        uint32_t windowMs = 200) const;

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

    void setImuOnlyConfig(int camId, const ImuOnlyEisConfig& config);
    bool calculate_imu_only_eis_offset(int camId,
                                       uint64_t frameTimestampNs,
                                       int32_t& offsetX,
                                       int32_t& offsetY,
                                       ImuOnlyEisOutput* out = nullptr);
    void resetImuOnlyState(int camId = -1);

private:
    bool integrateGyro(const std::vector<ImuSample>& samples,
                       float& thetaX,
                       float& thetaY,
                       float& thetaZ) const;

private:
    struct ImuOnlyCameraState {
        bool initialized;
        uint64_t lastIntegratedNs;
        uint64_t lastFrameNs;
        float qRaw[4];
        float qSmooth[4];
        int32_t lastOffsetX;
        int32_t lastOffsetY;
        ImuOnlyCameraState()
            : initialized(false), lastIntegratedNs(0), lastFrameNs(0),
              qRaw{1.0f,0.0f,0.0f,0.0f}, qSmooth{1.0f,0.0f,0.0f,0.0f},
              lastOffsetX(0), lastOffsetY(0) {}
    };

    ImuOnlyEisConfig imuOnlyCfg_[2];
    ImuOnlyCameraState imuOnlyState_[2];

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

