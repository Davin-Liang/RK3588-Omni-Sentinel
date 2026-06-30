/*
 * imu_eis.cpp - ICM45686防抖应用层接口实现
 *
 * 基于当前已调通的 /dev/icm45686 字符设备方案实现IMU读取、环形缓冲区和EIS偏移计算接口
 *
 * 日期: 2026-06-08
 */

#include "imu_eis.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define NS_PER_MS 1000000ULL
#define NS_PER_SEC 1000000000ULL


static inline float vec3_norm(float x, float y, float z)
{
    return std::sqrt(x*x + y*y + z*z);
}

static inline void mat3_mul_vec3(const float M[9], const float v[3], float out[3])
{
    out[0] = M[0]*v[0] + M[1]*v[1] + M[2]*v[2];
    out[1] = M[3]*v[0] + M[4]*v[1] + M[5]*v[2];
    out[2] = M[6]*v[0] + M[7]*v[1] + M[8]*v[2];
}

static inline void mat3_mul(const float A[9], const float B[9], float C[9])
{
    float T[9];
    for (int r=0; r<3; ++r) {
        for (int c=0; c<3; ++c) {
            T[r*3+c] = A[r*3+0]*B[0*3+c] + A[r*3+1]*B[1*3+c] + A[r*3+2]*B[2*3+c];
        }
    }
    memcpy(C, T, sizeof(T));
}

static inline void mat3_transpose(const float A[9], float At[9])
{
    At[0]=A[0]; At[1]=A[3]; At[2]=A[6];
    At[3]=A[1]; At[4]=A[4]; At[5]=A[7];
    At[6]=A[2]; At[7]=A[5]; At[8]=A[8];
}

static inline void make_K(float fx, float fy, float cx, float cy, float K[9])
{
    K[0]=fx;   K[1]=0.0f; K[2]=cx;
    K[3]=0.0f; K[4]=fy;   K[5]=cy;
    K[6]=0.0f; K[7]=0.0f; K[8]=1.0f;
}

static inline void make_K_inv(float fx, float fy, float cx, float cy, float Kinv[9])
{
    Kinv[0]=1.0f/fx; Kinv[1]=0.0f;    Kinv[2]=-cx/fx;
    Kinv[3]=0.0f;    Kinv[4]=1.0f/fy; Kinv[5]=-cy/fy;
    Kinv[6]=0.0f;    Kinv[7]=0.0f;    Kinv[8]=1.0f;
}

static inline void quat_identity(float q[4])
{
    q[0]=1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f;
}

static inline void quat_normalize(float q[4])
{
    float n = std::sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
    if (n < 1e-9f) { quat_identity(q); return; }
    q[0]/=n; q[1]/=n; q[2]/=n; q[3]/=n;
}

static inline void quat_mul(const float a[4], const float b[4], float out[4])
{
    float t[4];
    t[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
    t[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
    t[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
    t[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
    memcpy(out, t, sizeof(t));
}

static inline void quat_inverse(const float q[4], float qi[4])
{
    qi[0]=q[0]; qi[1]=-q[1]; qi[2]=-q[2]; qi[3]=-q[3];
}

static inline void quat_from_rotvec(const float w[3], float q[4])
{
    float angle = vec3_norm(w[0], w[1], w[2]);
    if (angle < 1e-9f) { quat_identity(q); return; }
    float half = 0.5f * angle;
    float s = std::sin(half) / angle;
    q[0] = std::cos(half);
    q[1] = w[0] * s;
    q[2] = w[1] * s;
    q[3] = w[2] * s;
    quat_normalize(q);
}

static inline void quat_to_mat3(const float qIn[4], float R[9])
{
    float q[4] = {qIn[0], qIn[1], qIn[2], qIn[3]};
    quat_normalize(q);
    float w=q[0], x=q[1], y=q[2], z=q[3];
    R[0]=1.0f-2.0f*(y*y+z*z); R[1]=2.0f*(x*y-w*z);     R[2]=2.0f*(x*z+w*y);
    R[3]=2.0f*(x*y+w*z);     R[4]=1.0f-2.0f*(x*x+z*z); R[5]=2.0f*(y*z-w*x);
    R[6]=2.0f*(x*z-w*y);     R[7]=2.0f*(y*z+w*x);     R[8]=1.0f-2.0f*(x*x+y*y);
}

static inline void quat_slerp(const float q0[4], const float q1[4], float alpha, float out[4])
{
    float b[4] = {q1[0], q1[1], q1[2], q1[3]};
    float dot = q0[0]*b[0] + q0[1]*b[1] + q0[2]*b[2] + q0[3]*b[3];
    if (dot < 0.0f) { dot=-dot; b[0]=-b[0]; b[1]=-b[1]; b[2]=-b[2]; b[3]=-b[3]; }
    if (dot > 0.9995f) {
        out[0] = q0[0] + alpha*(b[0]-q0[0]);
        out[1] = q0[1] + alpha*(b[1]-q0[1]);
        out[2] = q0[2] + alpha*(b[2]-q0[2]);
        out[3] = q0[3] + alpha*(b[3]-q0[3]);
        quat_normalize(out);
        return;
    }
    float theta0 = std::acos(std::max(-1.0f, std::min(1.0f, dot)));
    float theta = theta0 * alpha;
    float sinTheta = std::sin(theta);
    float sinTheta0 = std::sin(theta0);
    float s0 = std::cos(theta) - dot * sinTheta / sinTheta0;
    float s1 = sinTheta / sinTheta0;
    out[0] = s0*q0[0] + s1*b[0];
    out[1] = s0*q0[1] + s1*b[1];
    out[2] = s0*q0[2] + s1*b[2];
    out[3] = s0*q0[3] + s1*b[3];
    quat_normalize(out);
}

static inline float quat_angle(const float q[4])
{
    float w = std::max(-1.0f, std::min(1.0f, q[0]));
    return 2.0f * std::acos(w);
}

static inline void quat_clamp_angle(float q[4], float maxAngle)
{
    if (maxAngle <= 0.0f) return;
    quat_normalize(q);
    float angle = quat_angle(q);
    if (angle <= maxAngle || angle < 1e-9f) return;
    float scale = maxAngle / angle;
    float rotvec[3];
    float sinHalf = std::sqrt(std::max(0.0f, 1.0f - q[0]*q[0]));
    if (sinHalf < 1e-6f) return;
    rotvec[0] = q[1] / sinHalf * angle * scale;
    rotvec[1] = q[2] / sinHalf * angle * scale;
    rotvec[2] = q[3] / sinHalf * angle * scale;
    quat_from_rotvec(rotvec, q);
}

static inline void apply_homography_center(const float H[9], float x, float y, float& ox, float& oy)
{
    float w = H[6]*x + H[7]*y + H[8];
    if (std::fabs(w) < 1e-6f) { ox=x; oy=y; return; }
    ox = (H[0]*x + H[1]*y + H[2]) / w;
    oy = (H[3]*x + H[4]*y + H[5]) / w;
}

static inline void quat_to_euler_xyz(const float qIn[4], float e[3])
{
    float R[9]; quat_to_mat3(qIn, R);
    e[0] = std::atan2(R[7], R[8]);
    e[1] = std::asin(std::max(-1.0f, std::min(1.0f, -R[6])));
    e[2] = std::atan2(R[3], R[0]);
}

/**************************实现函数********************************************
 *函数原型:     uint64_t imu_get_time_ns(void)
 *功    能:     获取单调递增时间戳，单位ns
 *输入参数:     无
 *输出参数:     时间戳，单位ns
 ******************************************************************************/
uint64_t imu_get_time_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * NS_PER_SEC + (uint64_t)ts.tv_nsec;
}

/**************************实现函数********************************************
 *函数原型:     ImuRingBuffer::ImuRingBuffer(size_t maxSamples)
 *功    能:     构造IMU环形缓冲区
 *输入参数:     maxSamples - 最大缓存样本数量
 *输出参数:     无
 ******************************************************************************/
ImuRingBuffer::ImuRingBuffer(size_t maxSamples)
    : maxSamples_(maxSamples)
{
}

/**************************实现函数********************************************
 *函数原型:     void ImuRingBuffer::push(const ImuSample& sample)
 *功    能:     写入一个IMU样本
 *输入参数:     sample - IMU样本
 *输出参数:     无
 ******************************************************************************/
void ImuRingBuffer::push(const ImuSample& sample)
{
    std::lock_guard<std::mutex> lock(mutex_);

    samples_.push_back(sample);
    while (samples_.size() > maxSamples_) {
        samples_.pop_front();
    }
}

/**************************实现函数********************************************
 *函数原型:     void ImuRingBuffer::clear(void)
 *功    能:     清空环形缓冲区
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void ImuRingBuffer::clear()
{
    std::lock_guard<std::mutex> lock(mutex_);
    samples_.clear();
}

/**************************实现函数********************************************
 *函数原型:     size_t ImuRingBuffer::size(void) const
 *功    能:     获取当前缓存样本数量
 *输入参数:     无
 *输出参数:     样本数量
 ******************************************************************************/
size_t ImuRingBuffer::size() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return samples_.size();
}

/**************************实现函数********************************************
 *函数原型:     bool ImuRingBuffer::latest(ImuSample& sample) const
 *功    能:     获取最新IMU样本
 *输入参数:     sample - 输出样本
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool ImuRingBuffer::latest(ImuSample& sample) const
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (samples_.empty()) {
        return false;
    }

    sample = samples_.back();
    return true;
}

/**************************实现函数********************************************
 *函数原型:     bool ImuRingBuffer::getSamplesBetween(uint64_t startTimeNs,
 *                                                    uint64_t endTimeNs,
 *                                                    std::vector<ImuSample>& samples) const
 *功    能:     按时间范围获取IMU样本
 *输入参数:     startTimeNs - 起始时间戳，endTimeNs - 结束时间戳
 *输出参数:     samples - 输出样本列表
 *返 回 值:     true获取到样本，false没有样本
 ******************************************************************************/
bool ImuRingBuffer::getSamplesBetween(uint64_t startTimeNs,
                                      uint64_t endTimeNs,
                                      std::vector<ImuSample>& samples) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    samples.clear();
    if (startTimeNs > endTimeNs) {
        return false;
    }

    for (const auto& sample : samples_) {
        if (sample.timestampNs >= startTimeNs && sample.timestampNs <= endTimeNs) {
            samples.push_back(sample);
        }
    }

    return !samples.empty();
}

/**************************实现函数********************************************
 *函数原型:     Icm45686Reader::Icm45686Reader(size_t ringBufferSamples)
 *功    能:     构造ICM45686读取器
 *输入参数:     ringBufferSamples - 环形缓冲区最大样本数量
 *输出参数:     无
 ******************************************************************************/
Icm45686Reader::Icm45686Reader(size_t ringBufferSamples)
    : fd_(-1),
      sampleHz_(100.0f),
      running_(false),
      ringBuffer_(ringBufferSamples),
      totalSamples_(0),
      failedReads_(0)
{
}

/**************************实现函数********************************************
 *函数原型:     Icm45686Reader::~Icm45686Reader(void)
 *功    能:     析构读取器，停止线程并关闭设备
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
Icm45686Reader::~Icm45686Reader()
{
    stop();
    closeDevice();
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::openDevice(const std::string& devPath)
 *功    能:     打开ICM45686字符设备
 *输入参数:     devPath - 设备节点路径
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::openDevice(const std::string& devPath)
{
    closeDevice();

    fd_ = icm45686_open(devPath.c_str());
    if (fd_ < 0) {
        return false;
    }

    return true;
}

/**************************实现函数********************************************
 *函数原型:     void Icm45686Reader::closeDevice(void)
 *功    能:     关闭ICM45686字符设备
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void Icm45686Reader::closeDevice()
{
    if (fd_ >= 0) {
        icm45686_close(fd_);
        fd_ = -1;
    }
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::setAccelRange(uint8_t range)
 *功    能:     设置加速度计量程
 *输入参数:     range - 0/1/2/3对应±2G/±4G/±8G/±16G
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::setAccelRange(uint8_t range)
{
    if (fd_ < 0) {
        return false;
    }

    return icm45686_set_accel_fs(fd_, range) == 0;
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::setGyroRange(uint8_t range)
 *功    能:     设置陀螺仪量程
 *输入参数:     range - 0/1/2/3对应±250/±500/±1000/±2000DPS
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::setGyroRange(uint8_t range)
{
    if (fd_ < 0) {
        return false;
    }

    return icm45686_set_gyro_fs(fd_, range) == 0;
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::readSample(ImuSample& sample)
 *功    能:     主动读取一个IMU样本并打时间戳
 *输入参数:     sample - 输出样本
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::readSample(ImuSample& sample)
{
    icm45686_data_t data;

    if (fd_ < 0) {
        return false;
    }

    memset(&data, 0, sizeof(data));
    if (icm45686_read_data(fd_, &data) < 0) {
        return false;
    }

    sample.timestampNs = imu_get_time_ns();
    sample.accelX = data.accel_x;
    sample.accelY = data.accel_y;
    sample.accelZ = data.accel_z;
    sample.gyroX = data.gyro_x;
    sample.gyroY = data.gyro_y;
    sample.gyroZ = data.gyro_z;
    sample.temperature = data.temp;

    return true;
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::start(float sampleHz)
 *功    能:     启动后台读取线程
 *输入参数:     sampleHz - 应用层读取频率
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::start(float sampleHz)
{
    if (fd_ < 0 || sampleHz <= 0.0f) {
        return false;
    }

    if (running_.load()) {
        return true;
    }

    sampleHz_ = sampleHz;
    running_.store(true);
    worker_ = std::thread(&Icm45686Reader::readLoop, this);
    return true;
}

/**************************实现函数********************************************
 *函数原型:     void Icm45686Reader::stop(void)
 *功    能:     停止后台读取线程
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void Icm45686Reader::stop()
{
    if (!running_.load()) {
        return;
    }

    running_.store(false);
    if (worker_.joinable()) {
        worker_.join();
    }
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::isRunning(void) const
 *功    能:     查询读取线程是否正在运行
 *输入参数:     无
 *输出参数:     true运行中，false未运行
 ******************************************************************************/
bool Icm45686Reader::isRunning() const
{
    return running_.load();
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::getLatestSample(ImuSample& sample) const
 *功    能:     获取最新IMU样本
 *输入参数:     sample - 输出样本
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::getLatestSample(ImuSample& sample) const
{
    return ringBuffer_.latest(sample);
}

/**************************实现函数********************************************
 *函数原型:     bool Icm45686Reader::getSamplesBetween(uint64_t startTimeNs,
 *                                                     uint64_t endTimeNs,
 *                                                     std::vector<ImuSample>& samples) const
 *功    能:     从环形缓冲区中按时间范围获取样本
 *输入参数:     startTimeNs - 起始时间戳，endTimeNs - 结束时间戳
 *输出参数:     samples - 样本列表
 *返 回 值:     true成功，false失败
 ******************************************************************************/
bool Icm45686Reader::getSamplesBetween(uint64_t startTimeNs,
                                       uint64_t endTimeNs,
                                       std::vector<ImuSample>& samples) const
{
    return ringBuffer_.getSamplesBetween(startTimeNs, endTimeNs, samples);
}

bool Icm45686Reader::getAssistState(ImuAssistState& state, uint32_t windowMs) const
{
    ImuSample latestSample;
    if (!getLatestSample(latestSample)) {
        return false;
    }

    uint64_t endNs = latestSample.timestampNs;
    uint64_t windowNs = (uint64_t)windowMs * NS_PER_MS;
    uint64_t startNs = endNs > windowNs ? endNs - windowNs : 0;

    std::vector<ImuSample> samples;
    if (!getSamplesBetween(startNs, endNs, samples)) {
        samples.push_back(latestSample);
    }

    double gyro2Sum = 0.0;
    for (const auto& s : samples) {
        gyro2Sum += (double)s.gyroX * s.gyroX + (double)s.gyroY * s.gyroY + (double)s.gyroZ * s.gyroZ;
    }

    state.timestampNs = latestSample.timestampNs;
    state.accelX = latestSample.accelX;
    state.accelY = latestSample.accelY;
    state.accelZ = latestSample.accelZ;
    state.gyroX = latestSample.gyroX;
    state.gyroY = latestSample.gyroY;
    state.gyroZ = latestSample.gyroZ;
    state.accelNorm = vec3_norm(state.accelX, state.accelY, state.accelZ);
    state.gyroNorm = vec3_norm(state.gyroX, state.gyroY, state.gyroZ);
    state.gyroRms = samples.empty() ? state.gyroNorm : (float)std::sqrt(gyro2Sum / (double)samples.size());

    if (state.gyroRms < 0.03f) {
        state.vibrationLevel = 0;
    } else if (state.gyroRms < 0.15f) {
        state.vibrationLevel = 1;
    } else {
        state.vibrationLevel = 2;
    }

    return true;
}

size_t Icm45686Reader::bufferedSamples() const
{
    return ringBuffer_.size();
}

uint64_t Icm45686Reader::totalSamples() const
{
    return totalSamples_.load();
}

uint64_t Icm45686Reader::failedReads() const
{
    return failedReads_.load();
}

/**************************实现函数********************************************
 *函数原型:     void Icm45686Reader::readLoop(void)
 *功    能:     后台线程读取函数
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
void Icm45686Reader::readLoop()
{
    const uint64_t periodNs = (uint64_t)(NS_PER_SEC / sampleHz_);
    uint64_t nextTime = imu_get_time_ns();

    while (running_.load()) {
        ImuSample sample;

        if (readSample(sample)) {
            ringBuffer_.push(sample);
            totalSamples_.fetch_add(1);
        } else {
            failedReads_.fetch_add(1);
        }

        nextTime += periodNs;
        uint64_t now = imu_get_time_ns();
        if (nextTime > now) {
            uint64_t sleepNs = nextTime - now;
            struct timespec ts;
            ts.tv_sec = sleepNs / NS_PER_SEC;
            ts.tv_nsec = sleepNs % NS_PER_SEC;
            nanosleep(&ts, NULL);
        } else {
            nextTime = now;
        }
    }
}

/**************************实现函数********************************************
 *函数原型:     EisStabilizer::EisStabilizer(void)
 *功    能:     构造EIS防抖计算器
 *输入参数:     无
 *输出参数:     无
 ******************************************************************************/
EisStabilizer::EisStabilizer()
    : reader_(NULL),
      lastCostMs_(0.0),
      lastUsedSamples_(0),
      signX_(-1.0f),
      signY_(1.0f),
      maxOffsetPixel_(200)
{
}

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::bindReader(Icm45686Reader* reader)
 *功    能:     绑定IMU读取器
 *输入参数:     reader - 读取器指针
 *输出参数:     true成功，false失败
 ******************************************************************************/
bool EisStabilizer::bindReader(Icm45686Reader* reader)
{
    reader_ = reader;
    return reader_ != NULL;
}

/**************************实现函数********************************************
 *函数原型:     void EisStabilizer::setAxisSign(float signX, float signY)
 *功    能:     设置像素偏移方向符号，用于适配IMU安装方向和图像坐标系
 *输入参数:     signX - X方向符号，signY - Y方向符号
 *输出参数:     无
 ******************************************************************************/
void EisStabilizer::setAxisSign(float signX, float signY)
{
    signX_ = signX >= 0.0f ? 1.0f : -1.0f;
    signY_ = signY >= 0.0f ? 1.0f : -1.0f;
}

/**************************实现函数********************************************
 *函数原型:     void EisStabilizer::setMaxOffset(int32_t maxOffsetPixel)
 *功    能:     设置最大补偿像素偏移，防止异常陀螺数据导致输出过大
 *输入参数:     maxOffsetPixel - 最大偏移像素
 *输出参数:     无
 ******************************************************************************/
void EisStabilizer::setMaxOffset(int32_t maxOffsetPixel)
{
    maxOffsetPixel_ = maxOffsetPixel > 0 ? maxOffsetPixel : 200;
}

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::integrateGyro(const std::vector<ImuSample>& samples,
 *                                                float& thetaX, float& thetaY, float& thetaZ) const
 *功    能:     对时间窗口内陀螺仪角速度进行梯形积分
 *输入参数:     samples - IMU样本列表
 *输出参数:     thetaX/thetaY/thetaZ - 三轴角度增量，单位rad
 *返 回 值:     true成功，false失败
 ******************************************************************************/
bool EisStabilizer::integrateGyro(const std::vector<ImuSample>& samples,
                                  float& thetaX,
                                  float& thetaY,
                                  float& thetaZ) const
{
    thetaX = 0.0f;
    thetaY = 0.0f;
    thetaZ = 0.0f;

    if (samples.size() < 2) {
        return false;
    }

    for (size_t i = 1; i < samples.size(); ++i) {
        uint64_t dtNs = samples[i].timestampNs - samples[i - 1].timestampNs;
        float dt = (float)((double)dtNs / (double)NS_PER_SEC);

        thetaX += 0.5f * (samples[i - 1].gyroX + samples[i].gyroX) * dt;
        thetaY += 0.5f * (samples[i - 1].gyroY + samples[i].gyroY) * dt;
        thetaZ += 0.5f * (samples[i - 1].gyroZ + samples[i].gyroZ) * dt;
    }

    return true;
}

/**************************实现函数********************************************
 *函数原型:     bool EisStabilizer::calculate_eis_offset(float focalX, float focalY,
 *                                                       uint64_t targetTimestampNs,
 *                                                       uint32_t halfWindowMs,
 *                                                       int32_t& offsetX, int32_t& offsetY)
 *功    能:     计算防抖像素补偿量
 *输入参数:     focalX/focalY - 相机焦距，单位pixel；targetTimestampNs - 目标曝光时间戳；halfWindowMs - 时间窗口半径
 *输出参数:     offsetX/offsetY - 像素补偿量
 *返 回 值:     true成功，false失败
 ******************************************************************************/
bool EisStabilizer::calculate_eis_offset(float focalX,
                                         float focalY,
                                         uint64_t targetTimestampNs,
                                         uint32_t halfWindowMs,
                                         int32_t& offsetX,
                                         int32_t& offsetY)
{
    uint64_t costStartNs = imu_get_time_ns();
    uint64_t windowNs = (uint64_t)halfWindowMs * NS_PER_MS;
    uint64_t startNs;
    uint64_t endNs;
    std::vector<ImuSample> samples;
    float thetaX, thetaY, thetaZ;

    offsetX = 0;
    offsetY = 0;
    lastUsedSamples_ = 0;
    lastCostMs_ = 0.0;

    if (!reader_ || focalX <= 0.0f || focalY <= 0.0f || halfWindowMs == 0) {
        return false;
    }

    startNs = targetTimestampNs > windowNs ? targetTimestampNs - windowNs : 0;
    endNs = targetTimestampNs + windowNs;

    if (!reader_->getSamplesBetween(startNs, endNs, samples)) {
        return false;
    }

    lastUsedSamples_ = samples.size();
    if (!integrateGyro(samples, thetaX, thetaY, thetaZ)) {
        return false;
    }

    /*
     * 小角度近似：像素偏移 ≈ 焦距(pixel) × 角度(rad)。
     * 当前默认映射：gyroY影响水平画面偏移，gyroX影响垂直画面偏移。
     * 实际产品中需要根据IMU安装方向和相机坐标系调整符号或轴向。
     */
    offsetX = (int32_t)lroundf(signX_ * focalX * thetaY);
    offsetY = (int32_t)lroundf(signY_ * focalY * thetaX);

    offsetX = std::max(-maxOffsetPixel_, std::min(maxOffsetPixel_, offsetX));
    offsetY = std::max(-maxOffsetPixel_, std::min(maxOffsetPixel_, offsetY));

    lastCostMs_ = (double)(imu_get_time_ns() - costStartNs) / 1000000.0;
    return true;
}


void EisStabilizer::setImuOnlyConfig(int camId, const ImuOnlyEisConfig& config)
{
    if (camId < 0 || camId >= 2) {
        return;
    }
    imuOnlyCfg_[camId] = config;
    resetImuOnlyState(camId);
}

void EisStabilizer::resetImuOnlyState(int camId)
{
    if (camId >= 0 && camId < 2) {
        imuOnlyState_[camId] = ImuOnlyCameraState();
        return;
    }
    imuOnlyState_[0] = ImuOnlyCameraState();
    imuOnlyState_[1] = ImuOnlyCameraState();
}

bool EisStabilizer::calculate_imu_only_eis_offset(int camId,
                                                  uint64_t frameTimestampNs,
                                                  int32_t& offsetX,
                                                  int32_t& offsetY,
                                                  ImuOnlyEisOutput* out)
{
    uint64_t costStartNs = imu_get_time_ns();
    offsetX = 0;
    offsetY = 0;
    lastUsedSamples_ = 0;
    lastCostMs_ = 0.0;

    if (out) {
        *out = ImuOnlyEisOutput();
    }

    if (!reader_ || camId < 0 || camId >= 2) {
        return false;
    }

    ImuOnlyEisConfig& cfg = imuOnlyCfg_[camId];
    ImuOnlyCameraState& st = imuOnlyState_[camId];
    const uint64_t targetNs = frameTimestampNs + (uint64_t)cfg.timeOffsetNs;

    ImuSample latest;
    if (!reader_->getLatestSample(latest)) {
        return false;
    }

    if (!st.initialized) {
        quat_identity(st.qRaw);
        quat_identity(st.qSmooth);
        st.lastIntegratedNs = latest.timestampNs;
        st.lastFrameNs = targetNs;
        st.initialized = true;
        if (out) {
            out->valid = true;
        }
        return true;
    }

    uint64_t startNs = st.lastIntegratedNs;
    if (startNs == 0 || startNs > targetNs) {
        startNs = targetNs > 50 * NS_PER_MS ? targetNs - 50 * NS_PER_MS : 0;
    }

    std::vector<ImuSample> samples;
    if (!reader_->getSamplesBetween(startNs, targetNs, samples)) {
        uint64_t fallbackStart = latest.timestampNs > 50 * NS_PER_MS ? latest.timestampNs - 50 * NS_PER_MS : 0;
        reader_->getSamplesBetween(fallbackStart, latest.timestampNs, samples);
    }

    if (samples.size() < 2) {
        return false;
    }

    lastUsedSamples_ = samples.size();

    float lastGyroRaw[3] = {0.0f, 0.0f, 0.0f};
    float lastGyroB[3] = {0.0f, 0.0f, 0.0f};

    for (size_t i = 1; i < samples.size(); ++i) {
        if (samples[i].timestampNs <= samples[i - 1].timestampNs) {
            continue;
        }

        float dt = (float)((double)(samples[i].timestampNs - samples[i - 1].timestampNs) / (double)NS_PER_SEC);
        if (dt <= 0.0f || dt > 0.05f) {
            continue;
        }

        float gyroRawAvg[3] = {
            0.5f * (samples[i - 1].gyroX + samples[i].gyroX),
            0.5f * (samples[i - 1].gyroY + samples[i].gyroY),
            0.5f * (samples[i - 1].gyroZ + samples[i].gyroZ)
        };
        float gyroB[3];
        mat3_mul_vec3(cfg.R_B_imu_raw, gyroRawAvg, gyroB);

        float dtheta[3] = {gyroB[0] * dt, gyroB[1] * dt, gyroB[2] * dt};
        float dq[4];
        quat_from_rotvec(dtheta, dq);

        float qNew[4];
        quat_mul(st.qRaw, dq, qNew);
        memcpy(st.qRaw, qNew, sizeof(qNew));
        quat_normalize(st.qRaw);

        memcpy(lastGyroRaw, gyroRawAvg, sizeof(lastGyroRaw));
        memcpy(lastGyroB, gyroB, sizeof(lastGyroB));
    }

    st.lastIntegratedNs = samples.back().timestampNs;

    float dtFrame = 1.0f / 30.0f;
    if (st.lastFrameNs > 0 && targetNs > st.lastFrameNs) {
        dtFrame = (float)((double)(targetNs - st.lastFrameNs) / (double)NS_PER_SEC);
        if (dtFrame <= 0.0f || dtFrame > 0.2f) {
            dtFrame = 1.0f / 30.0f;
        }
    }
    st.lastFrameNs = targetNs;

    float tau = cfg.smoothTauSec > 0.001f ? cfg.smoothTauSec : 0.25f;
    float alpha = 1.0f - std::exp(-dtFrame / tau);
    if (alpha < 0.001f) alpha = 0.001f;
    if (alpha > 1.0f) alpha = 1.0f;

    float qSmoothNew[4];
    quat_slerp(st.qSmooth, st.qRaw, alpha, qSmoothNew);
    memcpy(st.qSmooth, qSmoothNew, sizeof(qSmoothNew));
    quat_normalize(st.qSmooth);

    float qRawInv[4];
    quat_inverse(st.qRaw, qRawInv);

    float qCompB[4];
    quat_mul(st.qSmooth, qRawInv, qCompB);
    quat_normalize(qCompB);
    quat_clamp_angle(qCompB, cfg.maxCompAngleRad);

    float RCompB[9];
    quat_to_mat3(qCompB, RCompB);

    float RCB[9];
    memcpy(RCB, cfg.extr.R_C_B, sizeof(RCB));
    float RBC[9];
    mat3_transpose(RCB, RBC);

    float tmp[9];
    float RCompC[9];
    mat3_mul(RCB, RCompB, tmp);
    mat3_mul(tmp, RBC, RCompC);

    float K[9], Kinv[9];
    make_K(cfg.intr.fx, cfg.intr.fy, cfg.intr.cx, cfg.intr.cy, K);
    make_K_inv(cfg.intr.fx, cfg.intr.fy, cfg.intr.cx, cfg.intr.cy, Kinv);

    float Htmp[9];
    float H[9];
    mat3_mul(K, RCompC, Htmp);
    mat3_mul(Htmp, Kinv, H);
    if (std::fabs(H[8]) > 1e-6f) {
        float inv = 1.0f / H[8];
        for (int i=0; i<9; ++i) H[i] *= inv;
    }

    float cx = cfg.intr.cx;
    float cy = cfg.intr.cy;
    float centerAfterX = cx;
    float centerAfterY = cy;
    apply_homography_center(H, cx, cy, centerAfterX, centerAfterY);

    int32_t ox = (int32_t)lroundf(cx - centerAfterX);
    int32_t oy = (int32_t)lroundf(cy - centerAfterY);

    if (cfg.maxOffsetPixel >= 0) {
        ox = std::max(-cfg.maxOffsetPixel, std::min(cfg.maxOffsetPixel, ox));
        oy = std::max(-cfg.maxOffsetPixel, std::min(cfg.maxOffsetPixel, oy));
    }

    if (cfg.maxOffsetStepPixel > 0) {
        ox = std::max(st.lastOffsetX - cfg.maxOffsetStepPixel,
                      std::min(st.lastOffsetX + cfg.maxOffsetStepPixel, ox));
        oy = std::max(st.lastOffsetY - cfg.maxOffsetStepPixel,
                      std::min(st.lastOffsetY + cfg.maxOffsetStepPixel, oy));
    }

    st.lastOffsetX = ox;
    st.lastOffsetY = oy;
    offsetX = ox;
    offsetY = oy;

    if (out) {
        out->valid = true;
        memcpy(out->H, H, sizeof(out->H));
        out->offsetX = ox;
        out->offsetY = oy;
        out->rollRad = std::atan2(RCompC[3], RCompC[0]);
        memcpy(out->gyroRaw, lastGyroRaw, sizeof(out->gyroRaw));
        memcpy(out->gyroB, lastGyroB, sizeof(out->gyroB));
        mat3_mul_vec3(RCB, lastGyroB, out->gyroCam);
        quat_to_euler_xyz(st.qRaw, out->rawAngleB);
        quat_to_euler_xyz(st.qSmooth, out->smoothAngleB);
    }

    lastCostMs_ = (double)(imu_get_time_ns() - costStartNs) / 1000000.0;

    if (cfg.debugLog) {
        float gyroCam[3];
        mat3_mul_vec3(RCB, lastGyroB, gyroCam);
        fprintf(stderr,
                "[IMU-only EIS Cam %d] offset=(%d,%d) gyroRaw=(%.4f,%.4f,%.4f) gyroB=(%.4f,%.4f,%.4f) gyroCam=(%.4f,%.4f,%.4f) roll=%.4f cost=%.3fms samples=%zu\n",
                camId, (int)offsetX, (int)offsetY,
                lastGyroRaw[0], lastGyroRaw[1], lastGyroRaw[2],
                lastGyroB[0], lastGyroB[1], lastGyroB[2],
                gyroCam[0], gyroCam[1], gyroCam[2],
                out ? out->rollRad : 0.0f,
                lastCostMs_, lastUsedSamples_);
    }

    return true;
}


double EisStabilizer::lastCostMs() const
{
    return lastCostMs_;
}

size_t EisStabilizer::lastUsedSamples() const
{
    return lastUsedSamples_;
}

