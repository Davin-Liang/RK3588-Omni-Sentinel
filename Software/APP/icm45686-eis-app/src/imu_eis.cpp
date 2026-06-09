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

double EisStabilizer::lastCostMs() const
{
    return lastCostMs_;
}

size_t EisStabilizer::lastUsedSamples() const
{
    return lastUsedSamples_;
}
