/*
 * imu_ahrs.c - Madgwick四元数姿态解算算法实现
 *
 * 适用于RK3588平台的ICM45686传感器应用
 *
 * 日期: 2026-05-05
 */

#include "imu_ahrs.h"
#include <math.h>

/* 四元数 */
static volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

/* 滤波参数 */
static volatile float beta = 0.1f;

/**************************实现函数********************************************
 *函数原型:     static float invSqrt(float x)
 *功　　能:     计算1/sqrt(x)，使用快速近似算法
 *输入参数:     x - 输入值
 *输出参数:     1/sqrt(x)的近似值
 ******************************************************************************/
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    
    return y;
}

/**************************实现函数********************************************
 *函数原型:     void IMU_AHRSupdate(float gx, float gy, float gz, 
 *                                   float ax, float ay, float az, 
 *                                   float mx, float my, float mz, 
 *                                   float sampleFreq)
 *功　　能:     更新四元数，使用Madgwick算法融合陀螺仪、加速度计和磁力计数据
 *输入参数:     gx/gy/gz - 陀螺仪数据 (rad/s)
 *              ax/ay/az - 加速度计数据 (m/s²)
 *              mx/my/mz - 磁力计数据 (可选，传0则不使用)
 *              sampleFreq - 采样频率 (Hz)
 *输出参数:     无
 ******************************************************************************/
void IMU_AHRSupdate(float gx, float gy, float gz, 
                    float ax, float ay, float az, 
                    float mx, float my, float mz, 
                    float sampleFreq)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz;
    float _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    /* 使用局部变量提高性能 */
    float local_q0 = q0;
    float local_q1 = q1;
    float local_q2 = q2;
    float local_q3 = q3;

    /* 如果磁力计数据无效，则使用纯IMU算法 */
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        IMU_AHRSupdateIMU(gx, gy, gz, ax, ay, az, sampleFreq);
        return;
    }

    /* 计算四元数乘积项 */
    q0q0 = local_q0 * local_q0;
    q0q1 = local_q0 * local_q1;
    q0q2 = local_q0 * local_q2;
    q0q3 = local_q0 * local_q3;
    q1q1 = local_q1 * local_q1;
    q1q2 = local_q1 * local_q2;
    q1q3 = local_q1 * local_q3;
    q2q2 = local_q2 * local_q2;
    q2q3 = local_q2 * local_q3;
    q3q3 = local_q3 * local_q3;

    /* 归一化磁力计测量值 */
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    /* 参考坐标系中的磁力计测量值 */
    _2q0mx = 2.0f * local_q0 * mx;
    _2q0my = 2.0f * local_q0 * my;
    _2q0mz = 2.0f * local_q0 * mz;
    _2q1mx = 2.0f * local_q1 * mx;
    hx = mx * q0q0 - _2q0my * local_q3 + _2q0mz * local_q2 + mx * q1q1 + _2q1 * my * local_q2 + _2q1 * mz * local_q3 - mx * q2q2 - mx * q3q3;
    hy = _2q0mx * local_q3 + my * q0q0 - _2q0mz * local_q1 + _2q1mx * local_q2 - my * q1q1 + my * q2q2 + _2q2 * mz * local_q3 - my * q3q3;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q0mx * local_q2 + _2q0my * local_q1 + mz * q0q0 + _2q1mx * local_q3 - mz * q1q1 + _2q2 * my * local_q3 - mz * q2q2 + mz * q3q3;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    /* 归一化加速度计测量值 */
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 计算梯度下降算法的误差 */
    _2q0 = 2.0f * local_q0;
    _2q1 = 2.0f * local_q1;
    _2q2 = 2.0f * local_q2;
    _2q3 = 2.0f * local_q3;
    _2q0q2 = 2.0f * local_q0q2;
    _2q2q3 = 2.0f * local_q2q3;

    s0 = _2q0 * q2q3 - _2q3 * q1q2 + _2q1 * q1q3 - _2q2 * q0q1 - ax;
    s1 = _2q3 * q0q1 + _2q2 * q1q3 - _2q1 * q0q0 - _2q0 * q1q2 - ay;
    s2 = _4bx * q1q3 + _2bz * q0q1 + _2q3 * q0q0 - _4bx * q0q2 - _2bz * local_q3 - az;
    s3 = _4bx * q0q2 - _2bz * q1q3 + _2q2 * q1q2 - _4bx * q1q1 - _2bz * local_q0;

    /* 归一化梯度 */
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* 计算四元数导数 */
    qDot1 = 0.5f * (-local_q1 * gx - local_q2 * gy - local_q3 * gz) - beta * s0;
    qDot2 = 0.5f * (local_q0 * gx + local_q2 * gz - local_q3 * gy) - beta * s1;
    qDot3 = 0.5f * (local_q0 * gy - local_q1 * gz + local_q3 * gx) - beta * s2;
    qDot4 = 0.5f * (local_q0 * gz + local_q1 * gy - local_q2 * gx) - beta * s3;

    /* 积分更新四元数 */
    local_q0 += qDot1 * (1.0f / sampleFreq);
    local_q1 += qDot2 * (1.0f / sampleFreq);
    local_q2 += qDot3 * (1.0f / sampleFreq);
    local_q3 += qDot4 * (1.0f / sampleFreq);

    /* 归一化四元数 */
    recipNorm = invSqrt(local_q0 * local_q0 + local_q1 * local_q1 + 
                        local_q2 * local_q2 + local_q3 * local_q3);
    local_q0 *= recipNorm;
    local_q1 *= recipNorm;
    local_q2 *= recipNorm;
    local_q3 *= recipNorm;

    /* 更新全局四元数 */
    q0 = local_q0;
    q1 = local_q1;
    q2 = local_q2;
    q3 = local_q3;
}

/**************************实现函数********************************************
 *函数原型:     void IMU_AHRSupdateIMU(float gx, float gy, float gz, 
 *                                      float ax, float ay, float az, 
 *                                      float sampleFreq)
 *功　　能:     更新四元数，仅使用陀螺仪和加速度计数据
 *输入参数:     gx/gy/gz - 陀螺仪数据 (rad/s)
 *              ax/ay/az - 加速度计数据 (m/s²)
 *              sampleFreq - 采样频率 (Hz)
 *输出参数:     无
 ******************************************************************************/
void IMU_AHRSupdateIMU(float gx, float gy, float gz, 
                       float ax, float ay, float az, 
                       float sampleFreq)
{
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    /* 使用局部变量提高性能 */
    float local_q0 = q0;
    float local_q1 = q1;
    float local_q2 = q2;
    float local_q3 = q3;

    /* 归一化加速度计测量值 */
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    /* 计算四元数乘积项 */
    _2q0 = 2.0f * local_q0;
    _2q1 = 2.0f * local_q1;
    _2q2 = 2.0f * local_q2;
    _2q3 = 2.0f * local_q3;
    _4q0 = 4.0f * local_q0;
    _4q1 = 4.0f * local_q1;
    _4q2 = 4.0f * local_q2;
    _8q1 = 8.0f * local_q1;
    _8q2 = 8.0f * local_q2;
    q0q0 = local_q0 * local_q0;
    q1q1 = local_q1 * local_q1;
    q2q2 = local_q2 * local_q2;
    q3q3 = local_q3 * local_q3;

    /* 计算梯度下降算法的误差 */
    s0 = _2q0 * q2q2 + _2q2 * local_q1 * local_q3 - _2q1 * q1q1 - _2q3 * local_q0 * local_q2 - ax;
    s1 = _2q3 * q1q1 + _2q1 * local_q0 * local_q2 - _2q0 * q2q2 - _2q2 * local_q2 * local_q3 - ay;
    s2 = _4q0 * local_q1 + _2q2 * local_q0 * local_q3 - _4q1 * q0q0 + _2q3 * ay - _8q1 * q1q1 - _8q2 * q2q2 - _2q2 * az;
    s3 = _4q1 * local_q2 + _2q3 * local_q0 * local_q1 - _4q2 * q0q0 + _2q0 * ay;

    /* 归一化梯度 */
    recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= recipNorm;
    s1 *= recipNorm;
    s2 *= recipNorm;
    s3 *= recipNorm;

    /* 计算四元数导数 */
    qDot1 = 0.5f * (-local_q1 * gx - local_q2 * gy - local_q3 * gz) - beta * s0;
    qDot2 = 0.5f * (local_q0 * gx + local_q2 * gz - local_q3 * gy) - beta * s1;
    qDot3 = 0.5f * (local_q0 * gy - local_q1 * gz + local_q3 * gx) - beta * s2;
    qDot4 = 0.5f * (local_q0 * gz + local_q1 * gy - local_q2 * gx) - beta * s3;

    /* 积分更新四元数 */
    local_q0 += qDot1 * (1.0f / sampleFreq);
    local_q1 += qDot2 * (1.0f / sampleFreq);
    local_q2 += qDot3 * (1.0f / sampleFreq);
    local_q3 += qDot4 * (1.0f / sampleFreq);

    /* 归一化四元数 */
    recipNorm = invSqrt(local_q0 * local_q0 + local_q1 * local_q1 + 
                        local_q2 * local_q2 + local_q3 * local_q3);
    local_q0 *= recipNorm;
    local_q1 *= recipNorm;
    local_q2 *= recipNorm;
    local_q3 *= recipNorm;

    /* 更新全局四元数 */
    q0 = local_q0;
    q1 = local_q1;
    q2 = local_q2;
    q3 = local_q3;
}

/**************************实现函数********************************************
 *函数原型:     void IMU_getYawPitchRoll(float *angles)
 *功　　能:     更新四元数，返回当前解算后的姿态数据
 *输入参数:     angles - 将要存放姿态角的数组首地址，依次为yaw, pitch, roll
 *输出参数:     无
 ******************************************************************************/
void IMU_getYawPitchRoll(float *angles)
{
    float local_q0 = q0;
    float local_q1 = q1;
    float local_q2 = q2;
    float local_q3 = q3;

    /* 四元数转欧拉角 */
    angles[0] = atan2(2.0f * (local_q1 * local_q2 + local_q0 * local_q3), 
                      local_q0 * local_q0 + local_q1 * local_q1 - local_q2 * local_q2 - local_q3 * local_q3) * (180.0f / M_PI);
    
    angles[1] = -asin(2.0f * (local_q1 * local_q3 - local_q0 * local_q2)) * (180.0f / M_PI);
    
    angles[2] = atan2(2.0f * (local_q0 * local_q1 + local_q2 * local_q3), 
                      local_q0 * local_q0 - local_q1 * local_q1 - local_q2 * local_q2 + local_q3 * local_q3) * (180.0f / M_PI);
}

/**************************实现函数********************************************
 *函数原型:     void IMU_getQuaternion(quaternion_t *q)
 *功　　能:     获取当前四元数
 *输入参数:     q - 四元数结构体指针
 *输出参数:     无
 ******************************************************************************/
void IMU_getQuaternion(quaternion_t *q)
{
    q->q0 = q0;
    q->q1 = q1;
    q->q2 = q2;
    q->q3 = q3;
}

/**************************实现函数********************************************
 *函数原型:     void IMU_init(float sampleFreq)
 *功　　能:     初始化姿态解算器
 *输入参数:     sampleFreq - 采样频率 (Hz)
 *输出参数:     无
 ******************************************************************************/
void IMU_init(float sampleFreq)
{
    /* 初始化四元数为单位四元数 */
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;

    /* 设置滤波参数 */
    beta = 0.1f;
}
