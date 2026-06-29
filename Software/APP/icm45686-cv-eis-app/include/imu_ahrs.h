/*
 * imu_ahrs.h - Madgwick四元数姿态解算算法头文件
 *
 * 适用于RK3588平台的ICM45686传感器应用
 *
 * 日期: 2026-05-05
 */

#ifndef __IMU_AHRS_H__
#define __IMU_AHRS_H__

#include <stdint.h>

/* 四元数结构体 */
typedef struct {
    float q0;  /* 四元数实部 */
    float q1;  /* 四元数虚部i */
    float q2;  /* 四元数虚部j */
    float q3;  /* 四元数虚部k */
} quaternion_t;

/* 姿态角结构体 */
typedef struct {
    float yaw;   /* 偏航角 (度) */
    float pitch; /* 俯仰角 (度) */
    float roll;  /* 横滚角 (度) */
} euler_angles_t;

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
                    float sampleFreq);

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
                       float sampleFreq);

/**************************实现函数********************************************
 *函数原型:     void IMU_getYawPitchRoll(float *angles)
 *功　　能:     更新四元数，返回当前解算后的姿态数据
 *输入参数:     angles - 将要存放姿态角的数组首地址，依次为yaw, pitch, roll
 *输出参数:     无
 ******************************************************************************/
void IMU_getYawPitchRoll(float *angles);

/**************************实现函数********************************************
 *函数原型:     void IMU_getQuaternion(quaternion_t *q)
 *功　　能:     获取当前四元数
 *输入参数:     q - 四元数结构体指针
 *输出参数:     无
 ******************************************************************************/
void IMU_getQuaternion(quaternion_t *q);

/**************************实现函数********************************************
 *函数原型:     void IMU_init(float sampleFreq)
 *功　　能:     初始化姿态解算器
 *输入参数:     sampleFreq - 采样频率 (Hz)
 *输出参数:     无
 ******************************************************************************/
void IMU_init(float sampleFreq);

#endif /* __IMU_AHRS_H__ */
