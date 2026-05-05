/*
 * inv_imu_driver.h - ICM45686官方驱动核心接口
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * 日期: 2026-05-05
 */

#ifndef __INV_IMU_DRIVER_H__
#define __INV_IMU_DRIVER_H__

#include "inv_imu_defs.h"
#include <linux/spi/spi.h>

/* 传输接口结构体 */
struct inv_imu_transport {
    int (*read_reg)(struct inv_imu_transport *t, uint8_t reg, 
                    uint8_t *buf, uint32_t len);
    int (*write_reg)(struct inv_imu_transport *t, uint8_t reg, 
                     const uint8_t *buf, uint32_t len);
    void *context;  /* 上下文指针 */
};

/* 设备结构体 */
struct inv_imu_device {
    struct inv_imu_transport transport;  /* 传输接口 */
    struct inv_imu_raw_data_t raw_data;  /* 原始数据 */
    struct inv_imu_data_t data;          /* 转换后数据 */
    
    uint8_t accel_fs;  /* 加速度计量程 */
    uint8_t gyro_fs;   /* 陀螺仪量程 */
    
    float accel_scale; /* 加速度计刻度因子 */
    float gyro_scale;  /* 陀螺仪刻度因子 */
};

/**************************实现函数********************************************
 *函数原型:     int inv_imu_init_hw(struct inv_imu_device *imu_dev)
 *功　　能:     初始化ICM45686硬件
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_init_hw(struct inv_imu_device *imu_dev);

/**************************实现函数********************************************
 *函数原型:     int inv_imu_read_raw_data(struct inv_imu_device *imu_dev)
 *功　　能:     读取传感器原始数据
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_read_raw_data(struct inv_imu_device *imu_dev);

/**************************实现函数********************************************
 *函数原型:     void inv_imu_convert_data(struct inv_imu_device *imu_dev)
 *功　　能:     将原始数据转换为物理量
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     无
 ******************************************************************************/
void inv_imu_convert_data(struct inv_imu_device *imu_dev);

/**************************实现函数********************************************
 *函数原型:     int inv_imu_set_accel_fs(struct inv_imu_device *imu_dev, uint8_t fs)
 *功　　能:     设置加速度计量程
 *输入参数:     imu_dev - 设备结构体指针, fs - 量程(0-3对应2G/4G/8G/16G)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_set_accel_fs(struct inv_imu_device *imu_dev, uint8_t fs);

/**************************实现函数********************************************
 *函数原型:     int inv_imu_set_gyro_fs(struct inv_imu_device *imu_dev, uint8_t fs)
 *功　　能:     设置陀螺仪量程
 *输入参数:     imu_dev - 设备结构体指针, fs - 量程(0-3对应250/500/1000/2000DPS)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_set_gyro_fs(struct inv_imu_device *imu_dev, uint8_t fs);

#endif /* __INV_IMU_DRIVER_H__ */
