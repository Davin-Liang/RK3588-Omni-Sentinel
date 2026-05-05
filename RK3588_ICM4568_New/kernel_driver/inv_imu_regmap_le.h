/*
 * inv_imu_regmap_le.h - ICM45686寄存器映射表（小端模式）
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * 日期: 2026-05-05
 */

#ifndef __INV_IMU_REGMAP_LE_H__
#define __INV_IMU_REGMAP_LE_H__

#include <linux/types.h>
#include <linux/kernel.h>

/* 传感器配置结构 */
struct inv_imu_sensor_config {
    uint8_t odr;          /* 输出数据率 */
    uint8_t fs;           /* 满量程 */
    uint8_t power_mode;   /* 电源模式 */
};

/* 设备配置结构 */
struct inv_imu_device_config {
    struct inv_imu_sensor_config accel;  /* 加速度计配置 */
    struct inv_imu_sensor_config gyro;   /* 陀螺仪配置 */
};

/* 寄存器配置项 */
struct inv_imu_reg_config {
    uint8_t reg_addr;    /* 寄存器地址 */
    uint8_t reg_val;     /* 寄存器值 */
};

/* 默认初始化配置 */
static const struct inv_imu_reg_config icm45686_default_config[] = {
    /* 电源管理：加速度计和陀螺仪设为高性能模式 */
    {ICM45686_REG_PWR_MGMT0, 0x0F},
    
    /* 加速度计：±2G量程，100Hz输出率 */
    {ICM45686_REG_ACCEL_CONFIG0, 0x10},
    
    /* 陀螺仪：±250DPS量程，100Hz输出率 */
    {ICM45686_REG_GYRO_CONFIG0, 0x10},
    
    /* 中断配置：INT1使能，高电平触发，推挽输出，锁存 */
    {ICM45686_REG_INT_CONFIG0, 0x09},
    
    /* 信号路径复位 */
    {ICM45686_REG_SIGNAL_PATH_RESET, 0x00},
};

/* 配置项数量 */
#define ICM45686_DEFAULT_CONFIG_LEN  ARRAY_SIZE(icm45686_default_config)

#endif /* __INV_IMU_REGMAP_LE_H__ */
