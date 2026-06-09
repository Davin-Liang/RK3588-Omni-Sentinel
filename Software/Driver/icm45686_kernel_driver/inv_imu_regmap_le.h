/*
 * inv_imu_regmap_le.h - ICM45686寄存器映射表
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

/*
 * 默认初始化配置
 * 注意：
 * 1. ICM45686的PWR_MGMT0地址为0x10；
 * 2. ACCEL_CONFIG0地址为0x1B，bit[6:4]为量程，bit[3:0]为ODR；
 * 3. GYRO_CONFIG0地址为0x1C，bit[7:4]为量程，bit[3:0]为ODR；
 * 4. 这里先不配置INT1，避免未连接INT1时引入额外干扰。
 */
static const struct inv_imu_reg_config icm45686_default_config[] = {
    /* 电源管理：使能加速度计和陀螺仪 */
    {ICM45686_REG_PWR_MGMT0, ICM45686_PWR_MGMT0_ACCEL_GYRO_ON},

    /* 加速度计：±2G量程，100Hz输出率 */
    {ICM45686_REG_ACCEL_CONFIG0, ICM45686_ACCEL_CONFIG0_DEFAULT},

    /* 陀螺仪：±250DPS量程，100Hz输出率 */
    {ICM45686_REG_GYRO_CONFIG0, ICM45686_GYRO_CONFIG0_DEFAULT},
};

/* 配置项数量 */
#define ICM45686_DEFAULT_CONFIG_LEN  ARRAY_SIZE(icm45686_default_config)

#endif /* __INV_IMU_REGMAP_LE_H__ */

