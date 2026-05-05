/*
 * inv_imu_defs.h - ICM45686惯性测量单元寄存器和位定义
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * 日期: 2026-05-05
 */

#ifndef __INV_IMU_DEFS_H__
#define __INV_IMU_DEFS_H__

#include <linux/types.h>

/* 寄存器地址 */
#define ICM45686_REG_WHO_AM_I        0x0F  /* 设备ID寄存器 */
#define ICM45686_REG_PWR_MGMT0       0x1F  /* 电源管理0寄存器 */
#define ICM45686_REG_ACCEL_CONFIG0   0x20  /* 加速度计配置0寄存器 */
#define ICM45686_REG_GYRO_CONFIG0    0x23  /* 陀螺仪配置0寄存器 */
#define ICM45686_REG_ACCEL_DATA_X0   0x3B  /* 加速度计X轴数据低字节 */
#define ICM45686_REG_GYRO_DATA_X0    0x43  /* 陀螺仪X轴数据低字节 */
#define ICM45686_REG_TEMP_DATA0      0x4F  /* 温度数据低字节 */
#define ICM45686_REG_INT_CONFIG0     0x53  /* 中断配置0寄存器 */
#define ICM45686_REG_INT_SOURCE0     0x55  /* 中断源0寄存器 */
#define ICM45686_REG_FIFO_CONFIG0    0x66  /* FIFO配置0寄存器 */
#define ICM45686_REG_FIFO_CONFIG1    0x67  /* FIFO配置1寄存器 */
#define ICM45686_REG_FIFO_WM_TH      0x68  /* FIFO水印阈值寄存器 */
#define ICM45686_REG_FIFO_COUNTH     0x69  /* FIFO计数高字节 */
#define ICM45686_REG_FIFO_COUNTL     0x6A  /* FIFO计数低字节 */
#define ICM45686_REG_FIFO_DATA       0x6B  /* FIFO数据寄存器 */
#define ICM45686_REG_SIGNAL_PATH_RESET 0x72  /* 信号路径复位寄存器 */
#define ICM45686_REG_ACCEL_CALIB     0x73  /* 加速度计校准寄存器 */
#define ICM45686_REG_GYRO_CALIB      0x76  /* 陀螺仪校准寄存器 */

/* 设备ID值 */
#define ICM45686_WHO_AM_I_VAL        0x68  /* ICM45686的设备ID */

/* 电源管理0寄存器位定义 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_MASK  0x03  /* 加速度计模式掩码 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_OFF   0x00  /* 加速度计关闭 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_LP    0x01  /* 加速度计低功耗模式 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_ULP   0x02  /* 加速度计超低功耗模式 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_HP    0x03  /* 加速度计高性能模式 */

#define ICM45686_PWR_MGMT0_GYRO_MODE_MASK   0x0C  /* 陀螺仪模式掩码 */
#define ICM45686_PWR_MGMT0_GYRO_MODE_OFF    0x00  /* 陀螺仪关闭 */
#define ICM45686_PWR_MGMT0_GYRO_MODE_LP     0x04  /* 陀螺仪低功耗模式 */
#define ICM45686_PWR_MGMT0_GYRO_MODE_ULP    0x08  /* 陀螺仪超低功耗模式 */
#define ICM45686_PWR_MGMT0_GYRO_MODE_HP     0x0C  /* 陀螺仪高性能模式 */

/* 加速度计配置0寄存器位定义 */
#define ICM45686_ACCEL_CONFIG0_ODR_MASK     0xF0  /* 输出数据率掩码 */
#define ICM45686_ACCEL_CONFIG0_FS_MASK      0x03  /* 满量程掩码 */
#define ICM45686_ACCEL_CONFIG0_FS_2G        0x00  /* 满量程2G */
#define ICM45686_ACCEL_CONFIG0_FS_4G        0x01  /* 满量程4G */
#define ICM45686_ACCEL_CONFIG0_FS_8G        0x02  /* 满量程8G */
#define ICM45686_ACCEL_CONFIG0_FS_16G       0x03  /* 满量程16G */

/* 陀螺仪配置0寄存器位定义 */
#define ICM45686_GYRO_CONFIG0_ODR_MASK      0xF0  /* 输出数据率掩码 */
#define ICM45686_GYRO_CONFIG0_FS_MASK       0x03  /* 满量程掩码 */
#define ICM45686_GYRO_CONFIG0_FS_250DPS     0x00  /* 满量程250DPS */
#define ICM45686_GYRO_CONFIG0_FS_500DPS     0x01  /* 满量程500DPS */
#define ICM45686_GYRO_CONFIG0_FS_1000DPS    0x02  /* 满量程1000DPS */
#define ICM45686_GYRO_CONFIG0_FS_2000DPS    0x03  /* 满量程2000DPS */

/* 中断配置0寄存器位定义 */
#define ICM45686_INT_CONFIG0_INT1_EN        0x01  /* INT1使能 */
#define ICM45686_INT_CONFIG0_INT1_ACTL      0x02  /* INT1极性 */
#define ICM45686_INT_CONFIG0_INT1_OPEN      0x04  /* INT1开漏/推挽 */
#define ICM45686_INT_CONFIG0_INT1_LATCH     0x08  /* INT1锁存 */

/* 中断源0寄存器位定义 */
#define ICM45686_INT_SOURCE0_DATA_RDY_INT   0x01  /* 数据就绪中断 */

/* 传感器数据结构 */
typedef struct {
    int16_t accel_x;  /* 加速度计X轴原始数据 */
    int16_t accel_y;  /* 加速度计Y轴原始数据 */
    int16_t accel_z;  /* 加速度计Z轴原始数据 */
    int16_t gyro_x;   /* 陀螺仪X轴原始数据 */
    int16_t gyro_y;   /* 陀螺仪Y轴原始数据 */
    int16_t gyro_z;   /* 陀螺仪Z轴原始数据 */
    int16_t temp;     /* 温度原始数据 */
} inv_imu_raw_data_t;

/* 传感器数据（转换后） */
typedef struct {
    float accel_x;  /* 加速度计X轴数据（单位：m/s²） */
    float accel_y;  /* 加速度计Y轴数据（单位：m/s²） */
    float accel_z;  /* 加速度计Z轴数据（单位：m/s²） */
    float gyro_x;   /* 陀螺仪X轴数据（单位：rad/s） */
    float gyro_y;   /* 陀螺仪Y轴数据（单位：rad/s） */
    float gyro_z;   /* 陀螺仪Z轴数据（单位：rad/s） */
    float temp;     /* 温度数据（单位：℃） */
} inv_imu_data_t;

#endif /* __INV_IMU_DEFS_H__ */
