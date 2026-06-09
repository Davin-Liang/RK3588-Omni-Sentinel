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

/*
 * ICM45686 User Bank 0 寄存器地址
 * 注意：
 * 1. ICM45686的数据寄存器从0x00开始连续排列；
 * 2. WHO_AM_I地址为0x72，默认设备ID为0xE9；
 * 3. 原MPU/ICM426xx类寄存器地址不能直接用于ICM45686。
 */
#define ICM45686_REG_ACCEL_DATA_X1      0x00  /* 加速度计X轴数据高字节 */
#define ICM45686_REG_ACCEL_DATA_X0      0x01  /* 加速度计X轴数据低字节 */
#define ICM45686_REG_ACCEL_DATA_Y1      0x02  /* 加速度计Y轴数据高字节 */
#define ICM45686_REG_ACCEL_DATA_Y0      0x03  /* 加速度计Y轴数据低字节 */
#define ICM45686_REG_ACCEL_DATA_Z1      0x04  /* 加速度计Z轴数据高字节 */
#define ICM45686_REG_ACCEL_DATA_Z0      0x05  /* 加速度计Z轴数据低字节 */

#define ICM45686_REG_GYRO_DATA_X1       0x06  /* 陀螺仪X轴数据高字节 */
#define ICM45686_REG_GYRO_DATA_X0       0x07  /* 陀螺仪X轴数据低字节 */
#define ICM45686_REG_GYRO_DATA_Y1       0x08  /* 陀螺仪Y轴数据高字节 */
#define ICM45686_REG_GYRO_DATA_Y0       0x09  /* 陀螺仪Y轴数据低字节 */
#define ICM45686_REG_GYRO_DATA_Z1       0x0A  /* 陀螺仪Z轴数据高字节 */
#define ICM45686_REG_GYRO_DATA_Z0       0x0B  /* 陀螺仪Z轴数据低字节 */

#define ICM45686_REG_TEMP_DATA1         0x0C  /* 温度数据高字节 */
#define ICM45686_REG_TEMP_DATA0         0x0D  /* 温度数据低字节 */

#define ICM45686_REG_PWR_MGMT0          0x10  /* 电源管理0寄存器 */

#define ICM45686_REG_INT1_CONFIG0       0x16  /* INT1中断配置0寄存器 */
#define ICM45686_REG_INT1_CONFIG1       0x17  /* INT1中断配置1寄存器 */
#define ICM45686_REG_INT1_CONFIG2       0x18  /* INT1中断配置2寄存器 */
#define ICM45686_REG_INT1_STATUS0       0x19  /* INT1中断状态0寄存器 */
#define ICM45686_REG_INT1_STATUS1       0x1A  /* INT1中断状态1寄存器 */

#define ICM45686_REG_ACCEL_CONFIG0      0x1B  /* 加速度计配置0寄存器 */
#define ICM45686_REG_GYRO_CONFIG0       0x1C  /* 陀螺仪配置0寄存器 */

#define ICM45686_REG_FIFO_CONFIG0       0x1D  /* FIFO配置0寄存器 */
#define ICM45686_REG_FIFO_CONFIG1_0     0x1E  /* FIFO水印阈值低字节 */
#define ICM45686_REG_FIFO_CONFIG1_1     0x1F  /* FIFO水印阈值高字节 */
#define ICM45686_REG_FIFO_CONFIG2       0x20  /* FIFO配置2寄存器 */
#define ICM45686_REG_FIFO_CONFIG3       0x21  /* FIFO配置3寄存器 */
#define ICM45686_REG_FIFO_CONFIG4       0x22  /* FIFO配置4寄存器 */
#define ICM45686_REG_FIFO_DATA          0x14  /* FIFO数据寄存器 */

#define ICM45686_REG_WHO_AM_I           0x72  /* 设备ID寄存器 */

/* 设备ID值 */
#define ICM45686_WHO_AM_I_VAL           0xE9  /* ICM45686设备ID */

/*
 * 电源管理0寄存器位定义
 * 当前驱动默认写0x0F，使能加速度计和陀螺仪工作。
 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_MASK  0x03  /* 加速度计模式掩码 */
#define ICM45686_PWR_MGMT0_GYRO_MODE_MASK   0x0C  /* 陀螺仪模式掩码 */
#define ICM45686_PWR_MGMT0_ACCEL_GYRO_ON    0x0F  /* 加速度计和陀螺仪使能 */

/*
 * 加速度计配置0寄存器位定义
 * bit[6:4]：ACCEL_UI_FS_SEL
 * bit[3:0]：ACCEL_ODR
 */
#define ICM45686_ACCEL_CONFIG0_FS_MASK       0x70
#define ICM45686_ACCEL_CONFIG0_FS_SHIFT      4
#define ICM45686_ACCEL_CONFIG0_ODR_MASK      0x0F

#define ICM45686_ACCEL_CONFIG0_FS_32G        0x00
#define ICM45686_ACCEL_CONFIG0_FS_16G        0x01
#define ICM45686_ACCEL_CONFIG0_FS_8G         0x02
#define ICM45686_ACCEL_CONFIG0_FS_4G         0x03
#define ICM45686_ACCEL_CONFIG0_FS_2G         0x04

/*
 * 陀螺仪配置0寄存器位定义
 * bit[7:4]：GYRO_UI_FS_SEL
 * bit[3:0]：GYRO_ODR
 */
#define ICM45686_GYRO_CONFIG0_FS_MASK        0xF0
#define ICM45686_GYRO_CONFIG0_FS_SHIFT       4
#define ICM45686_GYRO_CONFIG0_ODR_MASK       0x0F

#define ICM45686_GYRO_CONFIG0_FS_4000DPS     0x00
#define ICM45686_GYRO_CONFIG0_FS_2000DPS     0x01
#define ICM45686_GYRO_CONFIG0_FS_1000DPS     0x02
#define ICM45686_GYRO_CONFIG0_FS_500DPS      0x03
#define ICM45686_GYRO_CONFIG0_FS_250DPS      0x04
#define ICM45686_GYRO_CONFIG0_FS_125DPS      0x05
#define ICM45686_GYRO_CONFIG0_FS_62_5DPS     0x06
#define ICM45686_GYRO_CONFIG0_FS_31_25DPS    0x07
#define ICM45686_GYRO_CONFIG0_FS_15_625DPS   0x08

/* ODR配置值 */
#define ICM45686_ODR_6400HZ                  0x03
#define ICM45686_ODR_3200HZ                  0x04
#define ICM45686_ODR_1600HZ                  0x05
#define ICM45686_ODR_800HZ                   0x06
#define ICM45686_ODR_400HZ                   0x07
#define ICM45686_ODR_200HZ                   0x08
#define ICM45686_ODR_100HZ                   0x09
#define ICM45686_ODR_50HZ                    0x0A
#define ICM45686_ODR_25HZ                    0x0B
#define ICM45686_ODR_12_5HZ                  0x0C

/* 默认配置：±2G，±250DPS，100Hz */
#define ICM45686_ACCEL_CONFIG0_DEFAULT \
    ((ICM45686_ACCEL_CONFIG0_FS_2G << ICM45686_ACCEL_CONFIG0_FS_SHIFT) | ICM45686_ODR_100HZ)

#define ICM45686_GYRO_CONFIG0_DEFAULT \
    ((ICM45686_GYRO_CONFIG0_FS_250DPS << ICM45686_GYRO_CONFIG0_FS_SHIFT) | ICM45686_ODR_100HZ)

/* 中断配置位定义 */
#define ICM45686_INT_STATUS_EN_DRDY          0x04  /* INT1_CONFIG0中的DRDY状态使能 */

/* 传感器原始数据结构体 */
struct inv_imu_raw_data_t {
    int16_t accel_x;  /* 加速度计X轴原始数据 */
    int16_t accel_y;  /* 加速度计Y轴原始数据 */
    int16_t accel_z;  /* 加速度计Z轴原始数据 */
    int16_t gyro_x;   /* 陀螺仪X轴原始数据 */
    int16_t gyro_y;   /* 陀螺仪Y轴原始数据 */
    int16_t gyro_z;   /* 陀螺仪Z轴原始数据 */
    int16_t temp;     /* 温度原始数据 */
};

/* 传感器数据（转换后）结构体 - 使用整数避免内核浮点运算问题 */
/* 注意：实际物理值 = 值 / 1000.0f（单位：m/s² 或 rad/s）；温度 = 值 / 100.0f（℃） */
struct inv_imu_data_t {
    int32_t accel_x;  /* 加速度计X轴数据（放大1000倍，单位：m/s² × 1000） */
    int32_t accel_y;  /* 加速度计Y轴数据（放大1000倍，单位：m/s² × 1000） */
    int32_t accel_z;  /* 加速度计Z轴数据（放大1000倍，单位：m/s² × 1000） */
    int32_t gyro_x;   /* 陀螺仪X轴数据（放大1000倍，单位：rad/s × 1000） */
    int32_t gyro_y;   /* 陀螺仪Y轴数据（放大1000倍，单位：rad/s × 1000） */
    int32_t gyro_z;   /* 陀螺仪Z轴数据（放大1000倍，单位：rad/s × 1000） */
    int32_t temp;     /* 温度数据（放大100倍，单位：℃ × 100） */
};

#endif /* __INV_IMU_DEFS_H__ */

