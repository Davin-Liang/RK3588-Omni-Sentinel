/*
 * icm45686.h - ICM45686惯性测量单元驱动头文件
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * Copyright (c) 2026
 */

#ifndef __ICM45686_H__
#define __ICM45686_H__

#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>

/* ICM45686设备地址 */
#define ICM45686_I2C_ADDR     0x68  /* AD0=0时的I2C地址 */
#define ICM45686_I2C_ADDR_AD0 0x69  /* AD0=1时的I2C地址 */

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

/* 传感器数据结构 */
typedef struct {
    int16_t accel_x;  /* 加速度计X轴原始数据 */
    int16_t accel_y;  /* 加速度计Y轴原始数据 */
    int16_t accel_z;  /* 加速度计Z轴原始数据 */
    int16_t gyro_x;   /* 陀螺仪X轴原始数据 */
    int16_t gyro_y;   /* 陀螺仪Y轴原始数据 */
    int16_t gyro_z;   /* 陀螺仪Z轴原始数据 */
    int16_t temp;     /* 温度原始数据 */
} icm45686_raw_data_t;

/* 传感器数据（转换后） */
typedef struct {
    float accel_x;  /* 加速度计X轴数据（单位：m/s²） */
    float accel_y;  /* 加速度计Y轴数据（单位：m/s²） */
    float accel_z;  /* 加速度计Z轴数据（单位：m/s²） */
    float gyro_x;   /* 陀螺仪X轴数据（单位：rad/s） */
    float gyro_y;   /* 陀螺仪Y轴数据（单位：rad/s） */
    float gyro_z;   /* 陀螺仪Z轴数据（单位：rad/s） */
    float temp;     /* 温度数据（单位：℃） */
} icm45686_data_t;

/* 设备私有数据结构 */
typedef struct {
    struct i2c_client *i2c_client;  /* I2C客户端 */
    struct spi_device *spi_device;  /* SPI设备 */
    uint8_t addr;                   /* 设备地址 */
    uint8_t accel_fs;               /* 加速度计量程 */
    uint8_t gyro_fs;                /* 陀螺仪量程 */
    float accel_scale;              /* 加速度计缩放因子 */
    float gyro_scale;               /* 陀螺仪缩放因子 */
    struct mutex lock;              /* 互斥锁 */
    struct device *dev;             /* 设备指针 */
} icm45686_dev_t;

/* 函数声明 */

/* I2C接口函数 */
extern int icm45686_i2c_read(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
extern int icm45686_i2c_write(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

/* SPI接口函数 */
extern int icm45686_spi_read(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);
extern int icm45686_spi_write(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len);

/* 核心函数 */
extern int icm45686_init(icm45686_dev_t *dev);
extern int icm45686_read_raw_data(icm45686_dev_t *dev, icm45686_raw_data_t *data);
extern int icm45686_read_data(icm45686_dev_t *dev, icm45686_data_t *data);
extern int icm45686_set_accel_fs(icm45686_dev_t *dev, uint8_t fs);
extern int icm45686_set_gyro_fs(icm45686_dev_t *dev, uint8_t fs);
extern int icm45686_set_accel_odr(icm45686_dev_t *dev, uint8_t odr);
extern int icm45686_set_gyro_odr(icm45686_dev_t *dev, uint8_t odr);

extern int icm45686_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id);
extern int icm45686_spi_probe(struct spi_device *spi);
extern int icm45686_remove(struct i2c_client *client);
extern int icm45686_spi_remove(struct spi_device *spi);

#endif /* __ICM45686_H__ */
