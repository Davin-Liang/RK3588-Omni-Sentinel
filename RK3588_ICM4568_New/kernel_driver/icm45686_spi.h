/*
 * icm45686_spi.h - ICM45686 SPI设备驱动头文件
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * 日期: 2026-05-05
 */

#ifndef __ICM45686_SPI_H__
#define __ICM45686_SPI_H__

#include <linux/spi/spi.h>
#include <linux/gpio/consumer.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include "inv_imu_driver.h"

/* 设备私有数据结构 */
struct icm45686_data {
    struct spi_device *spi;              /* SPI设备指针 */
    struct inv_imu_device imu_dev;       /* IMU设备结构体 */
    struct gpio_desc *reset_gpio;        /* RESET引脚 */
    struct gpio_desc *int_gpio;          /* INT引脚 */
    
    struct cdev cdev;                    /* 字符设备 */
    dev_t devno;                         /* 设备号 */
    struct class *class;                 /* 设备类 */
    struct mutex lock;                   /* 互斥锁 */
    
    struct timer_list timer;             /* 数据读取定时器 */
    struct inv_imu_data_t sensor_data;   /* 传感器数据 */
    
    int open_count;                      /* 打开计数 */
};

/* IOCTL命令定义 */
#define ICM45686_IOC_MAGIC          'i'
#define ICM45686_IOC_READ_DATA      _IOR(ICM45686_IOC_MAGIC, 0, struct inv_imu_data_t)
#define ICM45686_IOC_SET_ACCEL_FS   _IOW(ICM45686_IOC_MAGIC, 1, uint8_t)
#define ICM45686_IOC_SET_GYRO_FS    _IOW(ICM45686_IOC_MAGIC, 2, uint8_t)

#endif /* __ICM45686_SPI_H__ */
