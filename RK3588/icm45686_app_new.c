/*
 * icm45686_app_new.c - ICM45686惯性测量单元应用程序
 *
 * 适用于RKS588ELF2板子的ICM45686传感器测试应用
 *
 * 日期: 2026-04-09
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "icm45686_driver.h"
#include "icm45686_transport.h"

/**************************实现函数********************************************
 *函数原型: 	    int main(int argc, char *argv[])
 *功　　能: 	  主函数，测试ICM45686传感器
 *输入参数：  argc - 命令行参数个数
 *            argv - 命令行参数数组
 *输出参数：  0 - 成功，非0 - 错误
 *******************************************************************************/
int main(int argc, char *argv[])
{
    int ret;
    icm45686_dev_t dev;
    icm45686_data_t data;
    int use_spi = 1;  /* 1: 使用SPI，0: 使用I2C */

    /* 初始化传输接口 */
    if (use_spi) {
        /* 初始化SPI接口 */
        ret = icm45686_spi_init(0, 0);  /* SPI0, CS0 */
        if (ret < 0) {
            fprintf(stderr, "初始化SPI接口失败\n");
            return 1;
        }

        /* 设置传输接口函数 */
        dev.transport.read_reg = icm45686_spi_read;
        dev.transport.write_reg = icm45686_spi_write;
        dev.transport.delay_us = icm45686_delay_us;
    } else {
        /* 初始化I2C接口 */
        ret = icm45686_i2c_init(0, ICM45686_I2C_ADDR);  /* I2C0, 地址0x68 */
        if (ret < 0) {
            fprintf(stderr, "初始化I2C接口失败\n");
            return 1;
        }

        /* 设置传输接口函数 */
        dev.transport.read_reg = icm45686_i2c_read;
        dev.transport.write_reg = icm45686_i2c_write;
        dev.transport.delay_us = icm45686_delay_us;
    }

    /* 初始化ICM45686 */
    ret = icm45686_init(&dev);
    if (ret < 0) {
        fprintf(stderr, "初始化ICM45686失败\n");
        return 1;
    }

    /* 循环读取数据 */
    printf("ICM45686传感器数据:\n");
    printf("加速度X (m/s²) | 加速度Y (m/s²) | 加速度Z (m/s²) | 陀螺仪X (rad/s) | 陀螺仪Y (rad/s) | 陀螺仪Z (rad/s) | 温度 (℃)\n");
    printf("-----------------------------------------------------------------------------------------------------------\n");

    while (1) {
        /* 读取传感器数据 */
        ret = icm45686_read_data(&dev, &data);
        if (ret < 0) {
            fprintf(stderr, "读取传感器数据失败\n");
            break;
        }

        /* 输出数据 */
        printf("%12.2f | %12.2f | %12.2f | %13.2f | %13.2f | %13.2f | %8.2f\n",
               data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x, data.gyro_y, data.gyro_z,
               data.temp);

        /* 延时100ms */
        usleep(100000);
    }

    return 0;
}
