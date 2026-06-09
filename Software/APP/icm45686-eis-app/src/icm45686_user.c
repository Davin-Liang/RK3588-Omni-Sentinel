/*
 * icm45686_user.c - ICM45686用户空间接口实现
 *
 * 适用于RK3588平台的ICM45686传感器应用
 *
 * 日期: 2026-05-05
 */

#include "icm45686_user.h"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>

/**************************实现函数********************************************
 *函数原型:     int icm45686_open(const char *dev_path)
 *功　　能:     打开ICM45686设备
 *输入参数:     dev_path - 设备路径 (如 "/dev/icm45686")
 *输出参数:     文件描述符，负数失败
 ******************************************************************************/
int icm45686_open(const char *dev_path)
{
    int fd;

    fd = open(dev_path, O_RDWR);
    if (fd < 0) {
        perror("Failed to open icm45686 device");
        return -1;
    }

    return fd;
}

/**************************实现函数********************************************
 *函数原型:     void icm45686_close(int fd)
 *功　　能:     关闭ICM45686设备
 *输入参数:     fd - 文件描述符
 *输出参数:     无
 ******************************************************************************/
void icm45686_close(int fd)
{
    if (fd >= 0) {
        close(fd);
    }
}

/**************************实现函数********************************************
 *函数原型:     int icm45686_read_data(int fd, icm45686_data_t *data)
 *功　　能:     读取传感器数据（自动从整数转换为物理值）
 *              内核驱动返回整数（×1000），此函数自动转换为浮点数
 *输入参数:     fd - 文件描述符, data - 数据结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int icm45686_read_data(int fd, icm45686_data_t *data)
{
    int ret;
    icm45686_raw_data_t raw_data;

    if (fd < 0 || data == NULL) {
        printf("Invalid parameters\n");
        return -1;
    }

    /* 从内核读取整数原始数据 */
    ret = ioctl(fd, ICM45686_IOC_READ_DATA, &raw_data);
    if (ret < 0) {
        perror("Failed to read icm45686 data");
        return -1;
    }

    /* 转换为浮点数物理值 */
    /* 加速度计：内核返回 ×1000，需要除以 1000.0f */
    data->accel_x = (float)raw_data.accel_x / 1000.0f;
    data->accel_y = (float)raw_data.accel_y / 1000.0f;
    data->accel_z = (float)raw_data.accel_z / 1000.0f;

    /* 陀螺仪：内核返回 ×1000，需要除以 1000.0f */
    data->gyro_x = (float)raw_data.gyro_x / 1000.0f;
    data->gyro_y = (float)raw_data.gyro_y / 1000.0f;
    data->gyro_z = (float)raw_data.gyro_z / 1000.0f;

    /* 温度：内核返回 ×100，需要除以 100.0f */
    data->temp = (float)raw_data.temp / 100.0f;

    return 0;
}

/**************************实现函数********************************************
 *函数原型:     int icm45686_set_accel_fs(int fd, uint8_t fs)
 *功　　能:     设置加速度计量程
 *输入参数:     fd - 文件描述符, fs - 量程(0-3对应2G/4G/8G/16G)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int icm45686_set_accel_fs(int fd, uint8_t fs)
{
    int ret;

    if (fd < 0 || fs > 3) {
        printf("Invalid parameters\n");
        return -1;
    }

    ret = ioctl(fd, ICM45686_IOC_SET_ACCEL_FS, &fs);
    if (ret < 0) {
        perror("Failed to set accelerometer full scale");
        return -1;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型:     int icm45686_set_gyro_fs(int fd, uint8_t fs)
 *功　　能:     设置陀螺仪量程
 *输入参数:     fd - 文件描述符, fs - 量程(0-3对应250/500/1000/2000DPS)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int icm45686_set_gyro_fs(int fd, uint8_t fs)
{
    int ret;

    if (fd < 0 || fs > 3) {
        printf("Invalid parameters\n");
        return -1;
    }

    ret = ioctl(fd, ICM45686_IOC_SET_GYRO_FS, &fs);
    if (ret < 0) {
        perror("Failed to set gyroscope full scale");
        return -1;
    }

    return 0;
}

