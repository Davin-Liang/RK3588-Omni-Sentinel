/*
 * icm45686_user.h - ICM45686用户空间接口头文件
 *
 * 适用于RK3588平台的ICM45686传感器应用
 *
 * 日期: 2026-05-05
 */

#ifndef __ICM45686_USER_H__
#define __ICM45686_USER_H__

#include <stdint.h>
#include <sys/ioctl.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 内核驱动返回的原始数据结构（整数，值×1000） */
typedef struct {
    int32_t accel_x;  /* 加速度计X轴原始值（×1000，单位：m/s² × 1000） */
    int32_t accel_y;  /* 加速度计Y轴原始值（×1000，单位：m/s² × 1000） */
    int32_t accel_z;  /* 加速度计Z轴原始值（×1000，单位：m/s² × 1000） */
    int32_t gyro_x;   /* 陀螺仪X轴原始值（×1000，单位：rad/s × 1000） */
    int32_t gyro_y;   /* 陀螺仪Y轴原始值（×1000，单位：rad/s × 1000） */
    int32_t gyro_z;   /* 陀螺仪Z轴原始值（×1000，单位：rad/s × 1000） */
    int32_t temp;     /* 温度原始值（×100，单位：℃ × 100） */
} icm45686_raw_data_t;

/* 转换后的物理量数据结构（浮点数） */
typedef struct {
    float accel_x;  /* 加速度计X轴数据 (m/s²) */
    float accel_y;  /* 加速度计Y轴数据 (m/s²) */
    float accel_z;  /* 加速度计Z轴数据 (m/s²) */
    float gyro_x;   /* 陀螺仪X轴数据 (rad/s) */
    float gyro_y;   /* 陀螺仪Y轴数据 (rad/s) */
    float gyro_z;   /* 陀螺仪Z轴数据 (rad/s) */
    float temp;     /* 温度数据 (℃) */
} icm45686_data_t;

/* IOCTL命令定义（使用整数类型） */
#define ICM45686_IOC_MAGIC          'i'
#define ICM45686_IOC_READ_DATA      _IOR(ICM45686_IOC_MAGIC, 0, icm45686_raw_data_t)
#define ICM45686_IOC_SET_ACCEL_FS   _IOW(ICM45686_IOC_MAGIC, 1, uint8_t)
#define ICM45686_IOC_SET_GYRO_FS    _IOW(ICM45686_IOC_MAGIC, 2, uint8_t)

/**************************实现函数********************************************
 *函数原型:     int icm45686_open(const char *dev_path)
 *功　　能:     打开ICM45686设备
 *输入参数:     dev_path - 设备路径 (如 "/dev/icm45686")
 *输出参数:     文件描述符，负数失败
 ******************************************************************************/
int icm45686_open(const char *dev_path);

/**************************实现函数********************************************
 *函数原型:     void icm45686_close(int fd)
 *功　　能:     关闭ICM45686设备
 *输入参数:     fd - 文件描述符
 *输出参数:     无
 ******************************************************************************/
void icm45686_close(int fd);

/**************************实现函数********************************************
 *函数原型:     int icm45686_read_data(int fd, icm45686_data_t *data)
 *功　　能:     读取传感器数据（自动从整数转换为物理值）
 *输入参数:     fd - 文件描述符, data - 数据结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int icm45686_read_data(int fd, icm45686_data_t *data);

/**************************实现函数********************************************
 *函数原型:     int icm45686_set_accel_fs(int fd, uint8_t fs)
 *功　　能:     设置加速度计量程
 *输入参数:     fd - 文件描述符, fs - 量程(0-3对应2G/4G/8G/16G)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int icm45686_set_accel_fs(int fd, uint8_t fs);

/**************************实现函数********************************************
 *函数原型:     int icm45686_set_gyro_fs(int fd, uint8_t fs)
 *功　　能:     设置陀螺仪量程
 *输入参数:     fd - 文件描述符, fs - 量程(0-3对应250/500/1000/2000DPS)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int icm45686_set_gyro_fs(int fd, uint8_t fs);

#ifdef __cplusplus
}
#endif

#endif /* __ICM45686_USER_H__ */

