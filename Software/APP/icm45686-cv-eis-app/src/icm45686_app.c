/*
 * icm45686_app.c - ICM45686应用程序主文件
 *
 * 适用于RK3588平台的ICM45686传感器应用
 *
 * 日期: 2026-05-05
 */

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "icm45686_user.h"
#include "imu_ahrs.h"

#define DEV_PATH "/dev/icm45686"
#define SAMPLE_FREQ 100.0f  /* 采样频率 (Hz) */

static int running = 1;

/**************************实现函数********************************************
 *函数原型:     static void sig_handler(int sig)
 *功　　能:     信号处理函数，用于优雅退出
 *输入参数:     sig - 信号编号
 *输出参数:     无
 ******************************************************************************/
static void sig_handler(int sig)
{
    running = 0;
    printf("\nExiting...\n");
}

/**************************实现函数********************************************
 *函数原型:     int main(int argc, char *argv[])
 *功　　能:     应用程序主函数
 *输入参数:     argc - 参数个数, argv - 参数数组
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int main(int argc, char *argv[])
{
    int fd;
    icm45686_data_t data;
    float angles[3];  /* yaw, pitch, roll */
    int count = 0;
    
    /* 注册信号处理 */
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);
    
    printf("ICM45686 IMU Application\n");
    printf("========================\n");
    
    /* 打开设备 */
    fd = icm45686_open(DEV_PATH);
    if (fd < 0) {
        printf("Failed to open ICM45686 device\n");
        return -1;
    }
    
    printf("Device opened successfully\n");
    
    /* 初始化姿态解算器 */
    IMU_init(SAMPLE_FREQ);
    printf("AHRS initialized\n");
    
    /* 设置加速度计和陀螺仪量程 */
    icm45686_set_accel_fs(fd, 0);  /* ±2G */
    icm45686_set_gyro_fs(fd, 0);   /* ±250DPS */
    printf("Sensor configured\n");
    
    printf("\n");
    printf("Data output format:\n");
    printf("Count | Accel (m/s²) XYZ | Gyro (rad/s) XYZ | Temp (°C) | Yaw Pitch Roll (°)\n");
    printf("------|-------------------|-------------------|-----------|------------------\n");
    
    /* 主循环 */
    while (running) {
        /* 读取传感器数据 */
        if (icm45686_read_data(fd, &data) < 0) {
            printf("Failed to read sensor data\n");
            break;
        }
        
        /* 更新姿态解算 */
        IMU_AHRSupdateIMU(data.gyro_x, data.gyro_y, data.gyro_z,
                          data.accel_x, data.accel_y, data.accel_z,
                          SAMPLE_FREQ);
        
        /* 获取姿态角 */
        IMU_getYawPitchRoll(angles);
        
        /* 输出数据 */
        printf("%5d | %6.2f %6.2f %6.2f | %6.2f %6.2f %6.2f | %8.2f | %6.2f %6.2f %6.2f\n",
               count++,
               data.accel_x, data.accel_y, data.accel_z,
               data.gyro_x, data.gyro_y, data.gyro_z,
               data.temp,
               angles[0], angles[1], angles[2]);
        
        fflush(stdout);
        
        /* 延时 */
        usleep((useconds_t)(1000000.0f / SAMPLE_FREQ));
    }
    
    printf("\n");
    
    /* 关闭设备 */
    icm45686_close(fd);
    printf("Device closed\n");
    
    return 0;
}
