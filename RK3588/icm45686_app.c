/*
 * icm45686_app.c - ICM45686惯性测量单元应用程序
 *
 * 适用于RK3588平台的ICM45686传感器测试应用
 *
 * Copyright (c) 2026
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define I2C_BUS_PATH "/dev/i2c-0"  /* I2C总线设备路径 */
#define ICM45686_ADDR 0x68          /* ICM45686设备地址 */

/* 寄存器地址 */
#define ICM45686_REG_WHO_AM_I        0x0F
#define ICM45686_REG_PWR_MGMT0       0x1F
#define ICM45686_REG_ACCEL_CONFIG0   0x20
#define ICM45686_REG_GYRO_CONFIG0    0x23
#define ICM45686_REG_ACCEL_DATA_X0   0x3B
#define ICM45686_REG_GYRO_DATA_X0    0x43
#define ICM45686_REG_TEMP_DATA0      0x4F

/* 设备ID值 */
#define ICM45686_WHO_AM_I_VAL        0x68

/* 电源管理0寄存器位定义 */
#define ICM45686_PWR_MGMT0_ACCEL_MODE_HP    0x03
#define ICM45686_PWR_MGMT0_GYRO_MODE_HP     0x0C

/* 加速度计配置0寄存器位定义 */
#define ICM45686_ACCEL_CONFIG0_FS_2G        0x00

/* 陀螺仪配置0寄存器位定义 */
#define ICM45686_GYRO_CONFIG0_FS_250DPS     0x00

/* 加速度计量程缩放因子 */
static const float accel_scale = 2.0f / 32768.0f;

/* 陀螺仪量程缩放因子 */
static const float gyro_scale = 250.0f / 32768.0f;

/* 写入单个寄存器 */
int i2c_write_reg(int fd, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return write(fd, buf, 2);
}

/* 读取单个寄存器 */
int i2c_read_reg(int fd, uint8_t reg, uint8_t *val)
{
    if (write(fd, &reg, 1) != 1) {
        return -1;
    }
    return read(fd, val, 1);
}

/* 读取多个寄存器 */
int i2c_read_regs(int fd, uint8_t reg, uint8_t *buf, int len)
{
    if (write(fd, &reg, 1) != 1) {
        return -1;
    }
    return read(fd, buf, len);
}

/* 初始化ICM45686 */
int icm45686_init(int fd)
{
    int ret;
    uint8_t val;

    /* 读取WHO_AM_I寄存器，确认设备 */
    ret = i2c_read_reg(fd, ICM45686_REG_WHO_AM_I, &val);
    if (ret != 1) {
        fprintf(stderr, "Failed to read WHO_AM_I register\n");
        return -1;
    }

    if (val != ICM45686_WHO_AM_I_VAL) {
        fprintf(stderr, "Invalid WHO_AM_I value: 0x%02x\n", val);
        return -1;
    }

    printf("ICM45686 detected\n");

    /* 配置电源管理0 - 启用加速度计和陀螺仪 */
    val = ICM45686_PWR_MGMT0_ACCEL_MODE_HP | ICM45686_PWR_MGMT0_GYRO_MODE_HP;
    ret = i2c_write_reg(fd, ICM45686_REG_PWR_MGMT0, val);
    if (ret != 2) {
        fprintf(stderr, "Failed to configure power management\n");
        return -1;
    }

    usleep(10000); /* 等待10ms */

    /* 配置加速度计 - 2G量程，100Hz输出数据率 */
    val = (0x03 << 4) | ICM45686_ACCEL_CONFIG0_FS_2G; /* 100Hz ODR */
    ret = i2c_write_reg(fd, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret != 2) {
        fprintf(stderr, "Failed to configure accelerometer\n");
        return -1;
    }

    /* 配置陀螺仪 - 250DPS量程，100Hz输出数据率 */
    val = (0x03 << 4) | ICM45686_GYRO_CONFIG0_FS_250DPS; /* 100Hz ODR */
    ret = i2c_write_reg(fd, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret != 2) {
        fprintf(stderr, "Failed to configure gyroscope\n");
        return -1;
    }

    printf("ICM45686 initialized successfully\n");
    return 0;
}

/* 读取传感器数据 */
int icm45686_read_data(int fd, float *accel_x, float *accel_y, float *accel_z,
                      float *gyro_x, float *gyro_y, float *gyro_z, float *temp)
{
    int ret;
    uint8_t buf[14];
    int16_t accel_x_raw, accel_y_raw, accel_z_raw;
    int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
    int16_t temp_raw;

    ret = i2c_read_regs(fd, ICM45686_REG_ACCEL_DATA_X0, buf, 14);
    if (ret != 14) {
        fprintf(stderr, "Failed to read sensor data\n");
        return -1;
    }

    /* 转换数据 */
    accel_x_raw = (buf[0] << 8) | buf[1];
    accel_y_raw = (buf[2] << 8) | buf[3];
    accel_z_raw = (buf[4] << 8) | buf[5];
    gyro_x_raw = (buf[6] << 8) | buf[7];
    gyro_y_raw = (buf[8] << 8) | buf[9];
    gyro_z_raw = (buf[10] << 8) | buf[11];
    temp_raw = (buf[12] << 8) | buf[13];

    /* 计算实际值 */
    *accel_x = accel_x_raw * accel_scale * 9.81f;
    *accel_y = accel_y_raw * accel_scale * 9.81f;
    *accel_z = accel_z_raw * accel_scale * 9.81f;
    *gyro_x = gyro_x_raw * gyro_scale;
    *gyro_y = gyro_y_raw * gyro_scale;
    *gyro_z = gyro_z_raw * gyro_scale;
    *temp = (temp_raw / 132.48f) + 25.0f;

    return 0;
}

int main(int argc, char *argv[])
{
    int fd;
    int ret;
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    float temp;

    /* 打开I2C设备 */
    fd = open(I2C_BUS_PATH, O_RDWR);
    if (fd < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }

    /* 设置从设备地址 */
    ret = ioctl(fd, I2C_SLAVE, ICM45686_ADDR);
    if (ret < 0) {
        perror("Failed to set I2C slave address");
        close(fd);
        return 1;
    }

    /* 初始化ICM45686 */
    ret = icm45686_init(fd);
    if (ret < 0) {
        close(fd);
        return 1;
    }

    /* 循环读取数据 */
    printf("ICM45686 sensor data:\n");
    printf("Accel X (m/s²) | Accel Y (m/s²) | Accel Z (m/s²) | Gyro X (dps) | Gyro Y (dps) | Gyro Z (dps) | Temp (°C)\n");
    printf("---------------------------------------------------------------------------------------------------\n");

    while (1) {
        ret = icm45686_read_data(fd, &accel_x, &accel_y, &accel_z,
                                &gyro_x, &gyro_y, &gyro_z, &temp);
        if (ret < 0) {
            break;
        }

        printf("%12.2f | %12.2f | %12.2f | %10.2f | %10.2f | %10.2f | %8.2f\n",
               accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, temp);

        usleep(100000); /* 100ms */
    }

    /* 关闭设备 */
    close(fd);
    return 0;
}
