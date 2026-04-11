/*
 * icm45686_driver.c - ICM45686惯性测量单元驱动实现
 *
 * 适用于RKS588ELF2板子的ICM45686传感器驱动
 *
 * 日期: 2026-04-09
 */

#include <stdio.h>
#include <math.h>

#include "icm45686_driver.h"

/* 加速度计量程缩放因子 */
static const float accel_scale[] = {
    2.0f / 32768.0f,  /* 2G */
    4.0f / 32768.0f,  /* 4G */
    8.0f / 32768.0f,  /* 8G */
    16.0f / 32768.0f  /* 16G */
};

/* 陀螺仪量程缩放因子 */
static const float gyro_scale[] = {
    250.0f / 32768.0f,   /* 250DPS */
    500.0f / 32768.0f,   /* 500DPS */
    1000.0f / 32768.0f,  /* 1000DPS */
    2000.0f / 32768.0f   /* 2000DPS */
};

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_read_reg(icm45686_dev_t *dev, uint8_t reg, uint8_t *val)
 *功　　能: 	  读取单个寄存器
 *输入参数：  dev - 设备结构体指针
 *            reg - 寄存器地址
 *输出参数：  val - 寄存器值
 *            0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_read_reg(icm45686_dev_t *dev, uint8_t reg, uint8_t *val)
{
    return dev->transport.read_reg(reg, val, 1);
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_write_reg(icm45686_dev_t *dev, uint8_t reg, uint8_t val)
 *功　　能: 	  写入单个寄存器
 *输入参数：  dev - 设备结构体指针
 *            reg - 寄存器地址
 *            val - 寄存器值
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_write_reg(icm45686_dev_t *dev, uint8_t reg, uint8_t val)
{
    return dev->transport.write_reg(reg, &val, 1);
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_soft_reset(icm45686_dev_t *dev)
 *功　　能: 	  软复位设备
 *输入参数：  dev - 设备结构体指针
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_soft_reset(icm45686_dev_t *dev)
{
    uint8_t val = 0x01;
    int ret;

    /* 复位信号路径 */
    ret = icm45686_write_reg(dev, ICM45686_REG_SIGNAL_PATH_RESET, val);
    if (ret < 0) {
        return ret;
    }

    /* 延时10ms */
    dev->transport.delay_us(10000);

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_get_who_am_i(icm45686_dev_t *dev, uint8_t *who_am_i)
 *功　　能: 	  获取设备ID
 *输入参数：  dev - 设备结构体指针
 *输出参数：  who_am_i - 设备ID
 *            0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_get_who_am_i(icm45686_dev_t *dev, uint8_t *who_am_i)
{
    return icm45686_read_reg(dev, ICM45686_REG_WHO_AM_I, who_am_i);
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_init(icm45686_dev_t *dev)
 *功　　能: 	  初始化ICM45686
 *输入参数：  dev - 设备结构体指针
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_init(icm45686_dev_t *dev)
{
    int ret;
    uint8_t val;

    /* 读取WHO_AM_I寄存器，确认设备 */
    ret = icm45686_get_who_am_i(dev, &val);
    if (ret < 0) {
        fprintf(stderr, "读取WHO_AM_I寄存器失败\n");
        return ret;
    }

    if (val != ICM45686_WHO_AM_I_VAL) {
        fprintf(stderr, "无效的WHO_AM_I值: 0x%02x\n", val);
        return -1;
    }

    printf("ICM45686设备已检测到\n");

    /* 软复位设备 */
    ret = icm45686_soft_reset(dev);
    if (ret < 0) {
        fprintf(stderr, "设备复位失败\n");
        return ret;
    }

    /* 配置电源管理0 - 启用加速度计和陀螺仪 */
    val = ICM45686_PWR_MGMT0_ACCEL_MODE_HP | ICM45686_PWR_MGMT0_GYRO_MODE_HP;
    ret = icm45686_write_reg(dev, ICM45686_REG_PWR_MGMT0, val);
    if (ret < 0) {
        fprintf(stderr, "配置电源管理失败\n");
        return ret;
    }

    /* 延时10ms */
    dev->transport.delay_us(10000);

    /* 配置加速度计 - 2G量程，100Hz输出数据率 */
    val = (0x03 << 4) | ICM45686_ACCEL_CONFIG0_FS_2G; /* 100Hz ODR */
    ret = icm45686_write_reg(dev, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret < 0) {
        fprintf(stderr, "配置加速度计失败\n");
        return ret;
    }

    /* 配置陀螺仪 - 250DPS量程，100Hz输出数据率 */
    val = (0x03 << 4) | ICM45686_GYRO_CONFIG0_FS_250DPS; /* 100Hz ODR */
    ret = icm45686_write_reg(dev, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret < 0) {
        fprintf(stderr, "配置陀螺仪失败\n");
        return ret;
    }

    /* 设置缩放因子 */
    dev->accel_fs = ICM45686_ACCEL_CONFIG0_FS_2G;
    dev->gyro_fs = ICM45686_GYRO_CONFIG0_FS_250DPS;
    dev->accel_scale = accel_scale[dev->accel_fs];
    dev->gyro_scale = gyro_scale[dev->gyro_fs];

    printf("ICM45686初始化成功\n");
    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_read_raw_data(icm45686_dev_t *dev, icm45686_raw_data_t *data)
 *功　　能: 	  读取原始数据
 *输入参数：  dev - 设备结构体指针
 *输出参数：  data - 原始数据结构体指针
 *            0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_read_raw_data(icm45686_dev_t *dev, icm45686_raw_data_t *data)
{
    uint8_t buf[14];
    int ret;

    /* 读取加速度计、陀螺仪和温度数据 */
    ret = dev->transport.read_reg(ICM45686_REG_ACCEL_DATA_X0, buf, 14);
    if (ret < 0) {
        fprintf(stderr, "读取原始数据失败\n");
        return ret;
    }

    /* 转换数据 */
    data->accel_x = (buf[0] << 8) | buf[1];
    data->accel_y = (buf[2] << 8) | buf[3];
    data->accel_z = (buf[4] << 8) | buf[5];
    data->gyro_x = (buf[6] << 8) | buf[7];
    data->gyro_y = (buf[8] << 8) | buf[9];
    data->gyro_z = (buf[10] << 8) | buf[11];
    data->temp = (buf[12] << 8) | buf[13];

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_read_data(icm45686_dev_t *dev, icm45686_data_t *data)
 *功　　能: 	  读取转换后的数据
 *输入参数：  dev - 设备结构体指针
 *输出参数：  data - 转换后的数据结构体指针
 *            0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_read_data(icm45686_dev_t *dev, icm45686_data_t *data)
{
    int ret;
    icm45686_raw_data_t raw_data;

    /* 读取原始数据 */
    ret = icm45686_read_raw_data(dev, &raw_data);
    if (ret < 0) {
        return ret;
    }

    /* 转换加速度计数据（单位：m/s²） */
    data->accel_x = raw_data.accel_x * dev->accel_scale * 9.81f;
    data->accel_y = raw_data.accel_y * dev->accel_scale * 9.81f;
    data->accel_z = raw_data.accel_z * dev->accel_scale * 9.81f;

    /* 转换陀螺仪数据（单位：rad/s） */
    data->gyro_x = raw_data.gyro_x * dev->gyro_scale * M_PI / 180.0f;
    data->gyro_y = raw_data.gyro_y * dev->gyro_scale * M_PI / 180.0f;
    data->gyro_z = raw_data.gyro_z * dev->gyro_scale * M_PI / 180.0f;

    /* 转换温度数据（单位：℃） */
    data->temp = (raw_data.temp / 132.48f) + 25.0f;

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_set_accel_fs(icm45686_dev_t *dev, uint8_t fs)
 *功　　能: 	  设置加速度计量程
 *输入参数：  dev - 设备结构体指针
 *            fs - 量程值
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_set_accel_fs(icm45686_dev_t *dev, uint8_t fs)
{
    int ret;
    uint8_t val;

    if (fs > 3) {
        return -1;
    }

    /* 读取当前配置 */
    ret = icm45686_read_reg(dev, ICM45686_REG_ACCEL_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    /* 更新量程配置 */
    val = (val & ~ICM45686_ACCEL_CONFIG0_FS_MASK) | fs;
    ret = icm45686_write_reg(dev, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    /* 更新缩放因子 */
    dev->accel_fs = fs;
    dev->accel_scale = accel_scale[fs];

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_set_gyro_fs(icm45686_dev_t *dev, uint8_t fs)
 *功　　能: 	  设置陀螺仪量程
 *输入参数：  dev - 设备结构体指针
 *            fs - 量程值
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_set_gyro_fs(icm45686_dev_t *dev, uint8_t fs)
{
    int ret;
    uint8_t val;

    if (fs > 3) {
        return -1;
    }

    /* 读取当前配置 */
    ret = icm45686_read_reg(dev, ICM45686_REG_GYRO_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    /* 更新量程配置 */
    val = (val & ~ICM45686_GYRO_CONFIG0_FS_MASK) | fs;
    ret = icm45686_write_reg(dev, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    /* 更新缩放因子 */
    dev->gyro_fs = fs;
    dev->gyro_scale = gyro_scale[fs];

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_set_accel_odr(icm45686_dev_t *dev, uint8_t odr)
 *功　　能: 	  设置加速度计输出数据率
 *输入参数：  dev - 设备结构体指针
 *            odr - 输出数据率值
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_set_accel_odr(icm45686_dev_t *dev, uint8_t odr)
{
    int ret;
    uint8_t val;

    if (odr > 0x0F) {
        return -1;
    }

    /* 读取当前配置 */
    ret = icm45686_read_reg(dev, ICM45686_REG_ACCEL_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    /* 更新输出数据率配置 */
    val = (val & ~ICM45686_ACCEL_CONFIG0_ODR_MASK) | (odr << 4);
    ret = icm45686_write_reg(dev, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_set_gyro_odr(icm45686_dev_t *dev, uint8_t odr)
 *功　　能: 	  设置陀螺仪输出数据率
 *输入参数：  dev - 设备结构体指针
 *            odr - 输出数据率值
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_set_gyro_odr(icm45686_dev_t *dev, uint8_t odr)
{
    int ret;
    uint8_t val;

    if (odr > 0x0F) {
        return -1;
    }

    /* 读取当前配置 */
    ret = icm45686_read_reg(dev, ICM45686_REG_GYRO_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    /* 更新输出数据率配置 */
    val = (val & ~ICM45686_GYRO_CONFIG0_ODR_MASK) | (odr << 4);
    ret = icm45686_write_reg(dev, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}
