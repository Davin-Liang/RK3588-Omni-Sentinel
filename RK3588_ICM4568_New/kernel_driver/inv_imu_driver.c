/*
 * inv_imu_driver.c - ICM45686官方驱动核心实现
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * 日期: 2026-05-05
 */

#include "inv_imu_driver.h"
#include "inv_imu_regmap_le.h"
#include <linux/delay.h>
#include <linux/module.h>

/**************************实现函数********************************************
 *函数原型:     static int inv_imu_write_reg(struct inv_imu_device *imu_dev, 
 *                                           uint8_t reg, uint8_t val)
 *功　　能:     写入单个寄存器
 *输入参数:     imu_dev - 设备结构体指针, reg - 寄存器地址, val - 寄存器值
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int inv_imu_write_reg(struct inv_imu_device *imu_dev, uint8_t reg, uint8_t val)
{
    return imu_dev->transport.write_reg(&imu_dev->transport, reg, &val, 1);
}

/**************************实现函数********************************************
 *函数原型:     static int inv_imu_read_reg(struct inv_imu_device *imu_dev, 
 *                                          uint8_t reg, uint8_t *val)
 *功　　能:     读取单个寄存器
 *输入参数:     imu_dev - 设备结构体指针, reg - 寄存器地址, val - 读取值指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int inv_imu_read_reg(struct inv_imu_device *imu_dev, uint8_t reg, uint8_t *val)
{
    return imu_dev->transport.read_reg(&imu_dev->transport, reg, val, 1);
}

/**************************实现函数********************************************
 *函数原型:     int inv_imu_init_hw(struct inv_imu_device *imu_dev)
 *功　　能:     初始化ICM45686硬件
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_init_hw(struct inv_imu_device *imu_dev)
{
    uint8_t who_am_i;
    int ret;
    int i;
    
    /* 读取设备ID */
    ret = inv_imu_read_reg(imu_dev, ICM45686_REG_WHO_AM_I, &who_am_i);
    if (ret < 0) {
        pr_err("Failed to read WHO_AM_I register\n");
        return ret;
    }
    
    if (who_am_i != ICM45686_WHO_AM_I_VAL) {
        pr_err("Invalid WHO_AM_I value: 0x%02x (expected 0x%02x)\n", 
               who_am_i, ICM45686_WHO_AM_I_VAL);
        return -ENODEV;
    }
    
    /* 执行软件复位 */
    ret = inv_imu_write_reg(imu_dev, ICM45686_REG_SIGNAL_PATH_RESET, 0x03);
    if (ret < 0) {
        pr_err("Failed to reset sensor\n");
        return ret;
    }
    
    msleep(10);
    
    /* 加载默认配置 */
    for (i = 0; i < ICM45686_DEFAULT_CONFIG_LEN; i++) {
        ret = inv_imu_write_reg(imu_dev, 
                               icm45686_default_config[i].reg_addr, 
                               icm45686_default_config[i].reg_val);
        if (ret < 0) {
            pr_err("Failed to write register 0x%02x\n", 
                   icm45686_default_config[i].reg_addr);
            return ret;
        }
    }
    
    msleep(50);
    
    /* 设置默认刻度因子 */
    imu_dev->accel_fs = ICM45686_ACCEL_CONFIG0_FS_2G;
    imu_dev->gyro_fs = ICM45686_GYRO_CONFIG0_FS_250DPS;
    imu_dev->accel_scale = 9.81f / 32768.0f * 2.0f;
    imu_dev->gyro_scale = (float)(250.0 * M_PI / 180.0) / 32768.0f;
    
    pr_info("ICM45686 initialized successfully\n");
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     int inv_imu_read_raw_data(struct inv_imu_device *imu_dev)
 *功　　能:     读取传感器原始数据
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_read_raw_data(struct inv_imu_device *imu_dev)
{
    uint8_t buf[14];
    int ret;
    
    /* 读取加速度计数据 (6字节) + 陀螺仪数据 (6字节) + 温度数据 (2字节) */
    ret = imu_dev->transport.read_reg(&imu_dev->transport, 
                                      ICM45686_REG_ACCEL_DATA_X0, 
                                      buf, 14);
    if (ret < 0) {
        pr_err("Failed to read sensor data\n");
        return ret;
    }
    
    /* 小端模式数据组装 */
    imu_dev->raw_data.accel_x = (int16_t)(buf[0] | (buf[1] << 8));
    imu_dev->raw_data.accel_y = (int16_t)(buf[2] | (buf[3] << 8));
    imu_dev->raw_data.accel_z = (int16_t)(buf[4] | (buf[5] << 8));
    imu_dev->raw_data.gyro_x = (int16_t)(buf[6] | (buf[7] << 8));
    imu_dev->raw_data.gyro_y = (int16_t)(buf[8] | (buf[9] << 8));
    imu_dev->raw_data.gyro_z = (int16_t)(buf[10] | (buf[11] << 8));
    imu_dev->raw_data.temp = (int16_t)(buf[12] | (buf[13] << 8));
    
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     void inv_imu_convert_data(struct inv_imu_device *imu_dev)
 *功　　能:     将原始数据转换为物理量
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     无
 ******************************************************************************/
void inv_imu_convert_data(struct inv_imu_device *imu_dev)
{
    /* 转换加速度计数据 (单位: m/s²) */
    imu_dev->data.accel_x = (float)imu_dev->raw_data.accel_x * imu_dev->accel_scale;
    imu_dev->data.accel_y = (float)imu_dev->raw_data.accel_y * imu_dev->accel_scale;
    imu_dev->data.accel_z = (float)imu_dev->raw_data.accel_z * imu_dev->accel_scale;
    
    /* 转换陀螺仪数据 (单位: rad/s) */
    imu_dev->data.gyro_x = (float)imu_dev->raw_data.gyro_x * imu_dev->gyro_scale;
    imu_dev->data.gyro_y = (float)imu_dev->raw_data.gyro_y * imu_dev->gyro_scale;
    imu_dev->data.gyro_z = (float)imu_dev->raw_data.gyro_z * imu_dev->gyro_scale;
    
    /* 转换温度数据 (单位: ℃) */
    imu_dev->data.temp = 25.0f + (float)imu_dev->raw_data.temp / 256.0f;
}

/**************************实现函数********************************************
 *函数原型:     int inv_imu_set_accel_fs(struct inv_imu_device *imu_dev, uint8_t fs)
 *功　　能:     设置加速度计量程
 *输入参数:     imu_dev - 设备结构体指针, fs - 量程(0-3对应2G/4G/8G/16G)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_set_accel_fs(struct inv_imu_device *imu_dev, uint8_t fs)
{
    uint8_t reg_val;
    int ret;
    
    if (fs > 3) {
        pr_err("Invalid accelerometer full scale: %u\n", fs);
        return -EINVAL;
    }
    
    /* 读取当前配置 */
    ret = inv_imu_read_reg(imu_dev, ICM45686_REG_ACCEL_CONFIG0, &reg_val);
    if (ret < 0) {
        pr_err("Failed to read ACCEL_CONFIG0 register\n");
        return ret;
    }
    
    /* 更新量程配置 */
    reg_val = (reg_val & ~ICM45686_ACCEL_CONFIG0_FS_MASK) | fs;
    ret = inv_imu_write_reg(imu_dev, ICM45686_REG_ACCEL_CONFIG0, reg_val);
    if (ret < 0) {
        pr_err("Failed to write ACCEL_CONFIG0 register\n");
        return ret;
    }
    
    /* 更新刻度因子 */
    imu_dev->accel_fs = fs;
    imu_dev->accel_scale = 9.81f / 32768.0f * (float)(1 << fs);
    
    pr_info("Accelerometer full scale set to %dG\n", 2 << fs);
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     int inv_imu_set_gyro_fs(struct inv_imu_device *imu_dev, uint8_t fs)
 *功　　能:     设置陀螺仪量程
 *输入参数:     imu_dev - 设备结构体指针, fs - 量程(0-3对应250/500/1000/2000DPS)
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int inv_imu_set_gyro_fs(struct inv_imu_device *imu_dev, uint8_t fs)
{
    uint8_t reg_val;
    int ret;
    float dps_range[] = {250.0f, 500.0f, 1000.0f, 2000.0f};
    
    if (fs > 3) {
        pr_err("Invalid gyroscope full scale: %u\n", fs);
        return -EINVAL;
    }
    
    /* 读取当前配置 */
    ret = inv_imu_read_reg(imu_dev, ICM45686_REG_GYRO_CONFIG0, &reg_val);
    if (ret < 0) {
        pr_err("Failed to read GYRO_CONFIG0 register\n");
        return ret;
    }
    
    /* 更新量程配置 */
    reg_val = (reg_val & ~ICM45686_GYRO_CONFIG0_FS_MASK) | fs;
    ret = inv_imu_write_reg(imu_dev, ICM45686_REG_GYRO_CONFIG0, reg_val);
    if (ret < 0) {
        pr_err("Failed to write GYRO_CONFIG0 register\n");
        return ret;
    }
    
    /* 更新刻度因子 */
    imu_dev->gyro_fs = fs;
    imu_dev->gyro_scale = dps_range[fs] * M_PI / 180.0f / 32768.0f;
    
    pr_info("Gyroscope full scale set to %.0fDPS\n", dps_range[fs]);
    return 0;
}

EXPORT_SYMBOL_GPL(inv_imu_init_hw);
EXPORT_SYMBOL_GPL(inv_imu_read_raw_data);
EXPORT_SYMBOL_GPL(inv_imu_convert_data);
EXPORT_SYMBOL_GPL(inv_imu_set_accel_fs);
EXPORT_SYMBOL_GPL(inv_imu_set_gyro_fs);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ICM45686 IMU Driver Core");
MODULE_AUTHOR("Developer");
