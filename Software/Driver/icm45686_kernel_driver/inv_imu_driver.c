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

/*
 * 固定点换算常数
 * 加速度输出单位：m/s² × 1000
 * 陀螺仪输出单位：rad/s × 1000
 */
#define ICM45686_GRAVITY_MS2_X1000       9807
#define ICM45686_DPS_TO_RAD_X1000_NUM    174533

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

    /*
     * 暂不执行软件复位。
     * 原驱动将0x72误作为信号路径复位寄存器，但0x72实际为WHO_AM_I。
     * 当前阶段优先保证寄存器地址、数据读取和基础配置正确。
     */
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

    msleep(100);

    /*
     * 设置默认刻度因子。
     * accel_scale保存加速度灵敏度，单位为LSB/g；
     * gyro_scale保存陀螺仪灵敏度×10，单位为(LSB/(dps))×10。
     */
    imu_dev->accel_fs = 0;         /* 用户接口0表示±2G */
    imu_dev->gyro_fs = 0;          /* 用户接口0表示±250DPS */
    imu_dev->accel_scale = 16384;  /* ±2G对应16384 LSB/g */
    imu_dev->gyro_scale = 1310;    /* ±250DPS对应131.0 LSB/(dps)，放大10倍 */

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
    /*
     * ICM45686 UI数据寄存器从0x00开始连续排列。
     * 注意：ICM45686默认数据字节序为Little Endian，
     * 因此这里按低字节在前、高字节在后的方式组装16位数据。
     */
    ret = imu_dev->transport.read_reg(&imu_dev->transport,
                                      ICM45686_REG_ACCEL_DATA_X1,
                                      buf, 14);
    if (ret < 0) {
        pr_err("Failed to read sensor data\n");
        return ret;
    }

    /* 默认Little Endian：低字节在前，高字节在后 */
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
 *功　　能:     将原始数据转换为物理量（整数运算，避免内核浮点问题）
 *              注意：转换后的值为实际物理值×1000，温度为实际值×100
 *输入参数:     imu_dev - 设备结构体指针
 *输出参数:     无
 ******************************************************************************/
void inv_imu_convert_data(struct inv_imu_device *imu_dev)
{
    int32_t accel_lsb_per_g = imu_dev->accel_scale;
    int32_t gyro_lsb_per_dps_x10 = imu_dev->gyro_scale;

    if (accel_lsb_per_g <= 0)
        accel_lsb_per_g = 16384;

    if (gyro_lsb_per_dps_x10 <= 0)
        gyro_lsb_per_dps_x10 = 1310;

    /*
     * 加速度计换算：
     * 输出(m/s²×1000) = raw / (LSB/g) × 9.807 × 1000
     */
    imu_dev->data.accel_x = (int32_t)((int64_t)imu_dev->raw_data.accel_x *
                                      ICM45686_GRAVITY_MS2_X1000 /
                                      accel_lsb_per_g);
    imu_dev->data.accel_y = (int32_t)((int64_t)imu_dev->raw_data.accel_y *
                                      ICM45686_GRAVITY_MS2_X1000 /
                                      accel_lsb_per_g);
    imu_dev->data.accel_z = (int32_t)((int64_t)imu_dev->raw_data.accel_z *
                                      ICM45686_GRAVITY_MS2_X1000 /
                                      accel_lsb_per_g);

    /*
     * 陀螺仪换算：
     * 输出(rad/s×1000) = raw / (LSB/dps) × pi/180 × 1000
     * gyro_lsb_per_dps_x10为灵敏度放大10倍后的整数值。
     */
    imu_dev->data.gyro_x = (int32_t)((int64_t)imu_dev->raw_data.gyro_x *
                                     ICM45686_DPS_TO_RAD_X1000_NUM /
                                     (gyro_lsb_per_dps_x10 * 1000));
    imu_dev->data.gyro_y = (int32_t)((int64_t)imu_dev->raw_data.gyro_y *
                                     ICM45686_DPS_TO_RAD_X1000_NUM /
                                     (gyro_lsb_per_dps_x10 * 1000));
    imu_dev->data.gyro_z = (int32_t)((int64_t)imu_dev->raw_data.gyro_z *
                                     ICM45686_DPS_TO_RAD_X1000_NUM /
                                     (gyro_lsb_per_dps_x10 * 1000));

    /*
     * 温度换算：
     * 手册中25℃输出为0 LSB，温度灵敏度为128 LSB/℃。
     */
    imu_dev->data.temp = 2500 + (int32_t)imu_dev->raw_data.temp * 100 / 128;
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
    uint8_t reg_fs;
    int ret;

    static const uint8_t accel_fs_reg_table[] = {
        ICM45686_ACCEL_CONFIG0_FS_2G,
        ICM45686_ACCEL_CONFIG0_FS_4G,
        ICM45686_ACCEL_CONFIG0_FS_8G,
        ICM45686_ACCEL_CONFIG0_FS_16G,
    };

    static const int accel_lsb_per_g_table[] = {
        16384,  /* ±2G */
        8192,   /* ±4G */
        4096,   /* ±8G */
        2048,   /* ±16G */
    };

    static const int accel_g_table[] = {2, 4, 8, 16};

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

    /* 更新量程配置，保持ODR位不变 */
    reg_fs = accel_fs_reg_table[fs];
    reg_val = (reg_val & ~ICM45686_ACCEL_CONFIG0_FS_MASK) |
              (reg_fs << ICM45686_ACCEL_CONFIG0_FS_SHIFT);

    ret = inv_imu_write_reg(imu_dev, ICM45686_REG_ACCEL_CONFIG0, reg_val);
    if (ret < 0) {
        pr_err("Failed to write ACCEL_CONFIG0 register\n");
        return ret;
    }

    /* 更新刻度因子 */
    imu_dev->accel_fs = fs;
    imu_dev->accel_scale = accel_lsb_per_g_table[fs];

    pr_info("Accelerometer full scale set to ±%dG\n", accel_g_table[fs]);
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
    uint8_t reg_fs;
    int ret;

    static const uint8_t gyro_fs_reg_table[] = {
        ICM45686_GYRO_CONFIG0_FS_250DPS,
        ICM45686_GYRO_CONFIG0_FS_500DPS,
        ICM45686_GYRO_CONFIG0_FS_1000DPS,
        ICM45686_GYRO_CONFIG0_FS_2000DPS,
    };

    /*
     * 陀螺仪灵敏度表，单位为(LSB/(dps))×10。
     * 例如±250DPS为131 LSB/(dps)，这里保存为1310。
     */
    static const int gyro_lsb_per_dps_x10_table[] = {
        1310,  /* ±250DPS */
        655,   /* ±500DPS */
        328,   /* ±1000DPS */
        164,   /* ±2000DPS */
    };

    static const int gyro_dps_table[] = {250, 500, 1000, 2000};

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

    /* 更新量程配置，保持ODR位不变 */
    reg_fs = gyro_fs_reg_table[fs];
    reg_val = (reg_val & ~ICM45686_GYRO_CONFIG0_FS_MASK) |
              (reg_fs << ICM45686_GYRO_CONFIG0_FS_SHIFT);

    ret = inv_imu_write_reg(imu_dev, ICM45686_REG_GYRO_CONFIG0, reg_val);
    if (ret < 0) {
        pr_err("Failed to write GYRO_CONFIG0 register\n");
        return ret;
    }

    /* 更新刻度因子 */
    imu_dev->gyro_fs = fs;
    imu_dev->gyro_scale = gyro_lsb_per_dps_x10_table[fs];

    pr_info("Gyroscope full scale set to ±%dDPS\n", gyro_dps_table[fs]);
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

