/*
 * icm45686.c - ICM45686惯性测量单元驱动实现
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * Copyright (c) 2026
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pm.h>
#include <linux/delay.h>
#include <linux/mutex.h>

#include "icm45686.h"

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

/* I2C读取函数 */
int icm45686_i2c_read(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int ret;

    mutex_lock(&dev->lock);
    ret = i2c_smbus_read_i2c_block_data(dev->i2c_client, reg, len, data);
    mutex_unlock(&dev->lock);

    return ret;
}

/* I2C写入函数 */
int icm45686_i2c_write(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int ret;

    mutex_lock(&dev->lock);
    ret = i2c_smbus_write_i2c_block_data(dev->i2c_client, reg, len, data);
    mutex_unlock(&dev->lock);

    return ret;
}

/* SPI读取函数 */
int icm45686_spi_read(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int ret;
    uint8_t cmd = reg | 0x80; /* 设置读取位 */

    mutex_lock(&dev->lock);
    ret = spi_write_then_read(dev->spi_device, &cmd, 1, data, len);
    mutex_unlock(&dev->lock);

    return ret;
}

/* SPI写入函数 */
int icm45686_spi_write(icm45686_dev_t *dev, uint8_t reg, uint8_t *data, uint16_t len)
{
    int ret;
    uint8_t *buf;

    buf = kzalloc(len + 1, GFP_KERNEL);
    if (!buf) {
        return -ENOMEM;
    }

    buf[0] = reg & 0x7F; /* 清除读取位 */
    memcpy(&buf[1], data, len);

    mutex_lock(&dev->lock);
    ret = spi_write(dev->spi_device, buf, len + 1);
    mutex_unlock(&dev->lock);

    kfree(buf);
    return ret;
}

/* 读取单个寄存器 */
static int icm45686_read_reg(icm45686_dev_t *dev, uint8_t reg, uint8_t *val)
{
    int ret;

    if (dev->i2c_client) {
        ret = icm45686_i2c_read(dev, reg, val, 1);
    } else if (dev->spi_device) {
        ret = icm45686_spi_read(dev, reg, val, 1);
    } else {
        return -EINVAL;
    }

    return ret;
}

/* 写入单个寄存器 */
static int icm45686_write_reg(icm45686_dev_t *dev, uint8_t reg, uint8_t val)
{
    int ret;

    if (dev->i2c_client) {
        ret = icm45686_i2c_write(dev, reg, &val, 1);
    } else if (dev->spi_device) {
        ret = icm45686_spi_write(dev, reg, &val, 1);
    } else {
        return -EINVAL;
    }

    return ret;
}

/* 初始化ICM45686 */
int icm45686_init(icm45686_dev_t *dev)
{
    int ret;
    uint8_t val;

    /* 读取WHO_AM_I寄存器，确认设备 */
    ret = icm45686_read_reg(dev, ICM45686_REG_WHO_AM_I, &val);
    if (ret < 0) {
        dev_err(dev->dev, "Failed to read WHO_AM_I: %d\n", ret);
        return ret;
    }

    if (val != ICM45686_WHO_AM_I_VAL) {
        dev_err(dev->dev, "Invalid WHO_AM_I value: 0x%02x\n", val);
        return -ENODEV;
    }

    dev_info(dev->dev, "ICM45686 detected\n");

    /* 复位信号路径 */
    val = 0x01;
    ret = icm45686_write_reg(dev, ICM45686_REG_SIGNAL_PATH_RESET, val);
    if (ret < 0) {
        dev_err(dev->dev, "Failed to reset signal path: %d\n", ret);
        return ret;
    }

    msleep(10);

    /* 配置电源管理0 - 启用加速度计和陀螺仪 */
    val = ICM45686_PWR_MGMT0_ACCEL_MODE_HP | ICM45686_PWR_MGMT0_GYRO_MODE_HP;
    ret = icm45686_write_reg(dev, ICM45686_REG_PWR_MGMT0, val);
    if (ret < 0) {
        dev_err(dev->dev, "Failed to configure power management: %d\n", ret);
        return ret;
    }

    msleep(10);

    /* 配置加速度计 - 2G量程，100Hz输出数据率 */
    val = (0x03 << 4) | ICM45686_ACCEL_CONFIG0_FS_2G; /* 100Hz ODR */
    ret = icm45686_write_reg(dev, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret < 0) {
        dev_err(dev->dev, "Failed to configure accelerometer: %d\n", ret);
        return ret;
    }

    /* 配置陀螺仪 - 250DPS量程，100Hz输出数据率 */
    val = (0x03 << 4) | ICM45686_GYRO_CONFIG0_FS_250DPS; /* 100Hz ODR */
    ret = icm45686_write_reg(dev, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret < 0) {
        dev_err(dev->dev, "Failed to configure gyroscope: %d\n", ret);
        return ret;
    }

    /* 设置缩放因子 */
    dev->accel_fs = ICM45686_ACCEL_CONFIG0_FS_2G;
    dev->gyro_fs = ICM45686_GYRO_CONFIG0_FS_250DPS;
    dev->accel_scale = accel_scale[dev->accel_fs];
    dev->gyro_scale = gyro_scale[dev->gyro_fs];

    dev_info(dev->dev, "ICM45686 initialized successfully\n");
    return 0;
}

/* 读取原始数据 */
int icm45686_read_raw_data(icm45686_dev_t *dev, icm45686_raw_data_t *data)
{
    int ret;
    uint8_t buf[14];

    if (dev->i2c_client) {
        ret = icm45686_i2c_read(dev, ICM45686_REG_ACCEL_DATA_X0, buf, 14);
    } else if (dev->spi_device) {
        ret = icm45686_spi_read(dev, ICM45686_REG_ACCEL_DATA_X0, buf, 14);
    } else {
        return -EINVAL;
    }

    if (ret < 0) {
        dev_err(dev->dev, "Failed to read raw data: %d\n", ret);
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

/* 读取转换后的数据 */
int icm45686_read_data(icm45686_dev_t *dev, icm45686_data_t *data)
{
    int ret;
    icm45686_raw_data_t raw_data;

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

/* 设置加速度计量程 */
int icm45686_set_accel_fs(icm45686_dev_t *dev, uint8_t fs)
{
    int ret;
    uint8_t val;

    if (fs > 3) {
        return -EINVAL;
    }

    ret = icm45686_read_reg(dev, ICM45686_REG_ACCEL_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~ICM45686_ACCEL_CONFIG0_FS_MASK) | fs;
    ret = icm45686_write_reg(dev, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    dev->accel_fs = fs;
    dev->accel_scale = accel_scale[fs];

    return 0;
}

/* 设置陀螺仪量程 */
int icm45686_set_gyro_fs(icm45686_dev_t *dev, uint8_t fs)
{
    int ret;
    uint8_t val;

    if (fs > 3) {
        return -EINVAL;
    }

    ret = icm45686_read_reg(dev, ICM45686_REG_GYRO_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~ICM45686_GYRO_CONFIG0_FS_MASK) | fs;
    ret = icm45686_write_reg(dev, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    dev->gyro_fs = fs;
    dev->gyro_scale = gyro_scale[fs];

    return 0;
}

/* 设置加速度计输出数据率 */
int icm45686_set_accel_odr(icm45686_dev_t *dev, uint8_t odr)
{
    int ret;
    uint8_t val;

    if (odr > 0x0F) {
        return -EINVAL;
    }

    ret = icm45686_read_reg(dev, ICM45686_REG_ACCEL_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~ICM45686_ACCEL_CONFIG0_ODR_MASK) | (odr << 4);
    ret = icm45686_write_reg(dev, ICM45686_REG_ACCEL_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/* 设置陀螺仪输出数据率 */
int icm45686_set_gyro_odr(icm45686_dev_t *dev, uint8_t odr)
{
    int ret;
    uint8_t val;

    if (odr > 0x0F) {
        return -EINVAL;
    }

    ret = icm45686_read_reg(dev, ICM45686_REG_GYRO_CONFIG0, &val);
    if (ret < 0) {
        return ret;
    }

    val = (val & ~ICM45686_GYRO_CONFIG0_ODR_MASK) | (odr << 4);
    ret = icm45686_write_reg(dev, ICM45686_REG_GYRO_CONFIG0, val);
    if (ret < 0) {
        return ret;
    }

    return 0;
}

/* I2C设备ID表 */
static const struct i2c_device_id icm45686_i2c_id[] = {
    { "icm45686", 0 },
    { }
};

MODULE_DEVICE_TABLE(i2c, icm45686_i2c_id);

/* SPI设备ID表 */
static const struct spi_device_id icm45686_spi_id[] = {
    { "icm45686", 0 },
    { }
};

MODULE_DEVICE_TABLE(spi, icm45686_spi_id);

/* 设备树匹配表 */
static const struct of_device_id icm45686_of_match[] = {
    { .compatible = "invensense,icm45686" },
    { }
};

MODULE_DEVICE_TABLE(of, icm45686_of_match);

/* I2C探测函数 */
int icm45686_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    icm45686_dev_t *dev;

    dev = devm_kzalloc(&client->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }

    dev->i2c_client = client;
    dev->dev = &client->dev;
    dev->addr = client->addr;
    mutex_init(&dev->lock);

    i2c_set_clientdata(client, dev);

    ret = icm45686_init(dev);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to initialize ICM45686: %d\n", ret);
        return ret;
    }

    dev_info(&client->dev, "ICM45686 I2C probe successful\n");
    return 0;
}

/* SPI探测函数 */
int icm45686_spi_probe(struct spi_device *spi)
{
    int ret;
    icm45686_dev_t *dev;

    dev = devm_kzalloc(&spi->dev, sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }

    dev->spi_device = spi;
    dev->dev = &spi->dev;
    mutex_init(&dev->lock);

    spi_set_drvdata(spi, dev);

    ret = icm45686_init(dev);
    if (ret < 0) {
        dev_err(&spi->dev, "Failed to initialize ICM45686: %d\n", ret);
        return ret;
    }

    dev_info(&spi->dev, "ICM45686 SPI probe successful\n");
    return 0;
}

/* I2C移除函数 */
int icm45686_remove(struct i2c_client *client)
{
    icm45686_dev_t *dev = i2c_get_clientdata(client);

    if (dev) {
        mutex_destroy(&dev->lock);
    }

    dev_info(&client->dev, "ICM45686 I2C removed\n");
    return 0;
}

/* SPI移除函数 */
int icm45686_spi_remove(struct spi_device *spi)
{
    icm45686_dev_t *dev = spi_get_drvdata(spi);

    if (dev) {
        mutex_destroy(&dev->lock);
    }

    dev_info(&spi->dev, "ICM45686 SPI removed\n");
    return 0;
}

/* I2C驱动结构体 */
static struct i2c_driver icm45686_i2c_driver = {
    .driver = {
        .name = "icm45686",
        .of_match_table = icm45686_of_match,
    },
    .probe = icm45686_i2c_probe,
    .remove = icm45686_remove,
    .id_table = icm45686_i2c_id,
};

/* SPI驱动结构体 */
static struct spi_driver icm45686_spi_driver = {
    .driver = {
        .name = "icm45686",
        .of_match_table = icm45686_of_match,
    },
    .probe = icm45686_spi_probe,
    .remove = icm45686_spi_remove,
    .id_table = icm45686_spi_id,
};

/* 模块初始化函数 */
static int __init icm45686_init_module(void)
{
    int ret;

    ret = i2c_add_driver(&icm45686_i2c_driver);
    if (ret < 0) {
        pr_err("Failed to register I2C driver: %d\n", ret);
        return ret;
    }

    ret = spi_register_driver(&icm45686_spi_driver);
    if (ret < 0) {
        pr_err("Failed to register SPI driver: %d\n", ret);
        i2c_del_driver(&icm45686_i2c_driver);
        return ret;
    }

    pr_info("ICM45686 driver initialized\n");
    return 0;
}

/* 模块退出函数 */
static void __exit icm45686_exit_module(void)
{
    spi_unregister_driver(&icm45686_spi_driver);
    i2c_del_driver(&icm45686_i2c_driver);
    pr_info("ICM45686 driver exited\n");
}

module_init(icm45686_init_module);
module_exit(icm45686_exit_module);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("ICM45686 Inertial Measurement Unit Driver for RK3588");
MODULE_VERSION("1.0");
