/*
 * icm45686_spi.c - ICM45686 SPI设备驱动实现
 *
 * 适用于RK3588平台的ICM45686传感器驱动
 *
 * 日期: 2026-05-05
 */

#include "icm45686_spi.h"
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

#define DRIVER_NAME "icm45686_spi"
#define DEVICE_NAME "icm45686"

/**************************实现函数********************************************
 *函数原型:     static int icm45686_spi_read(struct inv_imu_transport *t, 
 *                                           uint8_t reg, uint8_t *buf, uint32_t len)
 *功　　能:     SPI读寄存器（适配官方驱动的传输接口）
 *输入参数:     t - 传输结构体, reg - 寄存器地址, buf - 数据缓冲区, len - 长度
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_spi_read(struct inv_imu_transport *t, uint8_t reg, 
                              uint8_t *buf, uint32_t len)
{
    struct icm45686_data *data = container_of(t, struct icm45686_data, imu_dev.transport);
    struct spi_device *spi = data->spi;
    uint8_t tx_buf = reg | 0x80;  /* 最高位为0表示读 */
    int ret;
    
    ret = spi_write_then_read(spi, &tx_buf, 1, buf, len);
    if (ret < 0) {
        dev_err(&spi->dev, "SPI read failed\n");
        return ret;
    }
    
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     static int icm45686_spi_write(struct inv_imu_transport *t, 
 *                                            uint8_t reg, const uint8_t *buf, uint32_t len)
 *功　　能:     SPI写寄存器（适配官方驱动的传输接口）
 *输入参数:     t - 传输结构体, reg - 寄存器地址, buf - 数据缓冲区, len - 长度
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_spi_write(struct inv_imu_transport *t, uint8_t reg, 
                               const uint8_t *buf, uint32_t len)
{
    struct icm45686_data *data = container_of(t, struct icm45686_data, imu_dev.transport);
    struct spi_device *spi = data->spi;
    uint8_t *tx_buf;
    int ret;
    
    tx_buf = kmalloc(len + 1, GFP_KERNEL);
    if (!tx_buf)
        return -ENOMEM;
    
    tx_buf[0] = reg & 0x7F;  /* 最高位为1表示写 */
    memcpy(&tx_buf[1], buf, len);
    
    ret = spi_write(spi, tx_buf, len + 1);
    if (ret < 0) {
        dev_err(&spi->dev, "SPI write failed\n");
    }
    
    kfree(tx_buf);
    return ret;
}

/**************************实现函数********************************************
 *函数原型:     static ssize_t icm45686_read(struct file *filp, char __user *buf, 
 *                                           size_t count, loff_t *f_pos)
 *功　　能:     字符设备读操作
 *输入参数:     filp - 文件指针, buf - 用户缓冲区, count - 读取长度, f_pos - 文件位置
 *输出参数:     读取的字节数，负数失败
 ******************************************************************************/
static ssize_t icm45686_read(struct file *filp, char __user *buf, 
                              size_t count, loff_t *f_pos)
{
    struct icm45686_data *data = filp->private_data;
    struct inv_imu_data_t sensor_data;
    int ret;
    
    mutex_lock(&data->lock);
    
    /* 读取传感器数据 */
    ret = inv_imu_read_raw_data(&data->imu_dev);
    if (ret < 0) {
        mutex_unlock(&data->lock);
        return ret;
    }
    
    inv_imu_convert_data(&data->imu_dev);
    sensor_data = data->imu_dev.data;
    
    mutex_unlock(&data->lock);
    
    /* 复制数据到用户空间 */
    if (count > sizeof(sensor_data))
        count = sizeof(sensor_data);
    
    if (copy_to_user(buf, &sensor_data, count))
        return -EFAULT;
    
    return count;
}

/**************************实现函数********************************************
 *函数原型:     static long icm45686_ioctl(struct file *filp, unsigned int cmd, 
 *                                         unsigned long arg)
 *功　　能:     字符设备IOCTL操作
 *输入参数:     filp - 文件指针, cmd - IOCTL命令, arg - 参数
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static long icm45686_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    struct icm45686_data *data = filp->private_data;
    uint8_t fs;
    int ret;
    
    if (_IOC_TYPE(cmd) != ICM45686_IOC_MAGIC)
        return -ENOTTY;
    
    mutex_lock(&data->lock);
    
    switch (cmd) {
    case ICM45686_IOC_READ_DATA: {
        struct inv_imu_data_t sensor_data;
        
        ret = inv_imu_read_raw_data(&data->imu_dev);
        if (ret < 0) {
            mutex_unlock(&data->lock);
            return ret;
        }
        
        inv_imu_convert_data(&data->imu_dev);
        sensor_data = data->imu_dev.data;
        
        if (copy_to_user((void __user *)arg, &sensor_data, sizeof(sensor_data))) {
            mutex_unlock(&data->lock);
            return -EFAULT;
        }
        break;
    }
    case ICM45686_IOC_SET_ACCEL_FS:
        if (copy_from_user(&fs, (void __user *)arg, sizeof(fs))) {
            mutex_unlock(&data->lock);
            return -EFAULT;
        }
        ret = inv_imu_set_accel_fs(&data->imu_dev, fs);
        break;
    case ICM45686_IOC_SET_GYRO_FS:
        if (copy_from_user(&fs, (void __user *)arg, sizeof(fs))) {
            mutex_unlock(&data->lock);
            return -EFAULT;
        }
        ret = inv_imu_set_gyro_fs(&data->imu_dev, fs);
        break;
    default:
        mutex_unlock(&data->lock);
        return -ENOTTY;
    }
    
    mutex_unlock(&data->lock);
    return ret;
}

/**************************实现函数********************************************
 *函数原型:     static int icm45686_open(struct inode *inode, struct file *filp)
 *功　　能:     字符设备打开操作
 *输入参数:     inode - inode指针, filp - 文件指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_open(struct inode *inode, struct file *filp)
{
    struct icm45686_data *data = container_of(inode->i_cdev, struct icm45686_data, cdev);
    
    filp->private_data = data;
    
    mutex_lock(&data->lock);
    data->open_count++;
    mutex_unlock(&data->lock);
    
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     static int icm45686_release(struct inode *inode, struct file *filp)
 *功　　能:     字符设备释放操作
 *输入参数:     inode - inode指针, filp - 文件指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_release(struct inode *inode, struct file *filp)
{
    struct icm45686_data *data = filp->private_data;
    
    mutex_lock(&data->lock);
    data->open_count--;
    mutex_unlock(&data->lock);
    
    return 0;
}

/* 文件操作结构体 */
static const struct file_operations icm45686_fops = {
    .owner = THIS_MODULE,
    .read = icm45686_read,
    .unlocked_ioctl = icm45686_ioctl,
    .open = icm45686_open,
    .release = icm45686_release,
};

/**************************实现函数********************************************
 *函数原型:     static int icm45686_create_char_device(struct icm45686_data *data)
 *功　　能:     创建字符设备
 *输入参数:     data - 设备数据结构
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_create_char_device(struct icm45686_data *data)
{
    int ret;
    
    /* 分配设备号 */
    ret = alloc_chrdev_region(&data->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&data->spi->dev, "Failed to allocate char device region\n");
        return ret;
    }
    
    /* 初始化字符设备 */
    cdev_init(&data->cdev, &icm45686_fops);
    data->cdev.owner = THIS_MODULE;
    
    /* 添加字符设备 */
    ret = cdev_add(&data->cdev, data->devno, 1);
    if (ret < 0) {
        dev_err(&data->spi->dev, "Failed to add char device\n");
        unregister_chrdev_region(data->devno, 1);
        return ret;
    }
    
    /* 创建设备类 */
    data->class = class_create(THIS_MODULE, DEVICE_NAME);
    if (IS_ERR(data->class)) {
        dev_err(&data->spi->dev, "Failed to create class\n");
        cdev_del(&data->cdev);
        unregister_chrdev_region(data->devno, 1);
        return PTR_ERR(data->class);
    }
    
    /* 创建设备节点 */
    if (device_create(data->class, NULL, data->devno, NULL, DEVICE_NAME) == NULL) {
        dev_err(&data->spi->dev, "Failed to create device\n");
        class_destroy(data->class);
        cdev_del(&data->cdev);
        unregister_chrdev_region(data->devno, 1);
        return -EFAULT;
    }
    
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     static void icm45686_destroy_char_device(struct icm45686_data *data)
 *功　　能:     销毁字符设备
 *输入参数:     data - 设备数据结构
 *输出参数:     无
 ******************************************************************************/
static void icm45686_destroy_char_device(struct icm45686_data *data)
{
    if (data->class) {
        device_destroy(data->class, data->devno);
        class_destroy(data->class);
    }
    cdev_del(&data->cdev);
    unregister_chrdev_region(data->devno, 1);
}

/**************************实现函数********************************************
 *函数原型:     static int icm45686_probe(struct spi_device *spi)
 *功　　能:     SPI设备探测函数
 *输入参数:     spi - SPI设备结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_probe(struct spi_device *spi)
{
    struct icm45686_data *data;
    int ret;
    
    /* 分配设备数据结构 */
    data = devm_kzalloc(&spi->dev, sizeof(*data), GFP_KERNEL);
    if (!data)
        return -ENOMEM;
    
    /* 保存SPI设备指针 */
    data->spi = spi;
    spi_set_drvdata(spi, data);
    
    /* 配置SPI模式 (CPOL=1, CPHA=1) */
    spi->mode = SPI_MODE_3;
    spi->bits_per_word = 8;
    spi_setup(spi);
    
    /* 初始化RESET引脚 */
    data->reset_gpio = devm_gpiod_get(&spi->dev, "reset", GPIOD_OUT_LOW);
    if (IS_ERR(data->reset_gpio)) {
        dev_err(&spi->dev, "Failed to get reset GPIO\n");
        return PTR_ERR(data->reset_gpio);
    }
    
    /* 初始化INT引脚 */
    data->int_gpio = devm_gpiod_get_optional(&spi->dev, "interrupt", GPIOD_IN);
    if (IS_ERR(data->int_gpio)) {
        dev_err(&spi->dev, "Failed to get interrupt GPIO\n");
        return PTR_ERR(data->int_gpio);
    }
    
    /* 硬件复位 */
    gpiod_set_value(data->reset_gpio, 0);
    msleep(10);
    gpiod_set_value(data->reset_gpio, 1);
    msleep(50);
    
    /* 初始化传输接口 */
    data->imu_dev.transport.read_reg = icm45686_spi_read;
    data->imu_dev.transport.write_reg = icm45686_spi_write;
    data->imu_dev.transport.context = data;
    
    /* 初始化传感器寄存器 */
    ret = inv_imu_init_hw(&data->imu_dev);
    if (ret) {
        dev_err(&spi->dev, "Failed to initialize ICM45686\n");
        return ret;
    }
    
    /* 初始化互斥锁 */
    mutex_init(&data->lock);
    
    /* 创建字符设备 */
    ret = icm45686_create_char_device(data);
    if (ret) {
        dev_err(&spi->dev, "Failed to create char device\n");
        return ret;
    }
    
    dev_info(&spi->dev, "ICM45686 SPI driver loaded successfully\n");
    return 0;
}

/**************************实现函数********************************************
 *函数原型:     static int icm45686_remove(struct spi_device *spi)
 *功　　能:     SPI设备移除函数
 *输入参数:     spi - SPI设备结构体指针
 *输出参数:     0成功，负数失败
 ******************************************************************************/
static int icm45686_remove(struct spi_device *spi)
{
    struct icm45686_data *data = spi_get_drvdata(spi);
    
    /* 销毁字符设备 */
    icm45686_destroy_char_device(data);
    
    dev_info(&spi->dev, "ICM45686 SPI driver unloaded\n");
    return 0;
}

/* SPI设备匹配表 */
static const struct of_device_id icm45686_of_match[] = {
    { .compatible = "invensense,icm45686" },
    { /* Sentinel */ }
};
MODULE_DEVICE_TABLE(of, icm45686_of_match);

/* SPI驱动结构体 */
static struct spi_driver icm45686_spi_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .of_match_table = icm45686_of_match,
    },
    .probe = icm45686_probe,
    .remove = icm45686_remove,
};

module_spi_driver(icm45686_spi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ICM45686 SPI Driver for RK3588");
MODULE_AUTHOR("Developer");
MODULE_ALIAS("spi:icm45686");
