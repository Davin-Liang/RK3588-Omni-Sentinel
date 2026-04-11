/*
 * icm45686_transport.c - ICM45686惯性测量单元传输接口实现
 *
 * 适用于RKS588ELF2板子的ICM45686传感器驱动
 *
 * 日期: 2026-04-09
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>

#include "icm45686_transport.h"

static int i2c_fd = -1;  /* I2C文件描述符 */
static int spi_fd = -1;  /* SPI文件描述符 */
static uint8_t i2c_addr = 0;  /* I2C设备地址 */

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_i2c_init(int i2c_bus, uint8_t addr)
 *功　　能: 	  初始化I2C传输接口
 *输入参数：  i2c_bus - I2C总线号
 *            addr - I2C设备地址
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_i2c_init(int i2c_bus, uint8_t addr)
{
    char i2c_path[20];
    int ret;

    /* 构建I2C设备路径 */
    snprintf(i2c_path, sizeof(i2c_path), "/dev/i2c-%d", i2c_bus);

    /* 打开I2C设备 */
    i2c_fd = open(i2c_path, O_RDWR);
    if (i2c_fd < 0) {
        perror("打开I2C总线失败");
        return -1;
    }

    /* 设置从设备地址 */
    ret = ioctl(i2c_fd, I2C_SLAVE, addr);
    if (ret < 0) {
        perror("设置I2C从设备地址失败");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }

    i2c_addr = addr;
    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_spi_init(int spi_bus, uint8_t cs)
 *功　　能: 	  初始化SPI传输接口
 *输入参数：  spi_bus - SPI总线号
 *            cs - SPI片选号
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_spi_init(int spi_bus, uint8_t cs)
{
    char spi_path[20];
    uint8_t mode = SPI_MODE_3;  /* SPI模式3 */
    uint8_t bits = 8;           /* 8位数据 */
    uint32_t speed = 10000000;  /* 10MHz */
    int ret;

    /* 构建SPI设备路径 */
    snprintf(spi_path, sizeof(spi_path), "/dev/spidev%d.%d", spi_bus, cs);

    /* 打开SPI设备 */
    spi_fd = open(spi_path, O_RDWR);
    if (spi_fd < 0) {
        perror("打开SPI总线失败");
        return -1;
    }

    /* 设置SPI模式 */
    ret = ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    if (ret < 0) {
        perror("设置SPI模式失败");
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    /* 设置数据位 */
    ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    if (ret < 0) {
        perror("设置SPI数据位失败");
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    /* 设置时钟频率 */
    ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    if (ret < 0) {
        perror("设置SPI速度失败");
        close(spi_fd);
        spi_fd = -1;
        return -1;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_i2c_read(uint8_t reg, uint8_t *data, uint16_t len)
 *功　　能: 	  I2C读取函数
 *输入参数：  reg - 寄存器地址
 *            len - 数据长度
 *输出参数：  data - 读取的数据
 *            0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_i2c_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    int ret;

    if (i2c_fd < 0) {
        return -1;
    }

    /* 写入寄存器地址 */
    ret = write(i2c_fd, &reg, 1);
    if (ret != 1) {
        return -1;
    }

    /* 读取数据 */
    ret = read(i2c_fd, data, len);
    if (ret != len) {
        return -1;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_i2c_write(uint8_t reg, uint8_t *data, uint16_t len)
 *功　　能: 	  I2C写入函数
 *输入参数：  reg - 寄存器地址
 *            data - 写入的数据
 *            len - 数据长度
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_i2c_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    uint8_t *buf;
    int ret;

    if (i2c_fd < 0) {
        return -1;
    }

    /* 分配缓冲区 */
    buf = (uint8_t *)malloc(len + 1);
    if (!buf) {
        return -1;
    }

    /* 填充数据 */
    buf[0] = reg;
    memcpy(&buf[1], data, len);

    /* 写入数据 */
    ret = write(i2c_fd, buf, len + 1);
    free(buf);

    if (ret != len + 1) {
        return -1;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_spi_read(uint8_t reg, uint8_t *data, uint16_t len)
 *功　　能: 	  SPI读取函数
 *输入参数：  reg - 寄存器地址
 *            len - 数据长度
 *输出参数：  data - 读取的数据
 *            0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_spi_read(uint8_t reg, uint8_t *data, uint16_t len)
{
    struct spi_ioc_transfer tr[2];
    uint8_t cmd = reg | 0x80;  /* 设置读取位 */
    int ret;

    if (spi_fd < 0) {
        return -1;
    }

    /* 清空传输结构体 */
    memset(tr, 0, sizeof(tr));

    /* 传输命令 */
    tr[0].tx_buf = (unsigned long)&cmd;
    tr[0].rx_buf = 0;
    tr[0].len = 1;
    tr[0].delay_usecs = 0;
    tr[0].speed_hz = 10000000;
    tr[0].bits_per_word = 8;

    /* 传输数据 */
    tr[1].tx_buf = 0;
    tr[1].rx_buf = (unsigned long)data;
    tr[1].len = len;
    tr[1].delay_usecs = 0;
    tr[1].speed_hz = 10000000;
    tr[1].bits_per_word = 8;

    /* 执行传输 */
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(2), tr);
    if (ret < 0) {
        return -1;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    int icm45686_spi_write(uint8_t reg, uint8_t *data, uint16_t len)
 *功　　能: 	  SPI写入函数
 *输入参数：  reg - 寄存器地址
 *            data - 写入的数据
 *            len - 数据长度
 *输出参数：  0 - 成功，负数 - 错误
 *******************************************************************************/
int icm45686_spi_write(uint8_t reg, uint8_t *data, uint16_t len)
{
    struct spi_ioc_transfer tr;
    uint8_t *buf;
    int ret;

    if (spi_fd < 0) {
        return -1;
    }

    /* 分配缓冲区 */
    buf = (uint8_t *)malloc(len + 1);
    if (!buf) {
        return -1;
    }

    /* 填充数据 */
    buf[0] = reg & 0x7F;  /* 清除读取位 */
    memcpy(&buf[1], data, len);

    /* 清空传输结构体 */
    memset(&tr, 0, sizeof(tr));

    /* 设置传输参数 */
    tr.tx_buf = (unsigned long)buf;
    tr.rx_buf = 0;
    tr.len = len + 1;
    tr.delay_usecs = 0;
    tr.speed_hz = 10000000;
    tr.bits_per_word = 8;

    /* 执行传输 */
    ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    free(buf);

    if (ret < 0) {
        return -1;
    }

    return 0;
}

/**************************实现函数********************************************
 *函数原型: 	    void icm45686_delay_us(uint32_t us)
 *功　　能: 	  延时函数
 *输入参数：  us - 延时时间（微秒）
 *输出参数：  无
 *******************************************************************************/
void icm45686_delay_us(uint32_t us)
{
    usleep(us);
}
