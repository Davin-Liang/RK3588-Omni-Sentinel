# Makefile for ICM45686 SPI kernel driver

# 目标模块
obj-m += icm45686_spi.o
obj-m += inv_imu_driver.o

# 源文件
icm45686_spi-objs := icm45686_spi.o
inv_imu_driver-objs := inv_imu_driver.o

# 默认目标
all:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules

# 清理
clean:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) clean
	rm -f *.ko *.o *.mod.o *.mod.c *.symvers *.order

# 安装
install:
	$(MAKE) -C $(KERNEL_DIR) M=$(PWD) modules_install
	depmod -a

# 卸载
uninstall:
	rmmod icm45686_spi
	rmmod inv_imu_driver
