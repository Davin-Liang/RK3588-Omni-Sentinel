# Makefile for ICM45686 user application

# 编译器
CC = gcc

# 编译选项
CFLAGS = -Wall -O2 -lm

# 应用程序名称
APP_NAME = icm45686_app

# 目标文件
APP_OBJS = icm45686_app.o icm45686_user.o imu_ahrs.o

# 默认目标
all: $(APP_NAME)

# 编译应用程序
$(APP_NAME): $(APP_OBJS)
	$(CC) $(CFLAGS) -o $(APP_NAME) $(APP_OBJS)

# 清理
clean:
	rm -f $(APP_NAME) $(APP_OBJS)

# 运行应用程序
run:
	./$(APP_NAME)

# 规则
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 依赖
icm45686_app.o: icm45686_app.c icm45686_user.h imu_ahrs.h
icm45686_user.o: icm45686_user.c icm45686_user.h
imu_ahrs.o: imu_ahrs.c imu_ahrs.h

# 交叉编译
cross:
	arm-rockchip-linux-gnueabihf-gcc $(CFLAGS) -o $(APP_NAME)_arm $(APP_OBJS:.o=_arm.o)
