# Makefile for ICM45686 driver and application

# 编译器
CC = gcc

# 编译选项
CFLAGS = -Wall -O2

# 应用程序名称
APP_NAME = icm45686_app
NEW_APP_NAME = icm45686_app_new

# 目标文件
APP_OBJS = icm45686_app.o
NEW_APP_OBJS = icm45686_app_new.o icm45686_driver.o icm45686_transport.o

# 默认目标
all: app new_app

# 编译原始应用程序
app: $(APP_OBJS)
	$(CC) $(CFLAGS) -o $(APP_NAME) $(APP_OBJS)

# 编译新应用程序
new_app: $(NEW_APP_OBJS)
	$(CC) $(CFLAGS) -o $(NEW_APP_NAME) $(NEW_APP_OBJS)

# 清理
clean:
	rm -f $(APP_NAME) $(NEW_APP_NAME) $(APP_OBJS) $(NEW_APP_OBJS)

# 测试原始应用
run:
	./$(APP_NAME)

# 测试新应用
run_new:
	./$(NEW_APP_NAME)

# 规则
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 依赖
icm45686_app.o: icm45686_app.c
icm45686_app_new.o: icm45686_app_new.c icm45686_driver.h
icm45686_driver.o: icm45686_driver.c icm45686_driver.h icm45686_defs.h
icm45686_transport.o: icm45686_transport.c icm45686_transport.h
