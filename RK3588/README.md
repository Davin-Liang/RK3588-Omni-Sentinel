# ICM45686驱动与应用说明

## 项目概述

实现了适用于RK3588平台的ICM45686惯性测量单元(IMU)驱动和应用程序，支持I2C和SPI两种通信方式。

## 目录结构

```
RK3588/
├── icm45686_defs.h        # 寄存器和位定义
├── icm45686_transport.h   # 传输接口头文件
├── icm45686_transport.c   # 传输接口实现
├── icm45686_driver.h      # 驱动接口头文件
├── icm45686_driver.c      # 驱动核心实现
├── icm45686_app.c         # 原始应用程序
├── icm45686_app_new.c     # 新应用程序
├── icm45686.dtsi          # 设备树文件
├── Makefile               # 编译文件
└── README.md              # 说明文档
```

## 硬件连接

### RKS588ELF2板子SPI接口引脚配置

| ICM45686引脚 | RKS588ELF2引脚 | 功能描述 |
|-------------|---------------|--------|
| VDD         | 3.3V          | 电源   |
| VDDIO       | 3.3V          | 数字电源 |
| GND         | GND           | 接地   |
| SCK         | SPI0_CLK      | SPI时钟 |
| SDA/MOSI    | SPI0_MOSI     | SPI数据输出 |
| SDO/MISO    | SPI0_MISO     | SPI数据输入 |
| CS          | SPI0_CS0      | SPI片选 |
| INT1        | GPIO0_B6      | 中断1   |
| INT2        | GPIO0_B7      | 中断2   |
| AD0         | GND           | I2C地址选择(0x68) |

### 硬件连接注意事项

1. 确保电源供应稳定，推荐使用3.3V电源
2. 连接时注意信号线的长度和阻抗匹配
3. 为了减少噪声干扰，建议在电源和地之间添加去耦电容
4. 确保SPI时钟频率不超过ICM45686的最大支持频率(10MHz)

## 驱动说明

### 驱动架构

本驱动采用分层架构设计：

1. **传输层**：负责I2C和SPI通信
2. **驱动层**：实现传感器的初始化和配置
3. **应用层**：提供用户接口

### 驱动特性

1. **双通信模式**：支持I2C和SPI两种通信方式
2. **传感器初始化**：配置加速度计和陀螺仪的工作模式、量程和输出数据率
3. **数据读取**：支持读取原始数据和转换后的数据
4. **参数配置**：可动态调整加速度计和陀螺仪的量程和输出数据率
5. **设备树支持**：通过设备树配置传感器参数

### 驱动使用方法

1. **初始化传输接口**：选择I2C或SPI通信方式
2. **初始化设备**：调用`icm45686_init`函数
3. **读取数据**：调用`icm45686_read_data`函数
4. **配置参数**：根据需要调用相应的配置函数

## 设备树编译与安装

### 设备树编译

```bash
# 进入内核源码目录
cd /path/to/kernel/source

# 编译设备树
make dtbs

# 或者单独编译特定设备树文件
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- rk3588s-firefly-roc-rk3588s-pc.dtb
```

### 设备树安装

```bash
# 复制编译好的设备树文件到启动分区
sudo cp arch/arm64/boot/dts/rockchip/rk3588s-firefly-roc-rk3588s-pc.dtb /boot/

# 更新启动配置
sudo update-grub
```

### 设备树集成

将`icm45686.dtsi`文件包含到主设备树文件中：

```dts
// 在主设备树文件中添加
#include "icm45686.dtsi"

&spi0 {
    status = "okay";
    
    #include "icm45686.dtsi"
};
```

## 应用程序说明

### 应用程序功能

1. **icm45686_app.c**：原始应用程序，直接使用I2C通信
2. **icm45686_app_new.c**：新应用程序，使用驱动架构，支持I2C和SPI通信

### 应用程序编译与运行

```bash
# 编译所有应用程序
make all

# 运行原始应用程序（I2C）
make run

# 运行新应用程序（默认使用SPI）
make run_new
```

### 应用程序配置

在`icm45686_app_new.c`中，可以通过修改`use_spi`变量来选择通信方式：

```c
int use_spi = 1;  /* 1: 使用SPI，0: 使用I2C */
```

## 设备树配置

### SPI接口配置

```dts
&spi0 {
    status = "okay";
    
    icm45686: icm45686@0 {
        compatible = "invensense,icm45686";
        reg = <0>;
        spi-max-frequency = <10000000>;
        interrupt-parent = <&gpio0>;
        interrupts = <18 IRQ_TYPE_LEVEL_HIGH>;
        reset-gpios = <&gpio0 19 GPIO_ACTIVE_LOW>;
    };
};
```

### I2C接口配置

```dts
&i2c0 {
    status = "okay";
    
    icm45686: icm45686@68 {
        compatible = "invensense,icm45686";
        reg = <0x68>;
        interrupt-parent = <&gpio0>;
        interrupts = <18 IRQ_TYPE_LEVEL_HIGH>;
        reset-gpios = <&gpio0 19 GPIO_ACTIVE_LOW>;
    };
};
```

## 测试与验证

### 设备树验证

1. **检查设备树是否正确加载**：
   ```bash
   # 查看设备树节点
   ls /sys/firmware/devicetree/base/spi@fe610000/
   
   # 检查ICM45686节点是否存在
   ls /sys/firmware/devicetree/base/spi@fe610000/icm45686@0/
   ```

2. **检查设备是否被识别**：
   ```bash
   # 查看SPI设备
   ls /dev/spidev*
   
   # 查看I2C设备
   i2cdetect -y 0
   ```

### 驱动测试

1. **编译并加载驱动**：
   ```bash
   make driver
   make load
   ```

2. **检查驱动是否加载成功**：
   ```bash
   # 查看驱动模块
   lsmod | grep icm45686
   
   # 查看内核日志
   dmesg | grep icm45686
   ```

3. **运行应用程序**：
   ```bash
   make app
   make run
   ```

4. **观察传感器数据输出**：
   - 静止状态下，加速度计Z轴应接近9.81 m/s²
   - 移动传感器，观察数据变化是否合理
   - 旋转传感器，观察陀螺仪数据变化

### 数据格式

应用程序输出的数据格式如下：

```
Accel X (m/s²) | Accel Y (m/s²) | Accel Z (m/s²) | Gyro X (dps) | Gyro Y (dps) | Gyro Z (dps) | Temp (°C)
---------------------------------------------------------------------------------------------------
       0.12 |        -0.05 |         9.81 |       0.00 |       0.00 |       0.00 |     25.50
```

### 预期数据范围

- **加速度计**：±2G (约±19.62 m/s²)
- **陀螺仪**：±250DPS (约±4.36 rad/s)
- **温度**：-40℃ ~ +85℃

## 故障排查

### 设备树问题

1. **设备树编译失败**
   - 检查设备树语法是否正确
   - 确认包含的文件路径是否正确
   - 查看编译错误信息

2. **设备树加载失败**
   - 检查设备树文件是否正确放置
   - 确认启动配置是否正确
   - 查看内核启动日志

3. **设备节点未创建**
   - 检查设备树配置是否正确
   - 确认SPI/I2C总线是否启用
   - 查看内核日志获取详细信息

### 驱动问题

1. **传感器未检测到**
   - 检查硬件连接
   - 确认I2C/SPI地址正确
   - 检查电源供应
   - 查看内核日志获取详细错误信息

2. **数据输出异常**
   - 检查SPI时钟频率是否过高
   - 确认传感器量程设置正确
   - 检查硬件连接是否松动
   - 校准传感器

3. **驱动加载失败**
   - 确认内核版本兼容
   - 检查设备树配置是否正确
   - 查看内核日志获取详细错误信息
   - 确认编译环境正确

4. **应用程序无法访问设备**
   - 检查设备权限
   - 确认驱动已正确加载
   - 检查设备节点是否存在

### 调试技巧

1. **查看内核日志**：
   ```bash
   dmesg | grep -i icm45686
   dmesg | grep -i spi
   dmesg | grep -i i2c
   ```

2. **检查设备状态**：
   ```bash
   # 查看SPI设备
   ls -la /dev/spidev*
   
   # 查看I2C设备
   i2cdetect -y 0
   
   # 查看设备树节点
   find /sys/firmware/devicetree/base -name "*icm45686*"
   ```

3. **测试SPI通信**：
   ```bash
   # 使用spidev_test工具测试SPI通信
   ./spidev_test -D /dev/spidev0.0 -s 10000000
   ```

4. **测试I2C通信**：
   ```bash
   # 使用i2c-tools测试I2C通信
   i2cget -y 0 0x68 0x0F
   ```

## 代码规范

### 命名规范

- **驱动文件**：
  - 寄存器和位定义：`icm45686_defs.h`
  - 传输接口：`icm45686_transport.h`/`icm45686_transport.c`
  - 驱动接口：`icm45686_driver.h`/`icm45686_driver.c`
  - 应用程序：`icm45686_app.c`/`icm45686_app_new.c`
  - 设备树文件：`icm45686.dtsi`

- **变量和函数**：
  - 使用小写字母加下划线的命名方式
  - 函数名：`icm45686_功能描述`
  - 变量名：`描述性名称`

### 注释规范

- **文件头部**：包含版权信息、文件描述和版本信息
- **函数注释**：使用Doxygen风格的注释，包括函数功能、参数和返回值
- **代码注释**：关键代码添加详细注释，解释实现原理
- **注释语言**：使用中文注释，与原STM32F103工程保持一致

### 代码风格

- **缩进**：使用4个空格
- **行宽**：每行不超过80个字符
- **函数长度**：单个函数长度控制在100行以内
- **代码结构**：采用分层架构，与原STM32F103工程结构类似
- **错误处理**：完善的错误处理机制

### 注释示例

```c
/** @brief 初始化ICM45686
 *  @param[in] dev 设备结构体指针
 *  @return 0 on success, negative value on error
 */
int icm45686_init(icm45686_dev_t *dev)
{
    // 读取WHO_AM_I寄存器，确认设备
    // ...
}
```

## 版本信息

- **驱动版本**：1.0
- **应用版本**：1.0
- **支持平台**：RK3588
- **支持传感器**：ICM45686

## 参考资料

1. ICM45686数据手册
2. RK3588数据手册
3. Linux内核文档
4. SPI/I2C通信协议
