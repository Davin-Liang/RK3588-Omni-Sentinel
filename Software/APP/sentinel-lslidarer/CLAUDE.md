# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 构建系统

这是一个 ROS2 colcon 工作空间。在工作空间根目录下构建：

```bash
source /opt/ros/<ros2-distro>/setup.bash
colcon build --symlink-install
source install/setup.bash
```

单独编译某个包：

```bash
colcon build --packages-select <package-name>
```

## 包结构

### `turn_on_wheeltec_robot` — 机器人启动与传感器集成

顶层集成包，包含两个 C++ 节点：

- **`wheeltec_robot_node`** (`src/wheeltec_robot.cpp`, `src/Quaternion_Solution.cpp`) — 通过串口 (`/dev/wheeltec_controller`, 115200 波特率) 与 Wheeltec 底盘控制器通信。发布里程计、接收 cmd_vel、处理机器人状态机。
- **`ImuProcessor`** (`src/ImuProcessor.cpp`) — IMU 数据处理，发布姿态数据。

核心配置集中在 `config/wheeltec_param.yaml`。这一个文件控制机器人型号、雷达类型、IMU 模式和相机模式的选择。启动时可通过 launch 参数覆盖 YAML 默认值。

主要 launch 文件：
- `turn_on_wheeltec_robot.launch.py` — 顶层启动：启动底盘串口、EKF、joint state publisher、机器人模型描述。
- `base_serial.launch.py` — 启动 `wheeltec_robot_node`，可选择启动 IMU 节点（stm32 或 H30 模式）。
- `wheeltec_lidar.launch.py` — 雷达调度：从 YAML 读取 `lidar_type`，启动对应的驱动（lslidar / ldlidar / rplidar）。
- `wheeltec_camera.launch.py` — 相机启动。
- `wheeltec_ekf.launch.py` — 里程计 EKF 融合。

### `wheeltec_lidar_ros2/lslidar_ros2/` — 镭神 (LSLIDAR) 雷达驱动

两个子包：

- **`lslidar_msgs`** — 自定义 ROS2 接口：`LslidarInformation.msg`、`LslidarPacket.msg`，以及 10 个服务定义（电机速度、上下电、授时模式、网络配置等）。
- **`lslidar_driver`** — 驱动库 + 节点。采用策略模式：`lslidar_driver_node.cpp` 从 ROS 参数中读取 `lidar_type` 并实例化对应的驱动子类：
  - `LslidarCxDriver` — 机械式多线雷达（C16、C32 等）
  - `LslidarChDriver` — 905nm 混合固态雷达（CH16X1、CH32A、CH64W 等）
  - `LslidarLsDriver` — 1550nm 系列（LS25D、LS128S1 等）
  - `LslidarX10Driver` — 单线雷达（M10、N10、N301 等）

  发布自定义点云类型 `PointXYZIRT`（x、y、z、intensity、ring、time），同时可发布 LaserScan 消息。支持 PCAP 离线回放、组播、多种滤波和坐标变换。通过 `config/` 目录下的 YAML 文件配置（如 `lslidar_cx.yaml`、`lslidar_x10.yaml`）。

### `wheeltec_lidar_ros2/rplidar_ros/` — 思岚 (SLAMTEC) RPLIDAR 驱动

标准思岚 ROS2 驱动，支持 A1/A2/A3/S1/S2/S3/T1/C1 型号。串口通信。`launch/` 中 `view_*` 开头的文件在启动驱动的同时启动 RViz。

### `wheeltec_lidar_ros2/ldlidar_ros2/` — LD 系列雷达驱动

支持 LD06/LD19/STL06N/STL19P/STL26/STL27L 系列雷达。

## 运行命令

启动完整机器人：

```bash
ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py
```

启动时指定雷达类型：

```bash
ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py lidar_type:=ls_M10P_uart
```

单独启动某个雷达驱动：

```bash
ros2 launch lslidar_driver lslidar_ch_launch.py   # 905 系列
ros2 launch lslidar_driver lslidar_cx_launch.py   # 机械式多线
ros2 launch lslidar_driver lslidar_ls_launch.py   # 1550 系列
ros2 launch lslidar_driver lslidar_x10_launch.py  # 单线
```

## 关键依赖

系统包：`libpcl-dev`、`libpcap-dev`、`libyaml-cpp-dev`、`libboost-thread-dev`

ROS 包：`pcl-conversions`、`rosidl-default-generators`、`builtin-interfaces`、`sensor-msgs`、`nav-msgs`、`tf2`、`geometry-msgs`

`turn_on_wheeltec_robot` 包额外依赖 `wheeltec_robot_msg` 和 `serial`（与底盘的串口通信）。

## 配置流程

`wheeltec_param.yaml` → launch 文件读取 → 参数注入各节点。其中的 `lidar_type`、`imu_mode`、`car_mode`、`camera_mode` 字段决定了启动哪些子模块。雷达的角度裁剪参数也在该文件的 `x10` 和 `lscx` 键下配置。
