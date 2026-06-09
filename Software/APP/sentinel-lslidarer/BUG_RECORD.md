# BUG_RECORD — sentinel-lslidarer 问题记录

## 1. CLOCK_REALTIME 与 CLOCK_MONOTONIC 时间域不一致导致 get_closest_frame 返回无意义结果

**现象**: `get_closest_frame` 返回的帧与实际相机帧时间偏差巨大（可达数十年量级），融合算法中 LiDAR 点与图像内容完全错位。

**原因**: 调用方用 `CLOCK_REALTIME`（`clock_gettime(CLOCK_REALTIME, &ts)`）获取相机时间戳，但雷达 reader 线程使用 `CLOCK_MONOTONIC` 记录帧时间戳。`CLOCK_REALTIME` 从 Unix 纪元（1970-01-01）开始计数且受 NTP 时间调整影响，`CLOCK_MONOTONIC` 从系统启动开始计数且单调递增不受 NTP 影响。同一物理时刻两个时钟域的差值通常为数十年（约 1.5×10^18 ns），`get_closest_frame` 的线性扫描在这种量级偏差下完全失效。

**解决**: 
- API 文档和头文件注释明确标注 `cameraTsNs` 参数必须为 `CLOCK_MONOTONIC` 时间域
- 在 README 避坑章节首要位置说明时间戳一致性要求
- 融合线程（lidar-camera-fusion）统一切换到 `CLOCK_MONOTONIC` 获取相机时间戳

---

## 2. 调用方预分配缓冲区小于 max_points_per_frame() 导致点云静默截断

**现象**: 部分帧的尾部点云丢失，`frame.pointsCount` 显示为预分配的缓冲区大小（如 500），而非实际有效点数 540。融合结果在相机画面边缘（对应雷达后方区域）出现障碍物"消失"现象。

**原因**: 调用方手动硬编码分配 `new LidarPoint[500]`，但 `max_points_per_frame()` 返回 540（= N10Plus 理论最大点数）。`RingBuffer::copy_slot()` 为防止缓冲区溢出做了保护性截断：
```cpp
outFrame.pointsCount = std::min(count, maxPointsPerSlot_);
```
当预分配的 `outFrame.points` 数组小于 `maxPointsPerSlot_` 时，`memcpy` 只拷贝 `pointsCount` 个元素到缓冲区，尾部点云被静默截断。由于截断不发生越界或崩溃，问题极难排查。

**解决**:
- 调用方代码改为 `new LidarPoint[lidar.max_points_per_frame()]`，动态获取正确的缓冲区大小
- README 避坑章节增加缓冲区大小警告：**分配不足时自动截断，不会越界，但会丢失尾部点云数据**
- Demo 代码中使用 `max_points_per_frame()` 作为最佳实践示范

---

## 3. stop() 中线程 join 顺序错误导致永久阻塞

**现象**: 调用 `lidar.stop()` 后程序卡死，`readerThread_.join()` 永不返回，必须 `kill -9` 强杀。

**原因**: 最初版本 `stop()` 的实现顺序为：
```cpp
running_ = false;          // ① 通知退出
readerThread_.join();      // ② 等线程退出 — 永远等不到！
serialPort_->close();      // ③ 关串口 — 永远执行不到
```
reader 线程的主循环 `while(running_)` 在检查到 `false` 之前，正阻塞在 `::read(fd_, ...)` 调用中（内核态等待串口数据）。由于串口未关闭，`::read()` 永远不会返回，线程永远不会检查 `running_` 标志，`join()` 永久阻塞。

**解决**: 调整 `stop()` 中的操作顺序——**先关闭串口 fd 释放阻塞的 `::read()`，再 join 线程**：
```cpp
running_.store(false);          // ① 通知退出
serialPort_->close();           // ② 先关串口 → ::read() 返回错误 → 线程醒来
readerThread_.join();           // ③ 线程已退出循环，join 立即返回
```
原理：`close(fd)` 会使该 fd 上阻塞的 `::read()` 返回错误（`-1` 且 `errno` 通常为 `EBADF`），reader 线程从阻塞中醒来后检查 `running_` 为 `false`，正常退出循环。

---

## 4. 雷达启动后前 2-3 秒无帧数据

**现象**: `lidar.start()` 返回 true 后，应用层立即调用 `get_closest_frame()` 持续返回 false，约 2-3 秒后才有正常帧数据。

**原因**: 两层原因叠加：
1. **首圈丢弃**：`reader_loop_()` 中 `isFirstSweep=true` 时，跳过第一个不完整的半圈扫描（只从检测到圈边界后的点开始累积）。串口打开瞬间可能处于一帧的中间位置，数据不完整。
2. **电机加速**：雷达电机从上电到稳定 600 RPM（10Hz）需要约 2 秒。加速期间转速不稳定，角度数据可能不连续，但协议解码层面不会报错。

**解决**: 这是正常设计行为，无需修改代码。在 README 中明确说明 `start()` 返回后需等待 2-3 秒，并建议应用层在启动后自旋等待首帧：
```cpp
lidar.start();
// 等待首帧就绪
while (lidar.available_frames() == 0) {
    usleep(50000);  // 50ms
}
```

---

## 5. 串口路径硬编码与 udev 设备名漂移

**现象**: 雷达 USB 重新插拔后设备名从 `/dev/ttyACM0` 变为 `/dev/ttyACM1`，程序因找不到设备而启动失败。

**原因**: Linux 内核按 USB 枚举顺序分配 ttyACM 编号，重新插拔或系统重启后编号可能变化。默认配置中 `serialPort = "/dev/sentinel_lidar"` 依赖 udev 规则创建符号链接。

**解决**:
- 提供 `lidar_udev.sh` 脚本，通过雷达 USB 的 VID/PID 创建固定符号链接 `/dev/sentinel_lidar`
- 配置中默认路径使用符号链接名而非物理设备名
- README 中说明首次使用需执行 udev 脚本

---

## 6. CDC-ACM 虚拟串口波特率配置问题

**现象**: 对 ttyACM 设备执行 `tcsetattr` 后串口通信异常，无法正确读取数据包。

**原因**: ttyACM（CDC-ACM USB 虚拟串口）的波特率是虚拟值——数据通过 USB 总线传输，实际速率由 USB 协议控制。对 ACM 设备调用 `tcsetattr` 修改波特率可能破坏 USB CDC 协议栈的内部状态，导致数据错乱。

**解决**: 在 `SerialPort::open()` 中增加 ACM 设备检测逻辑，通过 `realpath` 解析设备路径判断是否为 ACM 设备。对 ACM 设备跳过 termios 波特率配置，仅使用内核默认的 USB CDC 参数。UART 物理串口仍正常配置 termios。实际测试确认数据读取正常后恢复对 ACM 设备的 termios 配置。

---

## 7. demo 角度屏蔽参数设为 0 导致全圈输出与预期不符

**现象**: 预期 demo 输出约 295-302 点（角度屏蔽 90°-240° 后），但实际输出约 530+ 点（全圈），与产品级使用场景不符。

**原因**: `demo.cpp` 中为演示全圈点云效果，显式设置了 `config.angleDisableMin = 0; config.angleDisableMax = 0;`，覆盖了 `LidarConfig` 默认的 90°-240° 屏蔽区间。这导致 demo 输出行为与 README 中标注的"角度屏蔽 90°-240°"描述不一致。

**解决**: demo 中保留全圈输出的设计意图（验证后方 20° 点的代码依赖全圈数据），但需在 DEMO-INSTRUCTIONS.md 中明确说明 demo 关闭了角度屏蔽，与默认产品配置不同。生产环境使用默认配置即可自动启用 90°-240° 屏蔽。
