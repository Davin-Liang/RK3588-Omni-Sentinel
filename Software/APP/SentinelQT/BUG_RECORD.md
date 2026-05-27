# BUG_RECORD — SentinelQT 问题记录

## 1. 预览线程关闭死锁

**现象**: 点击"关闭系统"后程序卡死，无法退出。

**原因**: `PreviewWorker::start()` 使用 `wait_get_preview()` 拉帧，内部调用 `ThreadSafeQueue::pop()` 无限阻塞。当相机停止产帧后，队列为空，`pop()` 永久挂起线程。`stop()` 设置 `running_ = false` 但线程卡在 `pop()` 内永远检查不到。

**解决**: SentinelVisioner 新增 `try_get_preview(camNum, timeoutMs)` 方法（内部使用 `try_pop` 带超时），`PreviewWorker` 改用 200ms 超时轮询。每次超时检查 `running_` 标志，确保最多 200ms 内响应退出信号。

---

## 2. previewWorker_ 内存泄漏

**现象**: `PreviewWorker` 对象在 `stop_preview_()` 中停掉线程后未释放，每次启停预览泄漏一个对象。

**原因**: `previewWorker_` 用 `new` 创建且无 QObject parent（移入子线程必须 orphan），但析构时只调了 `stop()` 未调 `delete`。`previewThread_` 有 parent 自动清理，`previewWorker_` 漏掉了。

**解决**: `stop_preview_()` 中 `QThread::wait()` 之后显式 `delete previewWorker_`。

---

## 3. 析构时 visioner_ 被预览线程 use-after-free

**现象**: 析构顺序不当，`delete visioner_` 时 PreviewWorker 子线程仍在 `wait_get_preview` 内部访问 visioner_。

**原因**: 析构函数先停预览再删 streamer/visioner，但预览线程只有 200ms 超时 + `wait(3000)` 等待。极端情况下线程未在 3 秒内退出，visioner 被提前释放。

**解决**: 析构顺序改为：
```
1. camera_stream_ctrl(false)    // 先停相机，停止产帧
2. stop_preview_()              // 停预览线程（队列无新帧，快速退出）
3. streamer_->remove_camera(0)  // 清理推流/录像
4. delete streamer_ / visioner_ // 最后释放
```

---

## 4. config.ini 找不到

**现象**: 修改 `config.ini` 内容不生效，程序始终用代码中的默认值。

**原因**: `QSettings("config.ini", QSettings::IniFormat)` 使用相对路径，查找位置取决于当前工作目录（`$PWD`），而非可执行文件所在目录。板端从不同路径启动时找不到文件。

**解决**: 改为 `QSettings(QCoreApplication::applicationDirPath() + "/config.ini", QSettings::IniFormat)`，始终从二进制所在目录读取。

---

## 5. 按钮 QMetaObject::connectSlotsByName 警告

**现象**: 启动时打印 `No matching signal for on_btn_start_stream_()` 等警告。

**原因**: `QMetaObject::connectSlotsByName` 是 Qt 自动按命名规则 `on_<widget>_<signal>()` 连接信号槽的机制。代码中已通过显式 `connect()` 连接，按名匹配找不到对应信号，产生无害警告。

**解决**: 无需处理。已在构造函数中显式 `connect()`，按名匹配失败不影响功能。

---

## 6. VIDIOC_S_PARM failed 警告

**现象**: 启动时打印 `VIDIOC_S_PARM failed: Inappropriate ioctl for device`。

**原因**: 摄像头驱动不支持修改帧率参数，`add_camera` 中尝试设置 30fps 失败。

**解决**: 无需处理。代码已打印 `(Ignore if not supported)`，不影响采集功能。

---

## 7. QStandardPaths runtime directory 警告

**现象**: 启动时打印 `QStandardPaths: runtime directory '/var/run' is not a directory, but a symbolic link`。

**原因**: Buildroot 的 `/var/run` 是符号链接指向 `/run`，Qt 内部运行时路径检查将其视为异常。

**解决**: 无需处理。不影响任何功能。

---

## 8. Failed to move cursor on screen DSI1

**现象**: 启动时打印 `Failed to move cursor on screen DSI1: -14`。

**原因**: DSI 触摸屏没有硬件光标，Qt eglfs 尝试移动光标失败。`-14` = EFAULT。

**解决**: 无需处理。触屏操作不受影响。

---

## 9. 双路预览颜色异常（皮肤呈蓝紫色"中毒"色）

**现象**: 两路相机的预览画面颜色异常，人的皮肤呈蓝紫色（中毒色），但推流画面颜色正常。

**原因**: RGA `rga_convert_to_rgb_full_` 目标格式为 `RK_FORMAT_RGB_888`，在 RK3588 little-endian 硬件上实际输出字节序为 [B][G][R]（低地址=Blue）。但 `QImage::Format_RGB888` 期望字节序 [R][G][B]（低地址=Red），导致 R↔B 通道交换。推流不受影响因为全程 NV12 无 RGB 转换。

**解决**: `rga_convert_to_rgb_full_` 目标格式改为 `RK_FORMAT_BGR_888`（sentinel-visioner），RGA 输出 BGR 字节序与 QImage 期望的 RGB 字节序匹配。

---

## 10. 删除按钮宽度/高度不合适

**现象**: 视频管理页面删除按钮文字"删除"显示不全（被截断），或按钮过高超出表格行。

**原因**: 按钮未设最小宽度，`padding: 2px 12px` 加上列宽不够。"操作"列 `resizeColumnsToContents` 可能缩得过窄。

**解决**: 按钮设置 `setMinimumWidth(60)`；"操作"列最小宽度 70px；padding 调整为 `3px 14px`。

---

## 11. 硬件监控缺百分号

**现象**: NPU 利用率显示为 `NPU 25/30/20` 无百分号。

**原因**: 格式字符串 `"NPU %1/%2/%3%"` 中 `%3%` 可能因 Qt `QString::arg()` 解析歧义，最后的 `%` 被吞掉。

**解决**: 格式字符串改为 `"NPU %1/%2/%3"`，后面拼接固定 `text += "%"`。RGA 同样修复。

---

## 12. 底部状态栏只能显示单路相机信息

**现象**: 底部状态栏文字只能显示一个事件（如 CAM0 推流中），CAM1 同时发生的状态变化会覆盖。

**原因**: `set_status_` 直接覆盖 `QLabel` 文本，无按相机分区存储。

**解决**: `set_status_` 自动检测消息前缀 "CAM0"/"CAM1"，按相机存储在 `camStatus_[0/1]` 数组中。`refresh_status_label_` 合并为 `"CAM0: xxx | CAM1: xxx"` 显示。全局消息（无前缀）仍直接显示。
