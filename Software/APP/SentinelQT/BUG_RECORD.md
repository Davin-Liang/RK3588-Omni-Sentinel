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

---

## 13. Web 状态推送导致 QT 标题栏 CPU 利用率消失

**现象**: 添加 Web 远程控制的状态推送（`push_status()`）后，QT 标题栏 `hwLabel` 中的 CPU 利用率不再更新，始终显示 `CPU --`。

**原因**: `get_hw_json_()` 被声明为 `const`，但内部通过 `const_cast<Widget*>(this)` 修改了 `prevCpuTotal_` 和 `prevCpuIdle_` 成员变量。这两个变量也被 `update_hw_usage_()` 使用，CPU 利用率依赖两次采样间的差值计算。`push_status()` 在 `update_hw_usage_()` 末尾调用 → `get_status_json_()` → `get_hw_json_()`，`get_hw_json_()` 读取 `/proc/stat` 后立即重置了缓存值。下一次 `update_hw_usage_()` 的采样间隔变得极短（两个函数几乎同时采样），差值趋近于 0，导致 CPU 计算为 0 或负数。

**解决**: `get_hw_json_()` 改用独立的 `static` 局部变量（`webPrevTotal` / `webPrevIdle`）保存上一次 Web 查询的 CPU 采样值，不再通过 `const_cast` 修改 Widget 成员变量。Web 和 QT 各自独立计算 CPU，互不干扰。

---

## 14. Web 暂停导致 PreviewWorker 超时告警

**现象**: 网页界面点击暂停后，终端不断打印 `[PreviewWorker] no frame for 30 cycles`，而 QT 界面的暂停按钮不会触发此告警。

**原因**: `web_pause_()` 实现时参考了 `on_btn_pause_()` 的主体逻辑，但遗漏了 `stop_preview_()` 调用。QT 的暂停流程是：停止录像→停止推流→**停止预览**→暂停相机。Web 的暂停缺少"停止预览"步骤，导致 PreviewWorker 子线程继续调用 `try_get_preview()` 轮询帧。相机暂停后 RGA 管线停止产出新帧，PreviewWorker 连续 30 次（×200ms = 6 秒）拿不到帧后打印超时告警。

**解决**: `web_pause_()` 增加 `stop_preview_()` 调用，与 QT 暂停逻辑完全对齐：停止推流→停止录像→停止预览→暂停相机。同时 `web_resume_()` 增加 `start_preview_()` 调用恢复预览线程。

---

## 15. 暂停状态下点击推流/录像导致系统卡死

**现象**: 相机暂停后，在网页点击"开始推流"或"开始录像"，整个 SentinelQT 进程卡死，需 `killall -9 SentinelQT` 强制终止。

**原因**: `web_start_stream_()` 和 `web_start_record_()` 未检查相机暂停状态，直接调用 `streamer_->start_stream()`。当相机暂停时 RGA 管线已停止产出帧，streamer 的推流线程调用 `wait_get_orig_copy_buffer()` 无限阻塞等帧 → 死锁。`BlockingQueuedConnection` 同步等待推流结果也一并卡住。

**解决**: `web_start_stream_()` 和 `web_start_record_()` 在执行前检测 `cameraPaused_[camNum]`，若为 true 则先调用 `visioner_->camera_pause(false)` 恢复 RGA 管线，再重启预览线程，最后执行推流/录像操作。

---

## 16. 数据回溯页面虚拟键盘无法弹出

**现象**: 切换到"数据回溯管理"页面（page 3），点击回溯秒数输入框，虚拟键盘不出现。融合参数页面的虚拟键盘正常。

**原因**: `keyboardContainer` 是 `pageFusion`（page 2）的子控件，当 QStackedWidget 切到 page 3 时 pageFusion 被隐藏，虚拟键盘不可见。此外 `eventFilter` 只检查 `fusionParamEdits_` 中的 QLineEdit，不包含回溯页的输入框。

**解决**: 将 `keyboardContainer` 从 `pageFusion` 内部移到 QStackedWidget 同级的 `wrapperLayout` 根层级，所有页面共享。同时 `eventFilter` 新增 `le == backtrackSecsEdit_` 条件识别回溯页输入框。

---

## 17. Web↔Qt 双向同步覆盖冲突

**现象**: Web 界面修改回溯秒数后，不到 1 秒即被 Qt 的周期性状态推送覆盖为旧值，两者相互覆盖无法稳定输入。

**原因**: `get_status_json_` 每 1 秒推送完整状态到 Web，覆盖用户在 Web 界面正在进行但尚未提交的修改。

**解决**: 为回溯秒数/摄像头选择引入 `btrDirty` 脏标志：Web 用户修改输入时置脏，阻止 Qt 推送覆盖；点击"回溯"提交后清脏标志恢复同步。同理为系统控制的录像分辨率（`sysDirty`）和融合参数（`fusionCfgDirty`）添加脏标志。

---

## 18. QDoubleSpinBox 无法安装虚拟键盘 eventFilter

**现象**: 回溯秒数使用 `QDoubleSpinBox`，调用 `lineEdit()->installEventFilter()` 编译报错 `protected`，改用 `findChild<QLineEdit*>()` 返回 nullptr。

**原因**: Qt5 中 `QAbstractSpinBox::lineEdit()` 是 protected 方法。`findChild<QLineEdit*>()` 在构造阶段返回 nullptr，因为 spinbox 内部 QLineEdit 是延迟创建的（首次 show 时才实例化）。

**解决**: 放弃 QDoubleSpinBox，改用与融合参数一致的 QLineEdit + QDoubleValidator 方案，构造时即可直接 `installEventFilter(this)`。

---

## 19. 虚拟键盘弹出后 FocusOut 立即隐藏

**现象**: 点击回溯秒数输入框，虚拟键盘一闪即消失，无法输入。

**原因**: eventFilter 新增 FocusOut 处理，输入框失焦时调用 `hide_keyboard()`。但键盘按钮自身获取焦点时也会触发输入框的 FocusOut，导致键盘弹出即被隐藏。

**解决**: FocusOut 中改用 `QTimer::singleShot(50ms)` 延迟判断，检查 `QApplication::focusWidget()` 是否在虚拟键盘上。若焦点在键盘按钮上则保留键盘，否则隐藏。

---

## 20. Web 暂停/恢复后 Qt 暂停按钮不更新

**现象**: Web 界面点击"暂停"，Web 按钮变为"恢复"，但 Qt 界面暂停按钮仍显示"暂停"。

**原因**: `update_camera_button_states_()` 仅更新推流/录像按钮的状态和样式，未处理暂停按钮。Web 的 REST handler `web_pause_`/`web_resume_` 调用此函数后暂停按钮文本和样式不变。

**解决**: 在 `update_camera_button_states_()` 中增加暂停按钮状态更新：根据 `cameraPaused_[camNum]` 设置按钮文本为"暂停"或"恢复"及对应样式。

---

## 21. MediaMTX HLS m3u8 文件 401 Unauthorized

**现象**: 板端 `wget http://127.0.0.1:8888/live/cam0/stream.m3u8` 返回 401，但内置播放器页面可访问。网页端无法播放 HLS 流。

**原因**: MediaMTX `authInternalUsers` 配置中 `any` 用户的 `read`/`playback` action 的 `path` 被误设为字面字符串 `all`，空 path 才表示任意路径。`all` 被当作路径名匹配，导致 m3u8 文件路径认证失败。

**解决**: 将 `path: all` 改为 `path:`（空值表示任意路径）。同时最终方案放弃 HLS，改用 MediaMTX WebRTC iframe 嵌入（端口 8889），零认证问题、零 CDN 依赖、延迟 <1s。

---

## 22. ffmpeg HLS 输出方案退化

**现象**: 修改 `ffmpeg_stream_open` 同时输出 RTSP + HLS，增加代码复杂度，且与已有 MediaMTX 功能重叠。

**原因**: 未意识到项目已部署 MediaMTX（COTS 流媒体服务器，自带 RTSP/HLS/WebRTC 多协议输出）。另起 ffmpeg HLS 输出属于重复造轮子。

**解决**: 回退 ffmpeg_stream_open 修改，保持单一 RTSP 推送职责。Web 前端改用 iframe 嵌入 MediaMTX 内置 WebRTC 播放器（`http://<ip>:8889/live/cam{i}/`），删除 hls.js CDN 依赖和 HLS 代理代码。

---

## 23. Web 融合第二次启动 segfault

**现象**: 通过 Web API 停止融合后再启动，`fusion_->start()` 后立即 segmentation fault。

**原因**: `web_fusion_start_()` 缺少 `SentinelYoloInfer` 创建逻辑。第一次停止时 `on_btn_fusion_toggle_()` 的禁用路径已将 `yoloInfer_` delete 并置 nullptr，但 `fusion_->stop()` 未清除内部的 `DetectionProvider`（std::function 仍持有已释放的 yoloInfer_ 指针）。第二次 Web 启动时 `fusion_->start()` 发起融合线程，线程调用过期 callback → 访问野指针 → segfault。

**解决**: 在 `web_fusion_start_()` 中添加与 `on_btn_fusion_toggle_()` 启用路径一致的 yoloInfer_ 懒创建 + provider 绑定逻辑。同时将融合 disable 路径的 yoloInfer_ 销毁改为检查 `osdEnabled_` 状态。

## 24. Web OSD 按钮点击返回"网络错误"

**现象**: 网页点击 OSD 按钮显示"网络错误"，桌面端 Qt OSD 按钮正常工作。

**原因**: cpp-httplib 对 HTTP POST 请求采用显式路由注册机制（`impl_->httpServer_.Post(path, handler)`），仅在 `web_server.cpp` 注册的路由才能被匹配。Widget 中的 `handle_web_command()` 路由表对 WebSocket 有效，但 HTTP 请求未注册 `/api/v1/cam/{0,1}/osd/start|stop` 四条路由，返回 404。

## 25. EIS 按钮网页点击返回"网络错误"

**现象**: 网页点击 EIS 按钮显示"网络错误"，桌面端 Qt EIS 按钮正常工作。

**原因**: 与 #24 OSD 按钮同因。cpp-httplib 对 HTTP POST 请求采用显式路由注册，未在 `web_server.cpp` 中注册 `/api/v1/cam/{0,1}/eis/start|stop` 四条 POST 路由。

**解决**: 在 `web_server.cpp` 中注册四条 EIS POST 路由。

---

## 26. EIS 偏移始终为 0

**现象**: EIS 初始化成功、IMU 数据正常读取（samples=4），但 `offset` 始终打印 (+0,+0)，晃动相机无变化。

**原因**: EIS 回调 `eis_offset_callback_()` 放在 `capture_thread_func_` 的 `if (targetNpuBuf != nullptr)` 块内。当 NPU 推理未启动时，`npuRgbPool` 的 8 个 buffer 全部推入 `npuTaskQueue` 后 `get_buffer()` 返回 null，整个 NPU 处理块被跳过，EIS 回调永不执行。

**解决**: 将 EIS 偏移计算提升到 `if (targetNpuBuf != nullptr)` 块外部，每帧独立于 NPU buffer 执行。偏移值仍仅在有 NPU buffer 时传入 `rga_process_to_rgb_()`。

---

## 27. 分辨率下拉框下拉箭头不显示

**现象**: `resCombo`/`resCombo1` 分辨率选择下拉框不显示下拉箭头，点击后能弹出菜单但箭头图标不可见。

**原因**: widget.ui 中 QComboBox 的 stylesheet 包含 `QComboBox::drop-down { border: none; width: 16px; }`。Qt 样式系统规则：一旦手动指定了 QComboBox 子控件的任意属性，就必须完整定义该子控件（`subcontrol-position`、`subcontrol-origin`、箭头图片等），否则 Qt 不会用默认渲染补全。`border: none` 清除了默认边框，但没有提供替代的箭头渲染，导致整个 drop-down 区域不可见。

**解决**: 删除 `QComboBox::drop-down` 自定义样式，让 Qt 使用默认下拉箭头渲染。下拉列表（`QAbstractItemView`）的样式保留，不影响功能。

---

## 28. QLabel tooltip 在嵌入式触屏上无法触发

**现象**: 融合图例帮助标签使用 `QLabel::setToolTip()` 设置悬停提示，在嵌入式触摸屏上点按无任何反应，提示不弹出。

**原因**: QToolTip 依赖鼠标 hover 事件触发（`QHelpEvent` → `QToolTip::showText()`）。嵌入式 Linux 触屏设备没有鼠标指针，触屏产生的是 `QTouchEvent`/`QMouseEvent(synthesized)`，不会产生 hover 状态。QLabel 的 tooltip 机制在纯触屏环境下完全失效。

**解决**: 改用 QPushButton + `clicked` 信号 + `QMessageBox::information()` 弹出说明。按钮在触屏上点击可靠触发，不依赖 hover。同时将按钮叠放在 TopDownView 右下角（子控件 + eventFilter 监听 Resize 事件动态定位），避免占用额外布局空间。

---

## 29. NVMe 集成时 RecordBufferPool 生命周期依赖

**现象**: 析构 SentinelQT 时偶发 crash，堆栈指向 `try_get_record_frame()` 内部访问已释放的 DMA buffer。

**原因**: `NvmeWorker` 持有 `SentinelStreamer*` 裸指针并调用 `try_get_record_frame()`，该函数访问 streamer 内部的 `RecordBufferPool`。析构顺序中 `streamer_->remove_camera()` 会销毁 `RecordBufferPool`。若 NvmeWorker 在 `remove_camera()` 之后才停止，worker 线程可能访问已释放的池内存。

**解决**: 析构顺序严格保证 `deinit_nvme_()`（stop worker + join thread + delete）在 `streamer_->remove_camera()` 之前执行。NvmeWorker::stop() 设置 `running_ = false`，线程在下一轮循环检查标志后退出，`QThread::wait(3000)` 确保线程完全结束。
