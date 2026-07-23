# BUG_RECORD — NVMe-SSD 问题记录

## 1. 作为子项目编译时 install 失败

**现象**: NVMe-SSD 作为 SentinelQT 子项目编译，`make install` 报错：
```
file INSTALL cannot find ".../nvme_demo": No such file or directory.
```

**原因**: CMakeLists.txt 中 `add_executable(nvme_demo EXCLUDE_FROM_ALL ...)` 跳过了 demo 编译，但 `install(TARGETS nvme_demo nvme_benchmark DESTINATION ./)` 仍尝试在 install 阶段查找二进制文件。`EXCLUDE_FROM_ALL` 只影响 `make all`，不影响 `install()` 指令。CMake 在 install 时找不到未编译的目标文件。

**解决**: 将 demo install 指令包裹在 standalone 构建判断中：
```cmake
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    install(TARGETS nvme_demo nvme_benchmark DESTINATION ./)
endif()
```
作为子项目时 `CMAKE_SOURCE_DIR != CMAKE_CURRENT_SOURCE_DIR`，install 指令被跳过。

---

## 2. export_trigger_video_clip 硬编码 RGB888 输入格式

**现象**: `export_trigger_video_clip()` 中 `expected_frame_size` 硬编码为 `W*H*3`（RGB888），且使用 swscale RGB→NV12 转换后送入 MPP 编码器。当上游存储的是 NV12 帧（如 RecordBufferPool 产出的 `W*H*3/2` 字节），尺寸检查失败，所有帧被丢弃，导出视频为空。

**原因**: 原始设计假设存储格式为 RGB888。但 sentinel-streamer 的 RecordBufferPool 全程 NV12，写入 NVMe 的也是 NV12 原始数据。RGB888 存储浪费 50% 空间，且多一次 RGA 转换。

**解决**: `export_trigger_video_clip()` 新增 `input_is_nv12` 参数。NV12 路径下 `expected_frame_size = W*H*3/2`，编码时直接 `memcpy` Y/UV plane 到 AVFrame（跳过 swscale），MPP 编码器原生消费 NV12。

---

## 3. 回溯导出视频时长为设定值的 2 倍

**现象**: 设置回溯 5 秒，导出 MP4 播放时长约 10 秒；设置 10 秒导出约 20 秒。时长精确翻倍。

**原因**: 编码器 PTS 使用固定步长 `pts_step = 90000 / fps = 90000 / 15 = 6000`，每帧递增 6000。实际摄像头帧率约 30fps，5 秒窗口采集 150 帧，按 15fps 逐帧编码 → 150/15 = 10 秒。PTS 与真实时间戳脱钩。

**解决**: 改为从 NVMe 帧真实时间戳计算 PTS：`pts = (timestamp_ns - base_ns) * 90000 / 1'000'000'000`。播放时长与采集窗口严格一致，自动适应任意实际帧率。

---

## 4. LiDAR 缓冲累积累积丢失帧间时间戳精度

**现象**: 回溯导出的雷达热力图所有点颜色相同（单色），缺少蓝→红的时间渐变。日志显示 `frameCount=1`，即使窗口内有数十帧雷达数据。

**原因**: `write_lidar_points_to_disk()` 使用 1MB 累积缓冲区，约 170 帧雷达数据（10Hz × 17s）拼成一个 NVMe 记录块并共享同一个 Header 时间戳。缓冲区内帧边界丢失，导出时无法恢复每帧的独立时间戳。

**解决**: 两步修复：
1. **帧内嵌帧头**：缓冲区中每帧前加 12 字节帧头 `[pointsCount:u32][timestampNs:u64]`。开销仅 0.2%，保留批量写盘的 I/O 效率。
2. **flush() 机制**：新增公开 `flush()` 方法，将缓冲区残留数据强制刷入 NVMe 写队列并等待落盘。`export_lidar_heatmap_png()` 在扫描前调用 `flush()`，确保最新数据已在磁盘上。

导出时逐帧头解析恢复每个点的真实时间戳，热力图实现正确的蓝→红时间渐变。

---

## 5. NvmeWorker 存 lidar 指针值拷贝导致运行时雷达启停不生效

**现象**: NVMe 初始化后启动雷达自动写入，但雷达数据从未写入 NVMe。回溯热力图始终输出 `no LiDAR points in window`。

**原因**: `NvmeWorker` 构造时接受 `SentinelLslidarer*` 并保存为**值拷贝**。Widget 在构造时调用 `init_nvme_()`，此时 `lidar_` 为 `nullptr`（雷达尚未启动）。后续 `init_nvme_()` 中自动启动雷达后 `lidar_` 已非空，但 NvmeWorker 内部仍存着构造时的 `nullptr` 副本，始终走 `if (lidar_ == nullptr) return` 跳过分支。

**解决**: NvmeWorker 改为存储 `SentinelLslidarer**`（指向 Widget 的 `lidar_` 成员）。每次轮询时解引用获取最新指针值。雷达启动/停止后自动感知，无需额外通知机制。

---

## 6. 写队列无限堆积导致 OOM

**现象**: 推流运行几分钟后 RSS 从 200MB 线性涨到 3GB，然后进程卡死或 OOM killed。关闭 NVMe 写入后不再复现。

**原因**: `data_queue_`（`std::queue<std::shared_ptr<DataBlock>>`）无容量上限。RK3588 板端 NVMe SSD 无主动散热，持续 45MB/s 写入几分钟后过热降速，writer 线程消费速度断崖下跌。生产者（NvmeWorker）继续以 15fps × 3MB/frame 入队，每帧 3MB 的 DataBlock 在内存中无限堆积。

**解决**: 三层防护：
1. **队列上限** `MAX_QUEUE_SIZE=16`：入队前检查，超限直接丢弃，内存最多 48MB。每 100 次丢弃打印计数
2. **flush() 超时**：等待队列排空从无限自旋改为 3 秒超时，防止 NVMe 降速时回溯导出永久阻塞主线程
3. **NvmeWorker 跳帧**：每 3 帧只写 1 次（45MB/s → 15MB/s），从源头降低 SSD 写压力

---

## 7. flush() 死锁主线程导致融合界面冻住

**现象**: 推流和融合运行一段时间后点击回溯，融合界面停止更新，回溯无响应，终端关不掉程序。

**原因**: `flush()` 中等待 `data_queue_` 排空的循环无超时。NVMe 降速后 writer 线程写不动，队列永远不空。回溯调用链（Web API → BlockingQueuedConnection → 主线程 → do_backtrack_ → export_lidar_heatmap_png → flush）把主线程永久阻塞。

**解决**: `flush()` 等待循环加 3 秒超时。超时后打印 `flush timeout` 日志，放弃等待继续导出（可能丢失最新几帧雷达数据，但不会死锁）。
