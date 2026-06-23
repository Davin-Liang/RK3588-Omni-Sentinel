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
