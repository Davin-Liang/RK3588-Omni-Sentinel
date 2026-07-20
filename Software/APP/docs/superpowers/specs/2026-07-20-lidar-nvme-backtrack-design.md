# 雷达数据 NVMe 存储与回溯导出 — 设计文档

**日期**: 2026-07-20  
**状态**: 设计完成，待评审  
**关联组件**: NVMe-SSD, SentinelQT (NvmeWorker), sentinel-lslidarer  

## 1. 需求概述

回溯导出时，除了现有的 MP4 视频，同时导出雷达数据的**静态 PNG 热力图**。

- 写入侧：NvmeWorker 将相机帧写入 NVMe 的同时，顺带写入雷达帧
- 导出侧：`do_backtrack_()` 除了调用 `export_trigger_video_clip()` 导出 MP4，额外调用 `export_lidar_heatmap_png()` 导出一张 PNG

用户场景：给客户/演示看 — 视频展示相机画面，PNG 热力图展示回溯窗口中雷达"看到了什么"。

## 2. 架构变更

改动集中在 3 个组件，不涉及 sentinel-lslidarer、lidar-camera-fusion、sentinel-streamer、web-control：

```
NvmeWorker (修改)
  └─ 新增 SentinelLslidarer* 引用
  └─ 轮询循环内：去重取最新帧 → write_lidar_points_to_disk()

NVMeDataManager (修改)
  └─ 新增 export_lidar_heatmap_png()

3rdparty/lodepng/ (新增)
  └─ 单文件 PNG 编码库，无外部依赖
```

## 3. 写入路径

### 3.1 NvmeWorker 修改

轮询循环内在视频帧之后，追加雷达写入：

```cpp
// 预分配缓冲区（构造时一次分配，复用）
LidarPoint lidarPointsBuf_[1200];
uint64_t lastLidarTs_ = 0;

// start() 循环内：
if (lidar_ != nullptr) {
    LidarFrame frame;
    frame.points = lidarPointsBuf_;
    if (lidar_->get_latest_frame(frame)) {
        if (frame.timestampNs != lastLidarTs_) {
            nvme_->write_lidar_points_to_disk(
                reinterpret_cast<uint8_t*>(frame.points),
                frame.pointsCount * sizeof(LidarPoint),
                frame.timestampNs);
            lastLidarTs_ = frame.timestampNs;
        }
    }
}
```

### 3.2 关键设计决策

| 决策 | 选择 | 理由 |
|------|------|------|
| 雷达指针可空 | `lidar_ == nullptr` 时跳过 | 融合未启动时雷达不可用，不影响 NVMe |
| 时间戳去重 | 比较 `lastLidarTs_` | 雷达 10Hz，NVMe 轮询 15-30fps，避免重复写入 |
| 复用已有 API | `write_lidar_points_to_disk()` | NVMeDataManager 已完整实现，零改动 |
| RingBuffer 并发读 | 安全 | SWCR 语义，`copy_slot()` 只读共享内存，写入方自己缓冲区 |
| 缓冲预分配 | `lidarPointsBuf_[1200]` 成员变量 | 避免每帧 14KB 堆分配 |

## 4. 导出路径

### 4.1 PNG 热力图渲染

`NVMeDataManager` 新增方法：

```
bool export_lidar_heatmap_png(
    uint64_t triggerTimestampNs,
    const std::string& outputPath,
    double timeWindowSec);
```

渲染流程：

1. **扫描 NVMe**: `pread()` 线性扫描，匹配 `Header.magic == 0xDEADBEEF` 且 `data_type == LIDAR`，筛选时间窗口 `[trigger - window, trigger]` 内的所有点记录
2. **计算渲染范围**: 遍历所有点得 `min/max X, Y`，外侧留 10% 边距；点数过少时最小范围 ±10m
3. **渲染 RGBA 像素缓冲**:
   - 黑色背景
   - 深灰虚线距离网格（10m 间距）+ 刻度数字
   - 每个 LidarPoint 映射为 2px 半径的圆
   - 时间→颜色: `(ts - minTs) / (maxTs - minTs)` 映射到 HSL 色相 240°→0°（蓝→青→绿→黄→红）
   - Alpha 叠加模式：重复像素叠加增亮，自然形成热点
   - 传感器原点标记（白色十字）
4. **图例**: 底部色条 + "早期" / "最新" 标签
5. **编码**: `lodepng_encode32_file()` 写盘

### 4.2 渲染参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| 图片分辨率 | 1200×1200 | 正方形象限 |
| 边距 | 10% | 自动范围外留白 |
| 最小范围 | ±10m | 点数稀少时的下限 |
| 网格间距 | 10m | 虚线 + 标注 |
| 点半径 | 2px | 小点累积形成热点 |
| 色相映射 | 240°→0° | 蓝→红 渐变 |
| 背景色 | #000000 | 纯黑 |

### 4.3 输出示例

```
文件: backtrack_alert_t3_20260720_143052_lidar.png
内容: 5.0s 窗口，~50 帧雷达 × ~500 点/帧 = ~25000 点叠加
```

### 4.4 lodepng

- 来源: https://github.com/lvandeve/lodepng (MIT License)
- 文件: `3rdparty/lodepng/lodepng.h` + `3rdparty/lodepng/lodepng.cpp`
- 用途: 将 RGBA 像素缓冲编码为 PNG 文件
- 替代 Qt/OpenCV/libpng：无额外依赖，适合 NVMe-SSD 纯 C++ 静态库

## 5. 导出编排

`widget.cpp` `do_backtrack_()` 末尾追加：

```cpp
// 现有：导出 MP4（不变）
for (int cam = camStart; cam <= camEnd; ++cam) {
    nvme_manager_->export_trigger_video_clip(...);
    files << fileName;
}

// 新增：导出雷达 PNG 热力图
if (nvme_manager_) {
    std::string pngPath = backtrackDir + "/backtrack_" + label
                        + "_" + tsStr + "_lidar.png";
    if (nvme_manager_->export_lidar_heatmap_png(triggerNs, pngPath, backSecs)) {
        files << QString::fromStdString(pngPath);
    }
}
```

**自动获得雷达导出的触发路径**:
- 手动回溯（Qt UI / Web API）
- 融合告警自动回溯

**不需要改动的**:
- Web API 文件列表 — 自动包含 `.png`
- WebSocket 推送 — 自动包含 PNG 路径
- `config.ini` — 不需要新开关
- Web 前端 index.html — 文件列表自然展示 PNG

## 6. 实现清单

| # | 文件 | 改动 | 行数估计 |
|---|------|------|----------|
| 1 | `3rdparty/lodepng/lodepng.h` | 新增 | ~2000 (外部库) |
| 2 | `3rdparty/lodepng/lodepng.cpp` | 新增 | ~6000 (外部库) |
| 3 | `NVMe-SSD/CMakeLists.txt` | 添加 lodepng 编译 | +3 |
| 4 | `NVMe-SSD/include/NVMeDataManager.h` | 声明 `export_lidar_heatmap_png()` | +10 |
| 5 | `NVMe-SSD/src/NVMeDataManager.cpp` | 实现热力图渲染 | +200 |
| 6 | `SentinelQT/nvme_worker.h` | 新增 `SentinelLslidarer*` + 缓冲区 | +5 |
| 7 | `SentinelQT/nvme_worker.cpp` | 雷达轮询 + 去重 | +25 |
| 8 | `SentinelQT/widget.cpp` | `do_backtrack_()` 追加 PNG 调用 | +10 |
| 9 | `SentinelQT/widget.h` | `init_nvme_` 传递 lidar 指针 | +2 |
| 10 | `SentinelQT/widget.cpp` | `init_nvme_` 创建 NvmeWorker 时传入 lidar | +2 |

非外部库的纯业务代码约 **250 行**。

## 7. 风险与边界

- **NVMe 空间**: 每帧雷达约 14KB（1200×12B），10Hz = 140KB/s，远小于视频（1080p NV12 = ~3MB/帧，15fps = 45MB/s），不构成瓶颈
- **雷达未启动**: `lidar_ == nullptr`，跳过写入；导出时无雷达记录 → 静默跳过，不影响视频导出
- **lodepng 线程安全**: 渲染在 `do_backtrack_()` 线程内同步执行（已为视频导出使用独立 `std::thread`），不引入新并发问题
- **向后兼容**: `export_lidar_heatmap_png()` 是新增 API，不影响现有 `export_trigger_video_clip()`
