# 雷达数据 NVMe 存储与 PNG 热力图回溯导出 — 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 回溯导出时，除 MP4 视频外额外导出一张雷达点云 PNG 热力图（时间→色相映射）。

**Architecture:** NvmeWorker 新增 SentinelLslidarer 引用以轮询雷达帧写入 NVMe（复用已有 write_lidar_points_to_disk）。NVMeDataManager 新增 export_lidar_heatmap_png() 扫描 NVMe 中 LIDAR 记录 → CPU 渲染 RGBA 像素缓冲 → lodepng 编码输出。Widget 层 do_backtrack_() 末尾追加一次 PNG 导出调用。

**Tech Stack:** C++14, lodepng (MIT), POSIX pread, 无新增外部依赖。

## Global Constraints

- C++14 标准，嵌入式 ARM64 Linux 目标
- 所有公共方法返回 bool 表示成功/失败
- 诊断输出 `fprintf(stderr, "[ComponentName] ...")` 格式
- 不使用 C++ 异常
- 预分配内存，避免运行时动态分配
- 交叉编译器: `aarch64-buildroot-linux-gnu`

---

### Task 1: 添加 lodepng 第三方库

**Files:**
- Create: `APP/3rdparty/lodepng/lodepng.h`
- Create: `APP/3rdparty/lodepng/lodepng.cpp`

**Produces:** `lodepng_encode32_file()` 函数，接受 RGBA 像素缓冲 + 宽高 → 写入 PNG 文件。

- [ ] **Step 1: 下载 lodepng.h**

从 https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.h 下载。

```bash
curl -L -o APP/3rdparty/lodepng/lodepng.h \
  https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.h
```

- [ ] **Step 2: 下载 lodepng.cpp**

```bash
curl -L -o APP/3rdparty/lodepng/lodepng.cpp \
  https://raw.githubusercontent.com/lvandeve/lodepng/master/lodepng.cpp
```

- [ ] **Step 3: 验证文件存在**

```bash
wc -l APP/3rdparty/lodepng/lodepng.h APP/3rdparty/lodepng/lodepng.cpp
```
预期: 两个文件非空，lodepng.cpp 约 6000+ 行。

- [ ] **Step 4: 提交**

```bash
git add APP/3rdparty/lodepng/
git commit -m "feat: 添加 lodepng 单文件 PNG 编码库 (MIT License)"
```

---

### Task 2: NVMe-SSD CMakeLists 添加 lodepng 编译

**Files:**
- Modify: `APP/NVMe-SSD/CMakeLists.txt`

**Consumes:** `APP/3rdparty/lodepng/lodepng.cpp` + `lodepng.h` (Task 1)

- [ ] **Step 1: 在 add_library 中添加 lodepng 源文件**

在 `APP/NVMe-SSD/CMakeLists.txt` 的 `add_library(nvme_data_manager_lib STATIC` 处添加 lodepng.cpp：

```cmake
add_library(nvme_data_manager_lib STATIC
    src/NVMeDataManager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/lodepng/lodepng.cpp
)
```

- [ ] **Step 2: 添加 lodepng 头文件路径**

在 `target_include_directories(nvme_data_manager_lib PUBLIC` 块内添加：

```cmake
target_include_directories(nvme_data_manager_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rdparty/lodepng
    ${FFMPEG_ROOT}/include
)
```

- [ ] **Step 3: 提交**

```bash
git add APP/NVMe-SSD/CMakeLists.txt
git commit -m "feat: NVMe-SSD CMake 引入 lodepng 编译"
```

---

### Task 3: NVMeDataManager 声明 export_lidar_heatmap_png()

**Files:**
- Modify: `APP/NVMe-SSD/include/NVMeDataManager.h`

**Consumes:** lodepng 就位 (Task 1)

**Produces:** `bool export_lidar_heatmap_png(...)` 公共方法声明。

- [ ] **Step 1: 在 public 区域添加方法声明**

在 `export_trigger_video_clip()` 声明之后、`get_queue_size()` 之前插入：

```cpp
// 导出雷达热力图 PNG（回溯窗口内所有 LiDAR 点按时间着色叠加）
bool export_lidar_heatmap_png(uint64_t trigger_timestamp_ns,
                               const std::string& output_path,
                               double time_window_sec = 5.0);
```

- [ ] **Step 2: 在 private 区域添加渲染辅助函数声明**

在 `writer_thread()` 声明前添加：

```cpp
// 热力图渲染辅助
struct LidarPointRecord {
    float x, y, intensity;
    uint64_t timestamp_ns;
};
void render_heatmap_pixels_(const std::vector<LidarPointRecord>& points,
                            int imgW, int imgH,
                            std::vector<uint8_t>& rgba);
```

- [ ] **Step 3: 提交**

```bash
git add APP/NVMe-SSD/include/NVMeDataManager.h
git commit -m "feat: NVMeDataManager 声明 export_lidar_heatmap_png 接口"
```

---

### Task 4: 实现 export_lidar_heatmap_png() 及热力图渲染

**Files:**
- Modify: `APP/NVMe-SSD/src/NVMeDataManager.cpp`

**Consumes:** NVMeDataManager.h 声明 (Task 3), lodepng 就位 (Task 1)

**Produces:** 完整的 PNG 热力图导出功能。

- [ ] **Step 1: 添加 lodepng include**

在 NVMeDataManager.cpp 顶部已有 include 区域添加：

```cpp
#include "lodepng.h"
#include <cmath>
#include <algorithm>
```

- [ ] **Step 2: 实现 export_lidar_heatmap_png()**

在文件末尾 `write_to_nvme()` 之后添加：

```cpp
bool NVMeDataManager::export_lidar_heatmap_png(uint64_t trigger_timestamp_ns,
                                                const std::string& output_path,
                                                double time_window_sec) {
    uint64_t window_ns = static_cast<uint64_t>(time_window_sec * 1'000'000'000.0);
    uint64_t start_ns = (trigger_timestamp_ns > window_ns)
                      ? (trigger_timestamp_ns - window_ns) : 0;
    uint64_t end_ns = trigger_timestamp_ns;

    // 1. 扫描 NVMe 收集 LiDAR 记录
    int read_fd = open(nvme_device_path_.c_str(), O_RDONLY);
    if (read_fd < 0) {
        fprintf(stderr, "[NVMeDataManager] heatmap: open failed: %s\n", strerror(errno));
        return false;
    }

    std::vector<LidarPointRecord> allPoints;
    off_t offset = 0;
    Header header;

    while (true) {
        ssize_t n = pread(read_fd, &header, sizeof(Header), offset);
        if (n != static_cast<ssize_t>(sizeof(Header))) break;
        if (header.magic_number != MAGIC_NUMBER) break;

        size_t record_payload = sizeof(Header) + header.data_size;
        size_t padding = (HEADER_ALIGNMENT - (record_payload % HEADER_ALIGNMENT)) % HEADER_ALIGNMENT;
        size_t record_size = record_payload + padding;

        if (header.data_type == static_cast<uint8_t>(DataType::LIDAR) &&
            header.timestamp_ns >= start_ns && header.timestamp_ns <= end_ns) {

            size_t pointCount = header.data_size / sizeof(LidarPoint);
            if (pointCount > 0 && pointCount <= 1200) {
                std::vector<uint8_t> buf(header.data_size);
                n = pread(read_fd, buf.data(), header.data_size, offset + sizeof(Header));
                if (n == static_cast<ssize_t>(header.data_size)) {
                    const LidarPoint* pts = reinterpret_cast<const LidarPoint*>(buf.data());
                    for (size_t i = 0; i < pointCount; ++i) {
                        LidarPointRecord r;
                        r.x = pts[i].x;
                        r.y = pts[i].y;
                        r.intensity = pts[i].intensity;
                        r.timestamp_ns = header.timestamp_ns;
                        allPoints.push_back(r);
                    }
                }
            }
        }

        offset += static_cast<off_t>(record_size);
    }

    close(read_fd);

    if (allPoints.empty()) {
        fprintf(stderr, "[NVMeDataManager] heatmap: no LiDAR points in window\n");
        return false;
    }

    // 2. 渲染 RGBA 像素缓冲
    const int kImgSize = 1200;
    std::vector<uint8_t> rgba(static_cast<size_t>(kImgSize) * kImgSize * 4, 0);
    render_heatmap_pixels_(allPoints, kImgSize, kImgSize, rgba);

    // 3. 编码 PNG 并写盘
    unsigned error = lodepng::encode(output_path, rgba, kImgSize, kImgSize);
    if (error) {
        fprintf(stderr, "[NVMeDataManager] heatmap: lodepng encode error: %s\n",
                lodepng_error_text(error));
        return false;
    }

    fprintf(stderr, "[NVMeDataManager] heatmap saved: %s (%zu points, %zu frames)\n",
            output_path.c_str(), allPoints.size(),
            allPoints.empty() ? 0 : 1);  // 简化：log 总点数
    return true;
}
```

- [ ] **Step 3: 实现 HSL→RGB 辅助函数**

在 `render_heatmap_pixels_()` 之前添加：

```cpp
namespace {

// hue: 0-360, sat: 0-1, light: 0-1  →  RGB 各分量 0-255
void hsl_to_rgb(float h, float s, float l,
                uint8_t& r, uint8_t& g, uint8_t& b) {
    auto hue2rgb = [](float p, float q, float t) -> float {
        if (t < 0.0f) t += 1.0f;
        if (t > 1.0f) t -= 1.0f;
        if (t < 1.0f/6.0f) return p + (q - p) * 6.0f * t;
        if (t < 1.0f/2.0f) return q;
        if (t < 2.0f/3.0f) return p + (q - p) * (2.0f/3.0f - t) * 6.0f;
        return p;
    };

    if (s == 0.0f) {
        auto v = static_cast<uint8_t>(l * 255.0f);
        r = g = b = v;
        return;
    }

    float q = (l < 0.5f) ? l * (1.0f + s) : l + s - l * s;
    float p = 2.0f * l - q;
    float hNorm = h / 360.0f;

    r = static_cast<uint8_t>(hue2rgb(p, q, hNorm + 1.0f/3.0f) * 255.0f);
    g = static_cast<uint8_t>(hue2rgb(p, q, hNorm) * 255.0f);
    b = static_cast<uint8_t>(hue2rgb(p, q, hNorm - 1.0f/3.0f) * 255.0f);
}

} // namespace
```

- [ ] **Step 4: 实现 render_heatmap_pixels_()**

```cpp
void NVMeDataManager::render_heatmap_pixels_(
        const std::vector<LidarPointRecord>& points,
        int imgW, int imgH,
        std::vector<uint8_t>& rgba) {

    if (points.empty()) return;

    // --- 计算渲染范围 ---
    float minX = points[0].x, maxX = points[0].x;
    float minY = points[0].y, maxY = points[0].y;
    uint64_t minTs = points[0].timestamp_ns;
    uint64_t maxTs = points[0].timestamp_ns;

    for (const auto& p : points) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
        if (p.timestamp_ns < minTs) minTs = p.timestamp_ns;
        if (p.timestamp_ns > maxTs) maxTs = p.timestamp_ns;
    }

    // 10% 边距 + 最小范围 ±10m
    float marginX = std::max((maxX - minX) * 0.1f, 0.5f);
    float marginY = std::max((maxY - minY) * 0.1f, 0.5f);
    float rangeMin = std::max(std::max(maxX - minX, maxY - minY) * 1.0f, 20.0f);
    (void)rangeMin;  // 保留用于最小范围约束

    float viewMinX = minX - marginX;
    float viewMaxX = maxX + marginX;
    float viewMinY = minY - marginY;
    float viewMaxY = maxY + marginY;

    // 确保正方形 + 至少 ±10m
    float cx = (viewMinX + viewMaxX) * 0.5f;
    float cy = (viewMinY + viewMaxY) * 0.5f;
    float half = std::max({viewMaxX - cx, viewMaxY - cy, 10.0f});
    viewMinX = cx - half;
    viewMaxX = cx + half;
    viewMinY = cy - half;
    viewMaxY = cy + half;

    float scaleX = imgW / (viewMaxX - viewMinX);
    float scaleY = imgH / (viewMaxY - viewMinY);
    float scale  = std::min(scaleX, scaleY);

    auto world_to_pixel = [&](float wx, float wy, int& px, int& py) {
        px = static_cast<int>((wx - cx) * scale + imgW / 2.0f);
        py = static_cast<int>((cy - wy) * scale + imgH / 2.0f);
    };

    // --- 黑色背景 ---
    std::fill(rgba.begin(), rgba.end(), 0);

    // --- 网格线 (10m 间距，深灰 #202020) ---
    {
        int gridStepM = 10;
        int mx0, my0, mx1, my1;
        world_to_pixel(viewMinX, 0.0f, mx0, my0);
        world_to_pixel(viewMaxX, 0.0f, mx1, my1);
        world_to_pixel(0.0f, viewMinY, mx0, my0);
        world_to_pixel(0.0f, viewMaxY, mx1, my1);

        for (int gm = static_cast<int>(std::floor(viewMinX / gridStepM)) * gridStepM;
             gm <= static_cast<int>(std::ceil(viewMaxX / gridStepM)) * gridStepM;
             gm += gridStepM) {
            int px, py;
            world_to_pixel(static_cast<float>(gm), 0.0f, px, py);
            if (px >= 0 && px < imgW) {
                for (int y = 0; y < imgH; y += 4) {  // 虚线
                    size_t idx = (static_cast<size_t>(y) * imgW + px) * 4;
                    rgba[idx] = 0x20; rgba[idx+1] = 0x20; rgba[idx+2] = 0x20; rgba[idx+3] = 255;
                }
            }
        }

        for (int gm = static_cast<int>(std::floor(viewMinY / gridStepM)) * gridStepM;
             gm <= static_cast<int>(std::ceil(viewMaxY / gridStepM)) * gridStepM;
             gm += gridStepM) {
            int px, py;
            world_to_pixel(0.0f, static_cast<float>(gm), px, py);
            if (py >= 0 && py < imgH) {
                for (int x = 0; x < imgW; x += 4) {  // 虚线
                    size_t idx = (static_cast<size_t>(py) * imgW + x) * 4;
                    rgba[idx] = 0x20; rgba[idx+1] = 0x20; rgba[idx+2] = 0x20; rgba[idx+3] = 255;
                }
            }
        }
    }

    // --- 传感器原点十字 (白色) ---
    {
        int ox, oy;
        world_to_pixel(0.0f, 0.0f, ox, oy);
        for (int dx = -10; dx <= 10; ++dx) {
            int px = ox + dx;
            if (px >= 0 && px < imgW && oy >= 0 && oy < imgH) {
                size_t idx = (static_cast<size_t>(oy) * imgW + px) * 4;
                rgba[idx] = rgba[idx+1] = rgba[idx+2] = 255; rgba[idx+3] = 255;
            }
        }
        for (int dy = -10; dy <= 10; ++dy) {
            int py = oy + dy;
            if (ox >= 0 && ox < imgW && py >= 0 && py < imgH) {
                size_t idx = (static_cast<size_t>(py) * imgW + ox) * 4;
                rgba[idx] = rgba[idx+1] = rgba[idx+2] = 255; rgba[idx+3] = 255;
            }
        }
    }

    // --- 渲染点：时间→色相 (240°→0°: 蓝→青→绿→黄→红)，alpha 叠加 ---
    float tsRange = (maxTs > minTs) ? static_cast<float>(maxTs - minTs) : 1.0f;
    int pointRadius = 2;

    for (const auto& pt : points) {
        float t = static_cast<float>(pt.timestamp_ns - minTs) / tsRange;
        float hue = 240.0f * (1.0f - t);  // 240°(蓝) → 0°(红)

        uint8_t cr, cg, cb;
        hsl_to_rgb(hue, 1.0f, 0.5f, cr, cg, cb);

        int cxPx, cyPx;
        world_to_pixel(pt.x, pt.y, cxPx, cyPx);

        for (int dy = -pointRadius; dy <= pointRadius; ++dy) {
            for (int dx = -pointRadius; dx <= pointRadius; ++dx) {
                if (dx*dx + dy*dy > pointRadius*pointRadius) continue;
                int px = cxPx + dx;
                int py = cyPx + dy;
                if (px < 0 || px >= imgW || py < 0 || py >= imgH) continue;
                size_t idx = (static_cast<size_t>(py) * imgW + px) * 4;
                // Alpha 叠加：新颜色与现有颜色混合
                uint8_t& er = rgba[idx];
                uint8_t& eg = rgba[idx+1];
                uint8_t& eb = rgba[idx+2];
                uint8_t& ea = rgba[idx+3];
                float alpha = 0.3f;  // 每点透明度，累积形成热点
                er = static_cast<uint8_t>(er * (1.0f - alpha) + cr * alpha);
                eg = static_cast<uint8_t>(eg * (1.0f - alpha) + cg * alpha);
                eb = static_cast<uint8_t>(eb * (1.0f - alpha) + cb * alpha);
                ea = 255;
            }
        }
    }
}
```

- [ ] **Step 5: 提交**

```bash
git add APP/NVMe-SSD/src/NVMeDataManager.cpp
git commit -m "feat: 实现 export_lidar_heatmap_png 热力图渲染与 PNG 导出"
```

---

### Task 5: NvmeWorker 新增雷达轮询

**Files:**
- Modify: `APP/SentinelQT/nvme_worker.h`
- Modify: `APP/SentinelQT/nvme_worker.cpp`

**Consumes:** SentinelLslidarer API (已有), NVMeDataManager::write_lidar_points_to_disk (已有 API)

- [ ] **Step 1: nvme_worker.h 添加 SentinelLslidarer 前向声明 + 新成员**

在 `class SentinelStreamer;` 下方添加前向声明，构造函数添加 lidar 参数，private 添加雷达相关成员：

```cpp
#ifndef NVME_WORKER_H
#define NVME_WORKER_H

#include <QObject>
#include <atomic>
#include <cstdint>

class SentinelStreamer;
class SentinelLslidarer;
struct LidarPoint;
class NVMeDataManager;

class NvmeWorker : public QObject
{
    Q_OBJECT

public:
    explicit NvmeWorker(SentinelStreamer* streamer,
                        NVMeDataManager* nvme,
                        SentinelLslidarer* lidar,
                        int numCameras,
                        QObject* parent = nullptr);

public slots:
    void start();
    void stop();

signals:
    void error(const QString& msg);

private:
    SentinelStreamer* streamer_;
    NVMeDataManager* nvme_;
    SentinelLslidarer* lidar_;
    int numCameras_;
    std::atomic<bool> running_{false};

    // 雷达轮询缓冲与去重 (1200 = kPointsPerSweep 理论最大值)
    LidarPoint lidarPointsBuf_[1200];
    uint64_t lastLidarTs_;
};

#endif // NVME_WORKER_H
```

- [ ] **Step 2: nvme_worker.cpp 更新构造函数**

```cpp
#include "nvme_worker.h"
#include "sentinel_streamer.h"
#include "sentinel_lslidarer.h"
#include "NVMeDataManager.h"

#include <QThread>
#include <cstdio>

NvmeWorker::NvmeWorker(SentinelStreamer* streamer,
                       NVMeDataManager* nvme,
                       SentinelLslidarer* lidar,
                       int numCameras,
                       QObject* parent)
    : QObject(parent)
    , streamer_(streamer)
    , nvme_(nvme)
    , lidar_(lidar)
    , numCameras_(numCameras)
    , lastLidarTs_(0)
{
}
```

- [ ] **Step 3: nvme_worker.cpp 在 start() 循环内追加雷达轮询**

在 `for (int cam = 0; cam < numCameras_; ++cam)` 循环后、`if (!gotAny)` 前插入：

```cpp
        // 雷达数据写入（与视频帧同步存储）
        if (lidar_ != nullptr) {
            LidarFrame frame;
            frame.points = lidarPointsBuf_;
            if (lidar_->get_latest_frame(frame)) {
                if (frame.timestampNs != lastLidarTs_) {
                    nvme_->write_lidar_points_to_disk(
                        reinterpret_cast<const uint8_t*>(frame.points),
                        frame.pointsCount * sizeof(LidarPoint),
                        frame.timestampNs);
                    lastLidarTs_ = frame.timestampNs;
                }
                gotAny = true;
            }
        }
```

注意: `LidarFrame` 和 `get_latest_frame` 来自 `sentinel_lslidarer.h`。

- [ ] **Step 4: 提交**

```bash
git add APP/SentinelQT/nvme_worker.h APP/SentinelQT/nvme_worker.cpp
git commit -m "feat: NvmeWorker 新增雷达轮询，同步写入 NVMe"
```

---

### Task 6: Widget 层传递 lidar 指针 + do_backtrack_ 导出 PNG

**Files:**
- Modify: `APP/SentinelQT/widget.cpp`

**Consumes:** NvmeWorker 新构造函数 (Task 5), export_lidar_heatmap_png (Task 4)

- [ ] **Step 1: init_nvme_() 传递 lidar_ 指针**

将 `widget.cpp:3508` 的 NvmeWorker 构造行从：

```cpp
nvme_worker_ = new NvmeWorker(streamer_, nvme_manager_, 2);
```

改为：

```cpp
nvme_worker_ = new NvmeWorker(streamer_, nvme_manager_, lidar_, 2);
```

> `lidar_` 是 Widget 的成员变量（widget.h:136），在融合启动时赋值，否则为 nullptr。

- [ ] **Step 2: do_backtrack_() 末尾追加 PNG 导出**

在 `do_backtrack_()` 的 for 循环（MP4 导出）之后、`return savedFiles;` 之前追加：

```cpp
// 导出雷达热力图 PNG
{
    std::string pngName = (QString("backtrack_%1_%2_lidar.png")
        .arg(label, tsStr)).toStdString();
    std::string pngPath = dir.absoluteFilePath(QString::fromStdString(pngName))
        .toStdString();

    if (nvme_manager_->export_lidar_heatmap_png(triggerNs, pngPath, backSecs)) {
        fprintf(stderr, "[SentinelQT] backtrack LiDAR heatmap saved: %s\n",
                pngPath.c_str());
        savedFiles.append(QString::fromStdString(pngName));
    }
}
```

- [ ] **Step 3: 提交**

```bash
git add APP/SentinelQT/widget.cpp
git commit -m "feat: do_backtrack_ 额外导出雷达 PNG 热力图"
```

---

### Task 7: 端到端验证（板端）

**前置条件:** 板端已部署，雷达运行中，相机推流/录像中（RecordBufferPool 有数据）。

- [ ] **Step 1: 确认 NVMe 中雷达数据在写入**

触发一次手动回溯后，可通过 NVMe demo 或 benchmark 验证 LIDAR 类型记录存在。如果已有 `nvme_benchmark`，可运行它扫描 NVMe 确认。

- [ ] **Step 2: 手动回溯验证 PNG 输出**

在 SentinelQT 回溯页面点击"手动回溯"，预期在 `/mnt/sdcard/backtrack/` 下同时出现 `.mp4` 和 `_lidar.png` 文件。

- [ ] **Step 3: 验证 PNG 内容**

将 PNG 复制到 PC 打开，验证：
- 黑色背景 + 灰色网格线 + 传感器十字
- 有色点云（蓝→红 渐变）
- 点分布与雷达实际扫描范围一致

- [ ] **Step 4: 验证雷达未启动时不影响视频导出**

停止融合（关闭雷达），再次手动回溯，预期：
- 视频正常导出
- 终端输出 `[NVMeDataManager] heatmap: no LiDAR points in window`
- 不崩溃，文件列表只含 `.mp4`

- [ ] **Step 5: 提交（如有微调）**

```bash
git add <修改的文件>
git commit -m "fix: 板端验证后的微调"
```
