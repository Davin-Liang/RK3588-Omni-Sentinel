# LiDAR 点云推流 OSD 叠加 — 设计文档

日期：2026-06-22

## 目标

在 sentinel-streamer 的 RTSP 推流画面中，绘制落在 YOLO 检测框内的 LiDAR 点云投影点，并标注目标距离。与现有 bbox OSD 独立开关。

## 数据流

```
LidarCameraFusion::fusion_thread_()
    │ fuse_data() 产出 FusionResult（含投影 UV）
    ▼
[新增] latestOsdSnapshot_ : std::mutex 保护的单份快照
    │
    ▼
LidarCameraFusion::try_get_lidar_osd_snapshot()
    │
    ▼
SentinelQT lambda bridge
    │ 转换 LidarOsdSnapshot → StreamLidarOsdBBox
    ▼
SentinelStreamer::set_lidar_osd_provider(lambda)
    │
    ▼
stream_thread_func_() 每帧调用 provider(5ms timeout)
    │
    ▼
draw_lidar_points_() 在 720p NV12 Y 平面画点 + 距离标签
    │
    ▼
MPP H.264 编码 → RTSP / MP4
```

## 改动清单

### 1. lidar-camera-fusion（融合库）

**1.1 FusionResult 新增字段** (`include/lidar_camera_fusion.h`)

```cpp
struct FusionResult {
    // ... 现有字段不变 ...
    const float*    bboxPointU;      // 投影像素 U 坐标（原始图像空间）
    const float*    bboxPointV;      // 投影像素 V 坐标（原始图像空间）
};
```

**1.2 新增预分配缓冲区** (`include/lidar_camera_fusion.h` 私有成员)

```cpp
float* bboxPointUBuf_;    // kMaxLidarPoints 个 float，扁平分段输出
float* bboxPointVBuf_;    // kMaxLidarPoints 个 float，扁平分段输出
float* tempU_;            // kMaxLidarPoints 个 float，按 LiDAR 点索引暂存投影 U
float* tempV_;            // kMaxLidarPoints 个 float，按 LiDAR 点索引暂存投影 V
float* lidarPointXBuf_;   // kMaxLidarPoints 个 float，LiDAR 点 X 暂存
float* lidarPointYBuf_;   // kMaxLidarPoints 个 float，LiDAR 点 Y 暂存
```

构造函数 `new (std::nothrow)` 分配，析构函数 `delete[]` 释放。

**1.3 fuse_data() 修改** (`src/lidar_camera_fusion.cpp`)

在 Pass 1（投影循环）中把每个点的 `(u, v)` 暂存到临时数组 `tempU_[i]` / `tempV_[i]`。
在 Pass 2（counting-sort 写回）中，与 `candidatePointBuf[writePos] = pointIndex` 同步写入：
```cpp
bboxPointUBuf[writePos] = tempU_[pointIndex];
bboxPointVBuf[writePos] = tempV_[pointIndex];
```

**1.4 新增 LidarOsdSnapshot 和同步机制** (`include/lidar_camera_fusion.h`，需新增 `#include <mutex>`)

```cpp
struct PerCameraLidarOsd {
    int      camNum;
    uint32_t imgWidth;
    uint32_t imgHeight;
    // 以下数组均为深拷贝，与 FusionResult 扁平分段布局一致
    // bboxPointU/V 对每个 LiDAR 点存一个投影坐标
    // bboxPointCounts[b] = 第 b 个 bbox 的点数
    std::vector<float>    bboxPointU;
    std::vector<float>    bboxPointV;
    std::vector<uint32_t> bboxPointCounts;
    uint32_t              bboxCount;
    // 检测框矩形（NPU 640x640 空间，用于距离标签定位）
    std::vector<uint32_t> bboxX1;
    std::vector<uint32_t> bboxY1;
    std::vector<uint32_t> bboxX2;
    std::vector<uint32_t> bboxY2;
    // 原始 LiDAR 点坐标（用于距离计算）
    std::vector<float>    lidarPointX;
    std::vector<float>    lidarPointY;
    std::vector<uint32_t> bboxPointIndices;
};

struct LidarOsdSnapshot {
    uint64_t timestampNs;
    PerCameraLidarOsd cameras[2];
    uint32_t camCount;
};
```

不使用队列，使用 `std::mutex` 保护的单一最新快照 `latestOsdSnapshot_`（fusion 线程写入，consumer 线程读取）。`LidarOsdSnapshot` 默认 `camCount=0` 表示无数据。

**1.5 快照构建** (`src/lidar_camera_fusion_thread.cpp`)

fusion 线程每轮迭代，在 `fuse_data()` 和 tracker update 之后，构建 `LidarOsdSnapshot`：

```cpp
// 注意: fuse_data() 累积调用（先 cam0 再 cam1），所有数据合并存储在扁平分段数组中。
// snapshot 构建需要按相机拆分。

// 1. 拷贝 LiDAR 点坐标到临时缓冲（所有相机共用同一帧 LiDAR 数据）
for (uint32_t i = 0; i < frame.pointsCount; ++i) {
    lidarPointXBuf_[i] = lidarPointsBuf_[i].x;
    lidarPointYBuf_[i] = lidarPointsBuf_[i].y;
}

// 2. 构建每相机快照（深拷贝到 vector）
LidarOsdSnapshot snap;
snap.timestampNs = frame.timestampNs;
snap.camCount = camCount_;

uint32_t globalBboxIdx = 0;  // 全局 bbox 索引（跨相机）
for (uint32_t c = 0; c < camCount_; ++c) {
    auto& cam = snap.cameras[c];
    cam.camNum = c;
    cam.imgWidth  = camConfigs_[c].imgWidth;
    cam.imgHeight = camConfigs_[c].imgHeight;

    uint32_t camBboxCount = fakeDetections_[c].size();
    cam.bboxCount = camBboxCount;
    if (camBboxCount == 0) continue;

    // 拷贝 bbox 坐标（NPU 640x640 空间）
    cam.bboxX1.resize(camBboxCount);
    cam.bboxY1.resize(camBboxCount);
    cam.bboxX2.resize(camBboxCount);
    cam.bboxY2.resize(camBboxCount);
    for (uint32_t b = 0; b < camBboxCount; ++b) {
        cam.bboxX1[b] = fakeDetections_[c][b].x1;
        cam.bboxY1[b] = fakeDetections_[c][b].y1;
        cam.bboxX2[b] = fakeDetections_[c][b].x2;
        cam.bboxY2[b] = fakeDetections_[c][b].y2;
    }

    // 拷贝 bboxPointCounts（每框点数）
    cam.bboxPointCounts.assign(&bboxPointCountsBuf[globalBboxIdx],
                                &bboxPointCountsBuf[globalBboxIdx + camBboxCount]);

    // 计算该相机在扁平数组中的起止偏移
    uint32_t pointStart = bboxOffsets[globalBboxIdx];
    uint32_t pointEnd   = (globalBboxIdx + camBboxCount < result_.bboxCount)
                            ? bboxOffsets[globalBboxIdx + camBboxCount]
                            : totalCandidateCount;
    uint32_t camPointCount = pointEnd - pointStart;

    // 拷贝投影点坐标和点索引（扁平分段布局的子区间）
    cam.bboxPointU.assign(&bboxPointUBuf_[pointStart],
                           &bboxPointUBuf_[pointStart + camPointCount]);
    cam.bboxPointV.assign(&bboxPointVBuf_[pointStart],
                           &bboxPointVBuf_[pointStart + camPointCount]);
    cam.bboxPointIndices.assign(&candidatePointBuf[pointStart],
                                 &candidatePointBuf[pointStart + camPointCount]);

    // 拷贝 LiDAR 点坐标（所有相机的点查找都使用同一份 LiDAR 帧数据）
    cam.lidarPointX.assign(lidarPointXBuf_, lidarPointXBuf_ + frame.pointsCount);
    cam.lidarPointY.assign(lidarPointYBuf_, lidarPointYBuf_ + frame.pointsCount);

    globalBboxIdx += camBboxCount;
}

// 3. 写入 mutex 保护的快照
{
    std::lock_guard<std::mutex> lock(osdSnapshotMutex_);
    latestOsdSnapshot_ = std::move(snap);
}
```

**1.6 新增公共 API 和私有成员**

```cpp
// 公共 API：拉取最新 LiDAR OSD 快照（非阻塞，线程安全）
bool try_get_lidar_osd_snapshot(LidarOsdSnapshot& out, int timeoutMs);

// 私有成员：
std::mutex          osdSnapshotMutex_;
LidarOsdSnapshot    latestOsdSnapshot_;   // camCount=0 表示无数据
```

实现：
```cpp
bool LidarCameraFusion::try_get_lidar_osd_snapshot(LidarOsdSnapshot& out, int timeoutMs) {
    (void)timeoutMs;
    std::lock_guard<std::mutex> lock(osdSnapshotMutex_);
    if (latestOsdSnapshot_.camCount == 0) return false;
    out = latestOsdSnapshot_;
    return true;
}
```

### 2. sentinel-streamer（推流组件）

**2.1 新增数据类型** (`include/sentinel_streamer.h`)

```cpp
enum class StreamLidarOsdMode {
    WITHOUT_LIDAR_OSD = 0,
    WITH_LIDAR_OSD    = 1,
};

struct StreamLidarOsdBBox {
    uint32_t x1, y1, x2, y2;       // NPU 640x640 空间（用于定位距离标签）
    float    distanceMeters;        // 平均距离
    std::vector<float> pointsU;     // pointCount 个 U 坐标（原始图像空间）
    std::vector<float> pointsV;     // pointCount 个 V 坐标（原始图像空间）
    uint32_t pointCount;            // = pointsU.size()
};

using StreamLidarOsdProvider = std::function<bool(
    int camNum,
    std::vector<StreamLidarOsdBBox>& out,
    int timeoutMs
)>;
```

**2.2 新增公共 API** (`include/sentinel_streamer.h`)

```cpp
void set_lidar_osd_provider(StreamLidarOsdProvider provider);
bool set_stream_lidar_osd_mode(int camNum, StreamLidarOsdMode mode);
```

**2.3 StreamerContext 新增字段** (`src/sentinel_streamer.cpp`)

```cpp
StreamLidarOsdMode     lidarOsdMode;
StreamLidarOsdProvider lidarOsdProvider;
```

**2.4 新增绘制函数** (`src/sentinel_streamer.cpp`)

```cpp
static void draw_lidar_points_(
    void* virtAddr,                  // 720p NV12 Y 平面
    int streamWidth, int streamHeight, // 1280, 720
    int srcWidth, int srcHeight,     // 原始帧分辨率（如 1920x1080）
    const std::vector<StreamLidarOsdBBox>& boxes
);
```

绘制逻辑：
- 每点 2×2 像素方块，遍历 `box.pointsU[j]` / `box.pointsV[j]`（vector，自有内存）
- 距离 < 5m → Y=240（亮白），5-15m → Y=180（中灰），> 15m → Y=120（暗灰）
- **点坐标变换**：`u_720p = u * 1280 / srcWidth`，`v_720p = v * 720 / srcHeight`（投影点在原始图像空间，简单等比缩放）
- **距离标签定位**：`(box.x1, box.y1)` 在 NPU 640×640 空间，复用 `draw_osd_boxes_()` 的 letterbox 逆变换（先逆 letterbox 到原始图像，再缩放到 720p）
- 距离标签 "3.2m" 复用 3×5 字体 2x 放大，白字暗底
- 裁剪到 [0, 1279] × [0, 719]

**2.5 线程函数修改** (`src/sentinel_streamer.cpp` — `stream_thread_func_()`)

在 bbox OSD 绘制之后、MPP 编码之前插入：

```cpp
if (ctx->lidarOsdMode == StreamLidarOsdMode::WITH_LIDAR_OSD && ctx->lidarOsdProvider) {
    std::vector<StreamLidarOsdBBox> lidarBoxes;
    if (ctx->lidarOsdProvider(ctx->camNum, lidarBoxes, 5)) {
        draw_lidar_points_(scaleBuf->virtAddr, 1280, 720, srcW, srcH, lidarBoxes);
    }
}
```

### 3. SentinelQT（触控界面桥接）

**3.1 LidarCameraFusion API 扩展**

`LidarCameraFusion` 新增：
```cpp
bool try_get_lidar_osd_snapshot(LidarOsdSnapshot& out, int timeoutMs);
```

fusion 线程每轮迭代后构建 `LidarOsdSnapshot`，通过 `osdSnapshotMutex_` 写入 `latestOsdSnapshot_`。

**3.2 Lambda provider 桥接**

在 `widget.cpp` 中新增方法 `setup_lidar_osd_provider_()`：

```cpp
void Widget::setup_lidar_osd_provider_() {
    streamer_->set_lidar_osd_provider(
        [this](int camNum, std::vector<StreamLidarOsdBBox>& out, int timeoutMs) {
            LidarOsdSnapshot snap;
            if (!fusion_->try_get_lidar_osd_snapshot(snap, timeoutMs))
                return false;
            for (uint32_t c = 0; c < snap.camCount; ++c) {
                if (snap.cameras[c].camNum != camNum) continue;
                auto& cam = snap.cameras[c];
                // 遍历每个 bbox，打包 StreamLidarOsdBBox
                uint32_t offset = 0;
                for (uint32_t b = 0; b < cam.bboxCount; ++b) {
                    StreamLidarOsdBBox box;
                    box.x1 = cam.bboxX1[b];
                    box.y1 = cam.bboxY1[b];
                    box.x2 = cam.bboxX2[b];
                    box.y2 = cam.bboxY2[b];
                    box.pointCount = cam.bboxPointCounts[b];
                    // 拷贝投影点坐标到 box（独立内存，不受 snap 析构影响）
                    box.pointsU.assign(cam.bboxPointU.begin() + offset,
                                       cam.bboxPointU.begin() + offset + box.pointCount);
                    box.pointsV.assign(cam.bboxPointV.begin() + offset,
                                       cam.bboxPointV.begin() + offset + box.pointCount);
                    // 计算平均距离
                    float sumDist = 0;
                    for (uint32_t j = 0; j < box.pointCount; ++j) {
                        uint32_t pi = cam.bboxPointIndices[offset + j];
                        sumDist += std::sqrt(cam.lidarPointX[pi] * cam.lidarPointX[pi] +
                                              cam.lidarPointY[pi] * cam.lidarPointY[pi]);
                    }
                    box.distanceMeters = sumDist / box.pointCount;
                    offset += box.pointCount;
                    out.push_back(box);
                }
            }
            return !out.empty();
        });
}
```

**3.3 LiDAR OSD 开关**

- UI 按钮：在相机控制栏新增 "LiDAR" 按钮
- Web API：新增 `POST /api/v1/cam/{0,1}/lidar-osd/start|stop`
- WebSocket status：新增 `lidarOsdEnabled` 字段

**3.4 融合启用时自动 setup**

`on_btn_fusion_toggle_()` 和 `web_fusion_start_()` 中，融合启动后调用 `setup_lidar_osd_provider_()`。

### 4. config.ini

无需新增配置节 —— LiDAR OSD 是运行时开关，不持久化。

## 线程安全

- **fusion 线程 ↔ streamer worker 线程**：`LidarOsdSnapshot` 使用 `std::vector` 深拷贝所有数据。`osdSnapshotMutex_` 保护 `latestOsdSnapshot_` 的读写。fusion 线程在锁内 move-assign，streamer 侧（通过 bridge lambda → `try_get_lidar_osd_snapshot()`）在锁内 copy。move-assign 后 snapshot 的 vectors 被移空，fusion 下轮迭代重建新的 snapshot 写入
- **bridge lambda → streamer**：`StreamLidarOsdBBox` 用 `std::vector<float>` 持有点坐标数据，从 `LidarOsdSnapshot` 深拷贝，lambda 返回后数据独立存在
- **draw_lidar_points_()**：在 streamer 的 worker 线程中执行，和同一帧的 `draw_osd_boxes_()` 同线程，无竞争
- **fusion_ 指针生命周期**：由 SentinelQT 主线程管理，仅在融合启动后设置 provider、融合停止前清空 provider，遵循现有 `yoloInfer_` / `streamer_` 的模式

## 独立开关行为矩阵

| Bbox OSD | LiDAR OSD | 画面效果 |
|----------|-----------|----------|
| OFF | OFF | 纯 720p 画面 |
| ON | OFF | 白色检测框 + "person 0.85" |
| OFF | ON | LiDAR 点簇 + 距离标签 "3.2m" |
| ON | ON | 白色框 + 标签 + 框内 LiDAR 点 + 独立距离标签 |

## 验证方法

1. 编译：`cd APP/lidar-camera-fusion && ./build.sh`，`cd APP/sentinel-streamer && ./build.sh`，`cd APP/SentinelQT && ./build.sh`
2. 板端运行 SentinelQT，启动激光雷达和融合
3. 启动单路推流，开启 LiDAR OSD（不开启 bbox OSD），确认画面出现 LiDAR 点簇和距离标签
4. 同时开启 bbox OSD 和 LiDAR OSD，确认白色框 + 点簇 + 距离标签并存
5. 关闭 LiDAR OSD，确认点簇消失但 bbox 仍在
6. 检查 720p RTSP 流画质，确认无花屏或帧率下降
7. 检查 Web 远程控制面板 LiDAR OSD 开关双向同步
