# LidarCameraFusion — 技术实现文档 (v2)

## 1. 概述

`LidarCameraFusion` 负责将 N10Plus 单线激光雷达点云与 YOLO 2D 检测框进行空间对齐（融合），并对融合后的目标进行多帧跟踪。

**v2 架构（当前）**: 融合阶段外参变换+内参投影将 LiDAR 点映射到相机图像平面。跟踪阶段对全点云做 **DBSCAN 2D 空间聚类** → **时间证据累积（帧间持久化确认）** → **评分制 bbox 认领** → Alpha-Beta 滤波 → **评分制贪心最近邻关联** → **5 状态生命周期管理**。

YOLO 检测结果通过 `DetectionProvider` 回调（`std::function`）从外部注入，融合模块不直接依赖 NPU 推理组件。未设置回调时回退到内部假检测（测试用）。

v2 相比 v1（CDC 角度链聚类 + 4 状态 + bboxIdx 门控）的关键升级见 `FUSION_PIPELINE.md`。

---

## 2. 数据流

```
YOLO bboxes                    LidarFrame (540 pts max)
     │                                │
     └──────────┬─────────────────────┘
                ▼
         fuse_data()
         ├── transform_point_()  ← 4x4 齐次外参（6 乘法 + 3 加法，z=0 优化）
         ├── project_point_()    ← pinhole 内参 (u=fx*cx/cz+cx, v=cy)
         ├── bbox 判定           ← first-hit 策略
         └── 计数排序写回        ← candidatePointBuf 展平存储
                │
                ▼
         FusionResult
         ├── bboxPointIndices[]  ← 点索引，按 bbox 分段
         ├── bboxPointCounts[]   ← 每个 bbox 的点数量
         └── bboxCount
                │
                ▼
         LidarTargetTracker::update()
         ├── cluster_bbox_points_()    ← 扫描顺序 CDC，bbox 内点
         ├── cluster_orphan_points_()  ← atan2 排序 CDC，未认领点
         ├── predict_tracks_()         ← Alpha-Beta 预测
         ├── associate_()              ← classId 门控 + 贪心 NN
         ├── manage_lifecycle_()       ← 状态机
         └── check_warnings_()         ← 迟滞 + 冷却
```

---

## 3. 融合算法（`fuse_data()`）

### 3.1 外参变换

针对 2D 单线雷达（z=0）优化，4×4 齐次变换降为 6 次乘法 + 3 次加法：

```
cx = T[0]*lx + T[1]*ly + T[3]
cy = T[4]*lx + T[5]*ly + T[7]    （单线雷达 cy≡0）
cz = T[8]*lx + T[9]*ly + T[11]
```

### 3.2 内参投影

标准 pinhole 模型：
```
u = fx * cx / cz + cx_principal
v = fy * cy / cz + cy_principal   （单线雷达 v≡cy_principal）
```

过滤条件：`cz <= 0`（相机后方）、`u ∉ [0, imgWidth)`、`v ∉ [0, imgHeight)`

### 3.3 bbox 分类（first-hit 策略）

对每个未分类的 LiDAR 点，遍历所有 bbox，首个命中即归属。已分类的点在后续 `fuse_data()` 调用中被跳过（`pointToBbox[i] >= 0`），实现多相机累积时每个点入至多一个框。

### 3.4 计数排序写回

Pass 1 统计每个 global bbox 的点数量（`bboxPointCountsBuf[gb]`）。Pass 2 计算偏移量，将点索引按组写入 `candidatePointBuf`。结果按 bbox 分段连续存储，`FusionResult` 只暴露只读指针。

---

## 4. 跟踪算法（`LidarTargetTracker`）

### 4.1 聚类

**扫描顺序 CDC（bbox 内点）**：`fuse_data()` 按扫描索引递增顺序遍历点云（`i=0..nPoints-1`），写入 `candidatePointBuf` 的点索引天然保持扫描顺序。利用此特性，**不需要额外排序**，直接线性扫描做 CDC 聚类。相邻点欧氏距离 < `clusterEpsMeters` → 归为同一簇，否则切分新簇。扫描完成后做 wrap-around 检查（第一个簇的起点与最后一个簇的终点距离），处理 360° 扫描的角度回绕。

**atan2 排序 CDC（孤儿点）**：未被 bbox 认领的 LiDAR 点按 `atan2(y,x)` 排序后做相同 CDC 聚类。这些点来自雷达视野内但不在任何 bbox 区域内的物体。

**簇评分**：
```
score = pointCount * 1.0 + distanceBonus * 1.0
distanceBonus = 1.0 - clamp(distMeters / 50.0, 0.0, 1.0)
```
点数为主，距离仅作弱 tie-breaker。附加规则：最高分簇点数 < 最大簇的 60% → 优先选最大簇。

**类别和置信度过滤**：融合线程仅保留 `classId=0` (person) 且 `confidence >= 0.75` 的检测框，非 person 和低置信度框在 `fusion_thread_()` 入口处过滤。

### 4.2 Alpha-Beta 滤波器

| 步骤 | 公式 | 说明 |
|------|------|------|
| 预测 | `x_pred = x + vx*dt` | 恒速模型 |
| 校正 | `x = x_pred + α * residual` | 位置更新 |
| 速度 | `vx = vx_pred + (β/dt) * residual` | 仅 `consecutiveHits >= minHitsForVelocity` 时更新 |

**dt 保护**：
- `dt <= minDtSec` → `dt = defaultDtSec`（回退值）
- `dt > maxDtSec` → `dt = 0`（跳过预测）
- 正常雷达帧率 ~10Hz → `dt ≈ 0.1s`

**速度初始化**：首帧 `vel=0`，连续命中 ≥2 帧后才开始更新速度。

**dt 过大场景**：匹配航迹用测量值重置位置+速度归零；未匹配航迹按 `maxStaleCoastingFrames` 加速删除。

### 4.3 数据关联

贪心最近邻，两步门控：

1. **classId 门控**：`requireClassIdMatch=true` 且 `track.classId != det.classId` → 跳过
2. **孤儿检测门控**：`det.isOrphan=true` → 只能匹配 coasting 的 Confirmed 航迹，且距离 < 1.0m（比正常门限更紧）
3. **距离门控**：欧氏距离 < `maxAssociationDistMeters`（默认 2.0m）

未匹配检测→创建 Tentative 航迹（孤儿检测除外）。未匹配航迹→递增 `consecutiveMisses`。

### 4.4 生命周期状态机

```
                     consecutiveHits >= minHitsToConfirm
Tentative ────────────────────────────────────────────→ Confirmed
    │                                                        │
    │ consecutiveMisses > maxTentativeMisses                  │ consecutiveMisses > 0
    │                                                        │
    ▼                                                        ▼
Deleted ←────────────────────────────────────────────── Coasting
             consecutiveMisses >= maxCoastingFrames
```

**Tentative**：容忍丢失 `maxTentativeMisses` 帧（默认 1）。**Confirmed**：丢失 1 帧即进入 Coasting。**Coasting**：持续预测外推，丢失 `maxCoastingFrames` 帧后删除。dt 过大时按 `maxStaleCoastingFrames` 加速删除。

**槽位管理**：预分配 `tracks_[50]` 数组，新建航迹时线性扫描 Deleted 槽位。

### 4.5 告警

**迟滞**：`warningEnterDistMeters`（进入）< `warningExitDistMeters`（退出），防止边界抖动。**冷却**：目标持续在危险区时每隔 `warningCooldownNs` 触发一次。**过滤**：仅 Confirmed、age ≥ `minConfirmedAgeForWarning` 的目标触发。

---

## 5. 内存模型

所有缓冲区在 `LidarTargetTracker` 构造时预分配（栈数组 + `new(nothrow)`），运行时零堆分配。

| 组件 | 典型大小 |
|------|---------|
| 双缓冲 tracks (working + snapshot) | ~8.4 KB |
| 预测状态数组 (x/y/vx/vy × 50) | ~0.8 KB |
| 检测缓存 (200 × 32B) | ~6.4 KB |
| 聚类缓冲区 (x/y/索引/分配 × 540) | ~8.6 KB |
| 关联缓冲区 | ~1.2 KB |
| **合计** | **~25 KB** |

---

## 6. 线程安全

双缓冲策略：`workingTracks_`（`update()` 无锁写入）→ mutex 拷贝→ `snapshotTracks_`（`copy_snapshot()` 加锁读取）。锁持有时间仅 memcpy 操作，竞争极小。

`DetectionProvider` 回调在融合线程中调用（`fusion_thread_()`），由调用方（SentinelQT）保证 `SentinelYoloInfer::try_get_fusion_result()` 的线程安全性（内部 `ThreadSafeQueue` 已加锁）。
