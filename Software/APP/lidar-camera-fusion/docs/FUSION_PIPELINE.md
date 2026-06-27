# 融合管线详细说明 (v2 优化版)

> RK3588 Omni-Sentinel  
> 单线机械式LiDAR(镭神N10Plus, 10Hz) + 双路OV2710相机(30fps)  
> YOLOv8 NPU推理 + DBSCAN全局聚类 + Alpha-Beta多目标跟踪  
> 5状态生命周期 + 评分制关联 + Web聚类可视化  
> C++14, 零ROS依赖, 线程安全双缓冲快照

---

## 一、整体架构

融合由独立线程 `fusion_thread_()` 驱动, 10ms周期循环:

```
步骤1  取YOLO检测(每路相机独立)
步骤2  检测是否hasYolo → 拿雷达帧(有YOLO时间对齐, 无YOLO取最新)
步骤3  fuse_data() 投影LiDAR点到相机像素(用于OSD显示)
步骤4  tracker_->update() DBSCAN聚类 + 预测 + 评分关联 + 5状态生命周期
步骤5  构建OSD快照(复用tracker聚类质心 + 聚类可视化数据)
步骤6  sleep(10ms)
```

**v2 核心优化**:
- CDC角度链聚类 → DBSCAN 2D空间聚类
- bbox认领从"质心必须在框内" → 评分制(框内外均可)
- 关联从"bboxIdx门控+classId门控" → 纯空间距离评分
- 4状态(Confirmed/Coasting) → 5状态(视觉雷达融合/纯雷达/丢失中)
- 新增Web端聚类圆可视化

---

## 二、YOLO检测层

每路相机独立拉取: `detectionProvider_(camNum, out, 33ms)`

**过滤**: `classId==0(person)` AND `confidence>=0.60`

**输出**: `fakeDetections_[cam] = vector<YoloBBox>`

```
YoloBBox { x1, y1, x2, y2, classId, confidence, timestampNs }
坐标系: NPU 640×640 RGB888 推理小图
```

**无YOLO时**: `fakeDetections_`为空 → `hasYolo=false` → 后续走纯雷达模式(全点为孤儿聚类)

---

## 三、LiDAR帧获取

**有YOLO**: `lidar_->get_closest_frame(yolo_timestamp_ns, frame)`  
在环形缓冲中找时间戳最接近的LiDAR帧, 实现时间对齐

**无YOLO**: `lidar_->get_latest_frame(frame)`  
直接取环形缓冲队尾的最新帧, 不依赖相机时间戳

**LiDAR帧结构**:
```
LidarFrame {
    LidarPoint points[540];  // 预分配
    uint32_t   pointsCount;  // 有效点数
    uint64_t   timestampNs;  // CLOCK_MONOTONIC 纳秒
}
LidarPoint { float x, y, intensity; }  // 2D笛卡尔坐标
```

---

## 四、fuse_data() 投影 (OSD用, 不影响跟踪)

每个LiDAR点 → 外参变换(T_LiDAR→Camera) → 针孔投影(u,v) → 检查是否落入YOLO bbox → 就近分配bbox中心 → 写入缓冲区

**输出缓冲区**(供OSD快照使用, tracker不依赖):
- `bboxPointCountsBuf[]` — 每个bbox内的点数
- `candidatePointBuf[]` — bbox内点的LiDAR索引
- `bboxPointUBuf_[] / bboxPointVBuf_[]` — 投影像素坐标

仅在 `hasYolo=true` 时执行。

---

## 五、Tracker核心: DBSCAN聚类 + 多目标跟踪

### 5.1 DBSCAN聚类 (dbscan_cluster_)

**输入**: 全部LiDAR点(540个), YOLO bbox列表, 相机外参+内参

**流程**:

a) **收集有效点**: 过滤零点 `(0,0)`, 过滤 `>maxPointDistanceMeters(30m)` 远点

b) **DBSCAN 2D空间聚类**:
- 邻域半径 = `dbscanEpsMeters(0.5m)`, 最小点数 = `dbscanMinPoints(5)`
- 对每个未访问点: 数邻居 → 点数不足则标记为噪声 → 点数足够则BFS扩张
- 噪声点不参与后续匹配
- 最多32簇, 每簇记录 `count, sumX, sumY, centroid(cx,cy)`

c) **簇过滤**:
- `count < dbscanMinPoints` → 丢弃
- 质心距离 `> maxClusterDistanceMeters(10m)` → 丢弃

**对比旧CDC方案**:
| 方面 | 旧CDC | 新DBSCAN |
|------|-------|----------|
| 聚类空间 | 按角度排序后1D扫描 | 直接2D笛卡尔空间 |
| 近距目标 | 角度间隙导致碎裂 | 2D邻域搜索不碎裂 |
| Wrap-around | 需要手动首尾合并 | 2D空间自然处理 |
| 复杂度 | O(n log n) (排序) | O(n²) (540点可忽略) |

**关键参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `dbscanEpsMeters` | 0.5 | DBSCAN邻域半径(米) |
| `dbscanMinPoints` | 5 | 核心点最小邻居数 |
| `maxPointDistanceMeters` | 30.0 | 收集点的最远距离 |
| `maxClusterDistanceMeters` | 10.0 | 簇质心最远距离 |
| `clusterPersistenceFrames` | 2 | 簇连续出现多少帧后才输出detection |

---

### 5.1b 时间证据累积层 (v2新增)

DBSCAN 输出的是单帧"瞬时几何结构"，同一目标在不同帧可能因密度变化而产生 cluster split/merge。直接把单帧 cluster 当 detection 会导致 tracking 输入不稳定。

解决方案：在 DBSCAN 和 detection 之间加帧间确认层。

```
每帧:
  dbscan_cluster_() → raw clusters (最多32个)
      ↓
  每个当前 cluster 质心匹配 clusterHistory_ 中最近的记录:
    距离 < dbscanEpsMeters → 连续命中, consecutiveFrames++, 更新位置
    距离 ≥ dbscanEpsMeters → 新建记录, consecutiveFrames = 1
    history 中未匹配的记录 → 标记老化, 超过 2 帧未命中则清除
      ↓
  consecutiveFrames >= clusterPersistenceFrames(2) → 输出为正式 detection
  consecutiveFrames <  2                             → 暂存, 不输出
```

**效果**:
- 瞬时噪声 cluster（单帧闪烁）被过滤
- cluster split/merge 不产生瞬时伪 detection
- 新参数 `clusterPersistenceFrames`（uint32_t, 默认 2, 热修改）

**内部开销**: ClusterRecord history[32], 质心距离匹配 O(32²) ≈ 可忽略。

---

### 5.2 簇→bbox认领 (评分制, 质心不强制在框内)

每个有效簇, 投影质心到每个相机, 对每个bbox计算认领得分:

```
pixelDist = 投影点在框内 ? 0 : 投影点到最近框边的欧氏距离(像素)
proximityFactor = max(0, 1 - pixelDist / bboxClaimMaxPixelDist)
score = 1.0 / (1 + centroidDistance) * proximityFactor
```

- `proximityFactor ≤ 0` 时该bbox不认领此簇(投影点太远)
- 每个bbox选择得分最高的簇 → 产出 **bbox检测** (`isOrphan=false`)
- 一簇最多被一个bbox认领

**设计意图**: 当人在图像边沿时, LiDAR质心可能投影到框外(特别是框不完全覆盖人体时)。旧方案要求质心必须在框内 → 边沿目标丢失。新方案对"框外但邻近"的簇仍给予评分 → 边沿目标也能认领到簇。

**未被任何bbox选中的簇** → **孤儿检测** (`isOrphan=true, bboxIdx=0xFFFFFFFF`)

**每个bbox最多一个检测**。每帧总检测数 = bbox detection数 + orphan detection数。

**关键参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `bboxClaimMaxPixelDist` | 100.0 | 投影点离bbox边缘的最大像素距离 |

---

### 5.3 Alpha-Beta预测 (predict_tracks_)

对所有非Deleted航迹:

```
dt = 当前帧时间 - 上次更新时间 (秒)
predX = posX + velX × dt
predY = posY + velY × dt
```

**参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `alpha` | 0.45 | 位置平滑(0~1, 越大越信观测) |
| `beta` | 0.20 | 速度估计(校正时使用) |
| `minHitsForVelocity` | 2 | 累积命中帧数后才启用速度估计 |

---

### 5.4 评分制关联 (associate_, v2重写)

**核心变更**: 移除 bboxIdx 门控和 classId 门控, 所有track对所有检测平等竞争, 纯空间距离评分。

#### bbox检测 → track

- 硬门限: `bboxAssocMaxDistMeters(1.5m)` — 超出此距离的 (track, detection) 对**直接跳过**

- 评分公式:
  ```
  score = 1.0 / (1 + distance²)
  ```

- 贪心策略: 每个检测找得分最高的未匹配track
- 评分维度: 距离越近 → 得分越高。pointCount 不参与关联评分（避免远处密集簇抢走近处稀疏 track）

#### 孤儿检测 → track

- 硬门限: `orphanAssocMaxDistMeters(1.0m)` — 超出直接跳过

- 仅匹配非Tentative、非Deleted的track (FusionTracking / PureRadarTracking / Lost)

- 评分公式同上: `score = 1.0 / (1 + distance²)`

- 孤儿检测**不创建新track** — 仅给已有航迹续命

#### 未匹配的bbox检测

→ 创建新Tentative航迹 (孤儿检测跳过此步骤)

**对比旧方案**:
| 方面 | 旧 | 新 |
|------|----|----|
| bboxIdx门控 | 跨框门限缩至25% | 无, track跨框自由竞争 |
| classId门控 | requireClassIdMatch | 移除 |
| 关联逻辑 | 最近邻(距离) | 评分制(纯距离, pointCount不参与) |
| 孤儿门限 | 0.5m, 仅Coasting | 1.0m, 排除Tentative/Deleted |

**关键参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `bboxAssocMaxDistMeters` | 1.5 | bbox检测关联硬门限(米) |
| `orphanAssocMaxDistMeters` | 1.0 | 孤儿检测关联硬门限(米) |

---

### 5.5 校正 (apply_correction_)

匹配成功:
```
miss = 0
hits++, age++

pos(X,Y) → Alpha-Beta滤波: pos = pred + alpha × (det - pred)
vel(X,Y) → Beta/dt滤波: vel = predVel + (beta/dt) × (det - predPred)
           (仅当 hits ≥ minHitsForVelocity 时更新速度)

更新检测元数据: bboxIdx, classId, confidence, pointCount, distanceMeters
```

**通过 bboxIdx 判断匹配类型**: 
- `bboxIdx != 0xFFFFFFFF` → 本次匹配来自 YOLO 认领簇
- `bboxIdx == 0xFFFFFFFF` → 本次匹配来自孤儿簇(纯雷达)

**未匹配**: `miss++, age++`

---

### 5.6 5状态生命周期 (manage_lifecycle_, v2重写)

**状态定义**:

| 状态 | 含义 | 颜色 |
|------|------|------|
| `Tentative` | 待确认, 新track试探期 | 蓝 `#4493f8` |
| `FusionTracking` | 视觉雷达融合跟踪, YOLO+LiDAR共同确认 | 绿 `#3fb950` |
| `PureRadarTracking` | 纯雷达跟踪, YOLO丢失但LiDAR续命中 | 黄 `#d29922` |
| `Lost` | 跟踪丢失中, 仅靠Alpha-Beta外推 | 红 `#da3633` |
| `Deleted` | 已删除, 槽位可回收 | 不绘制 |

**状态转换图**:

```
                         ┌──────────────────────────┐
                         │       Tentative          │
                         │    (新建, 未确认)          │
                         │       蓝 #4493f8          │
                         └──────┬──────┬────────────┘
                    miss>1(删)   │      hits≥3
                                 │      (确认)
                                 ▼
                         ┌──────────────────────────┐
            ┌────────────│    FusionTracking        │◄──────────┐
            │            │ (视觉雷达融合跟踪)          │           │
            │            │    绿 #3fb950             │           │
            │            └──────┬──────┬────────────┘           │
            │        YOLO丢失    │      no match                │
            │        +孤儿命中    │                              │
            │                   ▼                              │
            │            ┌──────────────────────────┐           │
            │    ┌───────│   PureRadarTracking      │───────┐   │
            │    │       │    (纯雷达跟踪)            │       │   │
            │    │       │    黄 #d29922             │       │   │
            │    │       └──────┬───────────────────┘       │   │
            │    │        no match                          │   │
            │    │              ▼                           │   │
            │    │       ┌──────────────────────────┐       │   │
            │    │       │         Lost             │       │   │
            │    └──────►│    (跟踪丢失中)            │◄──────┘   │
            │  孤儿命中   │    红 #da3633             │  孤儿命中  │
            │            └──────────┬───────────────┘           │
            │                 miss≥maxLostFrames                │
            │                       ▼                           │
            │               ┌──────────────────────────┐       │
            │               │        Deleted           │       │
            │               │  (已删除, 不再跟踪)        │       │
            │               └──────────────────────────┘       │
            │                                                   │
            └───────────────────────────────────────────────────┘
                     YOLO重新检测到 (bboxIdx ≠ 0xFFFFFFFF)
```

**转换规则**:

| 当前状态 | 条件 | 目标状态 |
|----------|------|----------|
| Tentative | hits ≥ minHitsToConfirm | FusionTracking |
| Tentative | misses > maxTentativeMisses | Deleted |
| FusionTracking | 匹配到YOLO认领簇 | FusionTracking (保持) |
| FusionTracking | 匹配到孤儿簇 | PureRadarTracking (YOLO丢失但雷达续命) |
| FusionTracking | no match | Lost |
| PureRadarTracking | 匹配到YOLO认领簇 | FusionTracking (YOLO恢复!) |
| PureRadarTracking | 匹配到孤儿簇 | PureRadarTracking (保持) |
| PureRadarTracking | no match | Lost |
| Lost | 匹配到YOLO认领簇 | FusionTracking (YOLO恢复) |
| Lost | 匹配到孤儿簇 | PureRadarTracking (雷达续命) |
| Lost | misses ≥ maxLostFrames | Deleted |
| Deleted | — | (槽位回收) |

**只有 YOLO 认领簇能创建新 track (Tentative)**— 孤儿簇仅维持已有航迹续命。

**maxTracks 满时淘汰策略**: 当槽位满且无 Deleted 可回收时，淘汰最老的 Lost track 腾出空间。若所有 track 均为 FusionTracking/PureRadarTracking（无 Lost），才丢弃新检测。

**设计要点**:
- `FusionTracking` 和 `PureRadarTracking` 首次丢失**立即**进入 Lost
- `FusionTracking → PureRadarTracking` 表达"YOLO暂时丢失, 雷达仍能看到"
- `PureRadarTracking → FusionTracking` 表达"YOLO重新检测到了"
- Lost 状态仍可被匹配(孤儿或YOLO), 实现航迹恢复

**关键参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `maxTentativeMisses` | 1 | 待定航迹最大连续丢失 |
| `minHitsToConfirm` | 3 | Tentative→FusionTracking连中帧数 |
| `maxLostFrames` | 20 | Lost最大存活帧数(~2秒@10Hz) |
| `maxTracks` | 50 | 最大航迹数 |

**移除参数**: `maxCoastingFrames`(→maxLostFrames), `maxStaleCoastingFrames`(冗余), `orphanMinHitsToConfirm`(从未使用), `requireClassIdMatch`(不再用classId门控)

---

### 5.7 告警 (check_warnings_)

对 FusionTracking 或 PureRadarTracking, `age ≥ minConfirmedAgeForWarning(2)`, 冷却 2 秒:

- **进入**: `dist < warningEnterDistMeters(0.5m)` → 触发告警回调
- **退出**: `dist > warningExitDistMeters(0.6m)` → 解除告警

---

### 5.8 快照 (update_snapshot_)

加锁拷贝 `workingTracks_` → `snapshotTracks_` (排除Deleted)  
同时加锁拷贝 `clusterVisBuf_[]` → 聚类可视化数据(质心+半径+类型)

供 `FusionWorker`(100ms轮询)、OSD距离标签、Web聚类可视化读取。

---

## 六、Web聚类可视化 (v2新增)

### 6.1 数据结构

```cpp
struct ClusterVisData {
    float    cx, cy;       // 质心 (LiDAR坐标, 米)
    float    radius;       // 簇内点到质心的最大距离 (米)
    uint32_t pointCount;   // 簇内点数
    bool     isOrphan;     // true=孤儿簇, false=YOLO认领簇
};
```

每帧最多32个簇, 通过 `copy_cluster_vis()` 导出。

### 6.2 传输路径

```
tracker_->update_snapshot_() → clusterVisBuf_[32] (加锁)
  → LidarCameraFusion::copy_cluster_vis() (加锁拷贝)
  → FusionWorker (100ms轮询)
  → Widget::on_tracking_updated_()
  → JSON序列化, 混入 "tracking" WebSocket 消息的 "clusters" 字段
  → Web前端 drawRadar() 渲染
```

### 6.3 Canvas绘制

- **YOLO认领簇**: 半透明绿 `rgba(63,185,80,0.25)` 填充圆
- **孤儿簇**: 半透明蓝 `rgba(68,147,248,0.25)` 填充圆
- 圆心 = 质心在俯视图上的投影, 半径 = `cluster.radius * scale` (clamp 60px)
- 标签: 点数
- 可通过 `clusterVisEnabled` 开关(warm-modifiable)

### 6.4 调参闭环

```
修改 config.ini / Web参数面板 → configure_tracker() 热更新
  → 聚类参数立刻生效
  → Web端实时看到聚类圆变化
  → 快速找到最优 dbscanEpsMeters / dbscanMinPoints
```

---

## 七、纯雷达模式 (YOLO无检测时)

```
hasYolo=false
→ get_latest_frame() 取最新雷达帧
→ 跳过 fuse_data() (无bbox可投影)
→ allBboxes为空
→ tracker全点归孤儿聚类
→ 只有孤儿检测参与预测+关联+生命周期
→ FusionTracking/PureRadarTracking track靠孤儿续命
→ Lost track靠孤儿恢复为PureRadarTracking
→ 最多撑 maxLostFrames(20帧, ~2秒)
```

**对比旧方案**: 旧方案孤儿只能匹配Coasting, 且保持Coasting; 新方案孤儿能匹配FusionTracking/PureRadarTracking/Lost, 并能将FusionTracking降级为PureRadarTracking或将Lost恢复为PureRadarTracking。

---

## 八、停止融合复位 (v2新增)

点击"停止融合" → `fusion_->reset_tracking()` → 清空所有track, 重置ID计数器 → 下次启动全新跟踪。

旧方案不自动复位, track状态跨启停残留。新方案每次启动都是干净状态。

---

## 九、关键设计权衡与已知短板

### 优点

- ✓ **DBSCAN 2D聚类**: 不受角度排序影响, 近距不碎裂, 无wrap-around特殊处理
- ✓ **评分制bbox认领**: 边沿目标(质心不在框内)也能认领簇
- ✓ **评分制关联**: 纯距离评分, 比点数加权更稳定, 不受簇大小干扰
- ✓ **5状态显式表达**: FusionTracking vs PureRadarTracking 区分YOLO有无
- ✓ **Lost可恢复**: 无论孤儿还是YOLO重新检测都能恢复航迹
- ✓ **纯雷达持续跟踪**: 无YOLO时孤儿簇维持已有track
- ✓ **Web聚类可视化**: 实时看聚类效果, 快速调参
- ✓ **停止融合复位**: 每次启动干净状态

### 短板

- ✗ **孤儿不创建新track**: 纯雷达无法独立发现全新目标, 需YOLO首次探测
- ✗ **bbox数<人数时**: 每框最多一个检测, 多出的人只能孤儿续命
- ✗ **DBSCAN O(n²)**: 540点可忽略, 未来若升级高线束雷达需考虑空间索引加速
- ✗ **投影和跟踪聚类分开**: OSD看到点 ≠ tracker检测到(历史问题, 本次未改)

### 已解决的旧短板

| 旧短板 | 解决方式 |
|--------|----------|
| 2.5m硬编码上限 | `maxClusterDistanceMeters` 可热修改, 默认10m |
| CDC近距碎裂 | DBSCAN 2D邻域搜索 |
| 质心必须在bbox内 | 评分制认领, `bboxClaimMaxPixelDist` 扩展框外容差 |
| bboxIdx跨框门控 | 移除, 纯空间距离评分 |
| Coasting永久不恢复 | Lost状态+孤儿匹配可恢复 |
| 停止后track残留 | stop时reset_tracking() |

---

## 十、可调参数完整清单 (config.ini [Fusion] 节)

### DBSCAN聚类

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `dbscanEpsMeters` | 0.5 | 邻域半径(米) | ✓ |
| `dbscanMinPoints` | 5 | 核心点最小邻居数 | ✓ |
| `maxPointDistanceMeters` | 30.0 | 收集点的最远距离 | ✓ |
| `maxClusterDistanceMeters` | 10.0 | 簇质心最远距离 | ✓ |
| `clusterPersistenceFrames` | 2 | 簇连续出现多少帧后才输出detection | ✓ |

### Bbox认领

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `bboxClaimMaxPixelDist` | 100.0 | 投影点离bbox边缘最大像素距离 | ✓ |

### 滤波

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `alpha` | 0.45 | 位置平滑(0~1) | ✓ |
| `beta` | 0.20 | 速度估计 | ✓ |
| `minHitsForVelocity` | 2 | 启用速度的最小命中帧数 | ✓ |

### 关联

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `bboxAssocMaxDistMeters` | 1.5 | bbox检测关联硬门限(米) | ✓ |
| `orphanAssocMaxDistMeters` | 1.0 | 孤儿检测关联硬门限(米) | ✓ |

### 生命周期

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `minHitsToConfirm` | 3 | Tentative→FusionTracking连中帧数 | ✓ |
| `maxTentativeMisses` | 1 | 待定最大连续丢失 | ✓ |
| `maxLostFrames` | 20 | Lost最大存活帧数 | ✓ |
| `maxTracks` | 50 | 最大航迹数 | ✓ |

### 告警

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `warningEnterDistMeters` | 0.5 | 进入告警区距离 | ✓ |
| `warningExitDistMeters` | 0.6 | 退出告警区距离 | ✓ |
| `minConfirmedAgeForWarning` | 2 | 最小告警年龄 | ✓ |
| `warningCooldownNs` | 2000000000 | 冷却(纳秒, 2秒) | ✓ |

### 可视化

| 参数 | 默认值 | 含义 | 热修改 |
|------|--------|------|--------|
| `clusterVisEnabled` | false | 是否显示聚类圆 | ✓ |
| `clusterVisOpacity` | 0.3 | 聚类圆透明度 | ✓ |

### 相机 (保持不变)

| 参数 | 含义 |
|------|------|
| `Cam0Fx/Fy/Cx/Cy` | 相机0内参 |
| `Cam0T0~T15` | 相机0外参矩阵(row-major, LiDAR→Camera) |
| `Cam1Fx/Fy/Cx/Cy` | 相机1内参 |
| `Cam1T0~T15` | 相机1外参矩阵(row-major, LiDAR→Camera) |

### 移除的参数

| 移除参数 | 原因 |
|----------|------|
| `clusterEpsMeters` | 重命名为 `dbscanEpsMeters` |
| `minClusterPoints` | 重命名为 `dbscanMinPoints` |
| `maxAssociationDistMeters` | 重命名为 `bboxAssocMaxDistMeters` |
| `maxOrphanAssocDistMeters` | 重命名为 `orphanAssocMaxDistMeters` |
| `maxCoastingFrames` | 重命名为 `maxLostFrames` |
| `maxStaleCoastingFrames` | 与 `maxCoastingFrames` 冗余 |
| `orphanMinHitsToConfirm` | 从未使用 |
| `requireClassIdMatch` | 不再使用classId门控 |

---

## 十二、v2 方案评审异议记录

> 第一轮（架构层面）和第二轮（设计细节层面）外部评审记录，含采纳/不采纳理由。

### 第一轮评审记录（架构层面）

> 结论: 系统已是工程级雏形，但仍是 cluster-driven heuristic tracking。

### 12.1 认同但不采纳的建议

---

#### 建议 1：贪心关联 → Hungarian 全局最优分配

**评审观点**: 贪心最近邻不保证全局最优，多目标密集时容易错配。应升级为 Hungarian 算法做全局 assignment。

**不采纳理由**:
- 当前规模 ~10 detection + ≤50 track，贪心与 Hungarian 差距 <5%
- Hungarian O(n³) 在 ARM Cortex-A76 上的开销不值得（虽然 n 小可以忽略）
- 这不是当前稳定性瓶颈，是优化而非修复
- **P2 后续优化**，当前不改

---

#### 建议 2：5 状态机合并为 "Tracked + sensor flags"

**评审观点**: FusionTracking 和 PureRadarTracking 本质只是"有没有 YOLO"，应合并为一个 Tracked 状态用布尔 flag 区分，减少状态机复杂度。

**不采纳理由**:
- 需求 7 明确要求"不同状态用不同颜色表示"，这是用户强需求
- 显式状态在调试日志、WebSocket 序列化、俯视图渲染中价值明确
- 两种状态的生命周期语义不同：FusionTracking 只能由 YOLO 首次创建，PureRadarTracking 只能由孤儿匹配形成
- 合并成 flag 会让 `manage_lifecycle_` 中的转换逻辑更不直观（嵌套 if-else 替代 flat switch）

---

#### 建议 3：YOLO 和 LiDAR 不应在同一评分空间混合

**评审观点**: YOLO（语义强空间弱）和 LiDAR（空间强语义弱）本质不同，不应混评。

**不采纳理由**:
- 融合的本质就是把不同传感器的观测放在同一空间竞争
- 当前方案已通过不同的硬门限区分两个模态：`bboxAssocMaxDistMeters`（YOLO 认领簇）和 `orphanAssocMaxDistMeters`（孤儿簇）
- 评分公式相同（纯距离）恰恰避免了模态间的不公平比较
- 如果为两个模态设计不同评分函数，反而引入更多超参数和调参复杂度

---

#### 建议 4：DBSCAN 应降级为"可视化/提案工具"，不作为 tracking 输入

**评审观点**: tracking 应该是 track-centric（航迹去解释观测），而不是 cluster-centric（聚类决定航迹）。DBSCAN 抖动直接导致 track 抖动。

**不采纳理由**:
- 这是 tracking-by-detection vs tracking-by-proposal 的范式之争，而非方案缺陷
- 540 点单线 LiDAR 上，不对点云做聚类就无法形成检测。逐点跟踪毫无意义
- DBSCAN 的稳健性由 `dbscanEpsMeters` 和 `dbscanMinPoints` 热修改保证——如果聚类效果不好，调参立即可见（Web 可视化）
- Alpha-Beta 滤波 + 多帧命中确认（`minHitsToConfirm=3`）已经提供了单帧聚类抖动的时间平滑
- track-centric 全概率框架（粒子滤波 / JPDA / MHT）对嵌入式 540 点系统是过度设计

---

#### 建议 5：升级为 "SORT + LiDAR hybrid + KF + 全概率框架"

**评审观点**: 应将整个系统升级为工业级 LiDAR-camera tracking 架构，包括卡尔曼滤波状态设计、Hungarian 匹配器、YOLO/LiDAR 融合权重模型。

**不采纳理由**:
- 这等于重建整个系统，而非优化当前管线
- 与项目原则"只做需求内的事，不搞未来可能用到的功能"冲突
- Alpha-Beta 滤波器对当前 10Hz 单线雷达场景已足够（KF 的优势在高速/非线性场景才明显）
- 当前系统是嵌入式实时系统，不是学术实验平台——复杂度预算有限

---

---

### 12.2 第二轮评审记录（设计细节层面）

> 核心洞察: 不是 DBSCAN vs point-tracking 的范式问题，而是缺了"temporal evidence accumulation layer"。

**评审核心论点**: DBSCAN cluster 不是 sensor measurement，而是 density-induced segmentation。同一个人在 5m/4.8m/5.2m 可能产出 1/2/1 个 cluster——这是拓扑不连续。tracking 最怕的就是 observation topology change。

**采纳的建议**: "multi-frame detection stabilization before association"——在 DBSCAN 输出和 detection 之间加帧间确认层。

| 建议 | 采纳/不采纳 | 理由 |
|------|------------|------|
| cluster EMA 平滑 | ✓ 采纳 | 降 jitter，降 frame-to-frame discontinuity |
| orphan persistence ≥2 帧 | ✓ 采纳 | temporal gating / hysteresis |
| 加 temporal evidence accumulation layer | ✓ **采纳** | cluster 连续出现≥2帧才输出detection，过滤瞬时噪声 |
| point-level tracking | ✗ 不采纳 | point 是不可辨识观测（无 identity），point-to-track 本质上仍是隐式聚类，复杂度搬了地方没消失 |
| KF 替代 AB filter | ✗ 不采纳 | 10Hz 行人场景 CV 模型足够，AB = 定增益 KF |

**本次设计迭代的核心收获**: 争论两轮的"DBSCAN 该不该进 tracking"是一个被错误聚焦的问题。真正的问题是单帧 cluster 的瞬时性——用帧间持久化确认解决，而非扔掉 DBSCAN。

---

### 12.3 已采纳的全部修改

| 来源 | 建议 | 采纳内容 |
|------|------|----------|
| 第一轮 | 去掉 pointCount 主导作用 | 三处评分公式改为 `1.0/(1+distance²)` |
| 第二轮 | cluster EMA 平滑 | 质心做 3 帧指数移动平均 |
| 第二轮 | orphan persistence | 孤儿簇需连续出现≥2帧 |
| 第二轮 | temporal evidence accumulation | 五-5.1b 节，全部 cluster 需 persistence 确认后输出 |

---

### 12.4 后续迭代可考虑的改进

| 优先级 | 内容 | 条件 |
|--------|------|------|
| P2 | 贪心 → Hungarian 全局分配 | 当 track 数 >30 且 ID switch 显著时 |
| P3 | Alpha-Beta → 2D Kalman Filter | 当需要估计加速度或传感器噪声协方差不一致时 |
| P4 | 孤儿创建新 track（纯雷达独立发现） | 当场景中出现大量未被 YOLO 覆盖的目标时 |
| P5 | 点云降采样 + 空间索引加速 DBSCAN | 当升级到多线 LiDAR（点数 >5000）时 |

---

## 十三、数据流总览

```
YOLO ──→ fakeDetections_ ──→ fuse_data(OSD用)
   │                              │
   │                         bbox点集 → OSD快照 → 推流画面
   │
   └──→ tracker_->update()
            │  [入口检查: frame.timestampNs != lastLidarTs_ → 新帧, 继续]
            │  [入口检查: frame.timestampNs == lastLidarTs_ → 重复帧, 跳过]
            │
            │ 注意: 融合循环 10ms, LiDAR 10Hz(100ms), 同一帧会被
            │ 调用 ~10 次。tracker 只在时间戳变化时才执行实际逻辑。
            │
            │ 另外: YOLO 队列 FIFO(try_pop 取最旧), 中间迭代会白白
            │ pop 掉结果。应先检查 LiDAR 新帧到达, 再取 YOLO。
            │
            ├─ dbscan_cluster_  (DBSCAN 2D空间聚类)
            │     ├─ 噪声点(点数不足) → 丢弃
            │     └─ 原始cluster(最多32个)
            │
            ├─ cluster_persistence_  (时间证据累积, 帧间确认)
            │     ├─ cluster质心匹配history → 连续命中计数
            │     ├─ >= clusterPersistenceFrames(2) → 输出stable detection
            │     └─ < 2帧 → 暂存不输出(过滤闪烁噪声)
            │
            ├─ bbox_claim_  (簇→bbox认领评分)
            │     ├─ 被认领 → bbox detection
            │     ├─ 未被认领 → orphan detection
            │     └─ 聚类可视化数据(clusterVisBuf_)
            │
            ├─ predict_tracks_  (Alpha-Beta)
            ├─ associate_       (评分制贪心最近邻)
            │     ├─ bbox检测 → 硬门限+评分 → track
            │     └─ 孤儿检测 → 硬门限+评分 → track(不创建新track)
            │
            ├─ apply_correction_ (滤波更新, 记录匹配类型)
            ├─ manage_lifecycle_ (5状态机: Tentative→Fusion⇄PureRadar→Lost→Deleted)
            ├─ check_warnings_   (告警: FusionTracking+PureRadarTracking)
            └─ update_snapshot_  (线程安全快照: tracks + clusterVis)
                  │
                  ├─ FusionWorker(100ms) → topDownView 俯视图(5状态着色)
                  │
                  ├─ WebSocket "tracking" (5Hz)
                  │     ├─ targets[] → Canvas drawRadar(4色目标+速度箭头)
                  │     └─ clusters[] → Canvas drawRadar(聚类圆)
                  │
                  └─ OSD距离标签 → 推流画面
```
