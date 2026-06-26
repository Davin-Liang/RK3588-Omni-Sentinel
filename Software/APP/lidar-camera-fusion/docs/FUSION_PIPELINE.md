# 融合管线详细说明

> RK3588 Omni-Sentinel  
> 单线机械式LiDAR(镭神N10Plus, 10Hz) + 双路OV2710相机(30fps)  
> YOLOv8 NPU推理 + 全局CDC聚类 + Alpha-Beta多目标跟踪  
> C++14, 零ROS依赖, 线程安全双缓冲快照

---

## 一、整体架构

融合由独立线程 `fusion_thread_()` 驱动, 10ms周期循环:

```
步骤1  取YOLO检测(每路相机独立)
步骤2  检测是否hasYolo → 拿雷达帧(有YOLO时间对齐, 无YOLO取最新)
步骤3  fuse_data() 投影LiDAR点到相机像素(用于OSD显示)
步骤4  tracker_->update() 全局聚类 + 预测 + 关联 + 生命周期
步骤5  构建OSD快照(复用tracker聚类质心)
步骤6  sleep(10ms)
```

---

## 二、YOLO检测层

每路相机独立拉取: `detectionProvider_(camNum, out, 33ms)`

**过滤**: `classId==0(person)` AND `confidence>=0.60`

**输出**: `fakeDetections_[cam] = vector<YoloBBox>`

```
YoloBBox { x1, y1, x2, y2, classId, confidence, timestampNs }
坐标系: NPU 640×640 RGB888 推理小图
```

**无YOLO时**: `fakeDetections_`为空 → `hasYolo=false` → 后续走纯雷达模式(仅孤儿跟踪)

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

## 五、Tracker核心: 全局聚类 + 多目标跟踪

### 5.1 全局聚类 (cluster_all_points_)

**输入**: 全部LiDAR点(540个), YOLO bbox列表, 相机外参+内参

**流程**:

a) **收集有效点**: 过滤零点 `(0,0)`, 过滤 >30m 远点

b) **按角度排序**(atan2): 确保CDC连续扫描特性可用

c) **CDC线性扫描聚类**:
- 相邻两点欧氏距离 < `clusterEpsMeters(0.5m)` → 同簇
- 距离 ≥ 0.5m → 新簇
- 最多32簇, 每簇记录 `startIdx, count, sumX, sumY`

d) **Wrap-around合并**: 首尾两点距离 < 0.5m → 合并首尾簇

e) **簇过滤**:
- `count < minClusterPoints(5)` → 丢弃
- 质心距离 > 2.5m → 丢弃 (硬编码)

**关键参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `clusterEpsMeters` | 0.5 | 两点聚集门限(米) |
| `minClusterPoints` | 5 | 最小簇点数 |

**评分公式**(bbox竞争用):

```
score = pointCount × (2.5 / centroidDistance)
```

近距离优先, 点数作为次要加权因子。

---

### 5.2 簇→bbox匹配

每个簇质心 → 通过外参投影到相机像素(u,v) → 检查落入哪个bbox

**两趟算法**:
1. **第一趟**: 收集所有 `(簇, bbox)` 候选对, 计算评分
2. **第二趟**: 每个bbox选评分最高的簇 → 产出 **bbox检测** (`isOrphan=false`)

未被任何bbox选中的簇 → **孤儿检测** (`isOrphan=true, bboxIdx=0xFFFFFFFF`)

每个bbox最多一个检测。每帧总检测数 = bbox检测数 + 孤儿检测数。

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

### 5.4 贪心最近邻关联 (associate_)

**匹配顺序**: bbox检测先处理, 孤儿检测后处理

#### bbox检测 → track

- 门限: `maxAssociationDistMeters = 1.0m`
- **同bboxIdx**: 全门限 1.0m
- **跨bboxIdx**: 门限缩至25% → 0.5m (防止track在不同框间跳变)
- classId门控: `requireClassIdMatch = true`
- 贪心策略: 每个检测找最近的未匹配track

#### 孤儿检测 → track

- **仅匹配Coasting状态**的已确认track (`consecutiveHits ≥ minHitsToConfirm=3`)
- 门限: `maxOrphanAssocDistMeters = 0.5m`
- 孤儿**不匹配** Confirmed或Tentative track
- 孤儿**不创建新track** ← 当前最大设计短板

#### 未匹配的bbox检测

→ 创建新Tentative航迹 (孤儿检测跳过此步骤)

---

### 5.5 校正 (apply_correction_)

**匹配成功**:
```
miss = 0
hits++, age++
pos(X,Y) → Alpha-Beta滤波: pos = pred + alpha × (det - pred)
vel(X,Y) → Beta/dt滤波: vel = predVel + (beta/dt) × (det - predPred)
          (仅当 hits ≥ minHitsForVelocity 时更新速度)
```

**状态恢复**(仅匹配成功时):
- bbox检测匹配 + Coasting → **恢复为 Confirmed**
- 孤儿检测匹配 + Coasting → **保持 Coasting** (不恢复, 防C/K震荡)

**未匹配**: `miss++, age++`

---

### 5.6 生命周期 (manage_lifecycle_)

航迹状态机:

```
                    ┌─────────────────────┐
                    │     Tentative       │
                    │  (新建, 未确认)      │
                    └──────┬──────┬───────┘
               miss>1(删)   │      hits≥3
                            │      (确认)
                            ▼
                    ┌─────────────────────┐
                    │     Confirmed       │
                    │  (正常跟踪中)         │
                    └──────────┬──────────┘
                          miss>0
                               │
                               ▼
                    ┌─────────────────────┐
                    │     Coasting        │
                    │  (短暂丢失, 预测外推)  │
                    └──────────┬──────────┘
                          miss≥20
                               │
                               ▼
                    ┌─────────────────────┐
                    │     Deleted         │
                    │  (删除, 槽位回收)     │
                    └─────────────────────┘
```

**参数**:

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `maxTentativeMisses` | 1 | 待定航迹最大连续丢失 |
| `minHitsToConfirm` | 3 | Tentative→Confirmed连中帧数 |
| `maxCoastingFrames` | 20 | Coasting最大存活帧数(~2秒) |
| `maxStaleCoastingFrames` | 20 | 同上(冗余保护) |
| `maxTracks` | 50 | 最大航迹数 |

---

### 5.7 告警 (check_warnings_)

仅 Confirmed track, `age ≥ minConfirmedAgeForWarning(2)`, 冷却 2 秒:

- **进入**: `dist < warningEnterDistMeters(0.5m)` → 触发告警回调
- **退出**: `dist > warningExitDistMeters(0.6m)` → 解除告警

---

### 5.8 快照 (update_snapshot_)

加锁拷贝 `workingTracks_` → `snapshotTracks_` (排除Deleted)

供 `FusionWorker`(100ms轮询)和OSD距离标签读取。

---

## 六、OSD快照与推流显示

**数据来源**: fuse_data的bbox点集 + tracker的聚类质心

| 字段 | 来源 | 用途 |
|------|------|------|
| bboxX1/Y1/X2/Y2 | YOLO检测框 | 框位置 |
| bboxPointU/V | fuse_data投影 | LiDAR点散点绘制 |
| bboxClusterDistMeters | tracker聚类质心 | 距离标签 |

流媒体绘制: NV12帧上画LiDAR点散点(按距离着色: <5m红, <15m黄, >15m青) + 距离标签

---

## 七、纯雷达模式 (YOLO无检测时)

```
hasYolo=false
→ get_latest_frame() 取最新雷达帧
→ 跳过 fuse_data() (无bbox可投影)
→ allBboxes为空
→ tracker全点归孤儿聚类
→ 只有孤儿检测参与预测+关联+生命周期
```

Confirmed track靠孤儿检测续命 → 保持Coasting → 最多撑20帧(~2秒)

---

## 八、关键设计权衡与已知短板

### 优点

- ✓ **全局聚类**: 免per-bbox分配污染, 一个物理目标一个簇
- ✓ **距离评分**: 近距人体优先级稳定高于远处墙壁/椅子
- ✓ **孤儿不回Confirm**: 消除C/K无限震荡
- ✓ **跨bbox门限收缩**: track不会在不同框间跳变
- ✓ **get_latest_frame**: 无YOLO时纯雷达不中断
- ✓ **bboxIdx绑定**: bbox检测优先匹配自己创建的track

### 短板

- ✗ **孤儿不创建新track**: track被删后永久丢失, 需YOLO重新发现
- ✗ **bbox数<人数时**: 每框最多一个检测, 多出的人只能孤儿续命
- ✗ **2.5m硬编码上限**: 远处目标无法跟踪
- ✗ **投影和聚类分开**: OSD能看到点 ≠ tracker能检测到
- ✗ **CDC聚类对极近距目标**(0.3m内)点数不足, 易碎裂

### 未启用的备选方案

`orphanMinHitsToConfirm=5`: 孤儿建track时更高确认门槛  
(参数已加, 逻辑已回退, 因孤儿建track误检太多暂关闭)

---

## 九、可调参数完整清单 (config.ini [Fusion] 节)

### 聚类

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `clusterEpsMeters` | 0.5 | 两点聚集门限(米) |
| `minClusterPoints` | 5 | 最小簇点数 |

### 滤波

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `alpha` | 0.45 | 位置平滑(0~1) |
| `beta` | 0.20 | 速度估计 |
| `minHitsForVelocity` | 2 | 启用速度的最小命中帧数 |

### 关联

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `maxAssociationDistMeters` | 1.0 | bbox检测关联门限(米) |
| `maxOrphanAssocDistMeters` | 0.5 | 孤儿检测关联门限(米) |
| `requireClassIdMatch` | true | 要求检测与track同类别 |

### 生命周期

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `minHitsToConfirm` | 3 | Tentative→Confirmed |
| `orphanMinHitsToConfirm` | 5 | 孤儿track确认帧数(未启用) |
| `maxTentativeMisses` | 1 | 待定最大连续丢失 |
| `maxCoastingFrames` | 20 | Coasting最大存活帧数 |
| `maxStaleCoastingFrames` | 20 | 同上 |
| `maxTracks` | 50 | 最大航迹数 |

### 告警

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `warningEnterDistMeters` | 0.5 | 进入告警区距离 |
| `warningExitDistMeters` | 0.6 | 退出告警区距离 |
| `minConfirmedAgeForWarning` | 2 | 最小告警年龄 |
| `warningCooldownNs` | 2000000000 | 冷却(纳秒, 2秒) |

### 相机

| 参数 | 含义 |
|------|------|
| `Cam1Fx/Fy/Cx/Cy` | 内参(362.24, 362.24, 309.31, 315.91) |
| `Cam1T0~T15` | 外参矩阵(row-major, LiDAR→Camera) |

---

## 十、数据流总览

```
YOLO ──→ fakeDetections_ ──→ fuse_data(OSD用)
   │                              │
   │                         bbox点集 → OSD快照 → 推流画面
   │
   └──→ tracker_->update()
            │
            ├─ cluster_all_points_  (全局CDC)
            │     ├─ bbox候选 → 评分排序 → bbox检测
            │     └─ 未选中 → 孤儿检测
            │
            ├─ predict_tracks_  (Alpha-Beta)
            ├─ associate_       (贪心最近邻)
            │     ├─ bbox检测 → 全门限/跨框缩限 → track
            │     └─ 孤儿检测 → 仅Coasting → track
            │
            ├─ apply_correction_ (滤波更新)
            ├─ manage_lifecycle_ (状态机)
            ├─ check_warnings_   (告警)
            └─ update_snapshot_  (线程安全快照)
                  │
                  ├─ FusionWorker(100ms) → topDownView 俯视图
                  └─ OSD距离标签 → 推流画面
```
