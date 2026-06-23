# LidarCameraFusion — 学习指南

## 目标

面试时能说清：做了什么、为什么这么设计、踩过什么坑。

---

## 第一层：能说清"做了什么"（面试讲项目用）

### 一句话概括

> 把单线激光雷达点云投影到 YOLO 检测框内，聚类出目标位置，用 Alpha-Beta 滤波器估计位置和速度，维持跨帧航迹 ID。脱离 ROS，C++14，RK3588 板端实跑。

### 数据流（能画出来）

```
N10Plus 雷达 (540 pts, 10Hz)      YOLO 检测 (暂用虚构框)
        │                                │
        └───────────┬────────────────────┘
                    ▼
             fuse_data()
             ├── 外参变换 (4x4 齐次, z=0 优化为 6 乘法)
             ├── 内参投影 (pinhole: u=fx*cx/cz+cx)
             ├── bbox 分类 (first-hit 策略)
             └── 计数排序写回 → FusionResult
                    │
                    ▼
            update_tracking()
            ├── CDC 聚类 (扫描顺序 + atan2 排序)
            ├── Alpha-Beta 预测 (恒速模型, dt 三层保护)
            ├── 贪心 NN 关联 (classId 门控)
            ├── Alpha-Beta 校正 (先增 hits 再判速度)
            ├── 生命周期管理 (Tentative→Confirmed→Coasting→Deleted)
            └── 告警检查 (迟滞 + 冷却节流)
```

### 关键代码（背下来）

```cpp
// 4 步用法，面试能张口就来
LidarCameraFusion fusion;
TrackerConfig cfg;
fusion.configure_tracker(cfg);          // 1. 配置
fusion.enable_tracking(true);           // 2. 启用
fusion.register_warning_callback(cb);   // 3. 告警回调

fusion.reset();
fusion.fuse_data(dets, imageTs, frame, camCfg);   // 4a. 融合
const FusionResult& r = fusion.result();
fusion.update_tracking(r, frame.points, frame.pointsCount,
                       dets.data(), dets.size(), frame.timestampNs);  // 4b. 跟踪

// 查询结果
fusion.copy_tracked_targets(snapshot, 50, &count);
```

### Qt 集成（能解释）

```cpp
// 通过快照接口 + 告警回调与 Qt 对接，无需继承 QObject
fusion_.register_warning_callback([](const TrackedTarget& t, void* self) {
    emit static_cast<FusionController*>(self)->proximityWarning(t);
}, this);

// QTimer 50ms 刷新一次快照，展示到 UI
void refreshTracking() {
    fusion_.copy_tracked_targets(snapshot, 50, &count);
    // 更新 UI 列表
}
// 告警回调在 fusion 线程，emit 需 Qt::QueuedConnection
```

---

## 第二层：能解释"为什么这么设计"（面试追问用）

### 决策 1：为什么用 Alpha-Beta 而不是全 Kalman？

| 方案 | Kalman | Alpha-Beta（我们选的） |
|------|--------|----------------------|
| 计算量 | 矩阵运算，6 状态 36 元素协方差 | 4 次浮点乘加 |
| 调参 | 过程噪声 Q + 观测噪声 R，难直观理解 | α 位置平滑 / β 速度平滑，直观 |
| 稳态等价 | Kalman 增益收敛后 = Alpha-Beta | 本身就是稳态 Kalman |
| RK3588 适用性 | NPU 不跑这个，CPU 跑浪费 | 50 个目标 × 4 浮点 = 可忽略 |

**话术**: "嵌入式平台没必要跑全 Kalman。Alpha-Beta 是稳态 Kalman 的等价形式，调参直观，计算量可忽略。唯一的代价是收敛速度不如 Kalman 自适应，但 10Hz 雷达帧率下差异不大。"

### 决策 2：为什么聚类用扫描顺序而不是 atan2 排序？

| 方案 | atan2 排序（最初考虑的） | 扫描顺序（最终选的） |
|------|------------------------|-------------------|
| 原理 | 对每个 bbox 内的点按 `atan2(y,x)` 排序后线性扫描 | 利用 `fuse_data` 按扫描索引递增写入的特性，点天然有序 |
| ±π 边界 | 雷达正后方相邻点 atan2 跳变，被错误切分为两个簇 | 不受影响 |
| 性能 | 需要额外 O(n log n) | O(n)，无需排序 |
| 360° 回绕 | 同上 | 首尾簇边界点距离 < 阈值即合并 |

**话术**: "单线雷达扫一圈 360 度。atan2 在正后方有 ±π 不连续性，我们的系统有两个摄像头一前一后，物体出现在正后方是常态，atan2 边界问题绕不开。用扫描索引天然顺序避免了这个问题。"

### 决策 3：孤儿检测为什么只续命不新建？

```
人有 YOLO 确认 → classId=person → 创建航迹 → 跟踪
人出相机       → bbox 消失 → 航迹 coasting → 孤儿 LiDAR 簇续命
墙/桌椅        → 无 YOLO → 孤儿簇 → 不能创建航迹 ← 这里拦截
```

**话术**: "雷达不知道那个点簇是人还是墙，只有 YOLO 知道。孤儿检测的唯一目的是给已经确认过的航迹'续命'——在相机暂时看不到的时候靠雷达维持。没有 YOLO 确认的点簇绝对不能创建新航迹，否则桌椅墙壁都会被当成目标。"

### 决策 4：为什么告警用迟滞（双阈值）？

```
进入告警：dist < 3.0m → 触发
解除告警：dist > 3.5m → 解除
3.0m ~ 3.5m：维持当前状态不翻
```

**话术**: "单阈值在边界处会反复触发/解除，一秒钟几十次，日志和 UI 都会炸。迟滞就是一个简单的施密特触发器——进入和退出用不同阈值。"

### 决策 5：为什么用双缓冲而不是锁全程？

```
update_tracking()           copy_tracked_targets()
       │                            │
  workingTracks_              Qt UI 线程
   (无锁读写)                      │
       │                      加锁 memcpy
  加锁 memcpy ←──────────── snapshotTracks_
       │
  解锁
```

**话术**: "`update_tracking` 做聚类+关联+校正，需要几百微秒。如果全程加锁，Qt UI 线程就要等几百微秒才能读到数据。双缓冲让计算在 working buffer 里无锁进行，最后 memcpy 到 snapshot 时才短暂加锁，UI 线程几乎不需要等待。"

---

## 第三层：能讲清 bug 和教训（面试加分项）

### 从 BUG_RECORD.md 选 3 个最有代表性的

**1. 虚构检测框固定位置导致墙壁抢夺航迹**

- 现象：板端测试人的航迹从 1.2m 跳到 3.5m，远处工位的雷达簇取代了人
- 原因：虚构 bbox 覆盖全图，1.2m 的人和 3.5m 的墙投影到同一像素区域，聚类评分只按点数+距离，墙的点多就赢
- 修复：收窄 bbox + 硬编码 2.5m 距离过滤排除远处簇
- **面试话术**: "YOLO 没就绪时我们用固定虚构框做测试，发现跟踪器会把墙壁也当成人。根因是没有 classId 来区分人和墙。这个问题的彻底解法是接入 YOLO——它告诉你'这个框里是人'，而雷达告诉你'这个人在哪'。两者互补，缺一不可。"

**2. 关联门限太小导致 ID 频繁新建**

- 现象：人在 1.2m 和 3.5m 之间走动，ID 从 1 暴涨到 52，平均走一趟换一个 ID
- 原因：两个位置相距 2.2m > 默认关联门限 2.0m。航迹移动到另一端时匹配不上旧航迹
- 修复：虚构框阶段调至 3.0m；YOLO 就绪后恢复 2.0m
- **面试话术**: "参数调优要看场景。2m 门限在有 YOLO 动态 bbox 的空间约束下完全够——人不会在两帧之间瞬移 2m。但虚构框覆盖全图时，2m 不够。这不是算法的问题，是测试条件的限制。接入 YOLO 后这个参数会自动回归合理值。"

**3. 孤儿检测误创建航迹导致跟踪污染**

- 现象：测试中出现大量短暂 Tentative 航迹，干扰输出
- 原因：孤儿聚类产出的检测在未匹配到已有航迹时，创建了新的 Tentative 航迹
- 修复：在"未匹配检测→创建航迹"阶段跳过 `isOrphan=true` 的检测
- **面试话术**: "这是一个设计疏忽——孤儿检测的定位是'续命已有航迹'，但代码里忘记在创建新航迹的路径上加拦截。修起来就一行 `continue`，但暴露了一个原则问题：每类检测的权限边界必须明确。孤儿检测可以匹配 coasting 航迹，但没有权力创建新航迹。"

---

## 怎么对着代码学

**别死记硬背。跟一遍数据流：**

1. 打开 `src/lidar_camera_fusion.cpp`，从 `fuse_data()` 开始
2. 跟一帧：`transform_point_()` → `project_point_()` → bbox 遍历 → `candidatePointBuf` 写回
3. 打开 `src/lidar_target_tracker.cpp`，找到 `update()`
4. 跟一帧：`cluster_bbox_points_()` → `predict_tracks_()` → `associate_()` → `manage_lifecycle_()` → `check_warnings_()`
5. 打开 `demo_tracking.cpp`，跑通 Test 1，理解合成数据怎么造的

**重点函数入口：**

| 函数 | 作用 | 在哪个文件 |
|------|------|----------|
| `fuse_data()` | 理解融合四步：变换→投影→分类→写回 | `lidar_camera_fusion.cpp` |
| `cluster_bbox_points_()` | 理解扫描顺序 CDC + wrap-around | `lidar_target_tracker.cpp` |
| `cluster_orphan_points_()` | 理解孤儿点聚类 | `lidar_target_tracker.cpp` |
| `predict_tracks_()` | 理解 dt 三层保护 | `lidar_target_tracker.cpp` |
| `associate_()` | 理解 classId 门控 + 孤儿检测限制 | `lidar_target_tracker.cpp` |
| `manage_lifecycle_()` | 理解状态机四态转换 | `lidar_target_tracker.cpp` |
| `check_warnings_()` | 理解迟滞双阈值 | `lidar_target_tracker.cpp` |
| `fusion_thread_()` | 理解虚构框怎么生成、怎么同步记录 bbox | `lidar_camera_fusion_thread.cpp` |
