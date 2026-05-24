# LidarCameraFusion — 激光雷达-相机融合与多目标跟踪

单线激光雷达（N10Plus）+ YOLO 2D 检测框融合，外参变换 + 内参投影 + 点云聚类 + Alpha-Beta 滤波 + 多目标跟踪。**脱离 ROS，无外部运行时依赖。**

---

## 快速上手（3 步）

```cpp
LidarCameraFusion fusion;

// 1. 配置跟踪器
TrackerConfig cfg;
cfg.warningEnterDistMeters = 0.5f;   // 进入告警距离
cfg.warningExitDistMeters  = 0.8f;   // 解除告警距离
fusion.configure_tracker(cfg);
fusion.register_warning_callback([](const TrackedTarget& t, void*) {
    printf("[WARN] #%u dist=%.2fm\n", t.id, t.distanceMeters);
}, nullptr);
fusion.enable_tracking(true);

// 2. 每帧融合 + 跟踪
fusion.reset();
fusion.fuse_data(detections, imageTs, lidarFrame, cameraCfg);
const FusionResult& r = fusion.result();
fusion.update_tracking(r, lidarFrame.points, lidarFrame.pointsCount,
                       detections.data(), detections.size(), lidarFrame.timestampNs);

// 3. 查询跟踪结果
TrackedTarget snapshot[50];
uint32_t count = 0;
fusion.copy_tracked_targets(snapshot, 50, &count);
for (uint32_t i = 0; i < count; ++i) {
    const TrackedTarget& t = snapshot[i];
    // t.id, t.posX, t.posY, t.velX, t.velY, t.distanceMeters, t.classId, t.state
}
```

---

## 线程模式（接入真实雷达）

```cpp
SentinelLslidarer lidar;
lidar.load_config(lidarCfg);
lidar.start();

LidarCameraFusion fusion;
fusion.configure_tracker(trackerCfg);
fusion.enable_tracking(true);
fusion.register_warning_callback(myCallback, nullptr);
fusion.start(&lidar, &camCfg, 1);   // 1 路相机

// 跟踪在内部线程自动运行，通过回调接收告警。随时可查询快照。
std::this_thread::sleep_for(std::chrono::seconds(30));
fusion.stop();
lidar.stop();
```

---

## 与后续 YOLO 推理类联合调试注意事项

### 当前状态
YOLO 推理类尚未实现，暂用 `generate_fake_detections_()` 生成**固定的虚构检测框**。这对跟踪功能的测试有重大影响：

### 虚构框导致的问题
| 现象 | 根因 | 影响 |
|------|------|------|
| 航迹被远处物体"抢走" | bbox 固定，覆盖多距离段 | 多个物体的雷达簇落入同一 bbox，聚类评分高的获胜 |
| ID 频繁新建 | 人体移动超出 bbox | 航迹丢失后重新检测到人 → 新建 ID，无法延续 |
| 墙壁/桌椅被误跟踪 | bbox 不区分物体类别 | 没有 YOLO 确认，任何雷达簇都可能被当成人 |
| 测试效果差 | 虚构框不能随人体移动 | 人必须站在 bbox 投影区域内才能触发检测 |

### YOLO 就绪后需要做的事情
1. **替换 `generate_fake_detections_()`**：从 YOLO 推理队列 pop 真实检测结果，替换 `lidar_camera_fusion_thread.cpp:54-56`
2. **classId 门控自动生效**：`requireClassIdMatch=true` 时，不同类型目标自动隔离，**不再需要距离过滤硬编码**
3. **bbox 跟随人体移动**：不再固定位置，人体走到哪里都有对应 bbox，**关联距离可恢复 2.0m**
4. **建议调整的参数**：

| 参数 | 虚构框测试值 | YOLO 就绪后建议 |
|------|-------------|-----------------|
| `maxAssociationDistMeters` | 3.0 | 2.0（YOLO 框限定了空间范围） |
| `requireClassIdMatch` | true | true（保持） |
| bbox 范围 | 虚构固定区域 | 由 YOLO 输出决定 |
| 距离过滤 | 硬编码 2.5m | **移除**，YOLO classId 可替代 |

5. **重新验证的场景**：多人类别交叉、人出相机→雷达续追→人回相机恢复、不同类别物体互不干扰

### 距离过滤说明
`lidar_target_tracker.cpp` 的聚类函数中硬编码了 `dist > 2.5f` 过滤远距离簇。此限制是为了在**无 YOLO 的虚构框测试阶段**防止远处工位/墙壁的雷达簇干扰跟踪。**YOLO 就绪后可以移除**，让跟踪器覆盖雷达全量程。

---

## 配合 Qt 工程使用

`LidarCameraFusion` 内部自带 fusion 线程，**无需继承 QObject，也不依赖 Qt**。通过快照接口 + 告警回调与 Qt 信号槽对接：

```cpp
class FusionController : public QObject {
    Q_OBJECT
public:
    FusionController() {
        TrackerConfig cfg;
        cfg.warningEnterDistMeters = 0.5f;
        cfg.warningExitDistMeters  = 0.8f;
        fusion_.configure_tracker(cfg);
        fusion_.enable_tracking(true);

        // 告警回调（在 fusion 线程调用）
        fusion_.register_warning_callback([](const TrackedTarget& t, void* self) {
            auto* ctrl = static_cast<FusionController*>(self);
            emit ctrl->proximityWarning(t.id, t.classId, t.distanceMeters);
        }, this);

        // Qt 定时器刷新（50ms = 20Hz，雷达 10Hz 够用）
        timer_ = new QTimer(this);
        connect(timer_, &QTimer::timeout, this, &FusionController::refreshTracking);
        timer_->start(50);
    }

    void start(SentinelLslidarer* lidar, const CameraConfig* cfg, uint32_t n) {
        fusion_.start(lidar, cfg, n);
    }

signals:
    void proximityWarning(uint32_t trackId, uint32_t classId, float distMeters);
    void trackingUpdated(const QVector<TrackedTarget>& targets);

private slots:
    void refreshTracking() {
        TrackedTarget snapshot[50];
        uint32_t count = 0;
        fusion_.copy_tracked_targets(snapshot, 50, &count);

        QVector<TrackedTarget> targets;
        targets.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            targets.append(snapshot[i]);
        }
        emit trackingUpdated(targets);
    }

private:
    LidarCameraFusion fusion_;
    QTimer* timer_;
};
```

### Qt 集成注意事项

| # | 注意点 | 说明 |
|---|--------|------|
| 1 | 线程安全 | `copy_tracked_targets()` 内部 mutex 保护，可在 Qt UI 线程调用 |
| 2 | 刷新频率 | 雷达 10Hz，Qt 定时器 50ms（20Hz）够用，不要超过 10ms |
| 3 | 告警回调线程 | 回调在 fusion 线程调用，`emit` 需 `Qt::QueuedConnection` |
| 4 | 头文件依赖 | `lidar_tracking_types.h` 独立无 Qt 依赖，Qt 工程只需 include 此文件 |
| 5 | 死锁风险 | 不要在告警回调里调用 `copy_tracked_targets()` |
| 6 | 展示字段 | `distanceMeters` 预计算，`state` 枚举转文字显示 |
| 7 | 配置时机 | `configure_tracker()` / `register_warning_callback()` 在 `start()` 前调用 |
| 8 | 启停顺序 | `stop()` → `copy_tracked_targets()`（最后一次查询）→ 析构 |

---

## 通过 Qt 控制融合/跟踪启停

`fuse_data()` 是否调用由调用方代码决定，`enable_tracking()` 控制跟踪开关。两者可独立控制：

```cpp
class FusionController : public QObject {
    Q_OBJECT
public:
    // 开关
    void setFusionEnabled(bool on)  { fusionEnabled_ = on; }
    void setTrackingEnabled(bool on) { fusion_.enable_tracking(on); }

    // 每帧由 QTimer 或信号触发
    void onNewFrame(const std::vector<YoloBBox>& dets, uint64_t imageTs,
                    const LidarFrame& frame, const CameraConfig& camCfg) {
        if (!fusionEnabled_) return;

        fusion_.reset();
        fusion_.fuse_data(dets, imageTs, frame, camCfg);

        // 跟踪可独立关闭：只融合不跟踪
        if (trackingEnabled_) {
            const FusionResult& r = fusion_.result();
            fusion_.update_tracking(r, frame.points, frame.pointsCount,
                                    dets.data(), dets.size(),
                                    frame.timestampNs);
        }
    }

signals:
    void fusionResultReady(const FusionResult& r);
    void trackingUpdated(const QVector<TrackedTarget>& targets);

private:
    LidarCameraFusion fusion_;
    bool fusionEnabled_   = true;
    bool trackingEnabled_ = true;  // 可独立关闭
};
```

三种典型用法：

| 场景 | `fusionEnabled_` | `trackingEnabled_` | 效果 |
|------|:---:|:---:|------|
| 融合 + 跟踪（正常） | ✓ | ✓ | 投影→跟踪，UI 展示航迹 |
| 仅融合 | ✓ | ✗ | 只做投影分类，看 FusionResult |
| 全部暂停 | ✗ | - | 空转，不做任何处理 |

Qt UI 上用两个 `QCheckBox` 或 `QPushButton` 绑定 `setFusionEnabled()` / `setTrackingEnabled()` 即可。

---

## 可调参数

详见 `lidar_tracking_types.h` 中的 `TrackerConfig` 结构体。

| 类别 | 参数 | 默认 | 说明 |
|------|------|------|------|
| 聚类 | `clusterEpsMeters` | 0.5 | 簇内点间距上限 (m) |
| 聚类 | `minClusterPoints` | 3 | 最小簇点数，少于丢弃 |
| 滤波 | `alpha` | 0.7 | 位置平滑增益 [0,1] |
| 滤波 | `beta` | 0.3 | 速度平滑增益 [0,1] |
| 滤波 | `minHitsForVelocity` | 2 | 速度初始化需连续命中帧数 |
| 关联 | `maxAssociationDistMeters` | 2.0 | 航迹-检测匹配门限 (m) |
| 关联 | `requireClassIdMatch` | true | classId 不一致时拒绝匹配 |
| 生命周期 | `minHitsToConfirm` | 3 | 确认航迹需连续命中帧数 |
| 生命周期 | `maxTentativeMisses` | 1 | Tentative 容忍丢失帧数 |
| 生命周期 | `maxCoastingFrames` | 5 | Coasting 最大外推帧数 |
| 生命周期 | `maxTracks` | 50 | 最大同时跟踪目标数 |
| 告警 | `warningEnterDistMeters` | 3.0 | 进入告警距离 (m) |
| 告警 | `warningExitDistMeters` | 3.5 | 解除告警距离 (m) |
| 告警 | `warningCooldownNs` | 2000000000 | 告警冷却时间 (ns) |

---

## Demo

```bash
# x86 本地（算法验证，不需要雷达硬件）
mkdir -p build && cd build && cmake .. && make -j$(nproc)
./lidar_camera_fusion_demo_tracking     # 9 个场景，30+ 项检查

# 板端（接真实雷达，30 秒自动停止）
./build.sh
scp install/lidar_camera_fusion_demo_thread root@<板端IP>:/tmp/
/tmp/lidar_camera_fusion_demo_thread    # Ctrl+C 可提前退出
```

---

## 编译 & 部署

```bash
./build.sh                                  # aarch64 交叉编译
ls install/lib/liblidar_camera_fusion_lib.a   # 完整库（含跟踪+线程）
ls install/include/                            # 头文件
```

---

## 核心架构

```
YOLO (暂未接入)                SentinelLslidarer (雷达)
      │                              │
      ▼                              ▼
std::vector<YoloBBox>            LidarFrame
      │                              │
      └──────────┬───────────────────┘
                 ▼
          fuse_data()              ← 外参变换 + 投影 + bbox 分类
                 │
                 ▼
          FusionResult              ← bboxPointIndices / bboxPointCounts
                 │
                 ▼
         update_tracking()
            ├── cluster_bbox_points_()    ← bbox 内点扫描顺序 CDC 聚类
            ├── cluster_orphan_points_()  ← 孤儿 LiDAR 点聚类（仅续命，不新建）
            ├── predict_tracks_()          ← Alpha-Beta 预测（dt 三层保护）
            ├── associate_()               ← classId 门控 + 贪心 NN
            ├── apply_correction_()        ← Alpha-Beta 校正
            ├── manage_lifecycle_()        ← Tentative→Confirmed→Coasting→Deleted
            └── check_warnings_()          ← 迟滞告警 + 冷却节流
```

---
