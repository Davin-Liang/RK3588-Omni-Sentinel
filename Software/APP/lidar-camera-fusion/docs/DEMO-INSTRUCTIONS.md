# LidarCameraFusion — Demo 运行指南

## 1. x86 本地测试（无需雷达硬件）

纯算法验证，不依赖真实雷达驱动和交叉编译器。

```bash
cd lidar-camera-fusion
./build_local.sh
./build/lidar_camera_fusion_demo_tracking
```

### 测试覆盖（9 个场景）

| # | 测试 | 验证内容 | 关键检查 |
|---|------|---------|---------|
| 1 | 静止目标 10 帧 | 位置稳定、速度收敛 | pos 误差 <5cm, vel≈0, state=Confirmed |
| 2 | 匀速运动 5m/s | 速度估计准确 | velY 误差 <0.5, state=Confirmed |
| 3 | 两目标交叉 | 关联稳定性 | 交叉后航迹未丢失，速度方向正确 |
| 4 | 目标出现/消失 | 生命周期管理 | Tentative 消失即删，Confirmed → Coasting → Deleted |
| 5 | 近距离告警 | 迟滞+冷却 | 安全距离不告警，进入告警区触发，持续冷却 |
| 6 | dt 异常 | 保护逻辑 | dt=0 不崩溃，dt=2s 跳过预测 |
| 7 | 空输入 | 鲁棒性 | 0 bbox + 0 points 不崩溃 |
| 8 | 边界输入 | 边界检查 | bboxCount 不一致打印警告，越界索引跳过 |
| 9 | 孤儿点续命 | 相机丢失后雷达维持 | Confirmed→Coasting（孤儿匹配）→Confirmed（恢复） |

### 期望输出

```
=== Results: 37 passed, 0 failed ===
```

---

## 2. 板端实跑测试（需要真实雷达）

### 硬件前提

- RK3588 板端
- N10Plus 激光雷达，串口 `/dev/sentinel_lidar`
- 交叉编译工具链

### 编译 & 部署

```bash
# 开发机上交叉编译
cd lidar-camera-fusion
./build.sh

# 拷贝到板端
scp install/lidar_camera_fusion_demo_thread root@<板端IP>:/tmp/
```

### 运行

```bash
# 板端
chmod +x /tmp/lidar_camera_fusion_demo_thread
/tmp/lidar_camera_fusion_demo_thread
```

程序运行 30 秒后自动停止。`Ctrl+C` 可提前退出。

### 运行中输出

```
[DEMO] lidar started
[DEMO] tracking enabled (warning at 0.5m)
[DEMO] fusion thread started, running 30 seconds...

[LidarCameraFusion] 100 iters | bboxes:1 matched:16 ... tracks:1
  #1 | pos=(-1.15,0.14) vel=(-0.01,0.00) dist=1.15m Confirmed

[WARNING] target 1 class=0 dist=0.44m pos=(-0.44,0.04)   ← 人走近 0.5m 告警
```

### 测试步骤

| 步骤 | 操作 | 预期 |
|------|------|------|
| 1 | 站在雷达正后方约 1.2m 处，静止 5 秒 | `#1` 创建 → Confirmed |
| 2 | 向侧面慢慢横跨（约 0.5m） | 航迹位置跟随移动，ID 不变 |
| 3 | 逐步走近雷达 | 距离递减，<0.5m 触发 `[WARNING]` |
| 4 | 退后 | 距离递增，>0.8m 告警解除 |
| 5 | 走到远处（>2.5m） | 航迹消失（距离过滤排除） |
| 6 | Ctrl+C 停止 | 打印最终快照，航迹信息 |

### 调整参数

编辑 `demo_thread.cpp` 中的 `trackerCfg`，重新编译：

```cpp
TrackerConfig trackerCfg;
trackerCfg.bboxAssocMaxDistMeters = 3.0f;   // 调大匹配距离
trackerCfg.warningEnterDistMeters   = 0.5f;   // 告警阈值
trackerCfg.maxLostFrames        = 20;     // 延长 coasting 记忆
trackerCfg.dbscanEpsMeters         = 1.0f;   // 放宽聚类距离
```

### 已知局限（虚构框阶段）

| 问题 | 表现 | 应对 |
|------|------|------|
| 人必须站在特定区域 | 虚构 bbox 固定，人体必须在其投影范围内 | 调整 bbox 坐标或使用 `demo_tracking` x86 测试算法 |
| 远处物体会干扰 | 工位/墙壁的雷达簇 >2.5m 被距离过滤硬编码排除 | YOLO 就绪后移除距离过滤 |
| ID 可能跳变 | 人走到 >3m 外再回来，匹配不上旧航迹 | 调大 `bboxAssocMaxDistMeters` |
