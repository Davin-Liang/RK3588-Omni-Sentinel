# BUG_RECORD — lidar-camera-fusion 问题记录

## 1. 全图虚构 bbox 导致墙壁抢夺航迹

**现象**: 板端测试时，人的航迹 `#1` 从 1.2m 跳到 3.5m，远处工位的雷达簇取代了人的航迹。

**原因**: 虚构 bbox 覆盖全图或过宽区域，远处物体的雷达簇和人体的雷达簇投影到同一 bbox 内。聚类评分按点数+距离加权，远处物体点数多时获胜，航迹被"抢走"。

**解决**: 
- 收窄虚构 bbox 范围，只覆盖人体预期出现的投影区域
- 核心聚类函数中增加距离过滤（硬编码 `dist > 2.5f` 排除远处簇）
- YOLO 就绪后：classId 门控 + 动态 bbox 彻底解决此问题

---

## 2. bbox 太窄导致人体边缘点被截断

**现象**: 板端测试时 `tracks:0` 频繁出现，人的航迹间歇性丢失。

**原因**: 虚构 bbox 收窄至 `[330, 410)` 后，人在 1.1m 处的激光点投影到 `u∈[330, 412]`，右边界点被 bbox 切掉，有效簇点数不足 `minClusterPoints`，聚类失败。

**解决**: 放宽 bbox 至 `[280, 460)`，配合距离过滤排除远处物体。

---

## 3. 关联门限太小导致往返走动时 ID 频繁新建

**现象**: 人在 1.2m 和 3.5m 两个位置之间往返走动，每次移动到另一边时 ID 从 `#1` 暴涨到 `#52`。

**原因**: 两个位置相距约 2.2m，超过默认关联门限 `maxAssociationDistMeters=2.0m`。航迹在移动过程中匹配不上另一端的检测，旧航迹被删，新航迹创建。

**解决**: 将 `maxAssociationDistMeters` 调至 3.0m。YOLO 就绪后恢复 2.0m（动态 bbox 限定了空间范围，不需要宽门限）。

---

## 4. 孤儿检测创建新航迹污染跟踪结果

**现象**: 测试过程中出现大量短暂的 Tentative 航迹（如 `#5`、`#8`），干扰跟踪输出。

**原因**: 孤儿聚类产出的检测在未匹配到任何已有航迹时，错误地创建了新的 Tentative 航迹。孤儿检测来自无类别信息的 LiDAR 簇，不应该有权创建新航迹。

**解决**: 在 `associate_()` 的"未匹配检测→创建新航迹"阶段，跳过 `isOrphan=true` 的检测。孤儿检测仅用于续命已有的 coasting 航迹。

---

## 5. 航迹 `consecutiveMisses` 判断用 `>` 导致删除延迟

**现象**: 目标消失 5 帧后航迹未删除，需要 6 帧才触发。

**原因**: Coasting 删除条件为 `consecutiveMisses > maxCoastingFrames`。默认 `maxCoastingFrames=5`，第 5 次丢失时 `consecutiveMisses=5` 不满足 `> 5`。

**解决**: 改为 `>=`，第 5 帧丢失即删除。

---

## 6. stale coasting 判断块缺删除语句

**现象**: dt 过大场景下 coasting 航迹没有被加速删除。

**原因**: `manage_lifecycle_()` 中 stale coasting 判断块的 body 只有注释，缺少 `track.state = TrackState::Deleted` 语句。

**解决**: 补上删除语句，同时将硬编码的帧数提取为 `maxStaleCoastingFrames` 配置参数。

---

## 7. `demo_tracking.cpp` 测试间 tracker 状态不重置

**现象**: Test 1 的航迹残留到 Test 2，导致 Test 2 的 `track confirmed` 检查失败。

**原因**: `demo_tracking.cpp` 中所有测试共享同一个 `LidarCameraFusion` 实例，但测试之间未重置 tracker 状态。

**解决**: 在 `LidarTargetTracker` 中新增 `reset()` 方法，通过 `LidarCameraFusion::reset_tracking()` 暴露，在每个测试开头调用。

---

## 8. cross-compiler 与 x86 编译混用

**现象**: x86 编译后的 `demo_tracking` 报 `aarch64-binfmt-P: Could not open '/lib/ld-linux-aarch64.so.1'`。

**原因**: build 目录残留了 aarch64 交叉编译的 CMake 缓存，x86 编译器尝试运行交叉编译产物。

**解决**: `rm -rf build` 后再 `cmake ..` 重新配置。`build_local.sh` 与 `build.sh` 应使用不同的 build 目录，或确保 `build.sh` 每次都重新 cmake。
