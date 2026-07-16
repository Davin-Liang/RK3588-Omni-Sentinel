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

---

## 9. load_fusion_config_ 自动修正覆写 T 矩阵

**现象**: config.ini 中 cam1 外参设为 `T0=1,T1=0,T8=0,T9=1`，但运行时实际行为是 `cx=ly`（T1=1），导致人在画面中心投影到 u=619 右边缘。

**原因**: `load_fusion_config_()` 中有一段硬编码兜底逻辑，检测 `T1==0 && T8==0` 时覆盖为 `T1=1, T8=-1`（`cx=ly, cz=-lx`）。此逻辑将合法的正交旋转矩阵（`cx=lx, cz=ly`）误判为"未配置"。

**解决**: 删除 cam0 和 cam1 两处的自动修正代码块。外参完全由 config.ini 决定。

---

## 10. per-bbox 聚类相互污染导致 track 跳变

**现象**: 两个人在画面中时，track 位置反复跳变，ID 频繁重建。bbox A 的检测有时拿到 bbox B 的人的簇。

**原因**: `fuse_data` 的 first-hit 策略将 LiDAR 点预分配给 bbox。两个 bbox 重叠时，先检查的 bbox 抢走后者的点。per-bbox 独立聚类在此基础上进行，污染无法消除。

**解决**: 重构为全局聚类（`cluster_all_points_`）：
- 全部 LiDAR 点统一排序+CDC 聚类，不预分配给 bbox
- 每个簇质心投影到相机像素后匹配最近的 bbox（距离评分`点数×2.5/距离`）
- 每个 bbox 选评分最高的簇（最优者得，而非先到先得）
- 未被选中的簇自动变为孤儿检测

---

## 11. 孤儿不建 track + Coasting 恢复逻辑死锁

**现象**: YOLO 短暂掉帧或 bbox 数<人数时，track 在 Confirmed/Coasting 之间反复震荡（C/K/C/K...），最终积累 10 帧 miss 被删，永久丢失。

**原因**: 三个设计互锁：
1. 孤儿检测只能匹配 Coasting 的 track（不能匹配 Confirmed）
2. 孤儿匹配成功后将 track 从 Coasting 恢复为 Confirmed
3. 恢复 Confirmed 后下一帧孤儿不再匹配 → 又变 Coasting
→ 无限循环 C/K 震荡

**解决**: 
- 孤儿匹配成功后**不恢复 Confirmed**，保持 Coasting 状态（仅 bbox 检测能恢复 Confirmed）
- 放宽孤儿匹配：删除 `track.state != Coasting` 限制，孤儿可匹配任意已确认航迹
- `maxCoastingFrames` 和 `maxStaleCoastingFrames` 从 5/2 提升至 20/20
- 后续回退孤儿放宽至仅 Coasting（因孤儿匹配 Confirmed 导致幽灵 track 增殖）

---

## 12. minClusterPoints=10 导致近距目标无法检测

**现象**: 人在 0.3m 处画面框内有 80+ LiDAR 点（OSD 可见），但 tracker `det=0` 完全检测不到。

**原因**: LiDAR 打到极近距离的人体上，点云覆盖宽角度，CDC 连续扫描聚类将 80 个点打散成多个不足 10 点的小簇，全部被 `minClusterPoints` 过滤。

**解决**: `minClusterPoints` 从 10 降至 5。后续可考虑动态门限（越近越宽松）。

---

## 13. bboxIdx 硬绑定导致跨框 track 跳变

**现象**: 两个 bbox 对应两个人时，track 的 bboxIdx 每帧可能变化（簇投影像素跨过两框中线），导致 track 在关联时跳配到另一个 bbox 的检测上。

**原因**: bboxIdx 绑定采用硬约束（pass=0 仅匹配同 bboxIdx 的 track），bboxIdx 变化时 pass=0 找不到匹配，pass=1 全门限匹配可能配到错误的 track。

**解决**: 改为软约束——同 bboxIdx 全门限 1.0m，跨 bboxIdx 门限缩至 25%（0.5m）。跨框匹配需要检测和 track 位于 0.5m 以内才允许。

---

## 当前局限

1. **孤儿不创建新 track** — track 被删后永久丢失，需 YOLO 重新发现才能恢复。当 YOLO 掉帧且 track 积累超过 20 帧 miss 时发生。曾尝试放开但造成大量幽灵 track，已回退。

2. **2.5m 硬编码上限** — 簇质心距离 >2.5m 直接丢弃，无法跟踪远处目标。在 `cluster_all_points_()` 和 bbox 匹配步骤中硬编码。

3. **每 bbox 最多一个检测** — 当 YOLO bbox 数 < 人数时，多余的人只能孤儿续命，不能获得独立 track。

4. **投影和聚类分离** — OSD 画面上的点云来自 `fuse_data()` 投影，tracker 的全局聚类独立运行。画面上能看到点 ≠ tracker 能检测到目标。

5. **CDC 聚类对极近距目标(≤0.3m)碎片化** — 点云覆盖宽角度，连续扫描聚类将点打散为多个小簇。

6. **config.ini 被 QSettings 二进制序列化(@Variant)覆盖** — 手动编辑的文本值被程序 `save_fusion_config_()` 重写为二进制格式，后续手动编辑需注意。

---

## 14. 停止融合后重新启动无法创建 track

**现象**: 第一次启用融合跟踪正常，关闭后再启用时 `track=0` 持续，bbox 认领正确但没有任何航迹被创建。

**原因**: `associate_()` 中 `detAssigned_[]` 数组在 `reset()` 和每帧开头都没有清零。第一次跟踪时构造函数 memset 了全内存，正常。第二次启动时 `reset()` 只清 track 相关数组，没碰 `detAssigned_`。数组中旧帧的 `true` 值残留，所有新的 detection 在"创建新 track"循环中被 `if (detAssigned_[j]) continue` 跳过。

**解决**: 在 `associate_()` 入口添加 `memset(detAssigned_, 0, sizeof(detAssigned_))`。

---

## 15. 同一 LiDAR 帧被 tracker 重复处理 10 次

**现象**: 融合循环 10ms 一次，LiDAR 10Hz（100ms 一帧）。同一帧点云被 DBSCAN 聚类 10 次：
- `clusterPersistenceFrames=2` 实际上只等了 20ms 而非 2 个真实 LiDAR 帧
- Alpha-Beta 预测 dt=0 → 被 clamp 到 defaultDtSec → 虚假预测累积

**原因**: `get_closest_frame()` 在每个循环迭代返回相同的 LiDAR 帧（时间戳相同）。tracker 没有检查时间戳是否变化，每 10ms 执行一次完整管线。

**解决**: (1) tracker `update()` 入口检查 `timestampNs == lastLidarTimestampNs_`，相同则直接 return。(2) 融合循环改为先取 LiDAR 帧再做去重，只有新帧到达才 drain YOLO 队列。

---

## 16. YOLO 结果队列 FIFO 导致慢相机数据被丢弃

**现象**: 两路 YOLO 帧率不同（~30fps / ~15fps），慢相机目标几乎无法被跟踪。`yolo=N` 频繁出现。

**原因**: (1) `ThreadSafeQueue` 用 `std::queue`，`try_pop` 返回**最旧**数据。(2) 融合循环每 10ms 无条件 pop YOLO 队列，但 tracker 因去重 9/10 次跳过——中间迭代把 YOLO 结果白白吃掉。(3) 慢相机队列积压慢，等到新 LiDAR 帧到达时队列已空。

**解决**: (1) 融合循环改为先等 LiDAR 新帧，再 drain YOLO 队列。(2) `ThreadSafeQueue` 新增 `drain_latest()` 方法：清空队列只保留最后一个。(3) 每路相机 YOLO drain 只在 LiDAR 新帧到达时执行。

---

## 17. 设备晃动后聚类永久消失

**现象**: 稍微移动装置（手遮雷达等），聚类圆剧减至 0，且之后无论怎么放都不恢复。

**原因**: `persist_clusters_()` 中 `clusterHistoryCount_` 只增不减。晃动产生大量噪声 cluster 记录填满 32 个槽位后，新 cluster 被 `if (clusterHistoryCount_ >= kMaxClusters) continue` 拦截，永远无法创建历史记录 → 无法通过 persistence 确认 → 无 detection。

**解决**: (1) 新 cluster 优先复用已失效的槽位 (`consecutiveFrames == 0`)。(2) 槽位未满时追加。 (3) 全满时淘汰最老的记录（`lastLidarTs` 最小）。

---

## 18. 一个 cluster 被多个 bbox 同时认领

**现象**: 有两个人但只有一个 track。日志显示其中一个 bbox 持续认领另一个 bbox 对应的 cluster。OSD 上 bbox[0] 有 80 个 LiDAR 点但只认领到 6 点的噪声簇。

**原因**: `bbox_claim_()` 中每 bbox 独立选最佳 cluster，没有互斥——cluster A 可以同时赢 bbox[0] 和 bbox[1]。一个 cluster 占两个 bbox，另一个 bbox 对应的真人 cluster 被挤出。

**解决**: (1) 冲突检测：统计每个 cluster 被几个 bbox 选中。被多个 bbox 选中的 cluster 只保留得分最高的 bbox。(2) 补选：被抢走 cluster 的 bbox 从剩余未认领 cluster 中重新选择。(3) 新增 `minBboxClaimPoints` 参数：点数 <10 的 cluster 不参与 bbox 竞争（过滤远处 6-7 点噪声簇）。

---

## 19. DBSCAN cluster 质心跳动导致远距目标永远不确认

**现象**: 远处目标（>2m）YOLO 持续检测到但永远不出 track。近距目标跟踪正常。

**原因**: `persist_clusters_()` 中历史匹配门限用了 `dbscanEpsMeters=0.5m`。远处目标 LiDAR 点稀疏，cluster 质心帧间跳动容易超过 0.5m → 每帧被当作新 cluster → `consecutiveFrames` 永远停在 1 → 永远不确认 → 不参与 bbox 认领。

**解决**: 历史匹配门限改为 `max(dbscanEpsMeters * 2.5, 1.0m)` = 默认 1.25m，足以容忍质心跳动但小于两目标间距（>2m），不会误匹配。

---

## 20. FusionTracking 丢一帧立即进入 Lost

**现象**: YOLO 偶尔掉一帧（bbox 数量从 2→1→2），对应 track 在 FusionTracking → Lost → FusionTracking 之间反复跳变。

**原因**: `manage_lifecycle_()` 中 FusionTracking/PureRadarTracking 的丢失条件为 `misses > 0`——丢第一帧就立即进入 Lost。YOLO 单帧漏检（常见于边沿目标、遮挡）就触发状态切换。

**解决**: 新增 `maxFusionMisses` 参数（默认 2），改为 `misses > maxFusionMisses` 才进入 Lost。给 YOLO 3 帧缓冲（~300ms）容忍间歇性漏检。

---

## 当前局限（v2 更新）

1. **孤儿不创建新 track** — 纯雷达无法独立发现新目标，需 YOLO 首次探测。当所有 track 因雷达全遮挡而 Deleted 后，系统依赖 YOLO 恢复。

2. **每 bbox 最多一个检测** — 当 YOLO bbox 数 < 人数时，多余的人只能孤儿续命。

3. **投影和跟踪聚类分离** — OSD 画面点来自 `fuse_data()` 独立投影，tracker 聚类独立运行。画面上有大量点 ≠ tracker 能检测到目标（如 bbox 80 个 OSD 点但 DBSCAN 只产出 6 点的噪声簇）。

4. **DBSCAN O(n²) 复杂度** — 540 点可忽略（~1.5M FLOPs），升级高线束雷达需空间索引加速。

---

## 21. kMaxLidarPoints=540 与 lslidarer 不同步导致 copy_slot 越界

**现象**: ASan 报 `heap-buffer-overflow`，`memcpy` 写入 11148 bytes 到 6480-byte 区域。崩溃在 `RingBuffer::copy_slot()` → `SentinelLslidarer::get_latest_frame()` → `LidarCameraFusion::fusion_thread_()`。启动融合后立即触发。

**原因**: lslidarer 的 `kPointsPerSweep` 从 540 扩容到 1200 后，`LidarCameraFusion` 的帧缓冲区 `lidarPointsBuf_` 仍按 `kMaxLidarPoints = 540` 分配（6480 bytes = 540 × 12）。`copy_slot()` 将新帧的点数据（~929 个点 = 11148 bytes）`memcpy` 到 540 点的缓冲区 → 越界。

**解决**: `lidar_camera_fusion.h` 和 `lidar_target_tracker.h` 的 `kMaxLidarPoints` 从 540 同步更新为 1200。三个组件（lslidarer / fusion / tracker）共用同一个点容量常量，任意一方改动必须同步。

5. **YOLO 检测到但 LiDAR 看不到的目标无法跟踪** — 单线雷达扫描面有限，超出扫描面的目标即使 YOLO 检测到也无法形成 LiDAR cluster。
