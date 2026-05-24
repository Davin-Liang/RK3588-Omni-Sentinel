# Bug 记录

## 1. 外参矩阵 T[5] 误设为 ly 系数

**现象**: 所有雷达点投影后 v 坐标超界，`outOfImageCount` 等于总点数，融合结果为空。

**原因**: `make_xy_to_xz_T()` 中设了 `T[5] = 1.0f`。4×4 行主序矩阵中 `T[5]` = row1 col1 = ly 系数，导致 `cy = ly`。单线雷达 `ly ∈ [3, 4]`，投影后 `v = fy*ly/ly + cy = 640`，超出 480 像素图像高度。

`z=0` 的项对应 row1 col2 = `T[6]`，而非 `T[5]`。且 z=0 时该项恒为 0，不需设任何值。

**修复**: 删除 `T[5] = 1.0f`。row1 全为零即可（`cy ≡ 0`）。

---

## 2. 相机外参朝向多次调整

**现象**: 板端联调时正后方（~180° 方位角）的雷达点始终匹配不到。

**过程**:
- 初版外参 `cz = ly`（朝 +Y）：后方点 `ly ≈ 0.12` 时 cz 太小，`|lx/ly| ≈ 6`，投影 u 飞出图像。
- 试 `cz = -ly`（朝 -Y）：正后方点 `ly > 0` 被判定为相机后方。
- 最终发现雷达坐标约定：`x = d·cos(az), y = d·sin(az)`，正后方 az≈180° 对应 `lx ≈ -1.0, ly ≈ 0.15`。相机应朝 -X 方向。

**修复**: 外参改为 `cx = ly, cz = -lx`（`T[1]=1, T[8]=-1`）。

---

## 3. 雷达盲区屏蔽正后方

**现象**: 雷达 demo 的 rear 区域方位角只到 173°，正后方 180° 无点。

**原因**: `LidarConfig` 默认 `angleDisableMin = 9000` (90°), `angleDisableMax = 24000` (240°)，正后方 180° 在盲区内。

**修复**: demo_thread 中设置 `angleDisableMin = 0; angleDisableMax = 0`。

---

## 4. udev 规则序列号不匹配

**现象**: 板端执行 `lidar_udev.sh` 后 `/dev/sentinel_lidar` 软链接未创建。

**原因**: CH9102 芯片的 udev 规则写死 `ATTRS{serial}=="0001"`，实际设备序列号为 `5B8E670822`。

**修复**: 删除 CH9102 规则中的串口号限制，仅匹配 vendor + product。同时修复脚本 `\r` 换行符问题。

---

## 5. `std::memset(result_, ...)` 缺少取地址符

**现象**: aarch64 交叉编译器报错 `cannot convert 'FusionResult' to 'void*'`。

**原因**: `result_` 是值类型 (`FusionResult`)，`memset` 需要 `void*` 指针。

**修复**: 改为 `std::memset(&result_, 0, sizeof(result_))`。

---

## 6. build.sh 换行符导致板端脚本错误

**现象**: 板端执行 `sudo bash build.sh` 报 `set: invalid option`、`$'\r': command not found`。

**原因**: Windows 编辑的文件带 CRLF 换行符。

**修复**: 板端执行 `dos2unix build.sh` 或直接手写规则。

---

## 7. 析构函数调 stop() 链接失败

**现象**: x86 本地编译 demo link 报 `undefined reference to LidarCameraFusion::stop()`。

**原因**: `stop()` 定义在 `lidar_camera_fusion_thread.cpp`，而 demo 只链接 `lidar_camera_fusion_core`（不含线程文件）。

**修复**: 将 `stop()` 移至 `lidar_camera_fusion.cpp`（核心文件），线程文件不再重复定义。

---

## 8. demo 验证逻辑未计入背景点

**现象**: `demo_single` 断言 `total == 100` 失败，实际为 80。

**原因**: 验证公式 `total = matched + behind + outOfImage` 未包含背景点（投影在图像内但未落入任何 bbox 的点），20 个背景点未被统计。

**修复**: 改为 `classified == 80`，加注释说明剩余 20 为背景点。
