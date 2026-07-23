# IMU-only EIS 集成说明

本版本 SentinelQT 防抖功能只保留 ICM45686 IMU-only 电子防抖链路，不再启用视觉 LK/RANSAC EIS，也不再使用 IMU assist 自适应视觉参数。

## 运行链路

```text
ICM45686 驱动 /dev/icm45686
  -> Icm45686Reader 后台采样
  -> EisStabilizer 计算 IMU-only offsetX/offsetY
  -> SentinelVisioner::set_eis_offset_callback
  -> RGA crop/resize 应用 offset
  -> 预览/推流/录像输出
```

## 控制入口

- Qt：相机防抖按钮。
- Web：`POST /api/v1/cam/{0,1}/eis/start` 与 `POST /api/v1/cam/{0,1}/eis/stop`。

## 配置入口

`config.ini` 中 `[EIS]` 只保留 IMU-only 参数，包括 IMU 设备路径、采样率、量程、相机内参、IMU 到机体系映射、机体系到相机系外参、最大补偿角、最大像素偏移和平滑时间常数。

## 调试建议

1. 先确认 `/dev/icm45686` 存在。
2. 打开防抖后观察日志 `[SentinelQT] IMU-only EIS initialized` 和 `[SentinelQT] cam X IMU-only EIS enabled`。
3. 开启 `eisRecordDebug=true` 时，可通过 streamer 输出 raw/eis 对照视频，观察防抖方向是否正确。
