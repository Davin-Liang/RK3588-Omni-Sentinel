# Robot Alarm GPIO Web Control Notes

本版本在原有 gpio97 语音报警自动拉低逻辑基础上，新增 Web 端手动控制：

- 默认状态：gpio97 输出高电平，机械臂状态显示为“正常运行状态”。
- 语音报警或融合告警触发：gpio97 拉低并锁存，机械臂状态显示为“停止工作状态”。
- Web 按钮“紧急急停”：手动拉低 gpio97，并锁存为停止状态。
- Web 按钮“恢复机械臂状态”：手动拉高 gpio97，并解除锁存，机械臂状态显示为“正常运行状态”。

新增后端接口：

- `GET /api/v1/robot-alarm/status`：读取当前 GPIO/机械臂状态。
- `POST /api/v1/robot-alarm/low`：拉低 gpio97，触发急停。
- `POST /api/v1/robot-alarm/high`：拉高 gpio97，恢复机械臂状态。

GPIO：P26-32 / GPIO3_A1 / Linux sysfs gpio97。
