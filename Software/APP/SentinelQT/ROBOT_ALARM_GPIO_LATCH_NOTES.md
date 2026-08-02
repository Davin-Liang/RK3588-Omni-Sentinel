# Robot Alarm GPIO Latch Notes

本次修改将机械臂急停 GPIO 改为“报警锁存低电平”策略。

- 默认状态：程序启动初始化 gpio97 为高电平。
- 报警触发：调用 `voice::playDangerWarning()` 后调用 `set_robot_alarm_gpio_(true)`，gpio97 拉低。
- 锁存策略：一旦拉低，运行期间所有 `set_robot_alarm_gpio_(false)` 请求都会被忽略，GPIO 保持低电平。
- 恢复方式：由 STM32/机械臂控制板复位，或重启 SentinelQT 后重新初始化为高电平。

修改文件：

1. `widget.h`
   - 新增 `robotAlarmGpioLatched_` 成员变量。

2. `widget.cpp`
   - 修改 `set_robot_alarm_gpio_(bool active)`，加入锁存逻辑。
   - 保留原有融合关闭、自动回溯关闭、程序退出处的 `set_robot_alarm_gpio_(false)` 调用，但锁存后这些调用不会恢复高电平。

GPIO 对应关系：

- RK3588 P26-32 / GPIO3_A1 / Linux gpio97
- 非报警：高电平
- 报警后：低电平，保持锁存
