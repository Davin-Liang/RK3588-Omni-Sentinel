# thermal-controller 实现文档

## 架构总览

```
ThermalController::tick()  [1s 调用]
  ├── read_sensors_()      读取 temp + cur_freq → 缓存
  └── evaluate_and_apply_()  [每 intervalSec 执行]
        ├── 策略评估 (4级回滞)
        └── write_max_freq_() x4 (A55 + A76x2 + NPU)
```

## 线程模型

ThermalController 无独立线程。由调用方（SentinelQT `clockTimer_`）的 1 秒定时器驱动 `tick()`。所有 sysfs 操作在调用线程（Qt 主线程）中同步执行，无锁。

## 核心数据流

1. `tick()` 每次调用内置 `read_sensors_()`：打开 4 个 sysfs 文件分别读 temp + 3 个 cur_freq，关闭后缓存
2. `tickCount_` 递增，若 `tickCount_ % intervalSec == 0` 且 `enabled == true`，执行 `evaluate_and_apply_()`
3. `evaluate_and_apply_()` 根据 tempC_ 和当前 level_ 进行回滞判断，确定新等级
4. 若等级变化（或初次），写入 4 个 max_freq 节点。写入前对比当前 max_freq，相同则跳过

## 配置校验

`validate_config_()` 在构造函数中执行：
- 温度阈值链：warmRecover < warmThreshold < hotRecover < hotThreshold < critRecover < critThreshold
- 频率单调性：Normal >= Warm >= Hot >= Critical (各级别)

校验失败打印 stderr 警告，回退到默认值，不阻止程序启动。
