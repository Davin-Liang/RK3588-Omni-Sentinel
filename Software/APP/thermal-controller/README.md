# thermal-controller — RK3588 温控调频组件

✨ 基于 sysfs 的 RK3588 用户态温控调频库，通过配置化 4 级策略主动限制 CPU 和 NPU 频率上限。

## 功能概述

- 读取 RK3588 soc-thermal 温度 + CPU/NPU 当前频率
- 4 级温度梯度策略（Normal / Warm / Hot / Critical）含回滞
- 控制 CPU A55 (policy0) + CPU A76 (policy4/policy6) + NPU 频率上限
- 不接管 governor（保持 schedutil / rknpu_ondemand），只调 max_freq
- 启动时自动恢复全速，退出时可配是否恢复
- 通过 config.ini [Thermal] 节配置，重启生效

## 管控范围

| 设备 | sysfs 节点 | 控制方式 |
|------|-----------|---------|
| CPU A55 (核 0-3) | `/sys/.../policy0/scaling_max_freq` | 写上限 |
| CPU A76 (核 4-5) | `/sys/.../policy4/scaling_max_freq` | 写上限 |
| CPU A76 (核 6-7) | `/sys/.../policy6/scaling_max_freq` | 写上限 |
| NPU | `/sys/class/devfreq/fdab0000.npu/max_freq` | 写上限 |

## 构建

```bash
cd thermal-controller && ./build.sh
```

## 依赖

无外部依赖，仅需 C++14 + POSIX 文件 I/O。
