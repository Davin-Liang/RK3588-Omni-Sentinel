# thermal-controller BUG_RECORD

## 1. sysfs 写入失败不重试导致频率未生效

**现象**: tick() 中 write_max_freq_() 返回后频率未变化

**原因**: fopen(path, "w") 失败（权限不足或节点不存在）时静默返回

**解决**: 失败时打印 `strerror(errno)` 到 stderr，下一个周期自动重试。write 调用本身在 fopen 成功的前提下几乎不会失败（内核 sysfs 写操作同步）

---

## 2. 频率默认值不在可用列表导致刷屏

**现象**: 进入 Warm 后每 2 秒重复打印同一频率写入日志：
```
[Thermal] CPU little max_freq: 1416000 -> 1400000
[Thermal] CPU little max_freq: 1416000 -> 1400000
...
```

**原因**: `cpuLittleWarm=1400000` 不在 policy0 A55 的可用频率列表中（最近值为 1416000）。内核将 `scaling_max_freq` 写入值自动 clamp 到最近可用频率。下一轮 `write_max_freq_()` 读回 1416000，与目标 1400000 比较不等，又写一遍——死循环。

**解决**: 两方面修复：
1. `evaluate_and_apply_()` 中 4 个 `write_max_freq_()` 调用移入等级变化守卫内（`level_ != prevLevel || tickCount_ == cfg_.intervalSec`），不再每周期无条件写
2. 修正默认值到合法频率：`cpuLittleWarm 1400000→1416000`、`cpuLittleHot 1000000→1008000`、`cpuBigCritical 800000→816000`
