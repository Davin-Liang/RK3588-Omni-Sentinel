# thermal-controller BUG_RECORD

## 1. sysfs 写入失败不重试导致频率未生效

**现象**: tick() 中 write_max_freq_() 返回后频率未变化

**原因**: fopen(path, "w") 失败（权限不足或节点不存在）时静默返回

**解决**: 失败时打印 `strerror(errno)` 到 stderr，下一个周期自动重试。write 调用本身在 fopen 成功的前提下几乎不会失败（内核 sysfs 写操作同步）
