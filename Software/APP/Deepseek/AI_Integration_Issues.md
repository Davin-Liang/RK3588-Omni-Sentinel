# DeepSeek 1.5B 大模型集成到 SentinelQT — 问题记录

## 时间线

2026-07-04 ~ 2026-07-06，将 DeepSeek-R1-Distill-Qwen-1.5B 推理能力以 `deepseek_ai_lib` 静态库形式集成到 SentinelQT，过程中遇到以下问题。

---

## 1. Segfault on startup (1): 缺少 CMAKE_AUTOMOC

**现象**: 程序启动后立即 `Segmentation fault`，无任何 AI 相关日志。

**原因**: `Deepseek/CMakeLists.txt` 作为独立静态库，未设置 `CMAKE_AUTOMOC ON`。`AIReportWorker` 使用了 `Q_OBJECT` 宏和 `signals:`/`slots:` 关键字，但 MOC (Meta-Object Compiler) 未处理这些代码，导致 `emit error(...)` 时信号内部索引无效，触发未定义行为。

**解决**: 在 `Deepseek/CMakeLists.txt` 中添加 `set(CMAKE_AUTOMOC ON)` 和 `set(CMAKE_AUTORCC ON)`。

**教训**: 任何包含 `Q_OBJECT` 的 CMake 子项目都需要单独开启 AUTOMOC。

---

## 2. Segfault on startup (2): 直接链接 librkllmrt.so 导致预加载崩溃

**现象**: 开启 AUTOMOC 后仍然 `Segmentation fault`。`strace` 显示崩溃发生在加载共享库阶段，程序甚至没有执行到 `main()` 中 AI 初始化代码。

**原因**: 通过 `target_link_libraries(... librkllmrt.so)` 直接链接，动态链接器在进程启动时加载 `librkllmrt.so` 及其依赖 `libgomp.so.1`（GCC OpenMP）。该 `.so` 的 `.init_array` 或静态构造器在非常早的阶段执行，与 buildroot 工具链编译的 SentinelQT 存在某种不兼容，触发崩溃。

**解决**: 改为运行时 `dlopen` 加载：
- CMakeLists.txt 不再链接 `librkllmrt.so`，改为链接 `${CMAKE_DL_LIBS}`（libdl）
- `deepseek_inference.cpp` 中 `LoadRkllmSymbols_()` 用 `dlopen` 搜索多个路径加载，用 `dlsym` 逐个解析函数指针
- 加载失败时返回 false，程序继续运行（只是 AI 功能不可用）

**教训**: 第三方 `.so` 库如果与主程序工具链不一致，应优先使用 `dlopen` 延迟加载，避免在动态链接器阶段崩溃。

---

## 3. Segfault on startup (3): `update_ai_status_snapshot_()` 竞态条件

**现象**: `strace` 显示 `SIGSEGV {si_addr=NULL}` 发生在读取 `/sys/class/thermal/thermal_zone0/temp` 和 `/proc/stat` 之后，即 `update_ai_status_snapshot_()` 中。

**原因**: `QTimer::singleShot(1000, ...)` 延迟初始化 AI Worker 与 `clockTimer_->start(1000)` 首次 tick 同时触发。如果 clockTimer_ 先于 AI Worker 初始化 tick，`aiReportWorker_` 还是 nullptr；但如果某些构造时序下 Worker 已被 `new` 创建（指针非空）但尚未完成 `moveToThread` + `connect` + `thread->start()`，主线程调用 `aiReportWorker_->updateStatus()` 会访问不完整初始化的对象，崩溃。

**解决**:
- 添加 `std::atomic<bool> aiWorkerReady_{false}` 标志
- 延迟初始化改为 `QTimer::singleShot(500, ...)`（早于 1000ms 首 tick）
- Worker 完全初始化后用 `aiWorkerReady_.store(true)` 标记就绪
- `update_ai_status_snapshot_()` 检查 `aiWorkerReady_.load()` 而非 `aiReportWorker_ != nullptr`

**教训**: 延迟初始化与定时回调之间存在竞态，不能只检查指针非空，必须用原子标志确保对象完全可用。

---

## 4. Segfault on startup (4): `rkllm_createDefaultParam()` ABI 不匹配

**现象**: 系统正常启动，AI 模型开始加载（`[DeepSeekInference] loaded librkllmrt.so from: librkllmrt.so`），随后立即 `Segmentation fault`。

**原因**: `rkllm_createDefaultParam()` 在 aarch64 上返回大结构体（`RKLLMParam` 约 204 字节），通过 x8 寄存器传递隐藏指针。函数指针类型被声明为 `typedef void* (*func_t)()`（返回 `void*`），编译器不设置 x8，导致函数内部向垃圾地址写入 → segfault。

**解决**: 使用内联汇编正确设置 x8：
```cpp
alignas(16) unsigned char paramBuf[256];
memset(paramBuf, 0, sizeof(paramBuf));
#ifdef __aarch64__
    __asm__ volatile("mov x8, %0" :: "r"(paramBuf) : "x8");
#endif
p_rkllm_createDefaultParam();
```

**教训**: `dlsym` 获取的函数指针，如果实际返回大结构体，必须按目标平台的 ABI 调用约定设置 x8（aarch64）/ 隐藏参数。不能随意声明返回类型。

---

## 5. 推理报错: `prompt gather than max context (prompt: 185 tokens, max context: 60)`

**现象**: AI 模型加载成功，但推理时报 `max context: 60`，实际设置是 `maxContextLen=2048`。

**原因**: 手动猜测的 `RKLLMParam` 结构体布局与真实 `rkllm.h` 不一致。`max_context_len` 在真实结构体中位于**偏移 8**（第 2 个字段），我错误地放在 offset 36。同时 `max_new_tokens` 在真实布局中是 offset 12，我在 offset 32。写入的值错位，导致 `max_context_len` 实际读取的是其他字段的值（60）。

**真实布局**（来自 `rkllm.h`）：
```
offset  0: model_path        (char*, 8B)
offset  8: max_context_len   (int32, 4B)   ← 关键：和 demo 代码顺序不同！
offset 12: max_new_tokens    (int32, 4B)
offset 16: top_k             (int32, 4B)
offset 20: top_p             (float, 4B)
offset 24: temperature       (float, 4B)
offset 28: repeat_penalty    (float, 4B)
offset 32: frequency_penalty (float, 4B)
offset 36: presence_penalty  (float, 4B)
offset 40: mirostat          (int32, 4B)
...
offset 52: skip_special_token (bool, 1B)
offset 88: extend_param.base_domain_id (int32, 4B)
// extend_param.reserved[112] → 总大小 204 字节
```

**解决**: 结合步骤 4 的 x8 修复，先调 `rkllm_createDefaultParam()` 获取正确默认值，再按真实偏移量覆盖需要的字段。

**教训**: 没有头文件时绝不要猜测结构体布局。优先通过 `dlopen` + 正确 ABI 调用获取默认值，再覆盖已知字段。

---

## 6. 枚举值 `LLMCallState` 不匹配

**现象**: 推理完成后 callback 可能未正确触发 `onFinish_()`。

**原因**: 真实 `rkllm.h` 中枚举值为：
```c
RKLLM_RUN_NORMAL  = 0
RKLLM_RUN_WAITING = 1  // 等待完整 UTF-8 字符
RKLLM_RUN_FINISH  = 2
RKLLM_RUN_ERROR   = 3
```
我错误地使用 `FINISH=1, ERROR=2`。

**解决**: 使用正确枚举值，并将 `RKLLM_RUN_WAITING`（等待多字节 UTF-8 拼接）也当作 token 输出处理。

---

## 7. OOM Killer: 内存不足

**现象**: 程序正常运行一段时间后被 `Killed`。

**原因**: `config.ini` 中 `ringBufferSlots=150`，每帧 1920×1080 NV12 约 3 MB，2 路相机共 150×3MB×2 = **890 MB**。加上 DeepSeek 1.5B W8A8 模型约 **1.5 GB**，推理时 KV Cache 额外占用约 500 MB，总内存需求超过 RK3588 的 3.7 GB 可用 DDR，触发 Linux OOM Killer。

**解决**: `ringBufferSlots=150` → `30`，节省 712 MB：
```
修改前: 150×3MB×2 = 890 MB
修改后:  30×3MB×2 = 178 MB
节省: 712 MB
```

**教训**: 在嵌入式平台（4 GB RAM）上同时运行摄像头管线和大模型推理，必须严格控制缓冲区内存占用。

---

## 8. 屏幕显示空间不足

**现象**: AI 报告在 QT 界面 `QTextEdit` 中显示不全，需要滚动查看。

**解决**: 在 `on_ai_report_ready_()` 中同时用 `fprintf(stderr, ...)` 打印完整报告到终端，方便开发调试。

---

## 架构决策

| 决策 | 方案 | 原因 |
|------|------|------|
| rkllm 库加载方式 | `dlopen` 运行时加载 | 避免直接链接导致进程启动时崩溃 |
| AI 初始化时机 | `QTimer::singleShot(500, ...)` 延迟初始化 | 避免阻塞 UI 启动，隔离崩溃影响 |
| 组件封装方式 | 独立静态库 `deepseek_ai_lib` | 与 web-control 一致，解耦 SentinelQT |
| 状态同步 | `std::atomic<bool>` + `QMutex` | 主线程写快照 / Worker 线程读，无锁竞争 |

---

## 关键代码文件

| 文件 | 作用 |
|------|------|
| `Software/APP/Deepseek/include/deepseek_inference.h` | rkllm API 封装类声明 |
| `Software/APP/Deepseek/src/deepseek_inference.cpp` | dlopen + dlsym + x8 ABI + 结构体偏移量修正 |
| `Software/APP/Deepseek/include/ai_report_worker.h` | QObject Worker 声明（QThread 异步推理） |
| `Software/APP/Deepseek/src/ai_report_worker.cpp` | Prompt 构建 + 推理调度 |
| `Software/APP/Deepseek/CMakeLists.txt` | 静态库构建（含 AUTOMOC + dlopen 配置） |
| `Software/APP/SentinelQT/widget.h` | aiWorkerReady_ 原子标志等成员 |
| `Software/APP/SentinelQT/widget.cpp` | AI 初始化、自动定时、状态快照、报告显示 |
| `Software/APP/SentinelQT/widget.ui` | btnAIAnalysis 按钮 + aiReportText 显示区 |
| `Software/APP/SentinelQT/config.ini` | [AI] 配置节 + ringBufferSlots 优化 |
