# DeepSeek 1.5B 大模型推理集成到 SentinelQT — 实施计划与进度

## 背景

用户已在 RK3588 板端成功运行 DeepSeek-R1-Distill-Qwen-1.5B：

```bash
root@elf2-buildroot:~/Deepseek/install/demo_Linux_aarch64
├── llm_demo                                    # 推理 demo
├── DeepSeek-R1-Distill-Qwen-1.5B_W8A8_RK3588.rkllm  # 量化模型
└── lib/librkllmrt.so                           # rkllm-runtime v1.1.4
# NPU driver: 0.9.8, Platform: RK3588
```

目标：将推理能力封装为可复用类，集成到 SentinelQT 中，实现**系统运行状态 AI 智能总结分析**。

---

## 板端真实路径

| 项目 | 路径 |
|------|------|
| 模型文件 | `/root/Deepseek/install/demo_Linux_aarch64/DeepSeek-R1-Distill-Qwen-1.5B_W8A8_RK3588.rkllm` |
| 运行时库 | `/root/Deepseek/install/demo_Linux_aarch64/lib/librkllmrt.so` |

---

## 实施进度总览

| 步骤 | 内容 | 状态 |
|------|------|------|
| 1 | 创建 `deepseek_inference.h` — rkllm API 封装类头文件 | ✅ 已完成 |
| 2 | 创建 `deepseek_inference.cpp` — rkllm API 封装类实现 | ✅ 已完成 |
| 3 | 创建 `ai_report_worker.h` — AI 报告 Worker 头文件 | ✅ 已完成 |
| 4 | 创建 `ai_report_worker.cpp` — AI 报告 Worker 实现 | ✅ 已完成 |
| 5 | 修改 `widget.ui` — 添加 AI 分析按钮和报告显示区域 | ✅ 已完成 |
| 6 | 修改 `widget.h` — 添加 AI 相关成员和 slot 声明 | ✅ 已完成 |
| 7 | 修改 `widget.cpp` — 集成 AI 分析功能到主界面 | ✅ 已完成 |
| 8 | 修改 `CMakeLists.txt` + `build.sh` — 添加 RKLLM 构建支持 | ✅ 已完成 |
| 9 | 修改 `config.ini` — 添加 `[AI]` 配置节 | ✅ 已完成 |
| 10 | **编译验证** — 开发机交叉编译通过 | 🔲 待执行 |
| 11 | **板端部署测试** — RK3588 上运行验证 | 🔲 待执行 |
| 12 | **Web API 集成（可选）** — REST API 远程触发 AI 分析 | 🔲 待执行 |
| 13 | **IMU 状态接入（可选）** — 将 icm45686 数据接入 AI 分析 | 🔲 待执行 |

---

## 已完成步骤详情

### 步骤 1-2：DeepSeekInference 封装类

**文件**：`Software/APP/SentinelQT/deepseek_inference.h` / `.cpp`

```cpp
class DeepSeekInference {
public:
    struct Config {
        std::string modelPath;
        int maxNewTokens    = 512;
        int maxContextLen   = 2048;
        float temperature   = 0.7f;
        float topP          = 0.9f;
        int   topK          = 40;
        float repeatPenalty = 1.1f;
    };
    bool initialize(const Config& cfg);
    bool isReady() const;
    std::string inferSync(const std::string& prompt, int timeoutMs = 30000);
    void destroy();
};
```

- 封装 rkllm C API（`rkllm_init` / `rkllm_run` / `rkllm_destroy`）
- `mutex + condition_variable` 将异步 callback 转为同步阻塞返回
- 自动添加 `<｜begin▁of▁sentence｜><｜User｜>` / `<｜Assistant｜>` prompt 模板
- `#ifdef HAS_RKLLM` 条件编译，无 SDK 时编译为 stub

### 步骤 3-4：AIReportWorker 异步工作线程

**文件**：`Software/APP/SentinelQT/ai_report_worker.h` / `.cpp`

- 参照 `FusionWorker` 模式：QObject + moveToThread + `std::atomic<bool> running_`
- 主线程每秒 `updateStatus()` 推送系统状态快照
- `requestReport()` → `buildPrompt_()` → `inferSync()` → emit `reportReady`
- 200ms 轮询间隔，防止重复请求

### 步骤 5-7：UI 集成

**修改文件**：`widget.ui`、`widget.h`、`widget.cpp`

- 标题栏新增 `btnAIAnalysis` 按钮（蓝色，紧凑型）
- 主页面新增 `aiReportText`（QTextEdit，只读，深色背景，默认隐藏）
- 构造函数：创建 AIReportWorker + QThread，从 `config.ini [AI]` 节加载配置
- `update_hw_usage_()` 末尾调用 `update_ai_status_snapshot_()` 每秒推送状态
- 析构函数：正确清理 worker 和线程（stop → quit → wait → delete）
- `on_btn_ai_analysis_()`：显示"正在分析…"→ `requestReport()`
- `on_ai_report_ready_()`：渲染报告（`<think>` 标签灰色斜体显示）

### 步骤 8：构建系统

**修改文件**：`CMakeLists.txt`、`build.sh`

- CMake：`-DRKLLM_RUNTIME_PATH=...` 条件编译，定义 `HAS_RKLLM` 宏
- build.sh：新增 `RKLLM_RUNTIME_PATH` 环境变量

### 步骤 9：配置文件

**修改文件**：`config.ini`

```ini
[AI]
modelPath=/root/Deepseek/install/demo_Linux_aarch64/DeepSeek-R1-Distill-Qwen-1.5B_W8A8_RK3588.rkllm
maxNewTokens=512
maxContextLen=2048
temperature=0.7
```

---

## 接下来需要执行的计划

### 步骤 10：编译验证（🔲 待执行）

```bash
# 1. 将 rkllm-runtime SDK 放到开发机可访问路径
#    （包含 include/rkllm.h 和 aarch64/librkllmrt.so）

# 2. 设置环境变量并编译
export RKLLM_RUNTIME_PATH=/path/to/rkllm-runtime/Linux/librkllm_api
cd Software/APP/SentinelQT
./build.sh
```

**预期结果**：CMake 输出 `RKLLM Runtime enabled: ...`，编译无错误。

**注意事项**：
- SentinelQT 使用 `aarch64-buildroot-linux-gnu` 工具链，Deepseek demo 使用 `aarch64-none-linux-gnu`。需验证 `librkllmrt.so` 在两个工具链下的 ABI 兼容性
- 如果链接失败，可能需要用 buildroot 工具链重新编译 `librkllmrt.so`，或确认 `.so` 是纯 C API（无 C++ name mangling 问题，通常兼容）

### 步骤 11：板端部署测试（🔲 待执行）

```bash
# 1. 将编译产物部署到 RK3588
scp install/SentinelQT root@<board_ip>:/root/
scp install/config.ini root@<board_ip>:/root/
scp /path/to/librkllmrt.so root@<board_ip>:/root/lib/

# 2. 板端运行
export LD_LIBRARY_PATH=/root/lib:$LD_LIBRARY_PATH
./SentinelQT

# 3. 验证
# - 点击标题栏 "AI 分析" 按钮
# - 观察 aiReportText 显示 "正在分析系统运行状态…"
# - 等待 20-60 秒
# - 看到中文系统状态分析报告
# - 多次点击不崩溃
# - 推理期间相机预览不卡顿
# - 退出程序无报错
```

### 步骤 12：Web API 集成（🔲 可选）

在 `widget.cpp` 的 `handle_web_command()` 中新增：
- `POST /api/v1/ai/report` — 触发 AI 分析
- `GET /api/v1/ai/report` — 获取最近一次结果

### 步骤 13：IMU 状态接入（🔲 可选）

当前 `update_ai_status_snapshot_()` 中 IMU 状态为占位字符串 `"未启用"`。后续接入 `icm45686-eis-app` 组件后，替换为真实 IMU 状态数据。

---

## 架构总览

```
SentinelQT 进程
├── 主线程 (Qt Event Loop)
│   ├── update_hw_usage_() → update_ai_status_snapshot_() [每秒]
│   ├── btnAIAnalysis → on_btn_ai_analysis_() → requestReport()
│   └── on_ai_report_ready_() → aiReportText 显示报告
│
├── AIReportWorker 子线程 (QThread)
│   ├── start() → DeepSeekInference::initialize()
│   ├── updateStatus() — mutex 保护的状态快照
│   ├── buildPrompt_() — 格式化中文 Prompt
│   └── requestReport() → inferSync() → emit reportReady
│
└── DeepSeekInference (封装 rkllm C API)
    ├── rkllm_init() → 加载 .rkllm 到 NPU
    ├── inferSync() → mutex+cv 转异步 callback 为同步
    └── rkllm_destroy() → 释放 NPU 资源
```

## 涉及文件清单

| 操作 | 文件 |
|------|------|
| 新建 | `Software/APP/SentinelQT/deepseek_inference.h` |
| 新建 | `Software/APP/SentinelQT/deepseek_inference.cpp` |
| 新建 | `Software/APP/SentinelQT/ai_report_worker.h` |
| 新建 | `Software/APP/SentinelQT/ai_report_worker.cpp` |
| 修改 | `Software/APP/SentinelQT/widget.h` |
| 修改 | `Software/APP/SentinelQT/widget.cpp` |
| 修改 | `Software/APP/SentinelQT/widget.ui` |
| 修改 | `Software/APP/SentinelQT/CMakeLists.txt` |
| 修改 | `Software/APP/SentinelQT/build.sh` |
| 修改 | `Software/APP/SentinelQT/config.ini` |
