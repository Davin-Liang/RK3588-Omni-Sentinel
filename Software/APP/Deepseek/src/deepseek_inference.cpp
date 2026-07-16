/**
 * @file    deepseek_inference.cpp
 * @brief   DeepSeek 大模型推理封装 — 底层 RKLLM C API 的 C++ 包装
 *
 * 职责：
 *   将 Rockchip rkllm C API（librkllmrt.so）封装为同步阻塞式的 C++ 类。
 *   上层（AIReportWorker）只需调用 inferSync("问题文本") 即可获得推理结果。
 *
 * 关键设计决策：
 *   1. 运行时 dlopen 加载 .so — 避免直接链接导致进程启动时崩溃
 *   2. 手动字段偏移写入结构体 — 不用 rkllm.h 头文件，避免 ABI 耦合
 *   3. mutex + condition_variable — 把异步 callback 转为同步阻塞返回
 *   4. aarch64 x8 内联汇编 — 正确处理大结构体返回值的 ABI 调用约定
 *
 * 调用栈：
 *   AIReportWorker::start()
 *     → initialize()         加载模型，dlopen + rkllm_init
 *     → inferSync(prompt)    触发推理，阻塞等待结果
 *       → rkllm_run()        NPU 推理（同步阻塞，内部回调流式输出 token）
 *       → raw_callback_()    C 回调 → onToken_/onFinish_/onError_
 *     → destroy()            释放 NPU 资源
 */

#include "deepseek_inference.h"

#ifdef HAS_RKLLM
#include <dlfcn.h>       // dlopen, dlsym, dlclose — 运行时动态加载 .so
#endif

#include <cstdio>
#include <cstring>

// ============================================================================
// Prompt 模板 — 必须与当前使用模型的 chat template 一致
//
// 旧模型 DeepSeek/Qwen 模板（参考）：
//   <｜begin▁of▁sentence｜><｜User｜>用户问题<｜Assistant｜>模型回答
//
// 当前模型 Llama-3.2-1B-Instruct 模板：
//   <|begin_of_text|><|start_header_id|>user<|end_header_id|>
//   用户问题
//   <|eot_id|><|start_header_id|>assistant<|end_header_id|>
//   模型回答
// ============================================================================
static const char* PROMPT_PREFIX  = "<|begin_of_text|><|start_header_id|>user<|end_header_id|>\n";
static const char* PROMPT_POSTFIX = "<|eot_id|><|start_header_id|>assistant<|end_header_id|>\n";

// ============================================================================
//                        HAS_RKLLM 分支 — 真实推理实现
// ============================================================================
#ifdef HAS_RKLLM

// ---- rkllm C API 的函数指针类型定义 ----
// 因为不包含 rkllm.h，这里手动声明所需的函数签名。
// 所有参数都用 void* 传递原始指针或缓冲区，由调用方保证类型正确。

typedef void* (*rkllm_createDefaultParam_t)();           // 返回默认参数结构体（通过 x8）
typedef int   (*rkllm_init_t)             (void*, void*, void*);  // handle, params, callback
typedef int   (*rkllm_run_t)              (void*, void*, void*, void*);  // handle, input, inferParams, userdata
typedef int   (*rkllm_destroy_t)          (void*);                     // handle

// ---- 从 librkllmrt.so 动态加载的函数指针 ----
// 进程启动时不加载 .so，只在首次 initialize() 时通过 dlopen 解析。
// 这样做的好处：即使 librkllmrt.so 不存在或有兼容问题，程序也能正常启动，
// 只是 AI 功能不可用。

static void* g_rkllm_so = nullptr;                        // dlopen 返回的 .so 句柄
static rkllm_createDefaultParam_t p_rkllm_createDefaultParam = nullptr;
static rkllm_init_t               p_rkllm_init               = nullptr;
static rkllm_run_t                p_rkllm_run                = nullptr;
static rkllm_destroy_t            p_rkllm_destroy            = nullptr;

// ============================================================================
// load_rkllm_symbols_() — 运行时动态加载 librkllmrt.so 并解析函数指针
//
// 搜索路径优先级：
//   1. LD_LIBRARY_PATH 中的 "librkllmrt.so"（系统标准查找）
//   2. "./lib/librkllmrt.so"（当前目录下的 lib 子目录）
//   3. "../lib/librkllmrt.so"（上级目录的 lib，适配板端部署结构）
//   4. 绝对路径 "/root/Deepseek/install/..."（板端实际部署路径）
//
// 返回值：true=加载成功，false=失败（.so 不存在或符号缺失）
// ============================================================================
static bool load_rkllm_symbols_()
{
    if (g_rkllm_so) return true;  // 已经加载过，直接返回

    const char* paths[] = {
        "librkllmrt.so",
        "./lib/librkllmrt.so",
        "../lib/librkllmrt.so",
        "/root/Deepseek/install/demo_Linux_aarch64/lib/librkllmrt.so",
        nullptr                                                // 哨兵
    };

    for (int i = 0; paths[i]; ++i) {
        g_rkllm_so = dlopen(paths[i], RTLD_NOW);               // RTLD_NOW: 立即解析所有符号
        if (g_rkllm_so) {
            fprintf(stderr, "[DeepSeekInference] loaded librkllmrt.so from: %s\n", paths[i]);
            break;
        }
    }

    if (!g_rkllm_so) {
        fprintf(stderr, "[DeepSeekInference] failed to load librkllmrt.so: %s\n", dlerror());
        return false;
    }

    // 逐个解析 API 函数（dlsym 按字符串名查找）
    p_rkllm_createDefaultParam = (rkllm_createDefaultParam_t)
        dlsym(g_rkllm_so, "rkllm_createDefaultParam");
    p_rkllm_init   = (rkllm_init_t)   dlsym(g_rkllm_so, "rkllm_init");
    p_rkllm_run    = (rkllm_run_t)    dlsym(g_rkllm_so, "rkllm_run");
    p_rkllm_destroy = (rkllm_destroy_t) dlsym(g_rkllm_so, "rkllm_destroy");

    if (!p_rkllm_createDefaultParam || !p_rkllm_init || !p_rkllm_run || !p_rkllm_destroy) {
        fprintf(stderr, "[DeepSeekInference] missing symbols in librkllmrt.so\n");
        dlclose(g_rkllm_so);
        g_rkllm_so = nullptr;
        return false;
    }

    return true;
}

// ============================================================================
// unload_rkllm_symbols_() — 卸载 .so（当前未使用，预留）
// ============================================================================
static void unload_rkllm_symbols_()
{
    if (g_rkllm_so) { dlclose(g_rkllm_so); g_rkllm_so = nullptr; }
    p_rkllm_createDefaultParam = nullptr;
    p_rkllm_init               = nullptr;
    p_rkllm_run                = nullptr;
    p_rkllm_destroy            = nullptr;
}

// ============================================================================
// LLMCallState 枚举 — rkllm callback 的状态码
//
// rkllm_run() 执行期间，每生成一个 token 都会调用一次 callback，
// 状态不同含义不同：
//   NORMAL  — 有新的 token 文本，result->text 指向 UTF-8 字符串
//   WAITING — 等待完整的多字节 UTF-8 字符（中文常见），也当作 token 处理
//   FINISH  — 推理正常完成
//   ERROR   — 推理出错
// ============================================================================
enum {
    RKLLM_RUN_NORMAL  = 0,
    RKLLM_RUN_WAITING = 1,   // 多字节 UTF-8 字符的中间状态
    RKLLM_RUN_FINISH  = 2,
    RKLLM_RUN_ERROR   = 3,
    RKLLM_RUN_GET_LAST_HIDDEN_LAYER = 4
};

// ============================================================================
// RKLLMResultLocal — 模拟 rkllm.h 中 RKLLMResult 的结构体布局
//
// 因为不 #include <rkllm.h>（避免 ABI 耦合），这里手动定义一个足够大的结构体。
// 我们只关心第一个字段 text (const char*)，后面的 pad 保证结构体对齐和大小正确。
// ============================================================================
struct RKLLMResultLocal {
    const char* text;        // 对应真实 RKLLMResult.text
    char _pad[64];           // 填充到足够大小，避免 out-of-bounds 访问
};

// ============================================================================
// raw_callback_() — C 风格回调，由 rkllm_run() 在推理过程中反复调用
//
// 签名：(void* result, void* userdata, int state)
//   result:   指向 RKLLMResult，内含新生成的 token 文本
//   userdata: 通过 rkllm_run 最后一个参数传入，这里传的是 DeepSeekInference*
//   state:    LLMCallState 枚举值
//
// 此函数在 rkllm_run() 内部被同步调用（不是异步线程），
// 因此可以直接访问 DeepSeekInference 对象的成员，无需额外线程同步。
// ============================================================================
static void raw_callback_(void* result, void* userdata, int state)
{
    DeepSeekInference* self = static_cast<DeepSeekInference*>(userdata);
    if (!self) return;

    if (state == RKLLM_RUN_NORMAL || state == RKLLM_RUN_WAITING) {
        // Token 输出：拼接到 resultText_
        if (result) {
            RKLLMResultLocal* r = static_cast<RKLLMResultLocal*>(result);
            if (r->text) {
                self->onToken_(r->text);
            }
        }
    } else if (state == RKLLM_RUN_FINISH) {
        self->onFinish_();       // 推理完成，通知 condition_variable
    } else if (state == RKLLM_RUN_ERROR) {
        self->onError_();        // 推理出错
    }
}

// ============================================================================
// 构造 / 析构
// ============================================================================
DeepSeekInference::DeepSeekInference()
    : handle_(nullptr), initialized_(false), finished_(false), error_(false) {}

DeepSeekInference::~DeepSeekInference() { destroy(); }

// ============================================================================
// initialize() — 加载 DeepSeek 模型到 NPU
//
// 流程：
//   1. dlopen 加载 librkllmrt.so，dlsym 解析所有函数指针
//   2. 调用 rkllm_createDefaultParam() 获取默认参数结构体
//      ⚠️ aarch64 上大结构体 (>16 字节) 通过 x8 寄存器返回隐藏指针。
//      必须用内联汇编设置 x8 指向我们的缓冲区，否则函数会向垃圾地址写入 → 崩溃。
//   3. 用 memcpy 按偏移量覆盖需要的字段（model_path, temperature, top_k 等）
//      偏移量来自 rkllm.h 的真实结构体定义（第 2 个字段起）
//   4. 调用 rkllm_init() 加载 .rkllm 模型文件，注册回调
// ============================================================================
bool DeepSeekInference::initialize(const Config& cfg)
{
    if (initialized_) {
        fprintf(stderr, "[DeepSeekInference] already initialized\n");
        return false;
    }

    fprintf(stderr, "[DeepSeekInference] initializing model: %s\n", cfg.modelPath.c_str());

    // ---- 第 1 步：运行时加载 librkllmrt.so ----
    if (!load_rkllm_symbols_()) {
        fprintf(stderr, "[DeepSeekInference] cannot load librkllmrt.so\n");
        return false;
    }

    fprintf(stderr, "[DeepSeekInference]   maxNewTokens=%d maxContextLen=%d "
            "temp=%.2f topP=%.2f topK=%d\n",
            cfg.maxNewTokens, cfg.maxContextLen, cfg.temperature, cfg.topP, cfg.topK);

    // ---- 第 2 步：获取默认参数结构体 (aarch64 x8 ABI) ----
    // RKLLMParam 约 204 字节，16 字节对齐
    alignas(16) unsigned char paramBuf[256];
    std::memset(paramBuf, 0, sizeof(paramBuf));

#ifdef __aarch64__
    // ⚠️ 关键：大结构体返回值通过 x8 寄存器传隐藏指针
    // 必须设置 x8 = paramBuf，让 rkllm_createDefaultParam() 把结果写入我们的缓冲区
    __asm__ volatile("mov x8, %0" :: "r"(paramBuf) : "x8");
#endif
    p_rkllm_createDefaultParam();   // paramBuf 现在包含正确布局的默认 RKLLMParam

    // ---- 第 3 步：按字段偏移量覆盖参数 ----
    // 偏移量来自真实 rkllm.h 的定义（第 2 个字段为 max_context_len）
    //
    // struct RKLLMParam {               offset   size
    //     const char* model_path;        0        8
    //     int32_t max_context_len;       8        4
    //     int32_t max_new_tokens;        12       4
    //     int32_t top_k;                 16       4
    //     float   top_p;                 20       4
    //     float   temperature;           24       4
    //     float   repeat_penalty;        28       4
    //     float   frequency_penalty;     32       4
    //     float   presence_penalty;      36       4
    //     int32_t mirostat;              40       4
    //     ... (更多字段)
    //     bool    skip_special_token;    52       1
    //     bool    is_async;              53       1
    //     ... (img_start/img_end/img_content 在 64/72/80)
    //     RKLLMExtendParam extend_param; 88       4+112=116
    // }; // 总大小 204 字节

    const char* modelPathPtr = cfg.modelPath.c_str();
    std::memcpy(paramBuf + 0,  &modelPathPtr, 8);     // model_path (指针)

    int32_t mcl = cfg.maxContextLen;
    std::memcpy(paramBuf + 8,  &mcl, 4);               // max_context_len

    int32_t mnt = cfg.maxNewTokens;
    std::memcpy(paramBuf + 12, &mnt, 4);               // max_new_tokens

    int32_t topK = cfg.topK;
    std::memcpy(paramBuf + 16, &topK, 4);              // top_k

    float topP = cfg.topP;
    std::memcpy(paramBuf + 20, &topP, 4);              // top_p

    float temp = cfg.temperature;
    std::memcpy(paramBuf + 24, &temp, 4);              // temperature

    float rp = cfg.repeatPenalty;
    std::memcpy(paramBuf + 28, &rp, 4);                // repeat_penalty

    // 以下字段保持 rkllm_createDefaultParam() 的默认值，不覆盖：
    //   offset 32: frequency_penalty (0.0)
    //   offset 36: presence_penalty  (0.0)
    //   offset 40-51: mirostat 参数 (0)
    //   offset 53: is_async (false)
    //   offset 64-80: img_start/end/content (nullptr)
    //   offset 88: extend_param.base_domain_id (0)

    bool sst = true;
    std::memcpy(paramBuf + 52, &sst, 1);               // skip_special_token = true

    // ---- 第 4 步：初始化模型 ----
    int ret = p_rkllm_init(&handle_, paramBuf, (void*)raw_callback_);
    if (ret != 0) {
        fprintf(stderr, "[DeepSeekInference] rkllm_init failed (ret=%d)\n", ret);
        return false;
    }

    initialized_ = true;
    fprintf(stderr, "[DeepSeekInference] init success\n");
    return true;
}

bool DeepSeekInference::isReady() const { return initialized_ && handle_ != nullptr; }

// ============================================================================
// inferSync() — 同步推理，阻塞直到完成或超时
//
// 调用前提：
//   - initialize() 已成功调用
//   - 不在 rkllm_run() 的回调中递归调用（单实例不支持并发推理）
//
// 流程：
//   1. 清空上次推理的状态（resultText_, finished_, error_）
//   2. 加上 prompt 模板前缀/后缀
//   3. 构造 RKLLMInput 和 RKLLMInferParam 结构体（手动按偏移量写入）
//   4. 调用 p_rkllm_run() — 阻塞执行，内部会反复调用 raw_callback_
//   5. callback 中 onToken_ 拼接文本，onFinish_ 设置完成标志
//   6. condition_variable 等待 finished_ == true（带超时保护）
//   7. 返回累积的完整文本
// ============================================================================
std::string DeepSeekInference::inferSync(const std::string& prompt, int timeoutMs)
{
    if (!isReady()) {
        fprintf(stderr, "[DeepSeekInference] not ready – call initialize() first\n");
        return {};
    }

    // ---- 第 1 步：重置状态 ----
    {
        std::lock_guard<std::mutex> lk(mtx_);
        resultText_.clear();
        finished_ = false;
        error_    = false;
    }

    // ---- 第 2 步：构建完整 Prompt ----
    std::string fullPrompt = std::string(PROMPT_PREFIX) + prompt + std::string(PROMPT_POSTFIX);
    fprintf(stderr, "[DeepSeekInference] inferSync prompt len=%zu\n", fullPrompt.size());

    // ---- 第 3 步：构造输入结构体 ----
    // RKLLMInput { RKLLMInputType input_type; union { const char* prompt_input; ... }; }
    // offset 0: input_type (int), offset 8: prompt_input (const char*)
    char inputBuf[32];
    std::memset(inputBuf, 0, sizeof(inputBuf));
    int inputType = 0;  // RKLLM_INPUT_PROMPT = 0
    std::memcpy(inputBuf, &inputType, 4);
    const char* promptPtr = fullPrompt.c_str();
    std::memcpy(inputBuf + 8, &promptPtr, 8);

    // RKLLMInferParam { RKLLMInferMode mode; ... }
    // offset 0: mode (int)
    char inferBuf[32];
    std::memset(inferBuf, 0, sizeof(inferBuf));
    int mode = 0;  // RKLLM_INFER_GENERATE = 0
    std::memcpy(inferBuf, &mode, 4);

    // ---- 第 4 步：执行 NPU 推理（阻塞） ----
    int ret = p_rkllm_run(handle_, inputBuf, inferBuf, static_cast<void*>(this));

    if (ret != 0) {
        fprintf(stderr, "[DeepSeekInference] rkllm_run failed (ret=%d)\n", ret);
        return {};
    }

    // ---- 第 5 步：等待 callback 设置完成标志 ----
    // 虽然 rkllm_run 是阻塞的，但 callback 可能在返回后才最后触发。
    // condition_variable 提供额外的同步保证。
    {
        std::unique_lock<std::mutex> lk(mtx_);
        if (!finished_ && !error_) {
            cv_.wait_for(lk, std::chrono::milliseconds(timeoutMs),
                         [this] { return finished_ || error_; });
        }
        if (error_) {
            fprintf(stderr, "[DeepSeekInference] inference error\n");
            return {};
        }
        if (!finished_) {
            fprintf(stderr, "[DeepSeekInference] inference timed out\n");
            return {};
        }
        return resultText_;
    }
}

// ============================================================================
// destroy() — 释放 NPU 资源
// ============================================================================
void DeepSeekInference::destroy()
{
    if (handle_) {
        fprintf(stderr, "[DeepSeekInference] destroying handle\n");
        void* h = handle_;
        handle_ = nullptr;
        p_rkllm_destroy(h);          // 释放 NPU 显存中的模型
    }
    initialized_ = false;
}

// ============================================================================
//                        !HAS_RKLLM 分支 — 空桩实现
//
// 编译时未设置 RKLLM_RUNTIME_PATH 时使用。
// 所有函数返回 false/空字符串，程序可以正常编译运行，只是 AI 功能不可用。
// ============================================================================
#else

DeepSeekInference::DeepSeekInference()
    : handle_(nullptr), initialized_(false), finished_(false), error_(false) {}
DeepSeekInference::~DeepSeekInference() { destroy(); }
bool DeepSeekInference::initialize(const Config&) {
    fprintf(stderr, "[DeepSeekInference] RKLLM not available (build without HAS_RKLLM)\n");
    return false;
}
bool DeepSeekInference::isReady() const { return false; }
std::string DeepSeekInference::inferSync(const std::string&, int) {
    fprintf(stderr, "[DeepSeekInference] RKLLM not available\n");
    return {};
}
void DeepSeekInference::destroy() { handle_ = nullptr; initialized_ = false; }

#endif // HAS_RKLLM

// ============================================================================
// 内部回调辅助方法（HAS_RKLLM 和 Stub 共用）
//
// 这三个方法由 raw_callback_ 在 rkllm_run() 执行期间同步调用。
// rkllm_run() 是阻塞的，所以这些回调都在 inferSync() 的调用线程中执行，
// 不存在多线程竞争。mutex 主要用于 condition_variable 的语义要求。
// ============================================================================

// 模型每生成一个 token，raw_callback_ 就调用一次
void DeepSeekInference::onToken_(const char* text)
{
    std::lock_guard<std::mutex> lk(mtx_);
    resultText_ += text;             // 追加到累积缓冲区
}

// 推理完成时触发
void DeepSeekInference::onFinish_()
{
    std::lock_guard<std::mutex> lk(mtx_);
    finished_ = true;
    cv_.notify_one();                // 唤醒 inferSync() 中等待的线程
}

// 推理出错时触发
void DeepSeekInference::onError_()
{
    std::lock_guard<std::mutex> lk(mtx_);
    error_ = true;
    finished_ = true;
    cv_.notify_one();                // 同样唤醒，让 inferSync() 返回空字符串
}
