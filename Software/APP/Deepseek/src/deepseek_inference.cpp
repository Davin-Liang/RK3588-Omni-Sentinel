#include "deepseek_inference.h"

#ifdef HAS_RKLLM
#include <dlfcn.h>
#endif

#include <cstdio>
#include <cstring>

// ---- Prompt template (must match the model's chat format) ----
// Unicode full-width characters – exactly as used in the Rockchip demo

static const char* PROMPT_PREFIX  = "<｜begin▁of▁sentence｜><｜User｜>";
static const char* PROMPT_POSTFIX = "<｜Assistant｜>";

// ============================================================================
// Implementation with RKLLM (runtime dlopen, NOT direct link)
// ============================================================================

#ifdef HAS_RKLLM

// ---- rkllm C API function pointer types ----

typedef void* (*rkllm_createDefaultParam_t)();
typedef int   (*rkllm_init_t)(void*, void*, void*);
typedef int   (*rkllm_run_t)(void*, void*, void*, void*);
typedef int   (*rkllm_destroy_t)(void*);

// ---- Dynamically loaded symbols ----

static void* g_rkllm_so = nullptr;

static rkllm_createDefaultParam_t p_rkllm_createDefaultParam = nullptr;
static rkllm_init_t               p_rkllm_init               = nullptr;
static rkllm_run_t                p_rkllm_run                = nullptr;
static rkllm_destroy_t            p_rkllm_destroy            = nullptr;

static bool load_rkllm_symbols_()
{
    if (g_rkllm_so) return true;  // already loaded

    // 尝试多个常见路径
    const char* paths[] = {
        "librkllmrt.so",
        "./lib/librkllmrt.so",
        "../lib/librkllmrt.so",
        "/root/Deepseek/install/demo_Linux_aarch64/lib/librkllmrt.so",
        nullptr
    };

    for (int i = 0; paths[i]; ++i) {
        g_rkllm_so = dlopen(paths[i], RTLD_NOW);
        if (g_rkllm_so) {
            fprintf(stderr, "[DeepSeekInference] loaded librkllmrt.so from: %s\n", paths[i]);
            break;
        }
    }

    if (!g_rkllm_so) {
        fprintf(stderr, "[DeepSeekInference] failed to load librkllmrt.so: %s\n", dlerror());
        return false;
    }

    p_rkllm_createDefaultParam = (rkllm_createDefaultParam_t)dlsym(g_rkllm_so, "rkllm_createDefaultParam");
    p_rkllm_init               = (rkllm_init_t)dlsym(g_rkllm_so, "rkllm_init");
    p_rkllm_run                = (rkllm_run_t)dlsym(g_rkllm_so, "rkllm_run");
    p_rkllm_destroy            = (rkllm_destroy_t)dlsym(g_rkllm_so, "rkllm_destroy");

    if (!p_rkllm_createDefaultParam || !p_rkllm_init || !p_rkllm_run || !p_rkllm_destroy) {
        fprintf(stderr, "[DeepSeekInference] missing symbols in librkllmrt.so\n");
        dlclose(g_rkllm_so);
        g_rkllm_so = nullptr;
        return false;
    }

    return true;
}

static void unload_rkllm_symbols_()
{
    if (g_rkllm_so) {
        dlclose(g_rkllm_so);
        g_rkllm_so = nullptr;
    }
    p_rkllm_createDefaultParam = nullptr;
    p_rkllm_init               = nullptr;
    p_rkllm_run                = nullptr;
    p_rkllm_destroy            = nullptr;
}

// ---- C callback → forward to the C++ instance ----
// 使用 int 代替枚举避免头文件依赖
enum {
    RKLLM_RUN_NORMAL  = 0,
    RKLLM_RUN_WAITING = 1,  // wait for complete UTF-8 character, ignore
    RKLLM_RUN_FINISH  = 2,
    RKLLM_RUN_ERROR   = 3,
    RKLLM_RUN_GET_LAST_HIDDEN_LAYER = 4
};

struct RKLLMResultLocal {
    const char* text;
    char _pad[64];  // 确保结构体大小足够，避免 ABI 问题
};

static void raw_callback_(void* result, void* userdata, int state)
{
    DeepSeekInference* self = static_cast<DeepSeekInference*>(userdata);
    if (!self) return;

    if (state == RKLLM_RUN_NORMAL || state == RKLLM_RUN_WAITING) {
        if (result) {
            RKLLMResultLocal* r = static_cast<RKLLMResultLocal*>(result);
            if (r->text) {
                self->onToken_(r->text);
            }
        }
    } else if (state == RKLLM_RUN_FINISH) {
        self->onFinish_();
    } else if (state == RKLLM_RUN_ERROR) {
        self->onError_();
    }
}

DeepSeekInference::DeepSeekInference()
    : handle_(nullptr)
    , initialized_(false)
    , finished_(false)
    , error_(false)
{
}

DeepSeekInference::~DeepSeekInference()
{
    destroy();
}

bool DeepSeekInference::initialize(const Config& cfg)
{
    if (initialized_) {
        fprintf(stderr, "[DeepSeekInference] already initialized\n");
        return false;
    }

    fprintf(stderr, "[DeepSeekInference] initializing model: %s\n", cfg.modelPath.c_str());

    // 运行时动态加载 librkllmrt.so
    if (!load_rkllm_symbols_()) {
        fprintf(stderr, "[DeepSeekInference] cannot load librkllmrt.so\n");
        return false;
    }

    fprintf(stderr, "[DeepSeekInference]   maxNewTokens=%d maxContextLen=%d "
            "temp=%.2f topP=%.2f topK=%d\n",
            cfg.maxNewTokens, cfg.maxContextLen,
            cfg.temperature, cfg.topP, cfg.topK);

    // 获取 rkllm_createDefaultParam() 返回的默认参数结构体。
    // aarch64 上大结构体 (>16B) 通过 x8 寄存器传隐藏指针返回，必须正确设置 x8。
    // RKLLMParam 实际大小 204 字节（含 extend_param.reserved[112]）
    alignas(16) unsigned char paramBuf[256];
    std::memset(paramBuf, 0, sizeof(paramBuf));

#ifdef __aarch64__
    // x8 = 返回值缓冲区地址
    __asm__ volatile("mov x8, %0" :: "r"(paramBuf) : "x8");
#endif
    p_rkllm_createDefaultParam();
    // paramBuf 现在包含正确布局的默认 RKLLMParam

    // 覆盖我们关心的字段（偏移量来自 rkllm.h 真实定义）
    // offset 0: model_path (const char*)
    const char* modelPathPtr = cfg.modelPath.c_str();
    std::memcpy(paramBuf + 0, &modelPathPtr, 8);

    // offset 8: max_context_len (int32_t)
    int32_t mcl = cfg.maxContextLen;
    std::memcpy(paramBuf + 8, &mcl, 4);

    // offset 12: max_new_tokens (int32_t)
    int32_t mnt = cfg.maxNewTokens;
    std::memcpy(paramBuf + 12, &mnt, 4);

    // offset 16: top_k (int32_t)
    int32_t topK = cfg.topK;
    std::memcpy(paramBuf + 16, &topK, 4);

    // offset 20: top_p (float)
    float topP = cfg.topP;
    std::memcpy(paramBuf + 20, &topP, 4);

    // offset 24: temperature (float)
    float temp = cfg.temperature;
    std::memcpy(paramBuf + 24, &temp, 4);

    // offset 28: repeat_penalty (float)
    float rp = cfg.repeatPenalty;
    std::memcpy(paramBuf + 28, &rp, 4);

    // offset 32: frequency_penalty (float) — keep default 0.0
    // offset 36: presence_penalty (float) — keep default 0.0
    // offset 40: mirostat (int32_t) — keep default 0
    // offset 44: mirostat_tau (float) — keep default
    // offset 48: mirostat_eta (float) — keep default

    // offset 52: skip_special_token (bool)
    bool sst = true;
    std::memcpy(paramBuf + 52, &sst, 1);

    // offset 53: is_async (bool) — keep default false
    // offset 64-80: img_start/img_end/img_content — keep default nullptr
    // offset 88: extend_param.base_domain_id — keep default 0

    int ret = p_rkllm_init(&handle_, paramBuf, (void*)raw_callback_);
    if (ret != 0) {
        fprintf(stderr, "[DeepSeekInference] rkllm_init failed (ret=%d)\n", ret);
        return false;
    }

    initialized_ = true;
    fprintf(stderr, "[DeepSeekInference] init success\n");
    return true;
}

bool DeepSeekInference::isReady() const
{
    return initialized_ && handle_ != nullptr;
}

std::string DeepSeekInference::inferSync(const std::string& prompt, int timeoutMs)
{
    if (!isReady()) {
        fprintf(stderr, "[DeepSeekInference] not ready – call initialize() first\n");
        return {};
    }

    // 重置状态
    {
        std::lock_guard<std::mutex> lk(mtx_);
        resultText_.clear();
        finished_ = false;
        error_    = false;
    }

    // 构建完整 prompt
    std::string fullPrompt = std::string(PROMPT_PREFIX) + prompt + std::string(PROMPT_POSTFIX);
    fprintf(stderr, "[DeepSeekInference] inferSync prompt len=%zu\n", fullPrompt.size());

    // 准备输入 (offset 0=input_type int, offset 8=prompt_input char*)
    char inputBuf[32];
    std::memset(inputBuf, 0, sizeof(inputBuf));
    int inputType = 0;  // RKLLM_INPUT_PROMPT
    std::memcpy(inputBuf, &inputType, 4);
    const char* promptPtr = fullPrompt.c_str();
    std::memcpy(inputBuf + 8, &promptPtr, 8);

    // 准备推理参数 (offset 0=mode int)
    char inferBuf[32];
    std::memset(inferBuf, 0, sizeof(inferBuf));
    int mode = 0;  // RKLLM_INFER_GENERATE
    std::memcpy(inferBuf, &mode, 4);

    // 执行推理（阻塞，callback 在内部被调用）
    int ret = p_rkllm_run(handle_, inputBuf, inferBuf, static_cast<void*>(this));

    if (ret != 0) {
        fprintf(stderr, "[DeepSeekInference] rkllm_run failed (ret=%d)\n", ret);
        return {};
    }

    // 等待 callback 设置 finished_ 标志
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

void DeepSeekInference::destroy()
{
    if (handle_) {
        fprintf(stderr, "[DeepSeekInference] destroying handle\n");
        void* h = handle_;
        handle_ = nullptr;
        p_rkllm_destroy(h);
    }
    initialized_ = false;
}

#else  // !HAS_RKLLM — stub implementations

DeepSeekInference::DeepSeekInference()
    : handle_(nullptr)
    , initialized_(false)
    , finished_(false)
    , error_(false)
{
}

DeepSeekInference::~DeepSeekInference()
{
    destroy();
}

bool DeepSeekInference::initialize(const Config&)
{
    fprintf(stderr, "[DeepSeekInference] RKLLM not available (build without HAS_RKLLM)\n");
    return false;
}

bool DeepSeekInference::isReady() const
{
    return false;
}

std::string DeepSeekInference::inferSync(const std::string&, int)
{
    fprintf(stderr, "[DeepSeekInference] RKLLM not available\n");
    return {};
}

void DeepSeekInference::destroy()
{
    handle_ = nullptr;
    initialized_ = false;
}

#endif // HAS_RKLLM

// ============================================================================
// Internal callback helpers (shared by both builds)
// ============================================================================

void DeepSeekInference::onToken_(const char* text)
{
    std::lock_guard<std::mutex> lk(mtx_);
    resultText_ += text;
}

void DeepSeekInference::onFinish_()
{
    std::lock_guard<std::mutex> lk(mtx_);
    finished_ = true;
    cv_.notify_one();
}

void DeepSeekInference::onError_()
{
    std::lock_guard<std::mutex> lk(mtx_);
    error_ = true;
    finished_ = true;
    cv_.notify_one();
}
