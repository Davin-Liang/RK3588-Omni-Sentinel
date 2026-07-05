#include "deepseek_inference.h"

#ifdef HAS_RKLLM
extern "C" {
#include <rkllm.h>
}
#endif

#include <cstdio>
#include <cstring>

// ---- Prompt template (must match the model's chat format) ----
// Unicode full-width characters – exactly as used in the Rockchip demo

static const char* PROMPT_PREFIX  = "<｜begin▁of▁sentence｜><｜User｜>";
static const char* PROMPT_POSTFIX = "<｜Assistant｜>";

// ============================================================================
// Implementation with RKLLM
// ============================================================================

#ifdef HAS_RKLLM

// ---- C callback → forward to the C++ instance ----

static void raw_callback_(RKLLMResult* result, void* userdata, LLMCallState state)
{
    DeepSeekInference* self = static_cast<DeepSeekInference*>(userdata);
    if (!self) return;

    if (state == RKLLM_RUN_NORMAL) {
        if (result && result->text) {
            self->onToken_(result->text);
        }
    } else if (state == RKLLM_RUN_FINISH) {
        self->onFinish_();
    } else if (state == RKLLM_RUN_ERROR) {
        self->onError_();
    }
    // RKLLM_RUN_GET_LAST_HIDDEN_LAYER – ignored
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
    fprintf(stderr, "[DeepSeekInference]   maxNewTokens=%d maxContextLen=%d "
            "temp=%.2f topP=%.2f topK=%d\n",
            cfg.maxNewTokens, cfg.maxContextLen,
            cfg.temperature, cfg.topP, cfg.topK);

    RKLLMParam param = rkllm_createDefaultParam();
    param.model_path = cfg.modelPath.c_str();

    param.top_k             = cfg.topK;
    param.top_p             = cfg.topP;
    param.temperature       = cfg.temperature;
    param.repeat_penalty    = cfg.repeatPenalty;
    param.frequency_penalty = 0.0f;
    param.presence_penalty  = 0.0f;

    param.max_new_tokens     = cfg.maxNewTokens;
    param.max_context_len    = cfg.maxContextLen;
    param.skip_special_token = true;
    param.extend_param.base_domain_id = 0;

    int ret = rkllm_init(reinterpret_cast<LLMHandle*>(&handle_),
                         &param,
                         raw_callback_);
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

    // 准备输入
    RKLLMInput input;
    std::memset(&input, 0, sizeof(input));
    input.input_type   = RKLLM_INPUT_PROMPT;
    input.prompt_input = const_cast<char*>(fullPrompt.c_str());

    // 准备推理参数
    RKLLMInferParam inferParams;
    std::memset(&inferParams, 0, sizeof(inferParams));
    inferParams.mode = RKLLM_INFER_GENERATE;

    // 执行推理（阻塞，callback 在内部被调用）
    int ret = rkllm_run(static_cast<LLMHandle>(handle_),
                        &input,
                        &inferParams,
                        static_cast<void*>(this));

    if (ret != 0) {
        fprintf(stderr, "[DeepSeekInference] rkllm_run failed (ret=%d)\n", ret);
        return {};
    }

    // rkllm_run 返回后，等待 callback 设置 finished_ 标志
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
        LLMHandle h = static_cast<LLMHandle>(handle_);
        handle_ = nullptr;
        rkllm_destroy(h);
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
