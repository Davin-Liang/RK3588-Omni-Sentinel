#ifndef DEEPSEEK_INFERENCE_H
#define DEEPSEEK_INFERENCE_H

#include <condition_variable>
#include <mutex>
#include <string>

/**
 * @brief DeepSeek 大模型推理封装类
 *
 * 封装 rkllm C API（librkllmrt.so），将异步 callback 转为同步阻塞调用。
 * 线程安全：单实例仅支持单次推理（不支持并发调用 inferSync）。
 *
 * 使用方式：
 *   DeepSeekInference ai;
 *   DeepSeekInference::Config cfg;
 *   cfg.modelPath = "/root/Deepseek/.../model.rkllm";
 *   ai.initialize(cfg);
 *   std::string result = ai.inferSync("你好");
 *   ai.destroy();
 */
class DeepSeekInference
{
public:
    struct Config {
        std::string modelPath;          ///< .rkllm 模型文件路径
        int maxNewTokens    = 512;      ///< 最大生成 token 数
        int maxContextLen   = 2048;     ///< 最大上下文长度
        float temperature   = 0.7f;     ///< 采样温度
        float topP          = 0.9f;     ///< nucleus sampling
        int   topK          = 40;       ///< top-k sampling
        float repeatPenalty = 1.1f;     ///< 重复惩罚
    };

    DeepSeekInference();
    ~DeepSeekInference();

    DeepSeekInference(const DeepSeekInference&) = delete;
    DeepSeekInference& operator=(const DeepSeekInference&) = delete;

    /** @brief 初始化模型，加载到 NPU
     *  @return true 成功, false 失败 */
    bool initialize(const Config& cfg);

    /** @brief 模型是否已就绪 */
    bool isReady() const;

    /**
     * @brief 同步推理（阻塞直到完成或超时）
     * @param prompt    用户输入文本（无需添加模板前缀/后缀，内部自动处理）
     * @param timeoutMs 超时毫秒数
     * @return 模型输出的完整文本，失败返回空字符串
     */
    std::string inferSync(const std::string& prompt, int timeoutMs = 30000);

    /** @brief 释放 NPU 资源 */
    void destroy();

    // ---- 以下方法由内部 C callback 调用，外部不应直接使用 ----
    void onToken_(const char* text);
    void onFinish_();
    void onError_();

private:
    void* handle_;      // LLMHandle (opaque, avoid rkllm.h in header)
    bool  initialized_;

    std::mutex              mtx_;
    std::condition_variable cv_;
    std::string             resultText_;
    bool                    finished_;
    bool                    error_;
};

#endif // DEEPSEEK_INFERENCE_H
