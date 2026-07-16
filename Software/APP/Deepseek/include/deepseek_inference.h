#ifndef DEEPSEEK_INFERENCE_H
#define DEEPSEEK_INFERENCE_H

#include <condition_variable>
#include <mutex>
#include <string>

/**
 * @brief DeepSeek 大模型推理封装类 — 将 RKLLM C API 包装为同步阻塞式 C++ 接口
 *
 * 职责：
 *   1. 运行时 dlopen 加载 librkllmrt.so，避免直接链接导致的启动崩溃
 *   2. 通过 rkllm_init 将 .rkllm 量化模型加载到 RK3588 NPU
 *   3. 将异步 callback 转为同步阻塞返回（mutex + condition_variable）
 *
 * 线程安全：
 *   - 单实例不支持并发推理（inferSync 不可重入）
 *   - mutex 保护 resultText_ / finished_ / error_，供 callback 和 inferSync 互斥访问
 *
 * 使用示例：
 * @code
 *   DeepSeekInference infer;
 *   DeepSeekInference::Config cfg;
 *   cfg.modelPath = "/root/Deepseek/model.rkllm";
 *   infer.initialize(cfg);
 *   std::string result = infer.inferSync("请分析系统状态", 60000);
 *   infer.destroy();
 * @endcode
 */
class DeepSeekInference
{
public:
    /**
     * @brief 推理参数配置
     *
     * 这些参数会在 initialize() 时写入 RKLLMParam 结构体，
     * 传递给 rkllm_init 控制模型行为。温度越低输出越确定，
     * topK/topP 控制采样的多样性。
     */
    struct Config {
        std::string modelPath;      ///< .rkllm 量化模型文件路径（板端绝对路径）
        int maxNewTokens    = 512;  ///< 最大生成 token 数（~200 字中文）
        int maxContextLen   = 1024; ///< 最大上下文窗口长度
        float temperature   = 0.7f; ///< 采样温度（0~1，越低越确定性）
        float topP          = 0.9f; ///< 核采样阈值（0~1）
        int   topK          = 40;   ///< Top-K 采样，仅保留概率最高的 K 个 token
        float repeatPenalty = 1.1f; ///< 重复惩罚因子（>1 抑制重复输出）
    };

    DeepSeekInference();
    ~DeepSeekInference();

    /// 禁止拷贝（内部持有 NPU 句柄，不可共享）
    DeepSeekInference(const DeepSeekInference&) = delete;
    DeepSeekInference& operator=(const DeepSeekInference&) = delete;

    /**
     * @brief 加载模型到 NPU，需在对端线程中调用（阻塞数秒）
     * @param cfg 推理参数配置，modelPath 必须指向有效的 .rkllm 文件
     * @return true=加载成功，false=失败（.so 不存在、模型路径错误、NPU 忙等）
     */
    bool initialize(const Config& cfg);

    /**
     * @brief 查询模型是否已成功加载
     * @return true=就绪，可安全调用 inferSync()
     */
    bool isReady() const;

    /**
     * @brief 执行同步推理，阻塞直到完成或超时
     * @param prompt   用户问题文本（不含模板前缀/后缀，由本函数自动添加）
     * @param timeoutMs 超时时间（毫秒），默认 30 秒。板端推理实际约 60~130 秒
     * @return 模型生成的完整回答文本；超时或出错返回空字符串
     *
     * @note 调用前必须已 initialize() 成功
     * @note 自动添加 <｜begin▁of▁sentence｜><｜User｜> 前缀和 <｜Assistant｜> 后缀
     * @note 此函数不可重入，同一时刻只能有一个 inferSync 在执行
     */
    std::string inferSync(const std::string& prompt, int timeoutMs = 30000);

    /**
     * @brief 释放 NPU 资源（卸载模型）
     * @note 析构函数会自动调用，也可手动提前释放
     */
    void destroy();

private:
    // ---- 以下三个方法由 C 回调 raw_callback_() 在 rkllm_run 执行期间调用 ----
    // rkllm_run 是阻塞的，所以它们在 inferSync 的调用线程中执行，无多线程竞争。
    // mutex 仅用于 condition_variable 的语义要求。

    /** @brief 模型每生成一个 token 时调用，追加到 resultText_ */
    void onToken_(const char* text);
    /** @brief 推理正常完成时调用，设置 finished_ 并唤醒 inferSync */
    void onFinish_();
    /** @brief 推理出错时调用，设置 error_ 并唤醒 inferSync（返回空字符串） */
    void onError_();

    // ---- 成员变量 ----

    void* handle_;               ///< rkllm_init 返回的 LLM 句柄（opaque，传给 rkllm_run/destroy）
    bool  initialized_;          ///< initialize() 是否成功，防止重复初始化

    std::mutex              mtx_;          ///< 保护 resultText_ / finished_ / error_ 的互斥锁
    std::condition_variable cv_;           ///< 条件变量，inferSync 等待 callback 设置完成标志
    std::string             resultText_;   ///< 累积的推理输出文本（token 逐个拼接）
    bool                    finished_;     ///< 推理是否完成（正常或异常）
    bool                    error_;        ///< 推理是否出错
};

#endif
