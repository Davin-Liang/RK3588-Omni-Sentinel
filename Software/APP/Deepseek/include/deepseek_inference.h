#ifndef DEEPSEEK_INFERENCE_H
#define DEEPSEEK_INFERENCE_H

#include <condition_variable>
#include <mutex>
#include <string>

class DeepSeekInference
{
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

    DeepSeekInference();
    ~DeepSeekInference();
    DeepSeekInference(const DeepSeekInference&) = delete;
    DeepSeekInference& operator=(const DeepSeekInference&) = delete;

    bool initialize(const Config& cfg);
    bool isReady() const;
    std::string inferSync(const std::string& prompt, int timeoutMs = 30000);
    void destroy();

    void onToken_(const char* text);
    void onFinish_();
    void onError_();

private:
    void* handle_;
    bool  initialized_;
    std::mutex              mtx_;
    std::condition_variable cv_;
    std::string             resultText_;
    bool                    finished_;
    bool                    error_;
};

#endif
