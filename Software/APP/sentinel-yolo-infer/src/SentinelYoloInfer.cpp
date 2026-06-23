#include "SentinelYoloInfer.h"
#include "Yolov8RknnEngine.h"

#include <chrono>
#include <ctime>
#include <iostream>

struct SentinelYoloInfer::InferThreadContext {
    explicit InferThreadContext(int cam) : camNum(cam) {}

    int camNum = -1;
    std::atomic<bool> running{false};
    std::thread worker;
    std::unique_ptr<Yolov8RknnEngine> engine;
    ThreadSafeQueue<YoloBBoxList> fusionQueue;
    ThreadSafeQueue<YoloBBoxList> osdQueue;
    uint64_t frameCount = 0;
};

namespace {

static uint64_t now_monotonic_ns() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ull + static_cast<uint64_t>(ts.tv_nsec);
}

static uint64_t dma_timestamp_ns(const DmaBuffer_t* buf) {
    if (!buf) return now_monotonic_ns();
    // sentinel-visioner 当前 DmaBuffer_t 使用 timestampUs 保存 CLOCK_MONOTONIC 微秒时间戳。
    if (buf->timestampUs != 0) {
        return static_cast<uint64_t>(buf->timestampUs) * 1000ull;
    }
    return now_monotonic_ns();
}

class NpuBufferGuard {
public:
    NpuBufferGuard(SentinelVisioner* visioner, int camNum, DmaBuffer_t* buf)
        : visioner_(visioner), camNum_(camNum), buf_(buf) {}
    ~NpuBufferGuard() {
        if (visioner_ && buf_) {
            visioner_->release_npu(camNum_, buf_);
        }
    }
    NpuBufferGuard(const NpuBufferGuard&) = delete;
    NpuBufferGuard& operator=(const NpuBufferGuard&) = delete;
    DmaBuffer_t* get() const { return buf_; }

private:
    SentinelVisioner* visioner_ = nullptr;
    int camNum_ = -1;
    DmaBuffer_t* buf_ = nullptr;
};

} // namespace

SentinelYoloInfer::SentinelYoloInfer(SentinelVisioner* visioner, const SentinelYoloInferConfig& config)
    : visioner_(visioner), config_(config) {}

SentinelYoloInfer::SentinelYoloInfer(SentinelVisioner* visioner, const std::string& modelPath)
    : visioner_(visioner) {
    config_.modelPath = modelPath;
}

SentinelYoloInfer::~SentinelYoloInfer() {
    stop_all();
}

bool SentinelYoloInfer::create_infer_thread(int camNum) {
    if (!visioner_) {
        std::cerr << "[SentinelYoloInfer] visioner is null" << std::endl;
        return false;
    }
    if (config_.modelPath.empty()) {
        std::cerr << "[SentinelYoloInfer] modelPath is empty" << std::endl;
        return false;
    }

    std::lock_guard<std::mutex> lock(contextsMutex_);
    auto it = contexts_.find(camNum);
    if (it != contexts_.end() && it->second->running.load()) {
        return true;
    }

    auto ctx = std::make_shared<InferThreadContext>(camNum);
    ctx->engine.reset(new Yolov8RknnEngine());
    if (!ctx->engine->init(config_.modelPath, config_.boxThreshold, config_.nmsThreshold)) {
        std::cerr << "[SentinelYoloInfer] init YOLOv8 RKNN engine failed, camNum=" << camNum << std::endl;
        return false;
    }

    ctx->running.store(true);
    ctx->worker = std::thread(&SentinelYoloInfer::infer_thread_loop_, this, ctx);
    contexts_[camNum] = ctx;

    std::cout << "[SentinelYoloInfer] infer thread started, camNum=" << camNum << std::endl;
    return true;
}

void SentinelYoloInfer::stop_infer_thread(int camNum) {
    std::shared_ptr<InferThreadContext> ctx;
    {
        std::lock_guard<std::mutex> lock(contextsMutex_);
        auto it = contexts_.find(camNum);
        if (it == contexts_.end()) return;
        ctx = it->second;
        contexts_.erase(it);
    }

    ctx->running.store(false);
    if (ctx->worker.joinable()) {
        ctx->worker.join();
    }
    if (ctx->engine) {
        ctx->engine->release();
    }
    std::cout << "[SentinelYoloInfer] infer thread stopped, camNum=" << camNum << std::endl;
}

void SentinelYoloInfer::stop_all() {
    std::vector<std::shared_ptr<InferThreadContext>> all;
    {
        std::lock_guard<std::mutex> lock(contextsMutex_);
        for (auto& kv : contexts_) {
            all.push_back(kv.second);
        }
        contexts_.clear();
    }

    for (auto& ctx : all) {
        if (!ctx) continue;
        ctx->running.store(false);
    }
    for (auto& ctx : all) {
        if (!ctx) continue;
        if (ctx->worker.joinable()) {
            ctx->worker.join();
        }
        if (ctx->engine) {
            ctx->engine->release();
        }
    }
}

bool SentinelYoloInfer::is_running(int camNum) const {
    auto ctx = get_context_(camNum);
    return ctx && ctx->running.load();
}

YoloBBoxList SentinelYoloInfer::wait_get_fusion_result(int camNum) {
    auto ctx = get_context_(camNum);
    if (!ctx) return {};
    return ctx->fusionQueue.pop();
}

bool SentinelYoloInfer::try_get_fusion_result(int camNum, YoloBBoxList& out, int timeoutMs) {
    auto ctx = get_context_(camNum);
    if (!ctx) return false;
    return ctx->fusionQueue.try_pop(out, timeoutMs);
}

YoloBBoxList SentinelYoloInfer::wait_get_osd_result(int camNum) {
    auto ctx = get_context_(camNum);
    if (!ctx) return {};
    return ctx->osdQueue.pop();
}

bool SentinelYoloInfer::try_get_osd_result(int camNum, YoloBBoxList& out, int timeoutMs) {
    auto ctx = get_context_(camNum);
    if (!ctx) return false;
    return ctx->osdQueue.try_pop(out, timeoutMs);
}

std::shared_ptr<SentinelYoloInfer::InferThreadContext> SentinelYoloInfer::get_context_(int camNum) const {
    std::lock_guard<std::mutex> lock(contextsMutex_);
    auto it = contexts_.find(camNum);
    return it == contexts_.end() ? nullptr : it->second;
}

void SentinelYoloInfer::infer_thread_loop_(std::shared_ptr<InferThreadContext> ctx) {
    if (!ctx || !ctx->engine) return;

    const int camNum = ctx->camNum;
    while (ctx->running.load()) {
        DmaBuffer_t* npuBuf = nullptr;

        if (config_.waitTimeoutMs > 0) {
            // 使用 try_get_npu 避免 stop_infer_thread 时卡死；其底层仍来自 sentinel-visioner 的 NPU 小图队列。
            npuBuf = visioner_->try_get_npu(camNum, config_.waitTimeoutMs);
            if (!npuBuf) continue;
        } else {
            // 需要极致低开销且无需优雅退出时，可以把 waitTimeoutMs 设为 <=0 使用纯阻塞接口。
            npuBuf = visioner_->wait_get_npu(camNum);
            if (!npuBuf) continue;
        }

        NpuBufferGuard guard(visioner_, camNum, npuBuf);
        const uint64_t timestampNs = dma_timestamp_ns(npuBuf);

        YoloBBoxList boxes;
        bool ok = ctx->engine->inferFromDmaBuffer(npuBuf->dmaFd,
                                                  npuBuf->virtAddr,
                                                  npuBuf->bufferSize,
                                                  npuBuf->width,
                                                  npuBuf->height,
                                                  timestampNs,
                                                  boxes);
        if (!ok) {
            std::cerr << "[SentinelYoloInfer] inference failed, camNum=" << camNum << std::endl;
            continue;
        }

        ++ctx->frameCount;
        {
            uint32_t personCount = 0;
            for (const auto& b : boxes) {
                if (b.classId == 0) ++personCount;
            }
            // YOLO 检测日志已关闭
        }

        if (!boxes.empty() || config_.pushEmptyResult) {
            ctx->fusionQueue.push(boxes);
            ctx->osdQueue.push(boxes);
        }
        // guard 析构时自动 release_npu(camNum, npuBuf)
    }
}
