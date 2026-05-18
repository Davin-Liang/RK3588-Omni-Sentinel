/**
 * @brief SentinelStreamer 核心实现 — 推流线程管理、帧循环调度
 */

#include "sentinel_streamer.h"
#include "sentinel-visioner.h"
#include "dma-buffer-pool.h"
#include "mpp_encoder.h"

#include <cstdio>
#include <cstring>
#include <new>
#include <thread>
#include <atomic>

// ---------------------------------------------------------------------------
// RGA 缩放函数 (定义在 rga_scaler.cpp)
// ---------------------------------------------------------------------------
extern bool rga_scale_nv12_1080p_to_720p(int srcFd, int dstFd);

// ============================================================================
// 全局状态回调
// ============================================================================

static StreamerCallback g_callback_ = nullptr;

static void notify_(int camNum, StreamerEvent event, const char* detail)
{
    if (g_callback_) g_callback_(camNum, event, detail);
}

// ============================================================================
// StreamerContext — 单路摄像头的推流/录像上下文
// ============================================================================

struct StreamerContext {
    int camNum;
    SentinelVisioner* visioner;

    // 状态
    std::atomic<bool> threadRunning{false};
    bool streamEnabled;
    bool recordEnabled;
    StreamOsdMode osdMode;

    // 录像参数
    RecordResolution recordResolution;

    // 线程
    std::thread workerThread;

    // 720p 中间缩放缓冲池
    DmaBufferPool* scale720pPool;

    // 推流编码器 (固定 1280×720)
    AVCodecContext* streamEncCtx;

    // 录像编码器 (可为 1080p 或 720p)
    AVCodecContext* recordEncCtx;

    // ffmpeg 子进程管道（推流）
    FILE* ffmpegPipe;
    char streamUrl[256];  // 保存 URL，用于断线重连

    // MP4 输出
    AVFormatContext* mp4Ctx;

    StreamerContext()
        : camNum(-1)
        , visioner(nullptr)
        , threadRunning(false)
        , streamEnabled(false)
        , recordEnabled(false)
        , osdMode(StreamOsdMode::WITHOUT_OSD)
        , recordResolution(RecordResolution::RES_1080P)
        , scale720pPool(nullptr)
        , streamEncCtx(nullptr)
        , recordEncCtx(nullptr)
        , ffmpegPipe(nullptr)
        , mp4Ctx(nullptr)
    {
        streamUrl[0] = '\0';
    }
};

// ============================================================================
// 推流线程主循环
// ============================================================================

static void stream_thread_func_(StreamerContext* ctx)
{
    fprintf(stderr, "[SentinelStreamer] cam=%d stream thread started\n", ctx->camNum);

    uint64_t firstTsUs = 0;  // 首帧时间戳，所有后续帧减它做 PTS 归零
    uint64_t lastLogTs = 0;
    int frameCount = 0;

    while (ctx->threadRunning.load(std::memory_order_acquire)) {
        // 步骤 1: 获取原始 1080p NV12 帧
        DmaBuffer_t* origBuf = ctx->visioner->wait_get_orig_copy_buffer(ctx->camNum);
        if (!origBuf) continue;

        uint64_t tsUs = origBuf->timestampUs;
        if (firstTsUs == 0) firstTsUs = tsUs;

        // 真实时间戳 → MPEG 时基 (90000)，减首帧偏移归零
        int64_t pts = static_cast<int64_t>(tsUs - firstTsUs) * 90000 / 1000000;

        // ----------------------------------------------------------------
        // 步骤 2: RGA 缩放 1080p → 720p (推流和/或 720p 录像共用)
        // ----------------------------------------------------------------
        DmaBuffer_t* scaleBuf = nullptr;
        bool needScale = ctx->streamEnabled ||
                         (ctx->recordEnabled &&
                          ctx->recordResolution == RecordResolution::RES_720P);

        if (needScale) {
            scaleBuf = ctx->scale720pPool->get_buffer();
            if (scaleBuf) {
                scaleBuf->timestampUs = tsUs;
                if (!rga_scale_nv12_1080p_to_720p(origBuf->dmaFd, scaleBuf->dmaFd)) {
                    ctx->scale720pPool->release_buffer(scaleBuf);
                    scaleBuf = nullptr;
                }
            }
        }

        // ----------------------------------------------------------------
        // 步骤 3: 编码 1080p 原始帧 → MP4 录像
        //         avcodec_send_frame 内部 dup(dmaFd)，调用后可归还 origBuf
        // ----------------------------------------------------------------
        if (ctx->recordEnabled &&
            ctx->recordResolution == RecordResolution::RES_1080P &&
            ctx->recordEncCtx)
        {
            encode_and_mux(ctx->recordEncCtx,
                           origBuf->virtAddr, 1920, 1080,
                           pts,
                           nullptr, ctx->mp4Ctx);
        }

        // ----------------------------------------------------------------
        // 步骤 4: 归还原始 1080p DMA 缓冲
        // ----------------------------------------------------------------
        ctx->visioner->release_orig_copy_buffer(ctx->camNum, origBuf);

        // ----------------------------------------------------------------
        // 步骤 5a: 720p RTSP 推流 (streamEncCtx → RTSP)
        // ----------------------------------------------------------------
        if (scaleBuf && ctx->streamEnabled && ctx->streamEncCtx) {
            encode_and_mux(ctx->streamEncCtx,
                           scaleBuf->virtAddr, 1280, 720,
                           pts,
                           ctx->ffmpegPipe, nullptr);
        }

        // ----------------------------------------------------------------
        // 步骤 5b: 720p MP4 录像 (recordEncCtx → MP4)
        // ----------------------------------------------------------------
        if (scaleBuf && ctx->recordEnabled &&
            ctx->recordResolution == RecordResolution::RES_720P &&
            ctx->recordEncCtx)
        {
            encode_and_mux(ctx->recordEncCtx,
                           scaleBuf->virtAddr, 1280, 720,
                           pts,
                           nullptr, ctx->mp4Ctx);
        }

        if (scaleBuf) {
            ctx->scale720pPool->release_buffer(scaleBuf);
        }

        // ffmpeg 子进程断线重连
        if (ctx->ffmpegPipe && ctx->streamEnabled && ferror(ctx->ffmpegPipe)) {
            fprintf(stderr, "[SentinelStreamer] cam=%d ffmpeg pipe broken, reconnecting...\n",
                    ctx->camNum);
            ffmpeg_stream_close(ctx->ffmpegPipe);
            ctx->ffmpegPipe = ffmpeg_stream_open(ctx->streamUrl);
            if (ctx->ffmpegPipe) {
                fprintf(stderr, "[SentinelStreamer] cam=%d ffmpeg reconnected\n", ctx->camNum);
                notify_(ctx->camNum, StreamerEvent::STREAM_STARTED, ctx->streamUrl);
            } else {
                fprintf(stderr, "[SentinelStreamer] cam=%d ffmpeg reconnect failed\n", ctx->camNum);
                notify_(ctx->camNum, StreamerEvent::ERROR, "ffmpeg reconnect failed");
            }
        }

        // FPS 诊断（每 30 帧打印一次）
        ++frameCount;
        if (frameCount % 30 == 0) {
            if (lastLogTs > 0 && tsUs > lastLogTs) {
                double fps = 30.0 * 1000000.0 / static_cast<double>(tsUs - lastLogTs);
                fprintf(stderr, "[SentinelStreamer] cam=%d FPS: %.1f\n",
                        ctx->camNum, fps);
            }
            lastLogTs = tsUs;
        }
    }

    fprintf(stderr, "[SentinelStreamer] cam=%d stream thread stopped\n", ctx->camNum);
}

// ============================================================================
// SentinelStreamer 实现
// ============================================================================

SentinelStreamer::SentinelStreamer()
{
    memset(contexts_, 0, sizeof(contexts_));
    ffmpeg_init_once();
}

SentinelStreamer::~SentinelStreamer()
{
    for (int i = 0; i < 2; ++i) {
        if (contexts_[i]) {
            remove_camera(i);
        }
    }
}

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------

bool SentinelStreamer::add_camera(int camNum, SentinelVisioner* visioner, int poolSize)
{
    if (camNum < 0 || camNum > 1) {
        fprintf(stderr, "[SentinelStreamer] camNum=%d out of range [0,1]\n", camNum);
        return false;
    }

    if (!visioner) {
        fprintf(stderr, "[SentinelStreamer] visioner is null\n");
        return false;
    }

    if (contexts_[camNum]) {
        fprintf(stderr, "[SentinelStreamer] cam=%d already added\n", camNum);
        return false;
    }

    StreamerContext* ctx = new (std::nothrow) StreamerContext();
    if (!ctx) {
        fprintf(stderr, "[SentinelStreamer] alloc StreamerContext failed\n");
        return false;
    }

    ctx->camNum   = camNum;
    ctx->visioner = visioner;

    // 创建 720p 中间缩放缓冲池
    ctx->scale720pPool = new (std::nothrow) DmaBufferPool();
    if (!ctx->scale720pPool) {
        fprintf(stderr, "[SentinelStreamer] alloc scale720pPool failed\n");
        delete ctx;
        return false;
    }
    if (!ctx->scale720pPool->alloc_pool(poolSize, 1280, 720, BufferFormat::NV12)) {
        fprintf(stderr, "[SentinelStreamer] scale720pPool alloc_pool failed\n");
        delete ctx->scale720pPool;
        delete ctx;
        return false;
    }

    // 编码器惰性创建：start_stream / start_record 时才各自初始化
    contexts_[camNum] = ctx;
    fprintf(stderr, "[SentinelStreamer] cam=%d added\n", camNum);
    return true;
}

bool SentinelStreamer::remove_camera(int camNum)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) {
        return false;
    }

    StreamerContext* ctx = contexts_[camNum];

    if (ctx->streamEnabled) stop_stream(camNum);
    if (ctx->recordEnabled) stop_record(camNum);

    mpp_encoder_close(&ctx->streamEncCtx);
    mpp_encoder_close(&ctx->recordEncCtx);

    if (ctx->scale720pPool) {
        ctx->scale720pPool->destroy_pool();
        delete ctx->scale720pPool;
        ctx->scale720pPool = nullptr;
    }

    delete ctx;
    contexts_[camNum] = nullptr;

    fprintf(stderr, "[SentinelStreamer] cam=%d removed\n", camNum);
    return true;
}

// ---------------------------------------------------------------------------
// 推流
// ---------------------------------------------------------------------------

bool SentinelStreamer::start_stream(int camNum, const char* rtspUrl)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) {
        fprintf(stderr, "[SentinelStreamer] cam=%d not found\n", camNum);
        return false;
    }

    StreamerContext* ctx = contexts_[camNum];

    if (ctx->streamEnabled) {
        fprintf(stderr, "[SentinelStreamer] cam=%d already streaming\n", camNum);
        return false;
    }

    if (!rtspUrl || !rtspUrl[0]) {
        fprintf(stderr, "[SentinelStreamer] cam=%d empty rtspUrl\n", camNum);
        return false;
    }

    // 编码器可能已销毁（上一轮 stop_stream），重建
    if (!ctx->streamEncCtx) {
        if (!mpp_encoder_open(&ctx->streamEncCtx, 1280, 720, 4000000)) {
            return false;
        }
    }

    ctx->ffmpegPipe = ffmpeg_stream_open(rtspUrl);
    if (!ctx->ffmpegPipe) {
        return false;
    }
    snprintf(ctx->streamUrl, sizeof(ctx->streamUrl), "%s", rtspUrl);

    ctx->streamEnabled = true;

    if (!ctx->threadRunning.load(std::memory_order_acquire)) {
        ctx->threadRunning.store(true, std::memory_order_release);
        ctx->workerThread = std::thread(stream_thread_func_, ctx);
    }

    fprintf(stderr, "[SentinelStreamer] cam=%d streaming started\n", camNum);
    notify_(camNum, StreamerEvent::STREAM_STARTED, rtspUrl);
    return true;
}

bool SentinelStreamer::stop_stream(int camNum)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) {
        return false;
    }

    StreamerContext* ctx = contexts_[camNum];

    if (!ctx->streamEnabled) {
        return false;
    }

    // 先停线程，再关输出、销毁编码器
    bool threadStopped = !ctx->recordEnabled;
    if (threadStopped) {
        fprintf(stderr, "[SentinelStreamer] DEBUG: stopping thread...\n");
        ctx->threadRunning.store(false, std::memory_order_release);
        if (ctx->workerThread.joinable()) {
            ctx->workerThread.join();
            fprintf(stderr, "[SentinelStreamer] DEBUG: thread joined\n");
        }
    }

    ctx->streamEnabled = false;
    fprintf(stderr, "[SentinelStreamer] DEBUG: closing ffmpeg pipe...\n");
    ffmpeg_stream_close(ctx->ffmpegPipe);
    ctx->ffmpegPipe = nullptr;

    // 只在确定线程已停时安全清理（编码器 + 可能延迟关闭的 mp4Ctx）
    if (threadStopped) {
        fprintf(stderr, "[SentinelStreamer] DEBUG: closing both encoders...\n");
        if (ctx->mp4Ctx) {
            mp4_output_close(&ctx->mp4Ctx);
        }
        mpp_encoder_close(&ctx->streamEncCtx);
        mpp_encoder_close(&ctx->recordEncCtx);
    }

    fprintf(stderr, "[SentinelStreamer] cam=%d streaming stopped\n", camNum);
    notify_(camNum, StreamerEvent::STREAM_STOPPED, nullptr);
    return true;
}

bool SentinelStreamer::is_streaming(int camNum) const
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) return false;
    return contexts_[camNum]->streamEnabled;
}

// ---------------------------------------------------------------------------
// 推流 OSD 模式
// ---------------------------------------------------------------------------

bool SentinelStreamer::set_stream_osd_mode(int camNum, StreamOsdMode mode)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) {
        return false;
    }

    StreamerContext* ctx = contexts_[camNum];

    if (ctx->streamEnabled) {
        fprintf(stderr, "[SentinelStreamer] cannot change OSD mode while streaming\n");
        return false;
    }

    ctx->osdMode = mode;

    if (mode == StreamOsdMode::WITH_OSD) {
        // 预留：有 OSD 的 720p 推流暂未实现
        fprintf(stderr, "[SentinelStreamer] OSD mode set to WITH_OSD (stub, not implemented)\n");
    }

    return true;
}

// ---------------------------------------------------------------------------
// 录像
// ---------------------------------------------------------------------------

bool SentinelStreamer::start_record(int camNum, const char* filePath,
                                    RecordResolution resolution)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) {
        fprintf(stderr, "[SentinelStreamer] cam=%d not found\n", camNum);
        return false;
    }

    StreamerContext* ctx = contexts_[camNum];

    if (ctx->recordEnabled) {
        fprintf(stderr, "[SentinelStreamer] cam=%d already recording\n", camNum);
        return false;
    }

    if (!filePath || !filePath[0]) {
        fprintf(stderr, "[SentinelStreamer] cam=%d empty filePath\n", camNum);
        return false;
    }

    int recWidth, recHeight, recBitRate;
    if (resolution == RecordResolution::RES_1080P) {
        recWidth  = 1920;
        recHeight = 1080;
        recBitRate = 8000000;
    } else {
        recWidth  = 1280;
        recHeight = 720;
        recBitRate = 4000000;
    }

    ctx->recordResolution = resolution;

    // 编码器可能已销毁（上一轮 stop_record）或分辨率不符，重建
    if (ctx->recordEncCtx &&
        (ctx->recordEncCtx->width != recWidth || ctx->recordEncCtx->height != recHeight))
    {
        mpp_encoder_close(&ctx->recordEncCtx);
    }

    if (!ctx->recordEncCtx) {
        if (!mpp_encoder_open(&ctx->recordEncCtx, recWidth, recHeight, recBitRate)) {
            return false;
        }
    }

    if (!mp4_output_open(&ctx->mp4Ctx, ctx->recordEncCtx, filePath)) {
        return false;
    }

    ctx->recordEnabled = true;

    if (!ctx->threadRunning.load(std::memory_order_acquire)) {
        ctx->threadRunning.store(true, std::memory_order_release);
        ctx->workerThread = std::thread(stream_thread_func_, ctx);
    }

    fprintf(stderr, "[SentinelStreamer] cam=%d recording started (%dp)\n",
            camNum, recHeight);
    notify_(camNum, StreamerEvent::RECORD_STARTED, filePath);
    return true;
}

bool SentinelStreamer::stop_record(int camNum)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) {
        return false;
    }

    StreamerContext* ctx = contexts_[camNum];

    if (!ctx->recordEnabled) {
        return false;
    }

    ctx->recordEnabled = false;

    // 只有线程已停的情况下才安全关闭 mp4Ctx 和销毁编码器
    // 否则推迟到 stop_stream 处理（等线程真的停了再关）
    if (!ctx->streamEnabled) {
        ctx->threadRunning.store(false, std::memory_order_release);
        if (ctx->workerThread.joinable()) {
            ctx->workerThread.join();
        }
        fprintf(stderr, "[SentinelStreamer] DEBUG: closing MP4...\n");
        mp4_output_close(&ctx->mp4Ctx);
        mpp_encoder_close(&ctx->streamEncCtx);
        mpp_encoder_close(&ctx->recordEncCtx);
    }

    fprintf(stderr, "[SentinelStreamer] cam=%d recording stopped\n", camNum);
    notify_(camNum, StreamerEvent::RECORD_STOPPED, nullptr);
    return true;
}

bool SentinelStreamer::is_recording(int camNum) const
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) return false;
    return contexts_[camNum]->recordEnabled;
}

// ---------------------------------------------------------------------------
// 状态回调
// ---------------------------------------------------------------------------

void SentinelStreamer::set_callback(StreamerCallback cb)
{
    g_callback_ = cb;
}
