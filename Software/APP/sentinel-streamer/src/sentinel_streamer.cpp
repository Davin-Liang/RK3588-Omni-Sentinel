/**
 * @brief SentinelStreamer 核心实现 — 推流线程管理、帧循环调度
 */

#include "sentinel_streamer.h"
#include "sentinel-visioner.h"
#include "dma-buffer-pool.h"
#include "record_buffer_pool.h"
#include "mpp_encoder.h"

#include <cstdio>
#include <cstring>
#include <new>
#include <thread>
#include <atomic>

// ---------------------------------------------------------------------------
// RGA 缩放函数 (定义在 rga_scaler.cpp)
// ---------------------------------------------------------------------------
extern bool rga_scale_nv12_to_720p(int srcFd, int srcWidth, int srcHeight, int dstFd);

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
    uint64_t baselineTsUs;  // 录制/推流开始时的系统时间，PTS 以此为基准
    StreamOsdMode osdMode;
    StreamOsdProvider osdProvider;

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

    // 录像帧环形缓冲池
    RecordBufferPool* recordPool;

    StreamerContext()
        : camNum(-1)
        , visioner(nullptr)
        , threadRunning(false)
        , streamEnabled(false)
        , recordEnabled(false)
        , baselineTsUs(0)
        , osdMode(StreamOsdMode::WITHOUT_OSD)
        , recordResolution(RecordResolution::RES_1080P)
        , scale720pPool(nullptr)
        , streamEncCtx(nullptr)
        , recordEncCtx(nullptr)
        , ffmpegPipe(nullptr)
        , mp4Ctx(nullptr)
        , recordPool(nullptr)
    {
        streamUrl[0] = '\0';
    }
};

// ============================================================================
// OSD 绘制 — NV12 矩形边框 + 文字标签（CPU 直接写 DMA buffer）
// ============================================================================

// 3x5 迷你点阵字体（数字 0-9 + "person" 字符 + '.'）
namespace {
struct Glyph { uint8_t w; uint8_t rows[5]; };  // 最大 5 行

bool glyphsInit = false;
Glyph g[128];

void initGlyphs_() {
    if (glyphsInit) return;
    // clang-format off
    g['0'] = {3, {0x7,0x5,0x5,0x5,0x7}};
    g['1'] = {3, {0x2,0x6,0x2,0x2,0x7}};
    g['2'] = {3, {0x7,0x1,0x7,0x4,0x7}};
    g['3'] = {3, {0x7,0x1,0x3,0x1,0x7}};
    g['4'] = {3, {0x5,0x5,0x7,0x1,0x1}};
    g['5'] = {3, {0x7,0x4,0x7,0x1,0x7}};
    g['6'] = {3, {0x7,0x4,0x7,0x5,0x7}};
    g['7'] = {3, {0x7,0x1,0x2,0x2,0x2}};
    g['8'] = {3, {0x7,0x5,0x7,0x5,0x7}};
    g['9'] = {3, {0x7,0x5,0x7,0x1,0x7}};
    g['.'] = {1, {0x0,0x0,0x0,0x0,0x1}};
    g['p'] = {3, {0x7,0x5,0x7,0x4,0x4}};
    g['e'] = {3, {0x7,0x4,0x7,0x4,0x7}};
    g['r'] = {3, {0x5,0x6,0x4,0x4,0x4}};
    g['s'] = {3, {0x7,0x4,0x7,0x1,0x7}};
    g['o'] = {3, {0x7,0x5,0x5,0x5,0x7}};
    g['n'] = {3, {0x7,0x5,0x5,0x5,0x5}};
    g[' '] = {2, {0x0,0x0,0x0,0x0,0x0}};
    // clang-format on
    glyphsInit = true;
}

// 在 Y 平面绘制 2x 放大的 3x5 字体字符（最终 6x10 px），返回绘制宽度
int draw_char_2x(uint8_t* yPlane, int stride, int cx, int cy, char ch, uint8_t val) {
    const Glyph& gl = g[static_cast<unsigned char>(ch)];
    if (gl.w == 0) return 0;
    for (int row = 0; row < 5; ++row) {
        for (int col = 0; col < gl.w; ++col) {
            if (gl.rows[row] & (1 << (gl.w - 1 - col))) {
                int py = cy + row * 2;
                int px = cx + col * 2;
                yPlane[py * stride + px] = val;
                yPlane[py * stride + px + 1] = val;
                yPlane[(py + 1) * stride + px] = val;
                yPlane[(py + 1) * stride + px + 1] = val;
            }
        }
    }
    return gl.w * 2 + 1;  // char width + 1px gap
}

// 绘制文字串，返回总宽度
int draw_text_2x(uint8_t* yPlane, int stride, int cx, int cy,
                  const char* text, uint8_t val) {
    initGlyphs_();
    int x = cx;
    for (const char* p = text; *p; ++p) {
        x += draw_char_2x(yPlane, stride, x, cy, *p, val);
    }
    return x - cx;
}
} // anonymous namespace

static void draw_osd_boxes_(void* virtAddr, const std::vector<StreamOsdBBox>& boxes,
                            int srcWidth, int srcHeight)
{
    if (!virtAddr || boxes.empty() || srcWidth <= 0 || srcHeight <= 0) return;

    initGlyphs_();

    uint8_t* yPlane = static_cast<uint8_t*>(virtAddr);
    const int kStride = 1280;
    const int kMaxY = 719;

    // 计算 letterbox 参数（与 rga_process_to_rgb_ 一致）
    float scale   = (std::min)(640.0f / srcWidth, 640.0f / srcHeight);
    float scaledH = srcHeight * scale;
    float padY    = (640.0f - scaledH) / 2.0f;

    for (const auto& b : boxes) {
        if (b.classId != 0) continue;   // 只叠加 person

        // 640x640 NPU letterbox → 原始分辨率
        float xo = b.x1 / scale;
        float yo = (b.y1 - padY) / scale;
        float wo = (b.x2 - b.x1) / scale;
        float ho = (b.y2 - b.y1) / scale;

        // 原始分辨率 → 1280x720
        int bx1 = static_cast<int>(xo * 1280.0f / srcWidth);
        int by1 = static_cast<int>(yo * 720.0f / srcHeight);
        int bx2 = static_cast<int>((xo + wo) * 1280.0f / srcWidth);
        int by2 = static_cast<int>((yo + ho) * 720.0f / srcHeight);

        // 裁剪
        bx1 = (std::max)(0, bx1); by1 = (std::max)(0, by1);
        bx2 = (std::min)(kStride - 1, bx2); by2 = (std::min)(kMaxY, by2);
        if (bx1 >= bx2 || by1 >= by2) continue;

        // 画白色边框（Y 平面 2px，不碰 UV）
        for (int x = bx1; x <= bx2; ++x) {
            yPlane[by1 * kStride + x] = 255;
            if (by1 + 1 < 720) yPlane[(by1 + 1) * kStride + x] = 255;
            yPlane[by2 * kStride + x] = 255;
            if (by2 - 1 >= 0) yPlane[(by2 - 1) * kStride + x] = 255;
        }
        for (int y = by1; y <= by2; ++y) {
            yPlane[y * kStride + bx1] = 255;
            if (bx1 + 1 < kStride) yPlane[y * kStride + bx1 + 1] = 255;
            yPlane[y * kStride + bx2] = 255;
            if (bx2 - 1 >= 0) yPlane[y * kStride + bx2 - 1] = 255;
        }

        // 画标签（框顶上方，深色底 + 白色字）
        char label[32];
        snprintf(label, sizeof(label), "person %.2f",
                 static_cast<double>(b.confidence));
        int labelW = 0;
        for (const char* p = label; *p; ++p) {
            labelW += g[static_cast<unsigned char>(*p)].w * 2 + 1;
        }
        int lx = bx1 + 3;
        int ly = (std::max)(2, by1 - 12);  // 标签 10px 高 + 2px 边距
        // 标签背景
        for (int y = ly; y < ly + 11 && y < 720; ++y) {
            for (int x = lx; x < lx + labelW + 4 && x < kStride; ++x) {
                yPlane[y * kStride + x] = 60;  // 深灰背景
            }
        }
        // 标签文字（白色）
        draw_text_2x(yPlane, kStride, lx + 2, ly + 1, label, 255);
    }
}

// ============================================================================
// 推流线程主循环
// ============================================================================

static void stream_thread_func_(StreamerContext* ctx)
{
    fprintf(stderr, "[SentinelStreamer] cam=%d stream thread started\n", ctx->camNum);

    uint64_t firstTsUs = ctx->baselineTsUs;
    uint64_t lastLogTs = 0;
    int frameCount = 0;

    fprintf(stderr, "[SentinelStreamer] baselineTsUs=%llu\n", (unsigned long long)firstTsUs);

    while (ctx->threadRunning.load(std::memory_order_acquire)) {
        // 步骤 1: 获取原始 1080p NV12 帧
        DmaBuffer_t* origBuf = ctx->visioner->wait_get_orig_copy_buffer(ctx->camNum);
        if (!origBuf) continue;

        // 步骤 1.5: 写入录像帧环形缓冲池（RGA DMA 硬件拷贝，供磁盘写入线程消费）
        if (ctx->recordPool) {
            ctx->recordPool->write_frame(
                origBuf->dmaFd, origBuf->width, origBuf->height, origBuf->timestampUs);
        }

        uint64_t tsUs = origBuf->timestampUs;

        // 跳过队列中积压的旧帧（时间戳在录制开始之前）
        if (tsUs < firstTsUs) {
            ctx->visioner->release_orig_copy_buffer(ctx->camNum, origBuf);
            continue;
        }

        // 真实时间戳 → MPEG 时基 (90000)，减基准偏移归零
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
                if (!rga_scale_nv12_to_720p(origBuf->dmaFd, origBuf->width, origBuf->height, scaleBuf->dmaFd)) {
                    ctx->scale720pPool->release_buffer(scaleBuf);
                    scaleBuf = nullptr;
                }
            }
        }

        // ----------------------------------------------------------------
        // 步骤 3: 原始帧直接编码 → MP4 录像
        //   - 1080p 录像: 使用 origBuf
        //   - 720p 录像且源已是 720p (如 USB 相机): 直接编码 origBuf，绕过 RGA 缩放
        // ----------------------------------------------------------------
        bool srcAlready720p = (origBuf->width == 1280 && origBuf->height == 720);
        if (ctx->recordEnabled && ctx->recordEncCtx) {
            if (ctx->recordResolution == RecordResolution::RES_1080P) {
                encode_and_mux(ctx->recordEncCtx,
                               origBuf->virtAddr, origBuf->width, origBuf->height,
                               pts,
                               nullptr, ctx->mp4Ctx);
            } else if (ctx->recordResolution == RecordResolution::RES_720P && srcAlready720p) {
                encode_and_mux(ctx->recordEncCtx,
                               origBuf->virtAddr, origBuf->width, origBuf->height,
                               pts,
                               nullptr, ctx->mp4Ctx);
            }
        }

        // ----------------------------------------------------------------
        // 步骤 4: 归还原始 DMA 缓冲
        // ----------------------------------------------------------------
        int srcW = origBuf->width;
        int srcH = origBuf->height;
        ctx->visioner->release_orig_copy_buffer(ctx->camNum, origBuf);

        // ----------------------------------------------------------------
        // 步骤 5a: 720p RTSP 推流 (streamEncCtx → RTSP)
        // ----------------------------------------------------------------
        if (scaleBuf && ctx->streamEnabled && ctx->streamEncCtx) {
            if (ctx->osdMode == StreamOsdMode::WITH_OSD && ctx->osdProvider) {
                std::vector<StreamOsdBBox> boxes;
                if (ctx->osdProvider(ctx->camNum, boxes, 5)) {
                    draw_osd_boxes_(scaleBuf->virtAddr, boxes, srcW, srcH);
                }
            }
            encode_and_mux(ctx->streamEncCtx,
                           scaleBuf->virtAddr, 1280, 720,
                           pts,
                           ctx->ffmpegPipe, nullptr);
        }

        // ----------------------------------------------------------------
        // 步骤 5b: 720p MP4 录像 (1080p 源 → RGA 缩放 → MP4)
        // ----------------------------------------------------------------
        if (scaleBuf && ctx->recordEnabled &&
            ctx->recordResolution == RecordResolution::RES_720P &&
            !srcAlready720p &&
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

    // 创建录像帧环形缓冲池（不立即分配 DMA，等 Widget 调 init_record_buffer）
    ctx->recordPool = new (std::nothrow) RecordBufferPool();
    if (!ctx->recordPool) {
        fprintf(stderr, "[SentinelStreamer] alloc RecordBufferPool failed\n");
        ctx->scale720pPool->destroy_pool();
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

    if (ctx->recordPool) {
        ctx->recordPool->destroy_pool();
        delete ctx->recordPool;
        ctx->recordPool = nullptr;
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
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        ctx->baselineTsUs = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;
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
    contexts_[camNum]->osdMode = mode;
    return true;
}

void SentinelStreamer::set_osd_provider(StreamOsdProvider provider)
{
    for (int i = 0; i < 2; ++i) {
        if (contexts_[i]) {
            contexts_[i]->osdProvider = provider;
        }
    }
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

    // 每次开始录像时更新基准时间戳，确保新文件 PTS 从 0 开始
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    ctx->baselineTsUs = (uint64_t)ts.tv_sec * 1000000LL + ts.tv_nsec / 1000LL;

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

// ---------------------------------------------------------------------------
// 录像帧缓冲
// ---------------------------------------------------------------------------

bool SentinelStreamer::init_record_buffer(int camNum, int slotCount,
                                           int width, int height)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) return false;
    StreamerContext* ctx = contexts_[camNum];
    if (!ctx->recordPool) return false;
    return ctx->recordPool->alloc_pool(slotCount, width, height);
}

bool SentinelStreamer::try_get_record_frame(int camNum, uint8_t** outData,
                                             size_t* outSize,
                                             uint64_t* outTimestampUs)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) return false;
    StreamerContext* ctx = contexts_[camNum];
    if (!ctx->recordPool) return false;
    return ctx->recordPool->try_get_record_frame(outData, outSize, outTimestampUs);
}

void SentinelStreamer::release_record_frame(int camNum, uint8_t* data)
{
    if (camNum < 0 || camNum > 1 || !contexts_[camNum]) return;
    StreamerContext* ctx = contexts_[camNum];
    if (ctx->recordPool) {
        ctx->recordPool->release_record_frame(data);
    }
}
