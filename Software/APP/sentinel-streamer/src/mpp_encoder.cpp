/**
 * @brief MPP 硬件编码器、ffmpeg 管道推流、MP4 复用器
 */

#include "mpp_encoder.h"

#include <cstdio>
#include <cstring>
#include <ctime>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}

// ============================================================================
// 全局初始化
// ============================================================================

static bool ffmpeg_initialized_ = false;

void ffmpeg_init_once()
{
    if (ffmpeg_initialized_) return;
    ffmpeg_initialized_ = true;
}

// ============================================================================
// MPP 编码器
// ============================================================================

bool mpp_encoder_open(AVCodecContext** outCtx, int width, int height, int bitRate)
{
    *outCtx = nullptr;

    const AVCodec* codec = avcodec_find_encoder_by_name("h264_rkmpp");
    if (!codec) {
        fprintf(stderr, "[MppEncoder] h264_rkmpp encoder not found\n");
        return false;
    }

    AVCodecContext* ctx = avcodec_alloc_context3(codec);
    if (!ctx) {
        fprintf(stderr, "[MppEncoder] avcodec_alloc_context3 failed\n");
        return false;
    }

    ctx->width     = width;
    ctx->height    = height;
    ctx->time_base = AVRational{1, 90000};
    ctx->framerate = AVRational{15, 1};
    ctx->gop_size  = 15;
    ctx->bit_rate  = bitRate;
    ctx->max_b_frames = 0;
    ctx->pix_fmt = AV_PIX_FMT_NV12;

    ctx->color_range     = AVCOL_RANGE_JPEG;
    ctx->color_primaries = AVCOL_PRI_BT709;
    ctx->color_trc       = AVCOL_TRC_BT709;
    ctx->colorspace      = AVCOL_SPC_BT709;

    av_opt_set_int(ctx->priv_data, "rc_mode", 0, 0);  // VBR
    av_opt_set_int(ctx->priv_data, "delay", 0, 0);

    int ret = avcodec_open2(ctx, codec, nullptr);
    if (ret < 0) {
        char errBuf[256];
        av_strerror(ret, errBuf, sizeof(errBuf));
        fprintf(stderr, "[MppEncoder] avcodec_open2 failed: %s\n", errBuf);
        avcodec_free_context(&ctx);
        return false;
    }

    *outCtx = ctx;
    return true;
}

void mpp_encoder_flush(AVCodecContext* encCtx, AVFormatContext* mp4Ctx)
{
    if (!encCtx) return;

    // 先收完编码器已缓冲的包（sent_frame 已产生的）
    AVPacket* pkt = av_packet_alloc();
    int64_t lastDts = INT64_MIN;
    int ret;
    while (true) {
        ret = avcodec_receive_packet(encCtx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
        if (ret < 0) break;
        if (mp4Ctx && pkt->size > 0) {
            pkt->stream_index = 0;
            av_write_frame(mp4Ctx, pkt);
        }
        av_packet_unref(pkt);
    }

    av_packet_free(&pkt);
}

void mpp_encoder_close(AVCodecContext** ctx)
{
    if (*ctx) {
        avcodec_free_context(ctx);
        *ctx = nullptr;
    }
}

// ============================================================================
// ffmpeg 子进程推流
// ============================================================================

FILE* ffmpeg_stream_open(const char* url)
{
    if (!url || !url[0]) return nullptr;

    char cmd[512];
    snprintf(cmd, sizeof(cmd),
             "ffmpeg -loglevel error -f h264 -i pipe:0 "
             "-c copy -rtsp_transport tcp -f rtsp %s",
             url);

    FILE* pipe = popen(cmd, "w");
    if (!pipe) {
        fprintf(stderr, "[MppEncoder] popen ffmpeg failed: %s\n", cmd);
        return nullptr;
    }

    fprintf(stderr, "[MppEncoder] ffmpeg stream started: %s\n", url);
    return pipe;
}

void ffmpeg_stream_close(FILE* pipe)
{
    if (pipe) {
        pclose(pipe);
        fprintf(stderr, "[MppEncoder] ffmpeg stream stopped\n");
    }
}

// ============================================================================
// MP4 复用器
// ============================================================================

bool mp4_output_open(AVFormatContext** outCtx, AVCodecContext* encCtx,
                     const char* filePath)
{
    *outCtx = nullptr;

    int ret = avformat_alloc_output_context2(outCtx, nullptr, "mp4", filePath);
    if (ret < 0 || !*outCtx) {
        fprintf(stderr, "[MppEncoder] avformat_alloc_output_context2(mp4) failed\n");
        return false;
    }

    AVStream* stream = avformat_new_stream(*outCtx, nullptr);
    if (!stream) {
        fprintf(stderr, "[MppEncoder] avformat_new_stream(mp4) failed\n");
        avformat_free_context(*outCtx);
        *outCtx = nullptr;
        return false;
    }

    ret = avcodec_parameters_from_context(stream->codecpar, encCtx);
    if (ret < 0) {
        fprintf(stderr, "[MppEncoder] avcodec_parameters_from_context(mp4) failed\n");
        avformat_free_context(*outCtx);
        *outCtx = nullptr;
        return false;
    }
    stream->time_base = encCtx->time_base;

    ret = avio_open(&(*outCtx)->pb, filePath, AVIO_FLAG_WRITE);
    if (ret < 0) {
        fprintf(stderr, "[MppEncoder] avio_open(mp4) failed\n");
        avformat_free_context(*outCtx);
        *outCtx = nullptr;
        return false;
    }

    ret = avformat_write_header(*outCtx, nullptr);
    if (ret < 0) {
        fprintf(stderr, "[MppEncoder] avformat_write_header(mp4) failed\n");
        avio_closep(&(*outCtx)->pb);
        avformat_free_context(*outCtx);
        *outCtx = nullptr;
        return false;
    }

    fprintf(stderr, "[MppEncoder] MP4 output opened: %s\n", filePath);
    return true;
}

void mp4_output_close(AVFormatContext** ctx)
{
    if (*ctx) {
        av_write_trailer(*ctx);
        avio_closep(&(*ctx)->pb);
        avformat_free_context(*ctx);
        *ctx = nullptr;
    }
}

// ============================================================================
// 编码 + 复用
// ============================================================================

bool ensure_enc_frame(AVFrame** frame, int width, int height)
{
    if (*frame && (*frame)->width == width && (*frame)->height == height)
        return true;  // 尺寸匹配，复用

    // 尺寸变化或首次分配
    av_frame_free(frame);
    *frame = av_frame_alloc();
    if (!*frame) return false;
    (*frame)->format = AV_PIX_FMT_NV12;
    (*frame)->width  = width;
    (*frame)->height = height;
    int ret = av_frame_get_buffer(*frame, 0);
    if (ret < 0) {
        av_frame_free(frame);
        return false;
    }
    return true;
}

bool encode_and_mux(AVCodecContext* encCtx, void* virtAddr, int width, int height,
                    int64_t pts,
                    FILE* streamPipe, AVFormatContext* mp4Ctx,
                    AVFrame* encFrame, AVPacket* encPkt)
{
    if (!encCtx || !virtAddr) return true;

    bool ownFrame = (encFrame == nullptr);
    AVFrame* frame = encFrame;
    if (ownFrame) {
        frame = av_frame_alloc();
        if (!frame) { fprintf(stderr, "[MppEncoder] av_frame_alloc failed\n"); return false; }
        frame->format = AV_PIX_FMT_NV12;
        frame->width  = width;
        frame->height = height;
        av_frame_get_buffer(frame, 0);
    } else {
        frame->pts = pts;
        frame->width  = width;
        frame->height = height;
    }

    uint8_t* src = static_cast<uint8_t*>(virtAddr);
    size_t ySize  = static_cast<size_t>(width) * height;
    std::memcpy(frame->data[0], src, ySize);
    std::memcpy(frame->data[1], src + ySize, ySize / 2);

    struct timespec tEnc0, tEnc1;
    clock_gettime(CLOCK_MONOTONIC, &tEnc0);

    int64_t sentPts = pts;
    int ret = avcodec_send_frame(encCtx, frame);
    if (ownFrame) av_frame_free(&frame);

    if (ret < 0 && ret != AVERROR_EOF) {
        char errBuf[256];
        av_strerror(ret, errBuf, sizeof(errBuf));
        fprintf(stderr, "[MppEncoder] avcodec_send_frame failed: %s\n", errBuf);
        return false;
    }

    bool ownPkt = (encPkt == nullptr);
    AVPacket* pkt = ownPkt ? av_packet_alloc() : encPkt;
    while (true) {
        ret = avcodec_receive_packet(encCtx, pkt);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) break;
        if (ret < 0) break;

        pkt->pts = sentPts;
        pkt->dts = sentPts;

        if (streamPipe && pkt->size > 0) {
            fwrite(pkt->data, 1, pkt->size, streamPipe);
            fflush(streamPipe);
        }

        if (mp4Ctx && pkt->size > 0) {
            pkt->stream_index = 0;
            av_write_frame(mp4Ctx, pkt);
        }

        av_packet_unref(pkt);
    }
    if (ownPkt) av_packet_free(&pkt);

    clock_gettime(CLOCK_MONOTONIC, &tEnc1);
    int64_t encUs = (tEnc1.tv_sec - tEnc0.tv_sec) * 1000000
                  + (tEnc1.tv_nsec - tEnc0.tv_nsec) / 1000;

    static constexpr int kWin = 100;
    static int64_t encWindow[100];
    static int encIdx = 0;
    static int encCnt = 0;
    encWindow[encIdx % kWin] = encUs;
    ++encIdx;
    if (encCnt < kWin) ++encCnt;

    if (encIdx % kWin == 0 && encCnt > 0) {
        int64_t sum = 0, vmin = encWindow[0], vmax = encWindow[0];
        for (int i = 0; i < encCnt; ++i) {
            int64_t v = encWindow[i];
            sum += v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }
        fprintf(stderr, "[EncodeLatency] avg=%ld us  min=%ld us  max=%ld us  (frames=%d)\n",
            (long)(sum / encCnt), (long)vmin, (long)vmax, encCnt);
    }

    return true;
}

void drain_encoder(AVCodecContext* encCtx,
                   FILE* streamPipe, AVFormatContext* mp4Ctx)
{
    if (!encCtx) return;

    avcodec_send_frame(encCtx, nullptr);

    int64_t drainPts = 0;
    AVPacket* pkt = av_packet_alloc();
    while (avcodec_receive_packet(encCtx, pkt) == 0) {
        pkt->pts = drainPts;
        pkt->dts = drainPts;
        drainPts += 3000;
        if (streamPipe && pkt->size > 0) {
            fwrite(pkt->data, 1, pkt->size, streamPipe);
        }
        if (mp4Ctx && pkt->size > 0) {
            pkt->stream_index = 0;
            av_write_frame(mp4Ctx, pkt);
        }
        av_packet_unref(pkt);
    }
    av_packet_free(&pkt);
}
