/**
 * @brief MPP 硬件编/复用器 + ffmpeg 子进程推流封装（内部头文件）
 */

#ifndef SENTINEL_STREAMER_MPP_ENCODER_H
#define SENTINEL_STREAMER_MPP_ENCODER_H

#include <cstdio>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
}

// ---- 全局 ----
void ffmpeg_init_once();

// ---- MPP 编码器 ----
bool mpp_encoder_open(AVCodecContext** outCtx, int width, int height, int bitRate);
void mpp_encoder_flush(AVCodecContext* encCtx, AVFormatContext* mp4Ctx);
void mpp_encoder_close(AVCodecContext** ctx);

// ---- ffmpeg 子进程推流（代替 RTSP API muxer） ----
/**
 * @brief 启动 ffmpeg 子进程，管道输入 H.264 裸流，命令行负责 RTSP 推流
 * @param url RTSP URL
 * @return FILE* 管道写端 / nullptr
 */
FILE* ffmpeg_stream_open(const char* url);

/**
 * @brief 关闭 ffmpeg 子进程管道
 */
void ffmpeg_stream_close(FILE* pipe);

// ---- MP4 复用器 ----
bool mp4_output_open(AVFormatContext** outCtx, AVCodecContext* encCtx, const char* filePath);
void mp4_output_close(AVFormatContext** ctx);

// ---- 编码 + 复用 ----
/**
 * @brief 编码一帧，输出到 ffmpeg 管道（推流）和/或 MP4（录像）
 */
bool encode_and_mux(AVCodecContext* encCtx, void* virtAddr, int width, int height,
                    int64_t pts,
                    FILE* streamPipe, AVFormatContext* mp4Ctx);

/**
 * @brief 排空编码器残余帧
 */
void drain_encoder(AVCodecContext* encCtx,
                   FILE* streamPipe, AVFormatContext* mp4Ctx);

#endif  // SENTINEL_STREAMER_MPP_ENCODER_H
