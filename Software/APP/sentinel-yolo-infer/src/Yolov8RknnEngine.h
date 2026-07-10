#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "SentinelYoloInfer.h"
#include "rknn_api.h"

class Yolov8RknnEngine {
public:
    Yolov8RknnEngine();
    ~Yolov8RknnEngine();

    Yolov8RknnEngine(const Yolov8RknnEngine&) = delete;
    Yolov8RknnEngine& operator=(const Yolov8RknnEngine&) = delete;

    bool init(const std::string& modelPath, float boxThreshold, float nmsThreshold,
              int npuCoreMask = 4);
    void release();
    bool isReady() const { return rknnCtx_ != 0; }

    /**
     * @brief 使用 SentinelVisioner 给出的 RGB888 NPU 小图 DMA buffer 做零拷贝推理。
     * @param dmaFd       NPU 小图 DMA-BUF fd
     * @param virtAddr    DMA buffer CPU 映射地址；用于 rknn_create_mem_from_fd
     * @param bufferSize  DMA buffer 字节数
     * @param width       图像宽度，必须与模型输入宽度一致，通常 640
     * @param height      图像高度，必须与模型输入高度一致，通常 640
     * @param timestampNs 帧时间戳，写入每个 YoloBBox
     * @param out         输出检测框列表
     */
    bool inferFromDmaBuffer(int dmaFd,
                            void* virtAddr,
                            int bufferSize,
                            int width,
                            int height,
                            uint64_t timestampNs,
                            std::vector<YoloBBox>& out);

    int modelWidth() const { return modelWidth_; }
    int modelHeight() const { return modelHeight_; }
    int modelChannel() const { return modelChannel_; }

private:
    bool queryModelInfo_();
    bool createOutputMems_();
    void destroyOutputMems_();
    bool collectOutputs_(std::vector<rknn_output>& outputs);
    void freeCollectedOutputs_(std::vector<rknn_output>& outputs);
    void postProcess_(const std::vector<rknn_output>& outputs, uint64_t timestampNs, std::vector<YoloBBox>& out);

private:
    rknn_context rknnCtx_ = 0;
    rknn_input_output_num ioNum_{};
    std::vector<rknn_tensor_attr> inputAttrs_;
    std::vector<rknn_tensor_attr> outputAttrs_;
    std::vector<rknn_tensor_attr> inputNativeAttrs_;
    std::vector<rknn_tensor_attr> outputNativeAttrs_;
    std::vector<rknn_tensor_mem*> outputMems_;

    int modelWidth_ = 0;
    int modelHeight_ = 0;
    int modelChannel_ = 0;
    bool isQuant_ = false;
    float boxThreshold_ = 0.25f;
    float nmsThreshold_ = 0.45f;
    int   npuCoreMask_  = 4;     // 默认 NPU Core 2，避免与 DeepSeek LLM 抢占
};
