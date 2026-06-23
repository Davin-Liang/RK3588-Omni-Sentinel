#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "sentinel-visioner.h"
#include "ThreadSafeQueue.h"

/**
 * @brief YOLOv8 检测框结果。
 *
 * 坐标默认位于 SentinelVisioner::wait_get_npu(camNum) 输出的 NPU 小图坐标系，
 * 即通常为 640x640 RGB888 letterbox 图像坐标系。
 */
#ifndef YOLO_BBOX_DEFINED
#define YOLO_BBOX_DEFINED
struct YoloBBox {
    uint32_t x1;          ///< 左上角像素坐标（包含）
    uint32_t y1;          ///< 左上角像素坐标（包含）
    uint32_t x2;          ///< 右下角像素坐标（不包含），宽度 = x2 - x1
    uint32_t y2;          ///< 右下角像素坐标（不包含），高度 = y2 - y1
    uint32_t classId;     ///< 类别 ID（COCO 格式）
    float confidence;     ///< 置信度 [0.0, 1.0]
    uint64_t timestampNs; ///< 该检测框对应的图像帧时间戳（CLOCK_MONOTONIC, ns）
};
#endif

using YoloBBoxList = std::vector<YoloBBox>;

struct SentinelYoloInferConfig {
    std::string modelPath;       ///< .rknn 模型路径
    float boxThreshold = 0.25f;  ///< 置信度阈值
    float nmsThreshold = 0.45f;  ///< NMS IoU 阈值
    int waitTimeoutMs = 200;     ///< 获取 NPU 小图的超时时间；<=0 时使用 wait_get_npu 阻塞等待
    bool pushEmptyResult = true; ///< 无目标帧是否仍向两个队列推送空 vector，用于保持 OSD 帧同步
};

/**
 * @brief 基于 SentinelVisioner + RKNN YOLOv8 的多摄像头推理类。
 *
 * 设计要点：
 * 1. 构造函数接收 SentinelVisioner*，推理线程通过其 wait_get_npu/try_get_npu 获取 NPU 小图。
 * 2. 每个摄像头编号对应一个独立推理线程、一个独立 RKNN context、两个结果队列。
 * 3. NPU 小图使用 dmaFd 导入 RKNN input tensor，避免 CPU memcpy。
 * 4. 推理完成后，同一份检测结果分别压入 fusion 队列和 osd 队列。
 * 5. 不论推理是否成功，都会调用 release_npu(camNum, buf) 归还 DMA buffer。
 */
class SentinelYoloInfer {
public:
    SentinelYoloInfer(SentinelVisioner* visioner, const SentinelYoloInferConfig& config);
    SentinelYoloInfer(SentinelVisioner* visioner, const std::string& modelPath);
    ~SentinelYoloInfer();

    SentinelYoloInfer(const SentinelYoloInfer&) = delete;
    SentinelYoloInfer& operator=(const SentinelYoloInfer&) = delete;

    /**
     * @brief 根据摄像头编号创建并启动对应的推理线程。
     * @return true 表示启动成功或线程已存在；false 表示参数/模型初始化失败。
     */
    bool create_infer_thread(int camNum);

    /** @brief 停止指定摄像头的推理线程。 */
    void stop_infer_thread(int camNum);

    /** @brief 停止所有推理线程。 */
    void stop_all();

    /** @brief 查询指定摄像头推理线程是否仍在运行。 */
    bool is_running(int camNum) const;

    /** @brief 阻塞获取供融合模块使用的推理结果。 */
    YoloBBoxList wait_get_fusion_result(int camNum);

    /** @brief 超时获取供融合模块使用的推理结果。 */
    bool try_get_fusion_result(int camNum, YoloBBoxList& out, int timeoutMs);

    /** @brief 阻塞获取供 OSD 叠加使用的推理结果。 */
    YoloBBoxList wait_get_osd_result(int camNum);

    /** @brief 超时获取供 OSD 叠加使用的推理结果。 */
    bool try_get_osd_result(int camNum, YoloBBoxList& out, int timeoutMs);

private:
    struct InferThreadContext;

    void infer_thread_loop_(std::shared_ptr<InferThreadContext> ctx);
    std::shared_ptr<InferThreadContext> get_context_(int camNum) const;

private:
    SentinelVisioner* visioner_ = nullptr;
    SentinelYoloInferConfig config_;

    mutable std::mutex contextsMutex_;
    std::unordered_map<int, std::shared_ptr<InferThreadContext>> contexts_;
};
