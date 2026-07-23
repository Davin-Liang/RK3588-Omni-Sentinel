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

    /**
     * @brief NPU 核心掩码（仅 RK3588 生效）
     *
     * 用于将 YOLO 推理绑定到指定 NPU 核心，避免与大模型抢占 NPU 资源。
     * 默认 RKNN_NPU_CORE_2(4)：YOLO 独占 Core 2，Core 0/1 留给 DeepSeek LLM。
     *
     * 可选值（来自 rknn_api.h）：
     *   RKNN_NPU_CORE_AUTO(0)   — 驱动自动分配
     *   RKNN_NPU_CORE_0(1)      — 仅使用 Core 0
     *   RKNN_NPU_CORE_1(2)      — 仅使用 Core 1
     *   RKNN_NPU_CORE_2(4)      — 仅使用 Core 2
     */
    int npuCoreMask = 4;  // RKNN_NPU_CORE_2

    /**
     * @brief CPU 亲和性掩码（用于将推理线程绑定到指定 CPU 核心）
     *
     * 通过 pthread_setaffinity_np 将 YOLO 推理线程限制在指定 CPU 核心上运行，
     * 避免抢占大核影响 UI 渲染、相机采集等延迟敏感任务。
     *
     * RK3588 CPU 布局（8 核）：
     *   0x0F (15)  = CPU 0-3  (4×A55 小核) — 推荐：YOLO 推理线程绑小核
     *   0xF0 (240) = CPU 4-7  (4×A76 大核) — 推荐：UI、相机采集线程
     *   0xFF (255) = CPU 0-7  (全部核心)   — 不限制
     *   0          = 不设置亲和性（默认）
     *
     * 注意：
     *   - 只影响 YOLO 推理线程的前/后处理 CPU 时间，不影响 NPU 硬件推理性能
     *   - NPU 是独立硬件加速器，其核心分配由 npuCoreMask 控制
     */
    int cpuAffinityMask = 0;  // 0 = 不绑定，交由内核调度
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
