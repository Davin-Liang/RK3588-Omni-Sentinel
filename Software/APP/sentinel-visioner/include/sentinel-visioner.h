#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>
#include <thread>
#include <atomic>
#include <memory>
#include <linux/videodev2.h>

#include "dma-buffer-pool.h"
#include "ThreadSafeQueue.h"

#include "im2d.h"
#include "drmrga.h"
#include <algorithm>
#include <functional>

// 内部结构体，用于保存每个 DMA Buffer 的信息
struct DmaBufferInfo {
    int index;
    int dmaFd;
    void* virtAddr;  ///< MMAP 地址，供 CPU 读取 JPEG 等压缩数据
};

/**
 * @brief 摄像头类型枚举，区分 MIPI CSI/ISP 摄像头和 USB UVC 摄像头
 */
enum class CameraType {
    ISP_CAM,  ///< MIPI CSI 摄像头 (默认，使用 MPLANE + NV12)
    USB_CAM   ///< USB UVC 摄像头 (使用单平面 + NV12/YUYV 协商)
};

// 内部结构体，用于保存单路摄像头的完整上下文信息
struct CameraContext {
    int camNum;
    int camFd;
    int epollFd;
    std::string deviceName;
    int width;
    int height;
    int bufferCount;
    bool isStreaming;
    std::vector<DmaBufferInfo> buffers;

    std::unique_ptr<std::thread> captureThread;
    std::atomic<bool> isThreadRunning;
    std::atomic<bool> isPaused;

    CameraType camType;                             ///< 摄像头类型
    int v4l2BufType;                                ///< V4L2 buffer type (MPLANE 或 SINGLE_PLANAR)
    unsigned int actualPixelFormat;                 ///< 实际协商后的 V4L2 pixel format
    int srcBytesPerLine;                            ///< USB 相机实际行跨度（字节），用于 RGA 导入正确 stride

    std::unique_ptr<DmaBufferPool> npuRgbPool;      ///< NPU RGB888 内存池
    std::unique_ptr<DmaBufferPool> origCopyPool;    ///< 原始大图(NV12) 拷贝池
    std::unique_ptr<DmaBufferPool> previewPool;      ///< 1080P RGB888 预览图像内存池
    std::unique_ptr<DmaBufferPool> usbConvertPool;  ///< USB YUYV→NV12 中间转换缓冲池
    std::unique_ptr<DmaBufferPool> usbSafePool;     ///< USB NV12 安全拷贝缓冲池（RGA 兼容性）
    std::unique_ptr<DmaBufferPool> mjpegDecodePool; ///< USB MJPG→NV12 软件解码输出池

    ThreadSafeQueue<DmaBuffer_t*> npuTaskQueue;      ///< 供 NPU 推理消费者消费的 640x640 RGB888 小图队列
    ThreadSafeQueue<DmaBuffer_t*> previewTaskQueue;   ///< 供预览消费者消费的 RGB888 图像队列
    ThreadSafeQueue<DmaBuffer_t*> processTaskQueue;  ///< 供推流/录像等后处理消费的原图队列

    // EIS 低通滤波状态（每路独立）
    float eisSmoothAlpha;
    int32_t prevEisOffsetX;
    int32_t prevEisOffsetY;
    bool eisPrevValid;

    CameraContext() : camFd(-1), epollFd(-1), isStreaming(false), isThreadRunning(false),
        isPaused(false), camType(CameraType::ISP_CAM),
        v4l2BufType(V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE),
        actualPixelFormat(V4L2_PIX_FMT_NV12), srcBytesPerLine(0),
        eisSmoothAlpha(0.7f), prevEisOffsetX(0), prevEisOffsetY(0), eisPrevValid(false) {}
};

class SentinelVisioner {
public:
    SentinelVisioner();
    ~SentinelVisioner();

    /**
     * @brief: 添加摄像头并初始化所有的 DMA 内存池
     * @param: deviceName - 设备节点路径 (如 "/dev/video0")
     * @param: width - 原始图像采集宽度 (如 1920)
     * @param: height - 原始图像采集高度 (如 1080)
     * @param: bufferCount - V4L2 及各级内存池的缓冲块数量
     * @param: camNum - 摄像头全局逻辑编号
     * @return: true 添加成功 / false 添加失败
     */
    bool add_camera(std::string& deviceName, int width, int height,
                    int bufferCount, int camNum,
                    CameraType camType = CameraType::ISP_CAM);

    /**
     * @brief: 开启/关闭特定摄像头的视频流及捕获线程
     * @param: camNum - 摄像头编号
     * @param: isOpen - true: 开启流和采集线程; false: 停止并等待线程退出
     * @return: true 操作成功 / false 操作失败
     */
    bool camera_stream_ctrl(int camNum, bool isOpen);

    /**
     * @brief: 暂停/恢复摄像头的 RGA 处理和队列输出 (硬件流保持，仅跳过处理)
     * @param: camNum - 摄像头编号
     * @param: paused - true: 暂停处理; false: 恢复处理
     */
    void camera_pause(int camNum, bool paused);

    /**
     * @brief: 阻塞等待并获取 NPU 推理图像数据
     * @param: camNum - 摄像头编号
     * @return: DmaBuffer_t* 640x640 RGB888 NPU 推理小图。若摄像头不存在，返回 nullptr
     */
    DmaBuffer_t* wait_get_npu(int camNum);

    /**
     * @brief: 带超时的阻塞等待 NPU 推理图像数据
     * @param: camNum - 摄像头编号
     * @param: timeoutMs - 超时毫秒数
     * @return: DmaBuffer_t* NPU 推理小图。超时返回 nullptr
     */
    DmaBuffer_t* try_get_npu(int camNum, int timeoutMs);

    /**
     * @brief: 归还 NPU 推理图像内存块
     * @param: camNum - 摄像头编号
     * @param: buf - 需要归还的内存块指针
     */
    void release_npu(int camNum, DmaBuffer_t* buf);

    /**
     * @brief: 阻塞等待并获取预览图像数据
     * @param: camNum - 摄像头编号
     * @return: DmaBuffer_t* RGB888 预览图像。若摄像头不存在，返回 nullptr
     */
    DmaBuffer_t* wait_get_preview(int camNum);

    /**
     * @brief: 带超时的阻塞等待预览图像数据
     * @param: camNum - 摄像头编号
     * @param: timeoutMs - 超时毫秒数
     * @return: DmaBuffer_t* 预览图像。超时返回 nullptr
     */
    DmaBuffer_t* try_get_preview(int camNum, int timeoutMs);

    /**
     * @brief: 归还预览图像内存块
     * @param: camNum - 摄像头编号
     * @param: buf - 需要归还的内存块指针
     */
    void release_preview(int camNum, DmaBuffer_t* buf);

    /**
     * @brief: 阻塞等待并获取拷贝好的原始高分辨率图像 (用于推流或存盘)
     * @param: camNum - 摄像头编号
     * @return: DmaBuffer_t* 可用原始图像内存块，直到有数据到来才会返回
     */
    DmaBuffer_t* wait_get_orig_copy_buffer(int camNum);

    /**
     * @brief: 推流/存盘完成后，将原始图像内存块归还给底层的拷贝内存池
     * @param: camNum - 摄像头编号
     * @param: buf - 需要归还的内存块指针
     */
    void release_orig_copy_buffer(int camNum, DmaBuffer_t* buf);

    void set_eis_offset_callback(std::function<bool(uint64_t timestampUs, int camNum,
                                 int32_t& offsetX, int32_t& offsetY)> callback);

    /**
     * @brief: 设置 EIS 偏移平滑系数（0~1，默认 0.7，越小越平滑）
     */
    void set_eis_smooth_alpha(float alpha);

private:
    std::unordered_map<int, std::unique_ptr<CameraContext>> _cameraContextMap;

    void release_camera_resources_(CameraContext* context);

    void capture_thread_func_(int camNum);

    bool rga_process_to_rgb_(int srcFd, int srcWidth, int srcHeight, int srcStride,
                             DmaBuffer_t* dstBuf, int horizontalOffset, int verticalOffset);

    bool rga_convert_to_rgb_full_(int srcFd, int srcWidth, int srcHeight, int srcStride,
                                   DmaBuffer_t* dstBuf);

    bool rga_copy_buffer_(int srcFd, int width, int height, int srcStride, DmaBuffer_t* dstBuf);

    bool rga_yuyv_to_nv12_(int srcFd, int srcWidth, int srcHeight, int srcStride, DmaBuffer_t* dstBuf);

    bool mjpeg_decode_to_nv12_(const uint8_t* jpegData, size_t jpegSize,
                               DmaBuffer_t* dstBuf);

    std::function<bool(uint64_t, int, int32_t&, int32_t&)> eis_offset_callback_;
};