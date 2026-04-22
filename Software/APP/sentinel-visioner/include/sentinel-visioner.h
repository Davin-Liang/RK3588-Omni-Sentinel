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

// 内部结构体，用于保存每个 DMA Buffer 的信息
struct DmaBufferInfo {
    int index;
    int dmaFd;
};

/**
 * @brief 打包传递给下游的 NPU 和 OSD 渲染任务
 * @note 包含了缩放给 NPU 推理用的小图，以及准备画框用的 720P OSD 底图
 */
struct NpuOSD {
    DmaBuffer_t* npuImage;  ///< 指向供 NPU 推理的 RGB888 小图
    DmaBuffer_t* osdImage;  ///< 指向供 OSD 叠加的 720P NV12 图像 (可能为 nullptr)
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

    std::unique_ptr<DmaBufferPool> npuRgbPool;      ///< NPU RGB888 内存池
    std::unique_ptr<DmaBufferPool> origCopyPool;    ///< 原始大图(NV12) 拷贝池
    std::unique_ptr<DmaBufferPool> osd720pPool;     ///< 720P OSD 图像内存池

    ThreadSafeQueue<NpuOSD> npuTaskQueue;           ///< 供 NPU 线程消费的任务队列
    ThreadSafeQueue<DmaBuffer_t*> processTaskQueue; ///< 供推流/录像等后处理消费的原图队列

    CameraContext() : camFd(-1), epollFd(-1), isStreaming(false), isThreadRunning(false) {}
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
                    int bufferCount, int camNum);

    /**
     * @brief: 开启/关闭特定摄像头的视频流及捕获线程
     * @param: camNum - 摄像头编号
     * @param: isOpen - true: 开启流和采集线程; false: 停止并等待线程退出
     * @return: true 操作成功 / false 操作失败
     */
    bool camera_stream_ctrl(int camNum, bool isOpen);

    /**
     * @brief: 阻塞等待并获取打包好的 NPU 和 OSD 图像数据
     * @param: camNum - 摄像头编号
     * @return: NpuOSD 结构体。若获取失败或摄像头不存在，返回 {nullptr, nullptr}
     */
    NpuOSD wait_get_npuOSD(int camNum);

    /**
     * @brief: 业务层使用完毕后，将 NPU 和 OSD 内存块归还给底层的内存池
     * @param: camNum - 摄像头编号
     * @param: npuOSD - 需要归还的任务结构体指针
     */
    void release_npuOSD(int camNum, NpuOSD* npuOSD);

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

private:
    std::unordered_map<int, std::unique_ptr<CameraContext>> _cameraContextMap;

    void release_camera_resources_(CameraContext* context);

    void capture_thread_func_(int camNum);

    bool rga_process_to_rgb_(int srcFd, int srcWidth, int srcHeight, 
                             DmaBuffer_t* dstBuf, int horizontalOffset, int verticalOffset);

    bool rga_scale_nv12_to_nv12_(int srcFd, int srcWidth, int srcHeight, DmaBuffer_t* dstBuf);

    bool rga_copy_buffer_(int srcFd, int width, int height, DmaBuffer_t* dstBuf);
};