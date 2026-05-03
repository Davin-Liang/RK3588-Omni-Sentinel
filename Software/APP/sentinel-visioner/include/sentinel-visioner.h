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

    // --- 新增：线程与控制标志 ---
    std::unique_ptr<std::thread> captureThread;
    std::atomic<bool> isThreadRunning;

    std::unique_ptr<DmaBufferPool> rgaOutputPool;
    ThreadSafeQueue<DmaBuffer_t*> targetTaskQueue;

    CameraContext() : camFd(-1), epollFd(-1), isStreaming(false), isThreadRunning(false) {}
};

class SentinelVisioner {
public:
    SentinelVisioner();
    ~SentinelVisioner();

    /**
     * @brief: 添加摄像头
     * @param: deviceName - 设备名
     * @param: width - 图像宽度
     * @param: height - 图像高度
     * @param: bufferCount - 需要分配的DMA Buffer内存块的数量
     * @param: camNum - 摄像头编号
     * @return: true 添加摄像头成功 / false 添加摄像头失败
     */
    bool add_camera(std::string& deviceName, int width, int height, 
                    int bufferCount, int camNum);

    /**
     * @brief: 开启/关闭特定摄像头的视频流 (并相应地启动/停止捕获线程)
     * @param: camNum - 摄像头编号
     * @param: isOpen - 开启/关闭视频流
     * @return: true 操作成功 / false 操作失败
     */
    bool camera_stream_ctrl(int camNum, bool isOpen);

    /**
     * @brief: 阻塞等待并获取处理好的 RGB888 DMA 内存块
     * @param: camNum - 摄像头编号
     * @return: DmaBuffer_t* 可用内存块，直到有数据到来才会返回 (找不到摄像头返回 nullptr)
     */
    DmaBuffer_t* wait_get_rga_buffer(int camNum);

    /**
     * @brief: 业务层使用完毕后，将 DMA 内存块归还给底层的内存池
     * @param: camNum - 摄像头编号
     * @param: buf - 需要归还的内存块指针
     */
    void release_rga_buffer(int camNum, DmaBuffer_t* buf);

private:
    // 私有变量：保存所有已添加的摄像头上下文。使用 unique_ptr 避免 map 扩容时的拷贝问题
    std::unordered_map<int, std::unique_ptr<CameraContext>> _cameraContextMap;

    // 私有内部方法：清理特定摄像头的资源
    void release_camera_resources_(CameraContext* context);

    // 私有内部方法：摄像头数据捕获工作线程
    void capture_thread_func_(int camNum);

    bool rga_process_to_rgb_(int srcFd, int srcWidth, int srcHeight, 
                             DmaBuffer_t* dstBuf, int horizontalOffset, int verticalOffset);
};
