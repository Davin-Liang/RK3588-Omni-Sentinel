#include "sentinel-visioner.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <cstring>
#include <cerrno>

SentinelVisioner::SentinelVisioner() {
}

SentinelVisioner::~SentinelVisioner() {
    // 析构时遍历所有摄像头，安全关闭视频流和线程，然后释放资源
    for (auto& pair : _cameraContextMap) {
        if (pair.second->isStreaming) {
            camera_stream_ctrl(pair.first, false);
        }
        release_camera_resources_(pair.second.get());
    }
    _cameraContextMap.clear();
}

bool SentinelVisioner::add_camera(std::string& deviceName, int width, int height, 
                                  int bufferCount, int camNum) {
    if (_cameraContextMap.find(camNum) != _cameraContextMap.end()) {
        std::cerr << "Camera number " << camNum << " already exists!" << std::endl;
        return false;
    }

    auto ctx = std::make_unique<CameraContext>();
    ctx->camNum = camNum;
    ctx->deviceName = deviceName;
    ctx->width = width;
    ctx->height = height;
    ctx->bufferCount = bufferCount;

    /* 初始化 NPU 内存池 (640x640 RGB) */
    ctx->npuRgbPool = std::make_unique<DmaBufferPool>();
    if (!ctx->npuRgbPool->alloc_pool(bufferCount, 640, 640, BufferFormat::RGB888)) {
        std::cerr << "初始化 NPU 内存池失败!———— " << camNum << std::endl;
        return false;
    }

    /* 初始化 原始图像拷贝 内存池 (保持和摄像头输出一致 ———— NV12) */
    ctx->origCopyPool = std::make_unique<DmaBufferPool>();
    if (!ctx->origCopyPool->alloc_pool(bufferCount, width, height, BufferFormat::NV12)) {
        std::cerr << "初始化原始图像拷贝内存池失败!————" << camNum << std::endl;
        return false;
    }

    /* 初始化 720P OSD 内存池 */
    ctx->osd720pPool = std::make_unique<DmaBufferPool>();
    if (!ctx->osd720pPool->alloc_pool(bufferCount, 1280, 720, BufferFormat::NV12)) {
        std::cerr << "初始化 720P OSD 内存池!————" << camNum << std::endl;
        return false;
    }

    // 打开设备节点
    ctx->camFd = open(deviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (ctx->camFd < 0) {
        std::cerr << "Failed to open " << deviceName << ": " << strerror(errno) << std::endl;
        return false;
    }

    // 设置图像格式 (假设默认使用 NV12 格式)
    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12; 
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(ctx->camFd, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "VIDIOC_S_FMT failed: " << strerror(errno) << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    // --- 尝试设置帧率为 30 FPS ---
    struct v4l2_streamparm streamparm = {};
    streamparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    streamparm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    streamparm.parm.capture.timeperframe.numerator = 1;
    streamparm.parm.capture.timeperframe.denominator = 30; // 想要 30 帧

    if (ioctl(ctx->camFd, VIDIOC_S_PARM, &streamparm) < 0) {
        std::cerr << "VIDIOC_S_PARM failed: " << strerror(errno) << " (Ignore if not supported)" << std::endl;
    } else {
        auto& tpf = streamparm.parm.capture.timeperframe;
        std::cout << "Current FPS set to: " << (float)tpf.denominator / tpf.numerator << std::endl;
    }

    // 请求分配内存 (MMAP 模式用于导出 DMA fd)
    struct v4l2_requestbuffers req = {};
    req.count = bufferCount;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(ctx->camFd, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "VIDIOC_REQBUFS failed: " << strerror(errno) << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    ctx->bufferCount = req.count;
    ctx->buffers.resize(ctx->bufferCount);

    // 导出 DMA fd 并将 Buffer 压入内核队列
    for (int i = 0; i < ctx->bufferCount; ++i) {
        ctx->buffers[i].index = i;

        struct v4l2_exportbuffer expbuf = {};
        expbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        expbuf.index = i;
        expbuf.flags = O_CLOEXEC | O_RDWR;

        if (ioctl(ctx->camFd, VIDIOC_EXPBUF, &expbuf) < 0) {
            std::cerr << "VIDIOC_EXPBUF failed: " << strerror(errno) << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
        ctx->buffers[i].dmaFd = expbuf.fd;

        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        // --- MPLANE 必须加这个 planes 数组 ---
        struct v4l2_plane planes[1] = {};
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
            std::cerr << "VIDIOC_QBUF failed: " << strerror(errno) << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
    }

    // 设置 epoll 监听
    ctx->epollFd = epoll_create1(0);
    if (ctx->epollFd < 0) {
        std::cerr << "epoll_create1 failed: " << strerror(errno) << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    struct epoll_event ev = {};
    ev.events = EPOLLIN; // 监听可读事件
    ev.data.fd = ctx->camFd;

    if (epoll_ctl(ctx->epollFd, EPOLL_CTL_ADD, ctx->camFd, &ev) < 0) {
        std::cerr << "epoll_ctl failed: " << strerror(errno) << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    // 保存上下文并移交所有权
    _cameraContextMap[camNum] = std::move(ctx);
    std::cout << "Camera " << camNum << " (" << deviceName << ") added successfully." << std::endl;

    return true;
}

bool SentinelVisioner::camera_stream_ctrl(int camNum, bool isOpen) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        std::cerr << "Camera number " << camNum << " not found!" << std::endl;
        return false;
    }

    CameraContext* ctx = it->second.get();
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

    if (isOpen) {
        if (ctx->isStreaming) return true;

        // 1. 开启视频流
        if (ioctl(ctx->camFd, VIDIOC_STREAMON, &type) < 0) {
            std::cerr << "VIDIOC_STREAMON failed: " << strerror(errno) << std::endl;
            return false;
        }
        ctx->isStreaming = true;

        // 2. 启动该摄像头的采集线程
        ctx->isThreadRunning = true;
        ctx->captureThread = std::make_unique<std::thread>(&SentinelVisioner::capture_thread_func_, this, camNum);
        
        std::cout << "Camera " << camNum << " stream & thread STARTED." << std::endl;
    } else {
        if (!ctx->isStreaming) return true;

        // 1. 停止线程：设置标志位为 false 并等待线程安全退出
        ctx->isThreadRunning = false;
        if (ctx->captureThread && ctx->captureThread->joinable()) {
            ctx->captureThread->join();
            ctx->captureThread.reset(); // 释放线程对象
        }

        // 2. 关闭视频流
        if (ioctl(ctx->camFd, VIDIOC_STREAMOFF, &type) < 0) {
            std::cerr << "VIDIOC_STREAMOFF failed: " << strerror(errno) << std::endl;
            return false;
        }
        ctx->isStreaming = false;
        
        std::cout << "Camera " << camNum << " stream & thread STOPPED." << std::endl;
    }

    return true;
}

void SentinelVisioner::capture_thread_func_(int camNum) {
    // 获取当前摄像头的上下文
    CameraContext* ctx = _cameraContextMap[camNum].get();
    
    const int MAX_EVENTS = 5;
    struct epoll_event events[MAX_EVENTS];

    // --- 新增：原始帧率统计变量 ---
    int raw_frame_count = 0;
    auto last_fps_time = std::chrono::steady_clock::now();

    std::cout << "[Thread] Camera " << camNum << " capture thread started." << std::endl;

    while (ctx->isThreadRunning) {
        // 设置超时时间为 1000 毫秒，避免线程死锁无法退出
        int nfds = epoll_wait(ctx->epollFd, events, MAX_EVENTS, 1000);
        
        if (nfds < 0) {
            if (errno == EINTR) continue; // 被信号中断，继续
            perror("[Thread] epoll_wait error");
            break;
        } else if (nfds == 0) {
            // 超时，继续循环检查 isThreadRunning 标志
            continue;
        }

        for (int i = 0; i < nfds; ++i) {
            if (events[i].data.fd == ctx->camFd && (events[i].events & EPOLLIN)) {
                
                // 1. 数据就绪，执行出队 (DQBUF)
                struct v4l2_buffer buf = {};
                buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                buf.memory = V4L2_MEMORY_MMAP;
                
                // --- MPLANE 必须加这个 planes 数组 ---
                struct v4l2_plane planes[1] = {};
                buf.m.planes = planes;
                buf.length = 1;

                if (ioctl(ctx->camFd, VIDIOC_DQBUF, &buf) < 0) {
                    if (errno == EAGAIN) continue;
                    perror("[Thread] VIDIOC_DQBUF failed");
                    ctx->isThreadRunning = false; // 发生严重错误，通知退出
                    break;
                }

                // 获取包含了最新一帧图像数据的 DMA fd
                int currentDmaFd = ctx->buffers[buf.index].dmaFd;
                // 获取最新一帧图像数据的时间戳
                uint64_t timestampUs = (uint64_t)buf.timestamp.tv_sec * 1000000LL + buf.timestamp.tv_usec;
                
                /* 从内存池获取空闲的 DMA 块 */
                DmaBuffer_t* targetNpuBuf = ctx->npuRgbPool->get_buffer();
                DmaBuffer_t* targetOsd720pBuf = ctx->osd720pPool->get_buffer();

                if (targetNpuBuf != nullptr) { // 这里并不做 targetOsd720pBuf 是否为空指针的判断，因为当并不对带框图像进行推流的时候，osd720pPool中会没有buffer
                    // TODO: 这里的偏移量(横向/纵向)通常由外部 IMU 陀螺仪计算后传入
                    // 此处模拟获取实时的防抖平移参数
                    int currentHorizOffset = 0; 
                    int currentVertOffset  = 0; 

                    // 记录时间戳
                    targetNpuBuf->timestampUs = timestampUs;
                    if(targetOsd720pBuf) targetOsd720pBuf->timestampUs = timestampUs;

                    auto start_time = std::chrono::high_resolution_clock::now();

                    // 操作 A: RGA 缩放并转码给 NPU (1080P NV12 -> 640 RGB888)
                    bool npuOk = rga_process_to_rgb_(currentDmaFd, ctx->width, ctx->height, 
                                                    targetNpuBuf, currentHorizOffset, 
                                                    currentVertOffset);

                    // 操作 B: RGA 缩放 (1080P NV12 -> 720P NV12)
                    bool scaleOk = true;
                    if (targetOsd720pBuf != nullptr) {
                        scaleOk = rga_scale_nv12_to_nv12_(currentDmaFd, ctx->width, ctx->height, 
                                                            targetOsd720pBuf);
                    }

                    if (npuOk && scaleOk) {
                        // 打包并 Push 到 NPU 队列
                        // targetOsd720pBuf是nullptr无所谓
                        NpuOSD task = {targetNpuBuf, targetOsd720pBuf};
                        ctx->npuTaskQueue.push(task);
                    } else {
                        // 处理失败，归还内存
                        std::cerr << "[RGA Error] 1080P NV12 -> 640 RGB888和1080P NV12 -> 720P NV12转换失败，立即归还避免内存泄漏." << std::endl;
                        ctx->npuRgbPool->release_buffer(targetNpuBuf);
                        if (targetOsd720pBuf != nullptr)
                            ctx->osd720pPool->release_buffer(targetOsd720pBuf);
                    }

                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "[time] RGA (NV12 -> RGB888 & EIS & scale): " << duration.count() << " ms." << std::endl;
                } else {
                    // 缓冲池干涸策略：通常意味着下游处理太慢，此时直接丢弃当前帧 (Drop Frame)
                    std::cerr << "[Thread] Warning: RGA buffer pool empty! Dropping frame." << std::endl;
                    if (targetNpuBuf) ctx->npuRgbPool->release_buffer(targetNpuBuf);
                    if (targetOsd720pBuf) ctx->osd720pPool->release_buffer(targetOsd720pBuf);
                }

                DmaBuffer_t* targetOrigBuf = ctx->origCopyPool->get_buffer();

                if (targetOrigBuf != nullptr) {
                    // 记录时间戳
                    targetOrigBuf->timestampUs = timestampUs;

                    bool copyOk = rga_copy_buffer_(currentDmaFd, ctx->width, ctx->height, 
                                                    targetOrigBuf);

                    if (copyOk) {
                        ctx->processTaskQueue.push(targetOrigBuf);
                    } else {
                        std::cerr << "[RGA Error] 拷贝图像失败，立即归还避免内存泄漏." << std::endl;
                        ctx->origCopyPool->release_buffer(targetOrigBuf);
                    }
                }

                // RGA（或其他）处理完毕后，将该 Buffer 重新入队交还给摄像头驱动
                if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                    perror("[Thread] VIDIOC_QBUF requeue failed");
                    ctx->isThreadRunning = false;
                    break;
                }
            }
        }
    }

    std::cout << "[Thread] Camera " << camNum << " capture thread exited." << std::endl;
}

void SentinelVisioner::release_camera_resources_(CameraContext* ctx) {
    if (!ctx) return;

    for (auto& bufInfo : ctx->buffers) {
        if (bufInfo.dmaFd >= 0) {
            close(bufInfo.dmaFd);
            bufInfo.dmaFd = -1;
        }
    }
    ctx->buffers.clear();

    if (ctx->epollFd >= 0) {
        close(ctx->epollFd);
        ctx->epollFd = -1;
    }

    if (ctx->camFd >= 0) {
        close(ctx->camFd);
        ctx->camFd = -1;
    }

    // ==========================================================
    // 新增: 释放针对该路摄像头的 RGA DMA 内存池
    // ==========================================================
    if (ctx->npuRgbPool) {
        ctx->npuRgbPool->destroy_pool();
        ctx->npuRgbPool.reset(); 
    }

    if (ctx->origCopyPool) {
        ctx->origCopyPool->destroy_pool();
        ctx->origCopyPool.reset(); 
    }
    if (ctx->osd720pPool) {
        ctx->osd720pPool->destroy_pool();
        ctx->osd720pPool.reset(); 
    }
}

NpuOSD SentinelVisioner::wait_get_npuOSD(int camNum) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return {nullptr, nullptr};
    }

    CameraContext* ctx = it->second.get();
    
    // 调用安全队列的阻塞方法，线程会在这里休眠，直到捕获线程 push 了新的一帧
    return ctx->npuTaskQueue.pop(); 
}

void SentinelVisioner::release_npuOSD(int camNum, NpuOSD* npuOSD) {
    if (npuOSD == nullptr) return;

    auto it = _cameraContextMap.find(camNum);
    if (it != _cameraContextMap.end()) {
        // 交还给对应摄像头的专用内存池
        it->second->npuRgbPool->release_buffer(npuOSD->npuImage);
        if (npuOSD->osdImage != nullptr)
            it->second->osd720pPool->release_buffer(npuOSD->osdImage);
    }
}

DmaBuffer_t* SentinelVisioner::wait_get_orig_copy_buffer(int camNum) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return nullptr;
    }

    CameraContext* ctx = it->second.get();
    
    // 调用安全队列的阻塞方法，线程会在这里休眠，直到捕获线程 push 了新的一帧
    return ctx->processTaskQueue.pop(); 
}

void SentinelVisioner::release_orig_copy_buffer(int camNum, DmaBuffer_t* buf) {
    if (buf == nullptr) return;

    auto it = _cameraContextMap.find(camNum);
    if (it != _cameraContextMap.end()) {
        // 交还给对应摄像头的专用内存池
        it->second->origCopyPool->release_buffer(buf);
    }
}

bool SentinelVisioner::rga_process_to_rgb_(int srcFd, int srcWidth, int srcHeight, 
                                           DmaBuffer_t* dstBuf, int horizontalOffset, int verticalOffset) {
    if (srcFd <= 0 || !dstBuf || dstBuf->dmaFd <= 0) {
        std::cerr << "[RGA Error] Invalid DMA fd!" << std::endl;
        return false;
    }

    bool ret = true;
    IM_STATUS ret_rga = IM_STATUS_NOERROR;
    
    rga_buffer_handle_t rga_handle_src = 0;
    rga_buffer_handle_t rga_handle_dst = 0;

    // MIPI 摄像头通常输入为 NV12 (YCrCb_420_SP)
    int srcFmt = RK_FORMAT_YCrCb_420_SP; 
    int dstFmt = RK_FORMAT_RGB_888;

    // 1. 导入 DMA Fd 生成 RGA Handle
    im_handle_param_t in_param = { srcWidth, srcHeight, srcFmt };
    rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) return false;

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, dstFmt };
    rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    // 2. 包装 RGA Buffer
    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, srcWidth, srcHeight, srcFmt, srcWidth, srcHeight);
    rga_buffer_t rga_buf_dst = wrapbuffer_handle(rga_handle_dst, dstBuf->width, dstBuf->height, dstFmt, dstBuf->width, dstBuf->height);

    // 3. 计算 Letterbox 参数
    float scale = std::min((float)dstBuf->width / srcWidth, (float)dstBuf->height / srcHeight);
    int scaled_w = srcWidth * scale;
    int scaled_h = srcHeight * scale;

    // 【核心防抖逻辑】：在默认居中的基础上，叠加有符号的外部补偿量
    int offset_x = (dstBuf->width - scaled_w) / 2 + horizontalOffset;
    int offset_y = (dstBuf->height - scaled_h) / 2 + verticalOffset;

    im_rect srect = {0, 0, srcWidth, srcHeight};
    im_rect drect = {offset_x, offset_y, scaled_w, scaled_h};

    // 4. 背景填充 (Padding)
    // 只要宽高没填满画布，或者发生了平移，就说明需要填充灰边防脏数据
    if (scaled_w != dstBuf->width || scaled_h != dstBuf->height || horizontalOffset != 0 || verticalOffset != 0) {
        im_rect dst_whole_rect = {0, 0, dstBuf->width, dstBuf->height};
        // 0xFF727272 对应 RGB 的灰色 (114, 114, 114)
        ret_rga = imfill(rga_buf_dst, dst_whole_rect, 0xFF727272); 
    }

    // 5. RGA 终极处理：格式转换 + 缩放 + 偏移写入
    rga_buffer_t pat; memset(&pat, 0, sizeof(rga_buffer_t));
    im_rect prect; memset(&prect, 0, sizeof(im_rect));
    
    ret_rga = improcess(rga_buf_src, rga_buf_dst, pat, srect, drect, prect, 0);
    if (ret_rga <= 0) {
        std::cerr << "[RGA Error] improcess failed." << std::endl;
        ret = false;
    }

    // 6. 释放 RGA 句柄
    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    return ret;
}

bool SentinelVisioner::rga_scale_nv12_to_nv12_(int srcFd, int srcWidth, int srcHeight, DmaBuffer_t* dstBuf) {
    // 检查参数合法性
    if (srcFd <= 0 || !dstBuf || dstBuf->dmaFd <= 0) {
        std::cerr << "[RGA Error] Invalid DMA fd for scaling!" << std::endl;
        return false;
    }

    // 源格式和目标格式保持一致，都是 NV12 (Rockchip 宏定义通常为 YCrCb_420_SP 或 YCbCr_420_SP)
    int fmt = RK_FORMAT_YCrCb_420_SP; 

    // 1. 导入 DMA Fd 生成 RGA Handle
    im_handle_param_t in_param = { srcWidth, srcHeight, fmt };
    rga_buffer_handle_t rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) {
        std::cerr << "[RGA Error] Failed to import source buffer fd: " << srcFd << std::endl;
        return false;
    }

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, fmt };
    rga_buffer_handle_t rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        std::cerr << "[RGA Error] Failed to import destination buffer fd: " << dstBuf->dmaFd << std::endl;
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    // 2. 包装 RGA Buffer
    // 跨距 (Stride) 默认与宽度对齐，如果你的分配器有特殊的步长，请修改最后的两个参数
    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, srcWidth, srcHeight, fmt, srcWidth, srcHeight);
    rga_buffer_t rga_buf_dst = wrapbuffer_handle(rga_handle_dst, dstBuf->width, dstBuf->height, fmt, dstBuf->width, dstBuf->height);

    // 3. 设置源和目标的矩形区域 (全图到全图映射)
    im_rect srect = {0, 0, srcWidth, srcHeight};
    im_rect drect = {0, 0, dstBuf->width, dstBuf->height};

    // 4. 执行 RGA 硬件缩放
    // 不需要背景填充(pat)和遮罩(prect)，全部置 0
    rga_buffer_t pat; memset(&pat, 0, sizeof(rga_buffer_t));
    im_rect prect; memset(&prect, 0, sizeof(im_rect));
    
    // 调用 improcess，RGA 会自动识别源和目标的大小差异并启动缩放引擎
    IM_STATUS ret_rga = improcess(rga_buf_src, rga_buf_dst, pat, srect, drect, prect, 0);

    // 5. 释放句柄 (极其重要，否则会造成内核 RGA 句柄泄漏)
    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    if (ret_rga <= 0) {
        std::cerr << "[RGA Error] improcess scale failed: " << imStrError(ret_rga) << std::endl;
        return false;
    }

    return true;
}

bool SentinelVisioner::rga_copy_buffer_(int srcFd, int width, int height, DmaBuffer_t* dstBuf) {
    if (srcFd <= 0 || !dstBuf || dstBuf->dmaFd <= 0) return false;

    // MIPI 摄像头通常输入为 NV12 (YCrCb_420_SP)
    int fmt = RK_FORMAT_YCrCb_420_SP; 

    im_handle_param_t in_param = { width, height, fmt };
    rga_buffer_handle_t rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) return false;

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, fmt };
    rga_buffer_handle_t rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, width, height, fmt, width, height);
    rga_buffer_t rga_buf_dst = wrapbuffer_handle(rga_handle_dst, dstBuf->width, dstBuf->height, fmt, dstBuf->width, dstBuf->height);

    // 调用 RGA 硬件拷贝
    IM_STATUS ret_rga = imcopy(rga_buf_src, rga_buf_dst);

    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    return ret_rga == IM_STATUS_SUCCESS;
}
