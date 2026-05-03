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

    // ==========================================================
    // 新增: 初始化针对 RGA 输出的 DMA 内存池
    // 目标大小: 680x680, 目标格式: RGB888
    // 缓冲块数量可以和 V4L2 保持一致，也可以根据下游消费速度略大一点
    // ==========================================================
    ctx->rgaOutputPool = std::make_unique<DmaBufferPool>();
    if (!ctx->rgaOutputPool->alloc_pool(bufferCount, 640, 640, BufferFormat::RGB888)) {
        std::cerr << "Failed to allocate RGA output DMA pool for camera " << camNum << std::endl;
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

                // raw_frame_count++;
                // auto now = std::chrono::steady_clock::now();
                // auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time).count();
                
                // if (elapsed >= 1000) {
                //     float fps = (raw_frame_count * 1000.0f) / elapsed;
                //     std::cout << "\033[1;32m[RAW FPS] Camera " << camNum << ": " << fps << " fps\033[0m" << std::endl;
                //     raw_frame_count = 0;
                //     last_fps_time = now;
                // }

                // 获取包含了最新一帧图像数据的 DMA fd
                int currentDmaFd = ctx->buffers[buf.index].dmaFd;

                // ==========================================================
                // 新增: RGA 处理与 DmaBufferPool 的联动
                // ==========================================================
                
                // 步骤 2.1: 从内存池获取一个空闲的 DMA 块用来接 RGA 的输出
                DmaBuffer_t* targetBuf = ctx->rgaOutputPool->get_buffer();

                if (targetBuf != nullptr) {
                    // TODO: 这里的偏移量(横向/纵向)通常由外部 IMU 陀螺仪计算后传入
                    // 此处模拟获取实时的防抖平移参数
                    int currentHorizOffset = 0; 
                    int currentVertOffset  = 0; 

                    auto start_time = std::chrono::high_resolution_clock::now();

                    // 执行硬件 RGA：NV12 -> RGB888 + 缩放 + 防抖平移
                    if (rga_process_to_rgb_(currentDmaFd, ctx->width, ctx->height, targetBuf, 
                                            currentHorizOffset, currentVertOffset)) {
                        // 转换成功，直接调用安全队列的 push，内部会自动加锁并 notify 消费者
                        ctx->targetTaskQueue.push(targetBuf);
                        
                    } else {
                        // 转换失败，立即归还避免内存泄漏
                        std::cerr << "[RGA Error] Process failed, returning buffer to pool." << std::endl;
                        ctx->rgaOutputPool->release_buffer(targetBuf);
                    }

                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    std::cout << "[time] RGA (NV12 -> RGB888 & EIS): " << duration.count() << " ms." << std::endl;
                } else {
                    // 缓冲池干涸策略：通常意味着下游处理太慢，此时直接丢弃当前帧 (Drop Frame)
                    std::cerr << "[Thread] Warning: RGA buffer pool empty! Dropping frame." << std::endl;
                }

                // ==========================================================

                // 2. RGA（或其他）处理完毕后，将该 Buffer 重新入队交还给摄像头驱动
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
    if (ctx->rgaOutputPool) {
        ctx->rgaOutputPool->destroy_pool();
        ctx->rgaOutputPool.reset(); 
    }
}

DmaBuffer_t* SentinelVisioner::wait_get_rga_buffer(int camNum) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return nullptr;
    }

    CameraContext* ctx = it->second.get();
    
    // 调用安全队列的阻塞方法，线程会在这里休眠，直到捕获线程 push 了新的一帧
    return ctx->targetTaskQueue.pop(); 
}

void SentinelVisioner::release_rga_buffer(int camNum, DmaBuffer_t* buf) {
    if (buf == nullptr) return;

    auto it = _cameraContextMap.find(camNum);
    if (it != _cameraContextMap.end()) {
        // 交还给对应摄像头的专用内存池
        it->second->rgaOutputPool->release_buffer(buf);
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
