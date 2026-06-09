#include "sentinel-visioner.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <cstring>
#include <cerrno>
#include <linux/dma-buf.h>

namespace {
void sync_dma_buf_for_device(int fd) {
    if (fd <= 0) return;
    struct dma_buf_sync sync = {};
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
}
}

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
                                  int bufferCount, int camNum, CameraType camType) {
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
    ctx->camType = camType;
    ctx->v4l2BufType = (camType == CameraType::ISP_CAM)
        ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
        : V4L2_BUF_TYPE_VIDEO_CAPTURE;

    // 打开设备节点
    ctx->camFd = open(deviceName.c_str(), O_RDWR | O_NONBLOCK);
    if (ctx->camFd < 0) {
        std::cerr << "Failed to open " << deviceName << ": " << strerror(errno) << std::endl;
        return false;
    }

    // 设置图像格式
    struct v4l2_format fmt = {};
    fmt.type = ctx->v4l2BufType;

    if (camType == CameraType::ISP_CAM) {
        fmt.fmt.pix_mp.width = width;
        fmt.fmt.pix_mp.height = height;
        fmt.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix_mp.field = V4L2_FIELD_NONE;
    } else {
        fmt.fmt.pix.width = width;
        fmt.fmt.pix.height = height;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
        fmt.fmt.pix.field = V4L2_FIELD_NONE;
    }

    if (ioctl(ctx->camFd, VIDIOC_S_FMT, &fmt) < 0) {
        if (camType == CameraType::USB_CAM) {
            // NV12 不支持，回退到 YUYV
            fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
            if (ioctl(ctx->camFd, VIDIOC_S_FMT, &fmt) < 0) {
                std::cerr << "VIDIOC_S_FMT failed for both NV12 and YUYV: "
                          << strerror(errno) << std::endl;
                release_camera_resources_(ctx.get());
                return false;
            }
            std::cout << "[USB Cam] NV12 unsupported, using YUYV." << std::endl;
        } else {
            std::cerr << "VIDIOC_S_FMT failed: " << strerror(errno) << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
    }

    // 回读实际协商后的格式和分辨率
    if (ioctl(ctx->camFd, VIDIOC_G_FMT, &fmt) == 0) {
        if (camType == CameraType::ISP_CAM) {
            ctx->actualPixelFormat = fmt.fmt.pix_mp.pixelformat;
            int actualW = fmt.fmt.pix_mp.width;
            int actualH = fmt.fmt.pix_mp.height;
            if (actualW != width || actualH != height) {
                std::cout << "[ISP Cam] Resolution negotiated: " << actualW << "x" << actualH
                          << " (requested " << width << "x" << height << ")" << std::endl;
                ctx->width = actualW;
                ctx->height = actualH;
            }
        } else {
            ctx->actualPixelFormat = fmt.fmt.pix.pixelformat;
            ctx->srcBytesPerLine = fmt.fmt.pix.bytesperline;
            int actualW = fmt.fmt.pix.width;
            int actualH = fmt.fmt.pix.height;
            if (actualW != width || actualH != height) {
                std::cout << "[USB Cam] Resolution negotiated: " << actualW << "x" << actualH
                          << " (requested " << width << "x" << height << ")" << std::endl;
                ctx->width = actualW;
                ctx->height = actualH;
            }
            if (ctx->srcBytesPerLine > 0 && ctx->srcBytesPerLine != ctx->width) {
                std::cout << "[USB Cam] bytesperline=" << ctx->srcBytesPerLine
                          << " width=" << ctx->width
                          << " fmt=" << (ctx->actualPixelFormat == V4L2_PIX_FMT_YUYV ? "YUYV" : "NV12")
                          << std::endl;
            }
        }
    } else {
        // G_FMT 失败，使用请求值
        ctx->actualPixelFormat = (camType == CameraType::ISP_CAM)
            ? V4L2_PIX_FMT_NV12 : V4L2_PIX_FMT_YUYV;
    }

    // USB YUYV 需要中间 NV12 转换缓冲池
    if (ctx->actualPixelFormat == V4L2_PIX_FMT_YUYV) {
        ctx->usbConvertPool = std::make_unique<DmaBufferPool>();
        if (!ctx->usbConvertPool->alloc_pool(bufferCount, ctx->width, ctx->height,
                                              BufferFormat::NV12)) {
            std::cerr << "USB convert pool allocation failed!" << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
    }

    // USB NV12 需要安全拷贝缓冲池（RGA 直接读 USB DMA-BUF 有硬件兼容问题）
    if (camType == CameraType::USB_CAM && ctx->actualPixelFormat == V4L2_PIX_FMT_NV12) {
        ctx->usbSafePool = std::make_unique<DmaBufferPool>();
        if (!ctx->usbSafePool->alloc_pool(4, ctx->width, ctx->height, BufferFormat::NV12)) {
            std::cerr << "USB safe pool allocation failed!" << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
    }

    // 初始化 DMA 内存池（在格式协商之后，使用实际分辨率）
    ctx->npuRgbPool = std::make_unique<DmaBufferPool>();
    if (!ctx->npuRgbPool->alloc_pool(bufferCount, 640, 640, BufferFormat::RGB888)) {
        std::cerr << "初始化 NPU 内存池失败!———— " << camNum << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    ctx->origCopyPool = std::make_unique<DmaBufferPool>();
    if (!ctx->origCopyPool->alloc_pool(bufferCount, ctx->width, ctx->height, BufferFormat::NV12)) {
        std::cerr << "初始化原始图像拷贝内存池失败!————" << camNum << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    ctx->previewPool = std::make_unique<DmaBufferPool>();
    if (!ctx->previewPool->alloc_pool(bufferCount, ctx->width, ctx->height, BufferFormat::RGB888)) {
        std::cerr << "初始化 1080P 预览图像内存池失败!————" << camNum << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }

    // --- 尝试设置帧率为 30 FPS ---
    struct v4l2_streamparm streamparm = {};
    streamparm.type = ctx->v4l2BufType;
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
    req.type = ctx->v4l2BufType;
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
        expbuf.type = ctx->v4l2BufType;
        expbuf.index = i;
        expbuf.flags = O_CLOEXEC | O_RDWR;

        if (ioctl(ctx->camFd, VIDIOC_EXPBUF, &expbuf) < 0) {
            std::cerr << "VIDIOC_EXPBUF failed: " << strerror(errno) << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
        ctx->buffers[i].dmaFd = expbuf.fd;

        struct v4l2_buffer buf = {};
        buf.type = ctx->v4l2BufType;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        struct v4l2_plane planes[1] = {};
        if (ctx->v4l2BufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            buf.m.planes = planes;
            buf.length = 1;
        }

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
    int type = ctx->v4l2BufType;

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

        std::cout << "Camera " << camNum << " capture thread STARTED." << std::endl;
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
                buf.type = ctx->v4l2BufType;
                buf.memory = V4L2_MEMORY_MMAP;

                struct v4l2_plane planes[1] = {};
                if (ctx->v4l2BufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                    buf.m.planes = planes;
                    buf.length = 1;
                }

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

                // DMA 缓存同步: 确保 USB 相机写入的数据对 RGA 硬件可见
                sync_dma_buf_for_device(currentDmaFd);

                // USB YUYV→NV12 格式转换
                int nv12DmaFd = currentDmaFd;
                DmaBuffer_t* convBufToRelease = nullptr;
                if (ctx->actualPixelFormat == V4L2_PIX_FMT_YUYV) {
                    DmaBuffer_t* convBuf = ctx->usbConvertPool->get_buffer();
                    if (convBuf == nullptr) {
                        std::cerr << "[Thread] USB convert pool empty! Dropping frame." << std::endl;
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    convBuf->timestampUs = timestampUs;
                    int srcStride = ctx->srcBytesPerLine > 0
                        ? ctx->srcBytesPerLine / 2 : ctx->width;
                    if (!rga_yuyv_to_nv12_(currentDmaFd, ctx->width, ctx->height,
                                           srcStride, convBuf)) {
                        std::cerr << "[RGA Error] YUYV->NV12 conversion failed!" << std::endl;
                        ctx->usbConvertPool->release_buffer(convBuf);
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    nv12DmaFd = convBuf->dmaFd;
                    convBufToRelease = convBuf;
                }

                // NV12 源 stride: usbConvertPool 无 padding，相机原生缓冲区可能有
                int nv12Stride = (convBufToRelease != nullptr)
                    ? ctx->width
                    : (ctx->srcBytesPerLine > 0 ? ctx->srcBytesPerLine : ctx->width);

                // 暂停模式: 跳过所有 RGA 处理和队列推送，只归还 buffer
                if (ctx->isPaused.load()) {
                    if (convBufToRelease != nullptr) {
                        ctx->usbConvertPool->release_buffer(convBufToRelease);
                    }
                    if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                        perror("[Thread] VIDIOC_QBUF requeue failed");
                        ctx->isThreadRunning = false;
                        break;
                    }
                    continue;
                }

                // USB 相机原生 NV12: 先 RGA 拷贝到安全池缓冲区，
                // 尽早归还相机缓冲区，后续所有 RGA 操作从安全池读取
                DmaBuffer_t* safeBuf = nullptr;
                DmaBuffer_t* safeBufToRelease = nullptr;
                if (ctx->camType == CameraType::USB_CAM && convBufToRelease == nullptr) {
                    safeBuf = ctx->usbSafePool->get_buffer();
                    if (!safeBuf) {
                        std::cerr << "[Thread] USB safe pool empty, drop frame" << std::endl;
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    safeBuf->timestampUs = timestampUs;
                    if (rga_copy_buffer_(nv12DmaFd, ctx->width, ctx->height,
                                         nv12Stride, safeBuf)) {
                        nv12DmaFd = safeBuf->dmaFd;
                        nv12Stride = safeBuf->width;
                        safeBufToRelease = safeBuf;
                    } else {
                        std::cerr << "[RGA Error] USB safe copy failed, drop frame" << std::endl;
                        ctx->usbSafePool->release_buffer(safeBuf);
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    // 拷贝完成后立即归还相机缓冲区
                    if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                        perror("[Thread] VIDIOC_QBUF requeue failed");
                        ctx->isThreadRunning = false;
                        break;
                    }
                }

                /* 从内存池获取空闲的 DMA 块 */
                DmaBuffer_t* targetNpuBuf = ctx->npuRgbPool->get_buffer();
                DmaBuffer_t* targetPreviewBuf = ctx->previewPool->get_buffer();

                if (targetNpuBuf != nullptr) {
                    // TODO: 这里的偏移量(横向/纵向)通常由外部 IMU 陀螺仪计算后传入
                    // 此处模拟获取实时的防抖平移参数
                    int currentHorizOffset = 0;
                    int currentVertOffset  = 0;

                    // 记录时间戳
                    targetNpuBuf->timestampUs = timestampUs;
                    if (targetPreviewBuf) targetPreviewBuf->timestampUs = timestampUs;

                    auto start_time = std::chrono::high_resolution_clock::now();

                    // 操作 A: RGA 缩放并转码给 NPU (1080P NV12 -> 640 RGB888)
                    bool npuOk = rga_process_to_rgb_(nv12DmaFd, ctx->width, ctx->height,
                                                    nv12Stride, targetNpuBuf,
                                                    currentHorizOffset, currentVertOffset);

                    // 操作 B: RGA 转码 (1080P NV12 -> 1080P RGB888 预览)
                    bool previewOk = true;
                    if (targetPreviewBuf != nullptr) {
                        previewOk = rga_convert_to_rgb_full_(nv12DmaFd, ctx->width, ctx->height,
                                                              nv12Stride, targetPreviewBuf);
                    }

                    if (npuOk) {
                        ctx->npuTaskQueue.push(targetNpuBuf);
                    } else {
                        std::cerr << "[RGA Error] NPU 转换失败，归还内存." << std::endl;
                        ctx->npuRgbPool->release_buffer(targetNpuBuf);
                    }

                    if (targetPreviewBuf != nullptr) {
                        if (previewOk) {
                            ctx->previewTaskQueue.push(targetPreviewBuf);
                        } else {
                            std::cerr << "[RGA Error] 预览转换失败，归还内存." << std::endl;
                            ctx->previewPool->release_buffer(targetPreviewBuf);
                        }
                    }

                    auto end_time = std::chrono::high_resolution_clock::now();
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                    if (raw_frame_count % 30 == 0)
                        std::cout << "[time] RGA (NPU + Preview): " << duration.count() << " ms." << std::endl;
                } else {
                    // 缓冲池干涸策略：通常意味着下游处理太慢，此时直接丢弃当前帧 (Drop Frame)
                    std::cerr << "[Thread] Warning: RGA buffer pool empty! Dropping frame." << std::endl;
                    if (targetNpuBuf) ctx->npuRgbPool->release_buffer(targetNpuBuf);
                    if (targetPreviewBuf) ctx->previewPool->release_buffer(targetPreviewBuf);
                }

                // 推流/录像用的 origCopy 缓冲区
                {
                    DmaBuffer_t* targetOrigBuf = ctx->origCopyPool->get_buffer();
                    if (targetOrigBuf != nullptr) {
                        targetOrigBuf->timestampUs = timestampUs;
                        bool copyOk = rga_copy_buffer_(nv12DmaFd, ctx->width, ctx->height,
                                                        nv12Stride, targetOrigBuf);
                        if (copyOk) {
                            ctx->processTaskQueue.push(targetOrigBuf);
                        } else {
                            std::cerr << "[RGA Error] 拷贝图像失败，立即归还避免内存泄漏." << std::endl;
                            ctx->origCopyPool->release_buffer(targetOrigBuf);
                        }
                    }
                }

                // 归还 USB 安全拷贝缓冲
                if (safeBufToRelease != nullptr) {
                    ctx->usbSafePool->release_buffer(safeBufToRelease);
                }

                // 归还 USB 转换缓冲
                if (convBufToRelease != nullptr) {
                    ctx->usbConvertPool->release_buffer(convBufToRelease);
                }

                // ISP/YUYV 路径在这里归还相机缓冲区；USB NV12 路径已在上方归还
                if (ctx->camType != CameraType::USB_CAM || convBufToRelease != nullptr) {
                    if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                        perror("[Thread] VIDIOC_QBUF requeue failed");
                        ctx->isThreadRunning = false;
                        break;
                    }
                }
            }
        }
    }

    std::cout << "[Thread] Camera " << camNum << " capture thread exited." << std::endl;
}

void SentinelVisioner::camera_pause(int camNum, bool paused) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) return;
    it->second->isPaused.store(paused);
    std::cout << "Camera " << camNum << (paused ? " PAUSED." : " RESUMED.") << std::endl;
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
    if (ctx->previewPool) {
        ctx->previewPool->destroy_pool();
        ctx->previewPool.reset();
    }

    if (ctx->usbConvertPool) {
        ctx->usbConvertPool->destroy_pool();
        ctx->usbConvertPool.reset();
    }

    if (ctx->usbSafePool) {
        ctx->usbSafePool->destroy_pool();
        ctx->usbSafePool.reset();
    }
}

DmaBuffer_t* SentinelVisioner::wait_get_npu(int camNum) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return nullptr;
    }
    return it->second->npuTaskQueue.pop();
}

DmaBuffer_t* SentinelVisioner::try_get_npu(int camNum, int timeoutMs) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return nullptr;
    }
    DmaBuffer_t* result = nullptr;
    it->second->npuTaskQueue.try_pop(result, timeoutMs);
    return result;
}

void SentinelVisioner::release_npu(int camNum, DmaBuffer_t* buf) {
    if (buf == nullptr) return;
    auto it = _cameraContextMap.find(camNum);
    if (it != _cameraContextMap.end()) {
        it->second->npuRgbPool->release_buffer(buf);
    }
}

DmaBuffer_t* SentinelVisioner::wait_get_preview(int camNum) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return nullptr;
    }
    return it->second->previewTaskQueue.pop();
}

DmaBuffer_t* SentinelVisioner::try_get_preview(int camNum, int timeoutMs) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        return nullptr;
    }
    DmaBuffer_t* result = nullptr;
    it->second->previewTaskQueue.try_pop(result, timeoutMs);
    return result;
}

void SentinelVisioner::release_preview(int camNum, DmaBuffer_t* buf) {
    if (buf == nullptr) return;
    auto it = _cameraContextMap.find(camNum);
    if (it != _cameraContextMap.end()) {
        it->second->previewPool->release_buffer(buf);
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
                                           int srcStride,
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
    im_handle_param_t in_param = { srcStride, srcHeight, srcFmt };
    rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) return false;

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, dstFmt };
    rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    // 2. 包装 RGA Buffer
    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, srcStride, srcHeight, srcFmt, srcStride, srcHeight);
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
    
    ret_rga = improcess(rga_buf_src, rga_buf_dst, pat, srect, drect, prect, IM_SYNC);
    if (ret_rga <= 0) {
        std::cerr << "[RGA Error] improcess failed." << std::endl;
        ret = false;
    }

    // 6. 释放 RGA 句柄
    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    return ret;
}

bool SentinelVisioner::rga_convert_to_rgb_full_(int srcFd, int srcWidth, int srcHeight,
                                                 int srcStride, DmaBuffer_t* dstBuf) {
    if (srcFd <= 0 || !dstBuf || dstBuf->dmaFd <= 0) {
        std::cerr << "[RGA Error] Invalid DMA fd!" << std::endl;
        return false;
    }

    int srcFmt = RK_FORMAT_YCrCb_420_SP;
    int dstFmt = RK_FORMAT_BGR_888;

    im_handle_param_t in_param = { srcStride, srcHeight, srcFmt };
    rga_buffer_handle_t rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) return false;

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, dstFmt };
    rga_buffer_handle_t rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, srcStride, srcHeight, srcFmt,
                                                  srcStride, srcHeight);
    rga_buffer_t rga_buf_dst = wrapbuffer_handle(rga_handle_dst, dstBuf->width, dstBuf->height, dstFmt,
                                                  dstBuf->width, dstBuf->height);

    im_rect srect = { 0, 0, srcWidth, srcHeight };
    im_rect drect = { 0, 0, dstBuf->width, dstBuf->height };

    rga_buffer_t pat;
    memset(&pat, 0, sizeof(rga_buffer_t));
    im_rect prect;
    memset(&prect, 0, sizeof(im_rect));

    IM_STATUS ret_rga = improcess(rga_buf_src, rga_buf_dst, pat, srect, drect, prect, IM_SYNC);

    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    if (ret_rga <= 0) {
        std::cerr << "[RGA Error] NV12 -> RGB888 failed: " << imStrError(ret_rga) << std::endl;
        return false;
    }

    return true;
}

bool SentinelVisioner::rga_copy_buffer_(int srcFd, int width, int height,
                                         int srcStride, DmaBuffer_t* dstBuf) {
    if (srcFd <= 0 || !dstBuf || dstBuf->dmaFd <= 0) return false;

    // MIPI 摄像头通常输入为 NV12 (YCrCb_420_SP)
    int fmt = RK_FORMAT_YCrCb_420_SP;

    im_handle_param_t in_param = { srcStride, height, fmt };
    rga_buffer_handle_t rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) return false;

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, fmt };
    rga_buffer_handle_t rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, srcStride, height, fmt, srcStride, height);
    rga_buffer_t rga_buf_dst = wrapbuffer_handle(rga_handle_dst, dstBuf->width, dstBuf->height, fmt, dstBuf->width, dstBuf->height);

    // 调用 RGA 硬件拷贝
    IM_STATUS ret_rga = imcopy(rga_buf_src, rga_buf_dst);

    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    return ret_rga == IM_STATUS_SUCCESS;
}

bool SentinelVisioner::rga_yuyv_to_nv12_(int srcFd, int srcWidth, int srcHeight,
                                          int srcStride, DmaBuffer_t* dstBuf) {
    if (srcFd <= 0 || !dstBuf || dstBuf->dmaFd <= 0) {
        std::cerr << "[RGA Error] Invalid DMA fd for YUYV->NV12!" << std::endl;
        return false;
    }

    int srcFmt = RK_FORMAT_YUYV_422;
    int dstFmt = RK_FORMAT_YCrCb_420_SP;

    im_handle_param_t in_param = { srcWidth, srcHeight, srcFmt };
    rga_buffer_handle_t rga_handle_src = importbuffer_fd(srcFd, &in_param);
    if (rga_handle_src <= 0) return false;

    im_handle_param_t dst_param = { dstBuf->width, dstBuf->height, dstFmt };
    rga_buffer_handle_t rga_handle_dst = importbuffer_fd(dstBuf->dmaFd, &dst_param);
    if (rga_handle_dst <= 0) {
        releasebuffer_handle(rga_handle_src);
        return false;
    }

    rga_buffer_t rga_buf_src = wrapbuffer_handle(rga_handle_src, srcWidth, srcHeight,
                                                  srcFmt, srcStride, srcHeight);
    rga_buffer_t rga_buf_dst = wrapbuffer_handle(rga_handle_dst, dstBuf->width, dstBuf->height,
                                                  dstFmt, dstBuf->width, dstBuf->height);

    im_rect srect = { 0, 0, srcWidth, srcHeight };
    im_rect drect = { 0, 0, dstBuf->width, dstBuf->height };

    rga_buffer_t pat;
    memset(&pat, 0, sizeof(rga_buffer_t));
    im_rect prect;
    memset(&prect, 0, sizeof(im_rect));

    IM_STATUS ret_rga = improcess(rga_buf_src, rga_buf_dst, pat, srect, drect, prect, IM_SYNC);

    releasebuffer_handle(rga_handle_src);
    releasebuffer_handle(rga_handle_dst);

    if (ret_rga <= 0) {
        std::cerr << "[RGA Error] YUYV_422 -> NV12 failed: "
                  << imStrError(ret_rga) << std::endl;
        return false;
    }
    return true;
}
