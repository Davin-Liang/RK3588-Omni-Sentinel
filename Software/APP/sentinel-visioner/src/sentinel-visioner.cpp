#include "sentinel-visioner.h"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <cstring>
#include <cerrno>
#include <linux/dma-buf.h>
#include <sys/mman.h>
#include <cmath>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libavutil/pixdesc.h>
}

namespace {
void sync_dma_buf_for_device(int fd) {
    if (fd <= 0) return;
    struct dma_buf_sync sync = {};
    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW;
    ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
}

int clamp_int(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

int align_down_even(int v) {
    return v & ~1;
}

int align_up_even(int v) {
    return (v + 1) & ~1;
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

    // USB MJPG 需要软件解码缓冲池（FFmpeg MJPG→NV12）
    if (ctx->actualPixelFormat == V4L2_PIX_FMT_MJPEG) {
        ctx->mjpegDecodePool = std::make_unique<DmaBufferPool>();
        if (!ctx->mjpegDecodePool->alloc_pool(bufferCount, ctx->width, ctx->height,
                                              BufferFormat::NV12)) {
            std::cerr << "MJPG decode pool allocation failed!" << std::endl;
            release_camera_resources_(ctx.get());
            return false;
        }
        std::cout << "[USB Cam] MJPG mode, " << ctx->width << "x" << ctx->height
                  << " bufferCount=" << bufferCount << std::endl;
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

    // 视觉 EIS 内部分析池：用于 LK 光流估计，不依赖 SentinelQT 预览线程是否开启。
    // 这里使用 640x360 RGB888 小图，既能降低 OpenCV 计算量，也避免 previewPool
    // 因 UI 预览未开启而耗尽时导致 EIS 不更新。
    ctx->visualEisPool = std::make_unique<DmaBufferPool>();
    if (!ctx->visualEisPool->alloc_pool(3, 640, 360, BufferFormat::RGB888)) {
        std::cerr << "初始化视觉 EIS 分析内存池失败!————" << camNum << std::endl;
        release_camera_resources_(ctx.get());
        return false;
    }
    std::cout << "[Visual EIS Debug] Camera " << camNum
              << " analysis pool allocated: 640x360 RGB888, count=3"
              << std::endl;

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

        // QUERYBUF 获取内存偏移，mmap 供 CPU 读取（MJPG 解码等场景）
        struct v4l2_buffer qbuf = {};
        qbuf.type = ctx->v4l2BufType;
        qbuf.memory = V4L2_MEMORY_MMAP;
        qbuf.index = i;
        struct v4l2_plane qplanes[1] = {};
        if (ctx->v4l2BufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
            qbuf.m.planes = qplanes;
            qbuf.length = 1;
        }
        if (ioctl(ctx->camFd, VIDIOC_QUERYBUF, &qbuf) == 0) {
            unsigned int offset = (ctx->v4l2BufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                ? qbuf.m.planes[0].m.mem_offset : qbuf.m.offset;
            unsigned int length = (ctx->v4l2BufType == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                ? qbuf.m.planes[0].length : qbuf.length;
            ctx->buffers[i].virtAddr = mmap(nullptr, length, PROT_READ | PROT_WRITE,
                                            MAP_SHARED, ctx->camFd, offset);
        } else {
            ctx->buffers[i].virtAddr = nullptr;
        }

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

    // 初始化视觉为主 EIS，默认关闭。
    // processHeight 按实际相机比例设置，避免视觉运动估计因缩放比例错误导致 dx/dy 失真。
    {
        VisionEisConfig vcfg;
        vcfg.camId = camNum;
        vcfg.inputWidth = ctx->width;
        vcfg.inputHeight = ctx->height;
        vcfg.processWidth = 640;
        vcfg.processHeight = std::max(1, (int)((double)vcfg.processWidth * ctx->height / ctx->width + 0.5));
        vcfg.maxOffsetPixel = 80;
        ctx->visualEis.reset(new VisionEisStabilizer(vcfg));
        ctx->visualEisEnabled.store(false);
        ctx->visualEisOffsetX.store(0);
        ctx->visualEisOffsetY.store(0);
        ctx->visualEisOffsetValid.store(false);
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

                int nv12DmaFd = currentDmaFd;
                DmaBuffer_t* convBufToRelease = nullptr;

                // USB MJPG→NV12 软件解码（FFmpeg）
                if (ctx->actualPixelFormat == V4L2_PIX_FMT_MJPEG) {
                    void* jpegVirtAddr = ctx->buffers[buf.index].virtAddr;
                    if (!jpegVirtAddr || buf.bytesused == 0) {
                        std::cerr << "[Thread] MJPG: no CPU mapping or zero size, drop" << std::endl;
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    DmaBuffer_t* decodeBuf = ctx->mjpegDecodePool->get_buffer();
                    if (!decodeBuf) {
                        std::cerr << "[Thread] MJPG decode pool empty! Dropping frame." << std::endl;
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    decodeBuf->timestampUs = timestampUs;
                    if (!mjpeg_decode_to_nv12_((const uint8_t*)jpegVirtAddr, buf.bytesused, decodeBuf)) {
                        std::cerr << "[Thread] MJPG decode failed, drop frame" << std::endl;
                        ctx->mjpegDecodePool->release_buffer(decodeBuf);
                        if (ioctl(ctx->camFd, VIDIOC_QBUF, &buf) < 0) {
                            perror("[Thread] VIDIOC_QBUF requeue failed");
                            ctx->isThreadRunning = false;
                            break;
                        }
                        continue;
                    }
                    nv12DmaFd = decodeBuf->dmaFd;
                    convBufToRelease = decodeBuf;
                }

                // USB YUYV→NV12 格式转换
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
                        if (ctx->actualPixelFormat == V4L2_PIX_FMT_MJPEG) {
                            ctx->mjpegDecodePool->release_buffer(convBufToRelease);
                        } else {
                            ctx->usbConvertPool->release_buffer(convBufToRelease);
                        }
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

                /* 从内存池获取空闲的 DMA 块。NPU 与预览各自独立，互不阻塞 */
                DmaBuffer_t* targetNpuBuf = ctx->npuRgbPool->get_buffer();
                DmaBuffer_t* targetPreviewBuf = ctx->previewPool->get_buffer();
                DmaBuffer_t* targetVisualEisBuf = nullptr;
                const bool visualEnabledNow = ctx->visualEisEnabled.load();
                if (visualEnabledNow) {
                    static thread_local int visualBranchDebugCnt = 0;
                    ++visualBranchDebugCnt;

                    if (!ctx->visualEis) {
                        if (visualBranchDebugCnt <= 5 || visualBranchDebugCnt % 30 == 0) {
                            std::cerr << "[Visual EIS Debug Cam " << camNum
                                      << "] enabled=1 but visualEis object is null."
                                      << std::endl;
                        }
                    } else if (!ctx->visualEisPool) {
                        if (visualBranchDebugCnt <= 5 || visualBranchDebugCnt % 30 == 0) {
                            std::cerr << "[Visual EIS Debug Cam " << camNum
                                      << "] enabled=1 but visualEisPool is null."
                                      << std::endl;
                        }
                    } else {
                        targetVisualEisBuf = ctx->visualEisPool->get_buffer();
                        if (!targetVisualEisBuf && (visualBranchDebugCnt <= 5 || visualBranchDebugCnt % 30 == 0)) {
                            std::cerr << "[Visual EIS Debug Cam " << camNum
                                      << "] enabled=1 but visualEisPool has no free buffer."
                                      << std::endl;
                        } else if (targetVisualEisBuf && (visualBranchDebugCnt <= 5 || visualBranchDebugCnt % 30 == 0)) {
                            std::cout << "[Visual EIS Debug Cam " << camNum
                                      << "] branch entered, buf=" << targetVisualEisBuf
                                      << " virt=" << targetVisualEisBuf->virtAddr
                                      << " dmaFd=" << targetVisualEisBuf->dmaFd
                                      << " size=" << targetVisualEisBuf->width << "x" << targetVisualEisBuf->height
                                      << std::endl;
                        }
                    }
                }

                auto start_time = std::chrono::high_resolution_clock::now();

                // 视觉 EIS 分析处理：先对“当前帧”计算 offset，再让后续 NPU/预览/录像分支使用该 offset。
                // 这样可以避免 15FPS cam0 使用上一帧 offset 带来的 66ms 级滞后。
                // 这一路不进入 Qt 预览队列；因此即使下位机 Qt 预览没有开启，
                // 只要 Web/Qt 打开防抖，就会持续输出 [Visual EIS Cam X] 日志并更新补偿量。
                if (targetVisualEisBuf != nullptr) {
                    targetVisualEisBuf->timestampUs = timestampUs;

                    bool analysisOk = rga_convert_to_rgb_full_(nv12DmaFd, ctx->width, ctx->height,
                                                               nv12Stride, targetVisualEisBuf);
                    if (analysisOk && targetVisualEisBuf->virtAddr) {
                        cv::Mat analysisMat(targetVisualEisBuf->height, targetVisualEisBuf->width,
                                            CV_8UC3, targetVisualEisBuf->virtAddr);

                        VisionImuAssistState imuState;
                        VisionImuAssistState* imuPtr = nullptr;
                        if (imu_assist_callback_ && imu_assist_callback_(timestampUs, camNum, imuState)) {
                            imuPtr = &imuState;
                        }

                        VisionEisResult vres;
                        ctx->visualEis->processFrame(analysisMat, timestampUs * 1000ULL, imuPtr, vres);
                        ctx->visualEisOffsetX.store(vres.offsetX);
                        ctx->visualEisOffsetY.store(vres.offsetY);
                        ctx->visualEisOffsetValid.store(vres.visualReliable || vres.usedFallback);

                        static thread_local int visualLogCount = 0;
                        ++visualLogCount;
                        // 调试阶段：开启后前 5 帧立即打印，之后每 30 帧打印一次，避免日志刷屏。
                        if (visualLogCount <= 5 || (visualLogCount % 30 == 0)) {
                            std::cout << "[Visual EIS Cam " << camNum << "] "
                                      << "reliable=" << vres.visualReliable
                                      << " fallback=" << vres.usedFallback
                                      << " offset=(" << vres.offsetX << "," << vres.offsetY << ")"
                                      << " dxdy=(" << vres.dx << "," << vres.dy << ")"
                                      << " pts=" << vres.trackedPoints
                                      << " inliers=" << vres.inliers
                                      << " alpha=" << vres.usedAlpha
                                      << " cost=" << vres.costMs << "ms"
                                      << " imu=" << (imuPtr ? 1 : 0)
                                      << std::endl;
                        }
                    } else {
                        static thread_local int visualFailCount = 0;
                        ++visualFailCount;
                        if (visualFailCount <= 5 || visualFailCount % 30 == 0) {
                            std::cerr << "[Visual EIS Debug Cam " << camNum << "] "
                                      << "analysis failed: analysisOk=" << analysisOk
                                      << " virt=" << targetVisualEisBuf->virtAddr
                                      << " src=" << ctx->width << "x" << ctx->height
                                      << " stride=" << nv12Stride
                                      << " dst=" << targetVisualEisBuf->width << "x" << targetVisualEisBuf->height
                                      << " dmaFd=" << targetVisualEisBuf->dmaFd
                                      << std::endl;
                        }
                    }

                    ctx->visualEisPool->release_buffer(targetVisualEisBuf);
                }

                // EIS 防抖偏移。
                // 新方案优先使用“视觉为主 + IMU辅助”模块刚刚对当前帧估计得到的 offset；
                // 这样后面的 NPU/预览/录像分支会尽量使用当前帧 offset，降低 15FPS cam0 的一帧滞后。
                // 如果未开启视觉 EIS，则兼容旧版外部 IMU offset 回调。
                int currentHorizOffset = 0;
                int currentVertOffset  = 0;
                bool eisActive = false;

                if (ctx->visualEisEnabled.load() && ctx->visualEisOffsetValid.load()) {
                    currentHorizOffset = ctx->visualEisOffsetX.load();
                    currentVertOffset  = ctx->visualEisOffsetY.load();
                    eisActive = true;
                } else if (eis_offset_callback_) {
                    int32_t eisX = 0, eisY = 0;
                    eisActive = eis_offset_callback_(timestampUs, camNum, eisX, eisY);
                    if (eisActive) {
                        currentHorizOffset = static_cast<int>(eisX);
                        currentVertOffset  = static_cast<int>(eisY);
                    }

                    // 仅旧版 IMU offset 回调使用这里的低通滤波。
                    // 新版视觉 EIS 已经在轨迹平滑阶段完成平滑，避免重复滤波造成滞后。
                    if (eisActive && ctx->eisPrevValid) {
                        float a = ctx->eisSmoothAlpha;
                        int sx = static_cast<int>(a * currentHorizOffset + (1.0f - a) * ctx->prevEisOffsetX);
                        int sy = static_cast<int>(a * currentVertOffset  + (1.0f - a) * ctx->prevEisOffsetY);
                        currentHorizOffset = sx;
                        currentVertOffset  = sy;
                    }
                    ctx->prevEisOffsetX = static_cast<int32_t>(currentHorizOffset);
                    ctx->prevEisOffsetY = static_cast<int32_t>(currentVertOffset);
                    ctx->eisPrevValid = eisActive;
                }

                // NPU 处理：有 buffer 就做，池空就跳过，不影响预览
                if (targetNpuBuf != nullptr) {
                    targetNpuBuf->timestampUs = timestampUs;

                    bool npuOk = rga_process_to_rgb_(nv12DmaFd, ctx->width, ctx->height,
                                                    nv12Stride, targetNpuBuf,
                                                    currentHorizOffset, currentVertOffset);
                    if (npuOk) {
                        // TODO: NPU 推理接入后改为 npuTaskQueue.push(targetNpuBuf)
                        //ctx->npuRgbPool->release_buffer(targetNpuBuf);
                        ctx->npuTaskQueue.push(targetNpuBuf);

                    } else {
                        std::cerr << "[RGA Error] NPU 转换失败，归还内存." << std::endl;
                        ctx->npuRgbPool->release_buffer(targetNpuBuf);
                    }
                }

                // 预览处理：有 buffer 就做，池空就跳过，不影响 NPU。
                // 新版视觉 EIS 的预览输出也会真正使用防抖后的画面，
                // 这样 SentinelQT 最终界面上按下“防抖开”后能直接看到稳定效果。
                if (targetPreviewBuf != nullptr) {
                    targetPreviewBuf->timestampUs = timestampUs;

                    // 第一步：先生成 raw RGB 预览，供 LK 光流估计当前帧运动。
                    bool previewOk = rga_convert_to_rgb_full_(nv12DmaFd, ctx->width, ctx->height,
                                                              nv12Stride, targetPreviewBuf);
                    if (previewOk) {
                        // 视觉 EIS 的运动估计已经在独立 visualEisPool 分支完成。
                        // 预览分支只负责把当前 offset 应用到当前预览图像，避免 UI 预览开关影响 EIS 计算。

                        // 如果 EIS 当前帧有可用 offset，则用当前帧估计出的 offset 重新生成防抖预览。
                        // targetPreviewBuf 会被覆盖成稳定后的 RGB 图像，随后推给 SentinelQT UI。
                        if (eisActive) {
                            bool stablePreviewOk = rga_process_to_rgb_(nv12DmaFd, ctx->width, ctx->height,
                                                                        nv12Stride, targetPreviewBuf,
                                                                        currentHorizOffset, currentVertOffset);
                            if (!stablePreviewOk) {
                                std::cerr << "[RGA Error] EIS preview conversion failed, fallback to raw preview." << std::endl;
                            }
                        }

                        ctx->previewTaskQueue.push(targetPreviewBuf);
                    } else {
                        std::cerr << "[RGA Error] 预览转换失败，归还内存." << std::endl;
                        ctx->previewPool->release_buffer(targetPreviewBuf);
                    }
                }

                // 连预览 buffer 都拿不到，说明预览池已枯竭（下游消费太慢）
                if (targetPreviewBuf == nullptr) {
                    // std::cerr << "[Thread] Warning: preview pool empty! Dropping frame." << std::endl;
                }

                // auto end_time = std::chrono::high_resolution_clock::now();
                // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
                // if (raw_frame_count % 30 == 0)
                //     std::cout << "[time] RGA (NPU + Preview): " << duration.count() << " ms." << std::endl;

                // 推流/录像用的 origCopy 缓冲区
                {
                    DmaBuffer_t* targetOrigBuf = ctx->origCopyPool->get_buffer();
                    if (targetOrigBuf != nullptr) {
                        targetOrigBuf->timestampUs = timestampUs;
                        targetOrigBuf->eisOffsetX = static_cast<int32_t>(currentHorizOffset);
                        targetOrigBuf->eisOffsetY = static_cast<int32_t>(currentVertOffset);
                        targetOrigBuf->eisActive = eisActive;
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

                // 归还 USB 转换/解码缓冲
                if (convBufToRelease != nullptr) {
                    if (ctx->actualPixelFormat == V4L2_PIX_FMT_MJPEG) {
                        ctx->mjpegDecodePool->release_buffer(convBufToRelease);
                    } else {
                        ctx->usbConvertPool->release_buffer(convBufToRelease);
                    }
                }

                // ISP/YUYV/MJPG 路径在这里归还相机缓冲区；USB NV12 路径已在上方归还
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

    if (ctx->visualEisPool) {
        ctx->visualEisPool->destroy_pool();
        ctx->visualEisPool.reset();
    }

    if (ctx->usbConvertPool) {
        ctx->usbConvertPool->destroy_pool();
        ctx->usbConvertPool.reset();
    }

    if (ctx->usbSafePool) {
        ctx->usbSafePool->destroy_pool();
        ctx->usbSafePool.reset();
    }

    if (ctx->mjpegDecodePool) {
        ctx->mjpegDecodePool->destroy_pool();
        ctx->mjpegDecodePool.reset();
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

void SentinelVisioner::set_eis_offset_callback(
    std::function<bool(uint64_t, int, int32_t&, int32_t&)> callback) {
    eis_offset_callback_ = std::move(callback);
}

void SentinelVisioner::set_eis_smooth_alpha(float alpha) {
    if (alpha < 0.0f) alpha = 0.0f;
    if (alpha > 1.0f) alpha = 1.0f;
    for (auto& kv : _cameraContextMap) {
        kv.second->eisSmoothAlpha = alpha;
    }
}


bool SentinelVisioner::set_visual_eis_config(int camNum, const VisionEisConfig& config) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        std::cerr << "Camera number " << camNum << " not found!" << std::endl;
        return false;
    }

    VisionEisConfig cfg = config;
    cfg.camId = camNum;
    if (cfg.inputWidth <= 0) cfg.inputWidth = it->second->width;
    if (cfg.inputHeight <= 0) cfg.inputHeight = it->second->height;
    if (cfg.processWidth <= 0) cfg.processWidth = 640;
    if (cfg.processHeight <= 0) {
        cfg.processHeight = std::max(1, (int)((double)cfg.processWidth * cfg.inputHeight / cfg.inputWidth + 0.5));
    }

    if (!it->second->visualEis) {
        it->second->visualEis.reset(new VisionEisStabilizer(cfg));
    } else {
        it->second->visualEis->setConfig(cfg);
    }

    it->second->visualEisOffsetX.store(0);
    it->second->visualEisOffsetY.store(0);
    it->second->visualEisOffsetValid.store(false);

    std::cout << "[Visual EIS] Camera " << camNum << " config updated: "
              << cfg.inputWidth << "x" << cfg.inputHeight
              << " process=" << cfg.processWidth << "x" << cfg.processHeight
              << " maxOffset=" << cfg.maxOffsetPixel << std::endl;
    return true;
}

bool SentinelVisioner::enable_visual_eis(int camNum, bool enable) {
    auto it = _cameraContextMap.find(camNum);
    if (it == _cameraContextMap.end()) {
        std::cerr << "Camera number " << camNum << " not found!" << std::endl;
        return false;
    }

    if (enable && !it->second->visualEis) {
        VisionEisConfig cfg;
        cfg.camId = camNum;
        cfg.inputWidth = it->second->width;
        cfg.inputHeight = it->second->height;
        cfg.processWidth = 640;
        cfg.processHeight = std::max(1, (int)((double)cfg.processWidth * cfg.inputHeight / cfg.inputWidth + 0.5));
        it->second->visualEis.reset(new VisionEisStabilizer(cfg));
    }

    if (it->second->visualEis) {
        it->second->visualEis->reset();
    }
    it->second->visualEisOffsetX.store(0);
    it->second->visualEisOffsetY.store(0);
    it->second->visualEisOffsetValid.store(false);
    it->second->visualEisEnabled.store(enable);

    std::cout << "[Visual EIS] Camera " << camNum << (enable ? " enabled." : " disabled.")
              << " threadRunning=" << it->second->isThreadRunning.load()
              << " paused=" << it->second->isPaused.load()
              << " hasEis=" << (it->second->visualEis ? 1 : 0)
              << " hasPool=" << (it->second->visualEisPool ? 1 : 0)
              << " size=" << it->second->width << "x" << it->second->height
              << std::endl;
    return true;
}

void SentinelVisioner::set_imu_assist_callback(
    std::function<bool(uint64_t, int, VisionImuAssistState&)> callback) {
    imu_assist_callback_ = std::move(callback);
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

    // 3. 计算 Letterbox / EIS 参数。
    // EIS 激活时采用“源图裁剪窗口平移 + 轻微 zoom”，为 X/Y 两个方向都预留补偿空间。
    // 关键修复：NV12/YUV 输入给 RGA 时，srect 的 x/y/w/h 必须保持偶数对齐，
    // 否则 improcess 很容易失败。之前 1.10 zoom 会产生 1745x981 这样的奇数裁剪，
    // 加上奇数 offset 后也会出现奇数 cropX/cropY，因此这里统一做偶数对齐和越界保护。
    im_rect srect = {0, 0, align_down_even(srcWidth), align_down_even(srcHeight)};

    if (horizontalOffset != 0 || verticalOffset != 0) {
        const float zoom = 1.10f;  // 约保留 9% 裁剪余量；可后续改成配置项
        int cropW = align_down_even(static_cast<int>(srcWidth / zoom));
        int cropH = align_down_even(static_cast<int>(srcHeight / zoom));
        if (cropW < 16 || cropW > srcWidth) cropW = align_down_even(srcWidth);
        if (cropH < 16 || cropH > srcHeight) cropH = align_down_even(srcHeight);

        float cropScale = std::min((float)dstBuf->width / cropW, (float)dstBuf->height / cropH);
        if (cropScale <= 0.0f) cropScale = 1.0f;

        // offset 的语义：希望输出画面向 offset 方向补偿。
        // 对源图裁剪来说，需要反向移动裁剪窗口，因此这里使用负号。
        int shiftSrcX = static_cast<int>(std::round(-horizontalOffset / cropScale));
        int shiftSrcY = static_cast<int>(std::round(-verticalOffset / cropScale));

        int cropX = (srcWidth - cropW) / 2 + shiftSrcX;
        int cropY = (srcHeight - cropH) / 2 + shiftSrcY;
        cropX = clamp_int(cropX, 0, srcWidth - cropW);
        cropY = clamp_int(cropY, 0, srcHeight - cropH);

        // NV12/YUV 裁剪起点也必须偶数对齐。对齐后再次防止越界。
        cropX = align_down_even(cropX);
        cropY = align_down_even(cropY);
        cropX = clamp_int(cropX, 0, srcWidth - cropW);
        cropY = clamp_int(cropY, 0, srcHeight - cropH);

        srect = {cropX, cropY, cropW, cropH};
    }

    float scale = std::min((float)dstBuf->width / srect.width, (float)dstBuf->height / srect.height);
    int scaled_w = align_down_even(static_cast<int>(srect.width * scale));
    int scaled_h = align_down_even(static_cast<int>(srect.height * scale));
    if (scaled_w <= 0) scaled_w = align_down_even(dstBuf->width);
    if (scaled_h <= 0) scaled_h = align_down_even(dstBuf->height);
    scaled_w = clamp_int(scaled_w, 2, align_down_even(dstBuf->width));
    scaled_h = clamp_int(scaled_h, 2, align_down_even(dstBuf->height));

    int offset_x = align_down_even((dstBuf->width - scaled_w) / 2);
    int offset_y = align_down_even((dstBuf->height - scaled_h) / 2);

    im_rect drect = {offset_x, offset_y, scaled_w, scaled_h};

    // 4. 背景填充 (Padding)
    if (scaled_w != dstBuf->width || scaled_h != dstBuf->height || horizontalOffset != 0 || verticalOffset != 0) {
        im_rect dst_whole_rect = {0, 0, dstBuf->width, dstBuf->height};
        ret_rga = imfill(rga_buf_dst, dst_whole_rect, 0xFF727272);
    }

    // 5. RGA 终极处理：格式转换 + 缩放 + 偏移写入
    rga_buffer_t pat; memset(&pat, 0, sizeof(rga_buffer_t));
    im_rect prect; memset(&prect, 0, sizeof(im_rect));
    
    ret_rga = improcess(rga_buf_src, rga_buf_dst, pat, srect, drect, prect, IM_SYNC);
    if (ret_rga <= 0) {
        std::cerr << "[RGA Error] improcess failed: " << imStrError(ret_rga)
                  << " src=" << srcWidth << "x" << srcHeight
                  << " stride=" << srcStride
                  << " dst=" << dstBuf->width << "x" << dstBuf->height
                  << " srect=(" << srect.x << "," << srect.y << ","
                  << srect.width << "," << srect.height << ")"
                  << " drect=(" << drect.x << "," << drect.y << ","
                  << drect.width << "," << drect.height << ")"
                  << " offset=(" << horizontalOffset << "," << verticalOffset << ")"
                  << std::endl;
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

    int srcFmt = RK_FORMAT_YVYU_422;  // 部分 USB 相机实际 U/V 与 YUYV 相反
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
        std::cerr << "[RGA Error] YVYU_422 -> NV12 failed: "
                  << imStrError(ret_rga) << std::endl;
        return false;
    }
    return true;
}

bool SentinelVisioner::mjpeg_decode_to_nv12_(const uint8_t* jpegData, size_t jpegSize,
                                            DmaBuffer_t* dstBuf) {
    if (!jpegData || jpegSize == 0 || !dstBuf || dstBuf->dmaFd <= 0) return false;

    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_MJPEG);
    if (!codec) {
        std::cerr << "[MJPG] avcodec_find_decoder failed" << std::endl;
        return false;
    }

    AVCodecContext* decCtx = avcodec_alloc_context3(codec);
    if (!decCtx) return false;

    if (avcodec_open2(decCtx, codec, nullptr) < 0) {
        avcodec_free_context(&decCtx);
        return false;
    }

    AVPacket* pkt = av_packet_alloc();
    if (!pkt) { avcodec_free_context(&decCtx); return false; }
    pkt->data = const_cast<uint8_t*>(jpegData);
    pkt->size = static_cast<int>(jpegSize);

    AVFrame* frame = av_frame_alloc();
    if (!frame) { av_packet_free(&pkt); avcodec_free_context(&decCtx); return false; }

    int ret = avcodec_send_packet(decCtx, pkt);
    if (ret < 0) {
        av_frame_free(&frame); av_packet_free(&pkt); avcodec_free_context(&decCtx);
        return false;
    }

    ret = avcodec_receive_frame(decCtx, frame);
    if (ret < 0) {
        av_frame_free(&frame); av_packet_free(&pkt); avcodec_free_context(&decCtx);
        return false;
    }

    static int dbgCount = 0;
    if (dbgCount++ == 0) {
        const char* fmtName = av_get_pix_fmt_name((AVPixelFormat)frame->format);
        fprintf(stderr, "[MJPG] decoded fmt=%s(%d) %dx%d range=%d\n",
                fmtName ? fmtName : "?", frame->format,
                frame->width, frame->height, frame->color_range);
    }

    // YUV → NV12: 复制 Y 平面，UV 交错写（兼容 420/422 子采样）
    uint8_t* dst = static_cast<uint8_t*>(dstBuf->virtAddr);
    size_t ySize = static_cast<size_t>(dstBuf->width) * dstBuf->height;

    if (frame->linesize[0] == dstBuf->width) {
        memcpy(dst, frame->data[0], ySize);
    } else {
        for (int r = 0; r < dstBuf->height; ++r)
            memcpy(dst + r * dstBuf->width, frame->data[0] + r * frame->linesize[0], dstBuf->width);
    }

    uint8_t* uvDst = dst + ySize;
    int uvWidth = dstBuf->width / 2;
    int uvHeight = dstBuf->height / 2;
    bool is422 = (frame->format == AV_PIX_FMT_YUVJ422P ||
                  frame->format == AV_PIX_FMT_YUV422P);
    int uvRowStep = is422 ? 2 : 1;  // 422: UV rows = full height, subsample to half

    for (int r = 0; r < uvHeight; ++r) {
        int srcR = r * uvRowStep;
        uint8_t* uSrc = frame->data[1] + srcR * frame->linesize[1];
        uint8_t* vSrc = frame->data[2] + srcR * frame->linesize[2];
        for (int c = 0; c < uvWidth; ++c) {
            *uvDst++ = uSrc[c];
            *uvDst++ = vSrc[c];
        }
    }

    av_frame_free(&frame);
    av_packet_free(&pkt);
    avcodec_free_context(&decCtx);
    return true;
}

