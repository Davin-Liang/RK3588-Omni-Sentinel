/**
 * @brief  棋盘格相机标定工具 — V4L2 原生捕获 + OpenCV 标定
 *
 * 用法:  ./camera_calibrate <device> <square_mm> [options]
 * 示例:  ./camera_calibrate /dev/video11 25
 *        ./camera_calibrate /dev/video21 30 --boards 8x6 --min-frames 15
 */

#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <vector>
#include <string>
#include <algorithm>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

static volatile bool gStop = false;
static void sigint_handler(int) { gStop = true; }

// ============================================================================
// 简易 V4L2 捕获类 — 支持 MMAP / USERPTR 双模式
// ============================================================================
class V4L2Capture {
public:
    V4L2Capture() : fd_(-1), width_(0), height_(0), stride_(0), pixelFormat_(0), mplane_(false) {
        memset(buffers_, 0, sizeof(buffers_));
    }

    ~V4L2Capture() { close(); }

    bool open(const char* device, int w, int h) {
        close();

        fd_ = ::open(device, O_RDWR);
        if (fd_ < 0) {
            fprintf(stderr, "[V4L2] Cannot open %s: %s\n", device, strerror(errno));
            return false;
        }

        // 查询能力
        v4l2_capability cap = {};
        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            fprintf(stderr, "[V4L2] VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
            close(); return false;
        }

        mplane_ = cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE;
        printf("[V4L2] %s driver=%s %s\n", device, cap.driver,
               mplane_ ? "MPLANE" : "SINGLE_PLANE");

        // 枚举格式，找一个能用的
        uint32_t fmt = try_format_(w, h, mplane_);
        if (fmt == 0) {
            close(); return false;
        }

        // 设置格式
        if (!set_format_(w, h, fmt, mplane_)) {
            close(); return false;
        }

        // 请求 buffer
        if (!request_buffers_(4, mplane_)) {
            close(); return false;
        }

        // 启动流
        int type = mplane_ ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                            : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0) {
            fprintf(stderr, "[V4L2] VIDIOC_STREAMON failed: %s\n", strerror(errno));
            close(); return false;
        }

        // 丢弃前几帧（AE 收敛）
        for (int i = 0; i < 10; ++i) {
            cv::Mat dummy;
            if (!read(dummy)) { close(); return false; }
        }
        printf("[V4L2] Stream started: %dx%d\n", width_, height_);
        return true;
    }

    void close() {
        if (fd_ >= 0) {
            int type = mplane_ ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                               : V4L2_BUF_TYPE_VIDEO_CAPTURE;
            ioctl(fd_, VIDIOC_STREAMOFF, &type);
        }
        for (int i = 0; i < kMaxBuf; ++i) {
            if (buffers_[i].start && buffers_[i].start != MAP_FAILED)
                munmap(buffers_[i].start, buffers_[i].length);
            if (buffers_[i].userPtr)
                free(buffers_[i].userPtr);
        }
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool read(cv::Mat& frame) {
        if (fd_ < 0) return false;

        v4l2_buffer buf = {};
        buf.type = mplane_ ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                           : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        v4l2_plane planes[1] = {};
        if (mplane_) {
            buf.m.planes = planes;
            buf.length   = 1;
        }

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) return false;
            fprintf(stderr, "[V4L2] DQBUF error: %s\n", strerror(errno));
            return false;
        }

        void* data = buffers_[buf.index].start;
        size_t len = buffers_[buf.index].length;

        // NV12 → BGR (处理 stride 对齐)
        if (pixelFormat_ == V4L2_PIX_FMT_NV12 || pixelFormat_ == V4L2_PIX_FMT_NV12M) {
            int s = (stride_ > 0 && stride_ >= width_) ? stride_ : width_;
            if (s == width_) {
                cv::Mat yuv(height_ * 3 / 2, width_, CV_8UC1, data);
                cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_NV12);
            } else {
                // stride > width: 逐行拷贝去 padding
                cv::Mat yuv(height_ * 3 / 2, width_, CV_8UC1);
                uint8_t* dst = yuv.data;
                const uint8_t* src = (const uint8_t*)data;
                // Y 平面: height_ 行, 每行 width_ 有效像素
                for (int r = 0; r < height_; ++r) {
                    memcpy(dst, src, width_);
                    dst += width_;
                    src += s;
                }
                // UV 平面: height_/2 行, 每行 width_ 有效像素
                for (int r = 0; r < height_ / 2; ++r) {
                    memcpy(dst, src, width_);
                    dst += width_;
                    src += s;
                }
                cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_NV12);
            }
        } else if (pixelFormat_ == V4L2_PIX_FMT_YUYV) {
            cv::Mat yuyv(height_, width_, CV_8UC2, data);
            cv::cvtColor(yuyv, frame, cv::COLOR_YUV2BGR_YUYV);
        } else if (pixelFormat_ == V4L2_PIX_FMT_MJPEG) {
            cv::Mat jpeg(1, len, CV_8UC1, data);
            frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
        } else {
            // 假设 RGB/BGR
            frame = cv::Mat(height_, width_, CV_8UC3, data).clone();
        }

        ioctl(fd_, VIDIOC_QBUF, &buf);
        return !frame.empty();
    }

    int width()  const { return width_; }
    int height() const { return height_; }

private:
    static const int kMaxBuf = 8;
    struct Buffer {
        void*  start = nullptr;
        size_t length = 0;
        void*  userPtr = nullptr;  // USERPTR fallback
    };

    int      fd_;
    int      width_, height_;
    int      stride_;    // Y plane 每行字节数（可能 > width_）
    uint32_t pixelFormat_;
    bool     mplane_;
    Buffer   buffers_[kMaxBuf];

    uint32_t try_format_(int w, int h, bool mplane) {
        // 优先级: NV12 > YUYV > MJPG
        uint32_t fmts[] = {
            V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV12M,
            V4L2_PIX_FMT_YUYV,
            V4L2_PIX_FMT_MJPEG,
            0
        };
        v4l2_fmtdesc fdesc = {};
        fdesc.type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                            : V4L2_BUF_TYPE_VIDEO_CAPTURE;

        while (ioctl(fd_, VIDIOC_ENUM_FMT, &fdesc) == 0) {
            for (int i = 0; fmts[i]; ++i) {
                if (fdesc.pixelformat == fmts[i]) {
                    printf("[V4L2] Found format: %c%c%c%c\n",
                           (char)(fmts[i] & 0xFF),
                           (char)((fmts[i] >> 8) & 0xFF),
                           (char)((fmts[i] >> 16) & 0xFF),
                           (char)((fmts[i] >> 24) & 0xFF));
                    return fmts[i];
                }
            }
            fdesc.index++;
        }
        return 0;
    }

    bool set_format_(int w, int h, uint32_t fmt, bool mplane) {
        if (mplane) {
            v4l2_format fmtDesc = {};
            fmtDesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ioctl(fd_, VIDIOC_G_FMT, &fmtDesc);
            fmtDesc.fmt.pix_mp.width       = w;
            fmtDesc.fmt.pix_mp.height      = h;
            fmtDesc.fmt.pix_mp.pixelformat = fmt;
            fmtDesc.fmt.pix_mp.num_planes  = (fmt == V4L2_PIX_FMT_NV12M) ? 2 : 1;
            if (ioctl(fd_, VIDIOC_S_FMT, &fmtDesc) < 0) {
                fprintf(stderr, "[V4L2] S_FMT MPLANE failed: %s\n", strerror(errno));
                return false;
            }
            width_  = fmtDesc.fmt.pix_mp.width;
            height_ = fmtDesc.fmt.pix_mp.height;
            stride_ = fmtDesc.fmt.pix_mp.plane_fmt[0].bytesperline;
            if (stride_ == 0) stride_ = width_;
            printf("[V4L2] MPLANE: got %dx%d stride=%d\n", width_, height_, stride_);
        } else {
            v4l2_format fmtDesc = {};
            fmtDesc.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            fmtDesc.fmt.pix.width       = w;
            fmtDesc.fmt.pix.height      = h;
            fmtDesc.fmt.pix.pixelformat = fmt;
            fmtDesc.fmt.pix.field       = V4L2_FIELD_ANY;
            if (ioctl(fd_, VIDIOC_S_FMT, &fmtDesc) < 0) {
                fprintf(stderr, "[V4L2] S_FMT failed: %s\n", strerror(errno));
                return false;
            }
            width_  = fmtDesc.fmt.pix.width;
            height_ = fmtDesc.fmt.pix.height;
            stride_ = fmtDesc.fmt.pix.bytesperline;
            if (stride_ == 0) stride_ = width_;
            printf("[V4L2] SINGLE: got %dx%d stride=%d\n",
                   width_, height_, stride_);
        }
        pixelFormat_ = fmt;
        return true;
    }

    bool request_buffers_(int count, bool mplane) {
        v4l2_requestbuffers req = {};
        req.type   = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                            : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.count  = count;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            fprintf(stderr, "[V4L2] REQBUFS(MMAP) failed, trying USERPTR...\n");
            req.memory = V4L2_MEMORY_USERPTR;
            if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
                fprintf(stderr, "[V4L2] REQBUFS(USERPTR) also failed: %s\n",
                        strerror(errno));
                return false;
            }
            return map_buffers_userptr_(req.count, mplane);
        }
        return map_buffers_mmap_(req.count, mplane);
    }

    bool map_buffers_mmap_(int count, bool mplane) {
        for (int i = 0; i < count; ++i) {
            v4l2_buffer buf = {};
            buf.type   = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index  = i;

            // MPLANE: 设置 planes，length = plane 数量
            v4l2_plane qplanes[1] = {};
            if (mplane) {
                buf.m.planes = qplanes;
                buf.length   = 1;
            }
            ioctl(fd_, VIDIOC_QUERYBUF, &buf);
            if (mplane) {
                if (buf.m.planes[0].length == 0) {
                    fprintf(stderr, "[V4L2] MPLANE plane length is 0\n");
                    return false;
                }
                buffers_[i].start = mmap(NULL, buf.m.planes[0].length,
                                          PROT_READ | PROT_WRITE,
                                          MAP_SHARED, fd_,
                                          buf.m.planes[0].m.mem_offset);
                buffers_[i].length = buf.m.planes[0].length;
            } else {
                ioctl(fd_, VIDIOC_QUERYBUF, &buf);
                buffers_[i].start = mmap(NULL, buf.length,
                                          PROT_READ | PROT_WRITE,
                                          MAP_SHARED, fd_, buf.m.offset);
                buffers_[i].length = buf.length;
            }

            if (buffers_[i].start == MAP_FAILED) {
                fprintf(stderr, "[V4L2] mmap[%d] failed: %s\n", i, strerror(errno));
                return false;
            }

            // QBUF
            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[V4L2] QBUF[%d] failed: %s\n", i, strerror(errno));
                return false;
            }
        }
        return true;
    }

    bool map_buffers_userptr_(int count, bool mplane) {
        // USERPTR 模式：自行分配内存
        size_t frameSize = width_ * height_ * 2; // NV12
        for (int i = 0; i < count; ++i) {
            buffers_[i].userPtr = malloc(frameSize);
            buffers_[i].length  = frameSize;
            if (!buffers_[i].userPtr) return false;

            v4l2_buffer buf = {};
            buf.type   = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                                : V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.index  = i;
            buf.m.userptr = (unsigned long)buffers_[i].userPtr;
            buf.length    = frameSize;

            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) {
                fprintf(stderr, "[V4L2] QBUF(USERPTR)[%d] failed: %s\n",
                        i, strerror(errno));
                return false;
            }
            buffers_[i].start = buffers_[i].userPtr;  // read() 从这里读
        }
        return true;
    }
};

// ============================================================================
// 参数解析
// ============================================================================
struct Args {
    std::string device;
    float squareMm = 25.0f;
    int boardW = 9;
    int boardH = 6;
    int minFrames = 20;
    int width = 1920;
    int height = 1080;
    std::string outFile;
    std::string saveDir;  // 非空则保存每帧 PNG 到此目录
};

static void print_usage(const char* prog) {
    fprintf(stderr,
        "用法: %s <device> <square_mm> [options]\n"
        "\n"
        "  <device>      相机设备路径, 例 /dev/video11\n"
        "  <square_mm>   棋盘格每格边长 (mm), 例 25\n"
        "\n"
        "选项:\n"
        "  --boards WxH      内角点数, 默认 9x6\n"
        "  --min-frames N    最少采集帧数 (默认 20)\n"
        "  --resolution WxH  目标分辨率 (默认 1920x1080)\n"
        "  --output FILE     输出文件路径\n"
        "\n"
        "示例:\n"
        "  %s /dev/video11 25\n"
        "  %s /dev/video21 30 --boards 8x6 --min-frames 15\n"
        "\n", prog, prog, prog);
}

static bool parse_args(int argc, char** argv, Args& a) {
    if (argc < 3) { print_usage(argv[0]); return false; }
    a.device    = argv[1];
    a.squareMm  = std::stof(argv[2]);

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--boards" && i + 1 < argc) {
            if (sscanf(argv[++i], "%dx%d", &a.boardW, &a.boardH) != 2) {
                fprintf(stderr, "Invalid --boards format, use WxH\n");
                return false;
            }
        } else if (arg == "--min-frames" && i + 1 < argc) {
            a.minFrames = std::stoi(argv[++i]);
        } else if (arg == "--resolution" && i + 1 < argc) {
            if (sscanf(argv[++i], "%dx%d", &a.width, &a.height) != 2) {
                fprintf(stderr, "Invalid --resolution format\n");
                return false;
            }
        } else if (arg == "--output" && i + 1 < argc) {
            a.outFile = argv[++i];
        } else if (arg == "--save-frames" && i + 1 < argc) {
            a.saveDir = argv[++i];
        } else {
            fprintf(stderr, "Unknown option: %s\n", arg.c_str());
            return false;
        }
    }

    if (a.outFile.empty()) {
        size_t pos = a.device.rfind('/');
        a.outFile = (pos != std::string::npos ? a.device.substr(pos + 1) : a.device)
                    + "_calib.txt";
    }
    if (a.boardW < 3 || a.boardH < 3) {
        fprintf(stderr, "棋盘格内角点至少 3×3\n");
        return false;
    }
    if (a.minFrames < 8) {
        fprintf(stderr, "最少采集 8 帧\n");
        return false;
    }
    return true;
}

// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv) {
    Args a;
    if (!parse_args(argc, argv, a)) return 1;

    signal(SIGINT,  sigint_handler);
    signal(SIGTERM, sigint_handler);

    printf("========================================\n");
    printf("  相机标定工具 (V4L2 原生捕获)\n");
    printf("  Device:    %s\n", a.device.c_str());
    printf("  棋盘格:    %d×%d  边长 %.1f mm\n", a.boardW, a.boardH, a.squareMm);
    printf("  最少帧数:  %d\n", a.minFrames);
    printf("  目标分辨率: %d×%d\n", a.width, a.height);
    printf("  输出文件:  %s\n", a.outFile.c_str());
    printf("========================================\n\n");

    // ---- 打开相机 ----
    V4L2Capture cap;
    if (!cap.open(a.device.c_str(), a.width, a.height)) {
        fprintf(stderr, "[Error] 无法打开 %s\n", a.device.c_str());
        return 1;
    }

    cv::Size imageSize(cap.width(), cap.height());
    cv::Size boardSize(a.boardW, a.boardH);

    // ---- 采集循环 ----
    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    std::vector<cv::Point3f> objTemplate;
    objTemplate.reserve(a.boardW * a.boardH);
    for (int i = 0; i < a.boardH; ++i)
        for (int j = 0; j < a.boardW; ++j)
            objTemplate.push_back(cv::Point3f(j * a.squareMm, i * a.squareMm, 0.0f));

    cv::Mat frame, gray;
    int captured = 0;
    float lastMeanX = -1.0f, lastMeanY = -1.0f;
    int missCount = 0;

    printf("开始采集 — 将棋盘格放到不同位置/角度...\n");
    printf("Ctrl+C 随时停止并进入标定\n\n");

    while (!gStop) {
        if (!cap.read(frame) || frame.empty()) {
            if (++missCount % 30 == 1)
                printf("  丢帧, 重试...\r");
            fflush(stdout);
            continue;
        }

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
            cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) {
            if (++missCount % 30 == 1)
                printf("  搜索棋盘格中... (已捕获 %d/%d)\r", captured, a.minFrames);
            fflush(stdout);
            continue;
        }

        // 亚像素精化
        cv::TermCriteria term(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), term);

        // 去重
        float meanX = 0.0f, meanY = 0.0f;
        for (auto& c : corners) { meanX += c.x; meanY += c.y; }
        meanX /= corners.size();
        meanY /= corners.size();

        if (captured > 0) {
            float dx = meanX - lastMeanX;
            float dy = meanY - lastMeanY;
            float dist = std::sqrt(dx * dx + dy * dy);
            if (dist < 60.0f) {
                printf("  位置变化太小 (%.0f px), 跳过\n", dist);
                continue;
            }
        }

        imagePoints.push_back(corners);
        objectPoints.push_back(objTemplate);
        lastMeanX = meanX;
        lastMeanY = meanY;
        captured++;

        printf("  \033[32m✓ 捕获 %2d/%2d\033[0m  角点中心 (%.0f, %.0f)\n",
               captured, a.minFrames, meanX, meanY);

        // 调试: 保存帧到磁盘
        if (!a.saveDir.empty()) {
            char path[256];
            snprintf(path, sizeof(path), "%s/frame_%02d.png",
                     a.saveDir.c_str(), captured);
            cv::imwrite(path, frame);
            printf("    已保存: %s\n", path);
        }

        if (captured >= a.minFrames) break;
    }

    cap.close();

    if (captured < 8) {
        fprintf(stderr, "\n[Error] 有效帧数不足 (%d < 8), 无法标定\n", captured);
        return 1;
    }

    printf("\n开始标定 (共 %d 帧)...\n", captured);

    // ---- 标定 ----
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs;
    std::vector<cv::Mat> rvecs, tvecs;

    int flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST;
    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    printf("\n");
    printf("========================================\n");
    printf("  标定结果  |  RMS: %.4f px  |  %dx%d\n",
           rms, imageSize.width, imageSize.height);
    printf("========================================\n");
    printf("\n  [原生内参]\n");
    printf("  fx=%.4f  fy=%.4f\n", fx, fy);
    printf("  cx=%.4f  cy=%.4f\n", cx, cy);
    if (distCoeffs.total() >= 4) {
        printf("  k1=%.6f  k2=%.6f\n",
               distCoeffs.at<double>(0), distCoeffs.at<double>(1));
    }

    // ---- 640×640 NPU 内参 ----
    const int NPU_W = 640, NPU_H = 640;
    float scale = std::min((float)NPU_W / imageSize.width,
                            (float)NPU_H / imageSize.height);
    int scaledW = (int)(imageSize.width * scale);
    int scaledH = (int)(imageSize.height * scale);
    int offsetX = (NPU_W - scaledW) / 2;
    int offsetY = (NPU_H - scaledH) / 2;

    float fx640 = fx * scale;
    float fy640 = fy * scale;
    float cx640 = cx * scale + offsetX;
    float cy640 = cy * scale + offsetY;

    printf("\n  [640×640 NPU 内参]\n");
    printf("  letterbox: scale=%.4f offset=(%d,%d)\n", scale, offsetX, offsetY);
    printf("  fx=%.4f  fy=%.4f\n", fx640, fy640);
    printf("  cx=%.4f  cy=%.4f\n", cx640, cy640);

    printf("\n  [config.ini 片段 — 替换 CamX 为 Cam0/Cam1]\n");
    printf("  ----------------------------------------\n");
    printf("  CamXFx=%.4f\n", fx640);
    printf("  CamXFy=%.4f\n", fy640);
    printf("  CamXCx=%.4f\n", cx640);
    printf("  CamXCy=%.4f\n", cy640);
    printf("  CamXImgWidth=%d\n", NPU_W);
    printf("  CamXImgHeight=%d\n", NPU_H);
    printf("========================================\n");

    // ---- 写入文件 ----
    FILE* fout = fopen(a.outFile.c_str(), "w");
    if (fout) {
        fprintf(fout, "# Camera Calibration Result\n");
        fprintf(fout, "# Device: %s\n", a.device.c_str());
        fprintf(fout, "# RMS: %.4f px\n", rms);
        fprintf(fout, "# Native: %dx%d\n", imageSize.width, imageSize.height);
        fprintf(fout, "# Frames: %d\n\n", captured);
        fprintf(fout, "[Native]\n");
        fprintf(fout, "fx=%.4f\nfy=%.4f\n", fx, fy);
        fprintf(fout, "cx=%.4f\ncy=%.4f\n", cx, cy);
        if (distCoeffs.total() >= 4)
            fprintf(fout, "k1=%.6f\nk2=%.6f\n",
                    distCoeffs.at<double>(0), distCoeffs.at<double>(1));
        fprintf(fout, "\n[NPU_640x640]\n");
        fprintf(fout, "fx=%.4f\nfy=%.4f\n", fx640, fy640);
        fprintf(fout, "cx=%.4f\ncy=%.4f\n", cx640, cy640);
        fprintf(fout, "imgWidth=%d\nimgHeight=%d\n", NPU_W, NPU_H);
        fprintf(fout, "\n[config.ini-snippet]\n");
        fprintf(fout, "# 复制到 [Fusion] 节，替换 CamX 为 Cam0 或 Cam1\n");
        fprintf(fout, "CamXFx=%.4f\n", fx640);
        fprintf(fout, "CamXFy=%.4f\n", fy640);
        fprintf(fout, "CamXCx=%.4f\n", cx640);
        fprintf(fout, "CamXCy=%.4f\n", cy640);
        fprintf(fout, "CamXImgWidth=%d\n", NPU_W);
        fprintf(fout, "CamXImgHeight=%d\n", NPU_H);
        fclose(fout);
        printf("\n结果已保存至: %s\n", a.outFile.c_str());
    }

    return 0;
}
