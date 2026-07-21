/**
 * @brief  棋盘格相机标定工具 — 原生 V4L2 捕获 + OpenCV 标定
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
#include <cmath>
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
// 简易 V4L2 捕获类 — 支持 MMAP / MPLANE / USERPTR
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

        v4l2_capability cap = {};
        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0) {
            fprintf(stderr, "[V4L2] VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
            close(); return false;
        }
        mplane_ = cap.capabilities & V4L2_CAP_VIDEO_CAPTURE_MPLANE;
        printf("[V4L2] %s driver=%s %s\n", device, cap.driver,
               mplane_ ? "MPLANE" : "SINGLE_PLANE");

        uint32_t fmt = try_format_(w, h, mplane_);
        if (fmt == 0) { close(); return false; }

        if (!set_format_(w, h, fmt, mplane_)) { close(); return false; }
        if (!request_buffers_(4, mplane_)) { close(); return false; }

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
        printf("[V4L2] Stream started: %dx%d stride=%d fmt=%c%c%c%c\n",
               width_, height_, stride_,
               (char)(pixelFormat_ & 0xFF), (char)((pixelFormat_ >> 8) & 0xFF),
               (char)((pixelFormat_ >> 16) & 0xFF), (char)((pixelFormat_ >> 24) & 0xFF));
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
        if (mplane_) { buf.m.planes = planes; buf.length = 1; }

        if (ioctl(fd_, VIDIOC_DQBUF, &buf) < 0) {
            if (errno == EAGAIN) return false;
            return false;
        }

        void* data = buffers_[buf.index].start;

        if (pixelFormat_ == V4L2_PIX_FMT_NV12 || pixelFormat_ == V4L2_PIX_FMT_NV12M) {
            static int nv12LogCount = 0;
            cv::Mat yuv(height_ * 3 / 2, width_, CV_8UC1);
            uint8_t* dst = yuv.data;
            const uint8_t* src = (const uint8_t*)data;
            // Y 平面：逐行拷贝，按 stride 跳过填充
            for (int r = 0; r < height_; ++r) {
                memcpy(dst, src, width_);
                dst += width_; src += stride_;
            }
            // UV 平面：尝试从 width*height 偏移开始（紧凑布局），
            // 因为 RKISP 可能在 Y 行用 stride 填充但 UV 从紧凑偏移开始
            src = (const uint8_t*)data + (size_t)width_ * height_;
            for (int r = 0; r < height_ / 2; ++r) {
                memcpy(dst, src, width_);
                dst += width_; src += (stride_ > width_ * 2 ? width_ : stride_);
            }
            cv::cvtColor(yuv, frame, cv::COLOR_YUV2BGR_NV12);
            if (++nv12LogCount == 1) {
                printf("[NV12] stride=%d width=%d height=%d uvOffset=%zu\n",
                       stride_, width_, height_, (size_t)width_ * height_);
                cv::Vec3b tl = frame.at<cv::Vec3b>(0, 0);
                cv::Vec3b tr = frame.at<cv::Vec3b>(0, width_ - 1);
                cv::Vec3b bl = frame.at<cv::Vec3b>(height_ - 1, 0);
                cv::Vec3b br = frame.at<cv::Vec3b>(height_ - 1, width_ - 1);
                printf("[NV12] corners BGR: TL(%d,%d,%d) TR(%d,%d,%d) BL(%d,%d,%d) BR(%d,%d,%d)\n",
                       tl[0], tl[1], tl[2], tr[0], tr[1], tr[2],
                       bl[0], bl[1], bl[2], br[0], br[1], br[2]);
            }
        } else if (pixelFormat_ == V4L2_PIX_FMT_YUYV) {
            cv::Mat yuyv(height_, width_, CV_8UC2, data);
            cv::cvtColor(yuyv, frame, cv::COLOR_YUV2BGR_YUYV);
        } else if (pixelFormat_ == V4L2_PIX_FMT_MJPEG) {
            cv::Mat jpeg(1, buf.length, CV_8UC1, data);
            frame = cv::imdecode(jpeg, cv::IMREAD_COLOR);
        } else {
            frame = cv::Mat(height_, width_, CV_8UC3, data).clone();
        }

        ioctl(fd_, VIDIOC_QBUF, &buf);
        return !frame.empty();
    }

    int width()  const { return width_; }
    int height() const { return height_; }

private:
    static const int kMaxBuf = 8;
    struct Buffer { void* start = nullptr; size_t length = 0; void* userPtr = nullptr; };

    int fd_, width_, height_, stride_;
    uint32_t pixelFormat_;
    bool mplane_;
    Buffer buffers_[kMaxBuf];

    uint32_t try_format_(int w, int h, bool mplane) {
        uint32_t fmts[] = { V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV12M,
                            V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_MJPEG, 0 };
        v4l2_fmtdesc fdesc = {};
        fdesc.type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                            : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        while (ioctl(fd_, VIDIOC_ENUM_FMT, &fdesc) == 0) {
            for (int i = 0; fmts[i]; ++i) {
                if (fdesc.pixelformat == fmts[i]) return fmts[i];
            }
            fdesc.index++;
        }
        return 0;
    }

    bool set_format_(int w, int h, uint32_t fmt, bool mplane) {
        if (mplane) {
            v4l2_format f = {};
            f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            ioctl(fd_, VIDIOC_G_FMT, &f);
            f.fmt.pix_mp.width = w;
            f.fmt.pix_mp.height = h;
            f.fmt.pix_mp.pixelformat = fmt;
            f.fmt.pix_mp.num_planes = (fmt == V4L2_PIX_FMT_NV12M) ? 2 : 1;
            if (ioctl(fd_, VIDIOC_S_FMT, &f) < 0) {
                fprintf(stderr, "[V4L2] S_FMT MPLANE failed\n");
                return false;
            }
            width_ = f.fmt.pix_mp.width;
            height_ = f.fmt.pix_mp.height;
            stride_ = f.fmt.pix_mp.plane_fmt[0].bytesperline;
            if (stride_ == 0) stride_ = width_;
            pixelFormat_ = f.fmt.pix_mp.pixelformat;  // 读回实际格式
        } else {
            v4l2_format f = {};
            f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            f.fmt.pix.width = w;
            f.fmt.pix.height = h;
            f.fmt.pix.pixelformat = fmt;
            f.fmt.pix.field = V4L2_FIELD_ANY;
            if (ioctl(fd_, VIDIOC_S_FMT, &f) < 0) {
                fprintf(stderr, "[V4L2] S_FMT failed\n");
                return false;
            }
            width_ = f.fmt.pix.width;
            height_ = f.fmt.pix.height;
            stride_ = f.fmt.pix.bytesperline;
            if (stride_ == 0) stride_ = width_;
            pixelFormat_ = f.fmt.pix.pixelformat;  // 读回实际格式
        }
        return true;
    }

    bool request_buffers_(int count, bool mplane) {
        v4l2_requestbuffers req = {};
        req.type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                          : V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.count = count;
        req.memory = V4L2_MEMORY_MMAP;
        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
            req.memory = V4L2_MEMORY_USERPTR;
            if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0) {
                fprintf(stderr, "[V4L2] REQBUFS failed: %s\n", strerror(errno));
                return false;
            }
            return map_userptr_(req.count, mplane);
        }
        return map_mmap_(req.count, mplane);
    }

    bool map_mmap_(int count, bool mplane) {
        for (int i = 0; i < count; ++i) {
            v4l2_buffer buf = {};
            buf.type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                              : V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            v4l2_plane qp[1] = {};
            if (mplane) { buf.m.planes = qp; buf.length = 1; }
            ioctl(fd_, VIDIOC_QUERYBUF, &buf);
            if (mplane) {
                buffers_[i].start = mmap(NULL, buf.m.planes[0].length,
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED, fd_, buf.m.planes[0].m.mem_offset);
                buffers_[i].length = buf.m.planes[0].length;
            } else {
                ioctl(fd_, VIDIOC_QUERYBUF, &buf);
                buffers_[i].start = mmap(NULL, buf.length,
                                         PROT_READ | PROT_WRITE,
                                         MAP_SHARED, fd_, buf.m.offset);
                buffers_[i].length = buf.length;
            }
            if (buffers_[i].start == MAP_FAILED) return false;
            ioctl(fd_, VIDIOC_QBUF, &buf);
        }
        return true;
    }

    bool map_userptr_(int count, bool mplane) {
        size_t frameSize = width_ * height_ * 2;
        for (int i = 0; i < count; ++i) {
            buffers_[i].userPtr = malloc(frameSize);
            buffers_[i].length = frameSize;
            v4l2_buffer buf = {};
            buf.type = mplane ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE
                              : V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_USERPTR;
            buf.index = i;
            buf.m.userptr = (unsigned long)buffers_[i].userPtr;
            buf.length = frameSize;
            if (ioctl(fd_, VIDIOC_QBUF, &buf) < 0) return false;
            buffers_[i].start = buffers_[i].userPtr;
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
    int boardW = 9, boardH = 6;
    int minFrames = 20;
    int width = 1920, height = 1080;
    std::string outFile;
    std::string saveDir;
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
        "\n", prog);
}

static bool parse_args(int argc, char** argv, Args& a) {
    if (argc < 3) { print_usage(argv[0]); return false; }
    a.device   = argv[1];
    a.squareMm = std::stof(argv[2]);
    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--boards" && i + 1 < argc) {
            if (sscanf(argv[++i], "%dx%d", &a.boardW, &a.boardH) != 2) return false;
        } else if (arg == "--min-frames" && i + 1 < argc) {
            a.minFrames = std::stoi(argv[++i]);
        } else if (arg == "--resolution" && i + 1 < argc) {
            if (sscanf(argv[++i], "%dx%d", &a.width, &a.height) != 2) return false;
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
        a.outFile = (pos != std::string::npos ? a.device.substr(pos + 1) : a.device) + "_calib.txt";
    }
    return true;
}

// ============================================================================
// main
// ============================================================================
int main(int argc, char** argv) {
    Args a;
    if (!parse_args(argc, argv, a)) return 1;

    signal(SIGINT, sigint_handler);
    signal(SIGTERM, sigint_handler);

    printf("========================================\n");
    printf("  相机标定工具 (原生 V4L2 + MPLANE) v4\n");
    printf("  Device:    %s\n", a.device.c_str());
    printf("  棋盘格:    %d×%d  边长 %.1f mm\n", a.boardW, a.boardH, a.squareMm);
    printf("  最少帧数:  %d\n", a.minFrames);
    printf("  分辨率:    %d×%d\n", a.width, a.height);
    printf("  输出文件:  %s\n", a.outFile.c_str());
    printf("========================================\n\n");

    V4L2Capture cap;
    if (!cap.open(a.device.c_str(), a.width, a.height)) {
        fprintf(stderr, "[Error] 无法打开 %s\n", a.device.c_str());
        return 1;
    }

    cv::Size imageSize(cap.width(), cap.height());
    cv::Size boardSize(a.boardW, a.boardH);

    std::vector<std::vector<cv::Point2f>> imagePoints;
    std::vector<std::vector<cv::Point3f>> objectPoints;

    std::vector<cv::Point3f> objTemplate;
    objTemplate.reserve(a.boardW * a.boardH);
    for (int i = 0; i < a.boardH; ++i)
        for (int j = 0; j < a.boardW; ++j)
            objTemplate.push_back(cv::Point3f(j * a.squareMm, i * a.squareMm, 0.0f));

    cv::Mat frame, gray;
    int captured = 0;
    float lastMeanX = -1, lastMeanY = -1;

    printf("自动采集 — 移动棋盘格到不同位置/角度 (Ctrl+C 停止)\n\n");

    int autoCaptureCooldown = 0;
    int missedFrames = 0;

    while (!gStop) {
        if (!cap.read(frame) || frame.empty()) continue;

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(gray, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK |
            cv::CALIB_CB_NORMALIZE_IMAGE);

        if (!found) {
            if (++missedFrames % 100 == 0)
                printf("  搜索棋盘格中... 已捕获 %d/%d\r", captured, a.minFrames);
            fflush(stdout);
            continue;
        }

        cv::TermCriteria term(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.001);
        cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), term);

        // 去重
        float mx = 0, my = 0;
        for (auto& c : corners) { mx += c.x; my += c.y; }
        mx /= corners.size(); my /= corners.size();

        if (captured > 0) {
            float dx = mx - lastMeanX, dy = my - lastMeanY;
            if (std::sqrt(dx * dx + dy * dy) < 40.0f) {
                // 位置没变，冷却递减
                if (autoCaptureCooldown > 0) --autoCaptureCooldown;
                continue;
            }
        }

        // 冷却中
        if (autoCaptureCooldown > 0) { --autoCaptureCooldown; continue; }

        lastMeanX = mx; lastMeanY = my;
        imagePoints.push_back(corners);
        objectPoints.push_back(objTemplate);
        captured++;
        autoCaptureCooldown = 5;

        printf("  \033[32m✓ 捕获 %2d/%2d\033[0m  角点中心 (%.0f, %.0f)\n",
               captured, a.minFrames, mx, my);

        if (!a.saveDir.empty()) {
            char path[256];
            snprintf(path, sizeof(path), "%s/frame_%02d_raw.png",
                     a.saveDir.c_str(), captured);
            cv::imwrite(path, frame);  // 原图
            cv::drawChessboardCorners(frame, boardSize, corners, true);
            snprintf(path, sizeof(path), "%s/frame_%02d_corner.png",
                     a.saveDir.c_str(), captured);
            cv::imwrite(path, frame);  // 带角点标注
            printf("    已保存: frame_%02d_raw.png + frame_%02d_corner.png\n",
                   captured, captured);
        }

        if (captured >= a.minFrames) break;
    }

    cap.close();

    if (captured < 8) {
        fprintf(stderr, "\n[Error] 有效帧数不足 (%d < 8)\n", captured);
        return 1;
    }

    printf("\n开始标定 (共 %d 帧)...\n", captured);

    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs   = cv::Mat::zeros(5, 1, CV_64F);
    std::vector<cv::Mat> rvecs, tvecs;

    // 固定 fx=fy（像素正方形），只用 k1 径向畸变（普通镜头 k1 足够）
    int flags = cv::CALIB_FIX_ASPECT_RATIO | cv::CALIB_ZERO_TANGENT_DIST
              | cv::CALIB_FIX_K2 | cv::CALIB_FIX_K3;

    double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize,
                                      cameraMatrix, distCoeffs, rvecs, tvecs, flags);

    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    printf("\n========================================\n");
    printf("  标定结果  |  RMS: %.4f px  |  %dx%d\n", rms, imageSize.width, imageSize.height);
    printf("========================================\n");
    printf("  [原生内参]\n");
    printf("  fx=%.4f  fy=%.4f\n", fx, fy);
    printf("  cx=%.4f  cy=%.4f\n", cx, cy);
    printf("  k1=%.6f  k2=%.6f  p1=%.6f  p2=%.6f\n",
           distCoeffs.at<double>(0), distCoeffs.at<double>(1),
           distCoeffs.at<double>(2), distCoeffs.at<double>(3));

    // ---- 640×640 NPU 内参 ----
    const int NPU_W = 640, NPU_H = 640;
    float scale = std::min((float)NPU_W / imageSize.width, (float)NPU_H / imageSize.height);
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
        fprintf(fout, "k1=%.6f\nk2=%.6f\n",
                distCoeffs.at<double>(0), distCoeffs.at<double>(1));
        fprintf(fout, "\n[NPU_640x640]\n");
        fprintf(fout, "fx=%.4f\nfy=%.4f\n", fx640, fy640);
        fprintf(fout, "cx=%.4f\ncy=%.4f\n", cx640, cy640);
        fprintf(fout, "imgWidth=%d\nimgHeight=%d\n", NPU_W, NPU_H);
        fprintf(fout, "\n[config.ini-snippet]\n");
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
