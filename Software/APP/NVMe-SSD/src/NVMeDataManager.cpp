#include "NVMeDataManager.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "lodepng.h"

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

namespace {

// On-disk LiDAR point layout matching sentinel-lslidarer's LidarPoint {float x, y, intensity}
struct LidarPointDisk {
    float x;
    float y;
    float intensity;
};

static_assert(sizeof(LidarPointDisk) == 3 * sizeof(float),
              "LidarPointDisk size must match LidarPoint {float x, y, intensity}");

// hue: 0-360, sat: 0-1, light: 0-1  →  RGB 各分量 0-255
void hsl_to_rgb(float h, float s, float l,
                uint8_t& r, uint8_t& g, uint8_t& b) {
    auto hue2rgb = [](float p, float q, float t) -> float {
        if (t < 0.0f) t += 1.0f;
        if (t > 1.0f) t -= 1.0f;
        if (t < 1.0f/6.0f) return p + (q - p) * 6.0f * t;
        if (t < 1.0f/2.0f) return q;
        if (t < 2.0f/3.0f) return p + (q - p) * (2.0f/3.0f - t) * 6.0f;
        return p;
    };

    if (s == 0.0f) {
        auto v = static_cast<uint8_t>(l * 255.0f);
        r = g = b = v;
        return;
    }

    float q = (l < 0.5f) ? l * (1.0f + s) : l + s - l * s;
    float p = 2.0f * l - q;
    float hNorm = h / 360.0f;

    r = static_cast<uint8_t>(hue2rgb(p, q, hNorm + 1.0f/3.0f) * 255.0f);
    g = static_cast<uint8_t>(hue2rgb(p, q, hNorm) * 255.0f);
    b = static_cast<uint8_t>(hue2rgb(p, q, hNorm - 1.0f/3.0f) * 255.0f);
}

} // namespace

NVMeDataManager::NVMeDataManager()
    : running_(false)
    , lidar_buffer_pos_(0)
    , imu_buffer_pos_(0)
    , nvme_fd_(-1)
    , write_buffer_(nullptr)
    , write_buffer_size_(0) {
}

NVMeDataManager::~NVMeDataManager() {
    shutdown();
}

bool NVMeDataManager::initialize(const char* device_path) {
    nvme_device_path_ = device_path;

    // 打开NVMe设备（使用 O_DIRECT 绕过 page cache，降低 CPU 占用）
    nvme_fd_ = open(nvme_device_path_.c_str(), O_WRONLY | O_DIRECT);
    if (nvme_fd_ < 0) {
        std::cerr << "Failed to open NVMe device: " << strerror(errno) << std::endl;
        return false;
    }

    // 设置页大小（通常为4KB）
    long page_size = sysconf(_SC_PAGESIZE);
    if (page_size <= 0) {
        std::cerr << "Failed to get page size" << std::endl;
        close(nvme_fd_);
        nvme_fd_ = -1;
        return false;
    }

    // 分配页对齐的缓冲池
    void* lidar_ptr = nullptr;
    void* imu_ptr = nullptr;

    if (posix_memalign(&lidar_ptr, page_size, BUFFER_SIZE) != 0 ||
        posix_memalign(&imu_ptr, page_size, BUFFER_SIZE) != 0) {
        std::cerr << "Failed to allocate aligned buffers" << std::endl;
        close(nvme_fd_);
        nvme_fd_ = -1;
        return false;
    }

    lidar_buffer_.assign(static_cast<uint8_t*>(lidar_ptr),
                       static_cast<uint8_t*>(lidar_ptr) + BUFFER_SIZE);
    imu_buffer_.assign(static_cast<uint8_t*>(imu_ptr),
                     static_cast<uint8_t*>(imu_ptr) + BUFFER_SIZE);

    // 预分配摄像头帧缓冲池（避免运行时 6MB 反复 malloc/free）
    for (int i = 0; i < CAMERA_POOL_SIZE; i++) {
        camera_pool_[i].data.reserve(CAMERA_MAX_SIZE + HEADER_ALIGNMENT);
        camera_pool_used_[i] = false;
    }

    // 分配页对齐的写入缓冲区（O_DIRECT 要求缓冲区地址512B对齐）
    write_buffer_size_ = CAMERA_MAX_SIZE + sizeof(Header) + HEADER_ALIGNMENT;
    void* wbuf = nullptr;
    if (posix_memalign(&wbuf, HEADER_ALIGNMENT, write_buffer_size_) != 0) {
        std::cerr << "Failed to allocate aligned write buffer" << std::endl;
        close(nvme_fd_);
        nvme_fd_ = -1;
        return false;
    }
    write_buffer_ = static_cast<uint8_t*>(wbuf);

    // 启动写入线程
    running_ = true;
    writer_thread_ = std::thread(&NVMeDataManager::writer_thread, this);

    return true;
}

void NVMeDataManager::shutdown() {
    if (running_) {
        running_ = false;
        queue_cv_.notify_all();
        if (writer_thread_.joinable()) {
            writer_thread_.join();
        }
    }

    // 排空队列，触发自定义 deleter 将池块归还（此时池成员仍有效）
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        while (!data_queue_.empty()) {
            data_queue_.pop();
        }
    }

    if (nvme_fd_ >= 0) {
        close(nvme_fd_);
        nvme_fd_ = -1;
    }

    // 释放缓冲池内存
    lidar_buffer_.clear();
    imu_buffer_.clear();

    // 释放页对齐写入缓冲区
    free(write_buffer_);
    write_buffer_ = nullptr;
    write_buffer_size_ = 0;
}

void NVMeDataManager::prepare_header(Header& header, DataType type, uint64_t timestamp, uint32_t data_size) {
    header.magic_number = MAGIC_NUMBER;
    header.data_type = static_cast<uint8_t>(type);
    header.timestamp_ns = timestamp;
    header.data_size = data_size;
}

bool NVMeDataManager::write_to_nvme(const Header* header, const uint8_t* data,
                                     size_t data_size, size_t padding_size) {
    if (nvme_fd_ < 0) return false;

    size_t total = sizeof(Header) + data_size + padding_size;
    if (total > write_buffer_size_) return false;

    // 将 header + payload + 零填充合并写入页对齐缓冲区（O_DIRECT 要求）
    memcpy(write_buffer_, header, sizeof(Header));
    memcpy(write_buffer_ + sizeof(Header), data, data_size);
    if (padding_size > 0) {
        memset(write_buffer_ + sizeof(Header) + data_size, 0, padding_size);
    }

    ssize_t written = write(nvme_fd_, write_buffer_, total);
    if (written < 0) {
        std::cerr << "write to NVMe failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

void NVMeDataManager::render_heatmap_pixels_(
        const std::vector<LidarPointRecord>& points,
        int imgW, int imgH,
        std::vector<uint8_t>& rgba) {

    if (points.empty()) return;

    // --- 计算渲染范围 ---
    float minX = points[0].x, maxX = points[0].x;
    float minY = points[0].y, maxY = points[0].y;
    uint64_t minTs = points[0].timestamp_ns;
    uint64_t maxTs = points[0].timestamp_ns;

    for (const auto& p : points) {
        if (p.x < minX) minX = p.x;
        if (p.x > maxX) maxX = p.x;
        if (p.y < minY) minY = p.y;
        if (p.y > maxY) maxY = p.y;
        if (p.timestamp_ns < minTs) minTs = p.timestamp_ns;
        if (p.timestamp_ns > maxTs) maxTs = p.timestamp_ns;
    }

    // 10% 边距 + 最小范围 ±10m
    float marginX = std::max((maxX - minX) * 0.1f, 0.5f);
    float marginY = std::max((maxY - minY) * 0.1f, 0.5f);
    float viewMinX = minX - marginX;
    float viewMaxX = maxX + marginX;
    float viewMinY = minY - marginY;
    float viewMaxY = maxY + marginY;

    // 确保正方形 + 至少 ±10m
    float cx = (viewMinX + viewMaxX) * 0.5f;
    float cy = (viewMinY + viewMaxY) * 0.5f;
    float half = std::max({viewMaxX - cx, viewMaxY - cy, 10.0f});
    viewMinX = cx - half;
    viewMaxX = cx + half;
    viewMinY = cy - half;
    viewMaxY = cy + half;

    float scaleX = imgW / (viewMaxX - viewMinX);
    float scaleY = imgH / (viewMaxY - viewMinY);
    float scale  = std::min(scaleX, scaleY);

    auto world_to_pixel = [&](float wx, float wy, int& px, int& py) {
        px = static_cast<int>((wx - cx) * scale + imgW / 2.0f);
        py = static_cast<int>((cy - wy) * scale + imgH / 2.0f);
    };

    // --- 黑色背景 ---
    std::fill(rgba.begin(), rgba.end(), 0);

    // --- 网格线 (10m 间距，深灰 #202020) ---
    {
        int gridStepM = 10;

        for (int gm = static_cast<int>(std::floor(viewMinX / gridStepM)) * gridStepM;
             gm <= static_cast<int>(std::ceil(viewMaxX / gridStepM)) * gridStepM;
             gm += gridStepM) {
            int px, py;
            world_to_pixel(static_cast<float>(gm), 0.0f, px, py);
            if (px >= 0 && px < imgW) {
                for (int y = 0; y < imgH; y += 4) {  // 虚线
                    size_t idx = (static_cast<size_t>(y) * imgW + px) * 4;
                    rgba[idx] = 0x20; rgba[idx+1] = 0x20; rgba[idx+2] = 0x20; rgba[idx+3] = 255;
                }
            }
        }

        for (int gm = static_cast<int>(std::floor(viewMinY / gridStepM)) * gridStepM;
             gm <= static_cast<int>(std::ceil(viewMaxY / gridStepM)) * gridStepM;
             gm += gridStepM) {
            int px, py;
            world_to_pixel(0.0f, static_cast<float>(gm), px, py);
            if (py >= 0 && py < imgH) {
                for (int x = 0; x < imgW; x += 4) {  // 虚线
                    size_t idx = (static_cast<size_t>(py) * imgW + x) * 4;
                    rgba[idx] = 0x20; rgba[idx+1] = 0x20; rgba[idx+2] = 0x20; rgba[idx+3] = 255;
                }
            }
        }
    }

    // --- 传感器原点十字 (白色) ---
    {
        int ox, oy;
        world_to_pixel(0.0f, 0.0f, ox, oy);
        for (int dx = -10; dx <= 10; ++dx) {
            int px = ox + dx;
            if (px >= 0 && px < imgW && oy >= 0 && oy < imgH) {
                size_t idx = (static_cast<size_t>(oy) * imgW + px) * 4;
                rgba[idx] = rgba[idx+1] = rgba[idx+2] = 255; rgba[idx+3] = 255;
            }
        }
        for (int dy = -10; dy <= 10; ++dy) {
            int py = oy + dy;
            if (ox >= 0 && ox < imgW && py >= 0 && py < imgH) {
                size_t idx = (static_cast<size_t>(py) * imgW + ox) * 4;
                rgba[idx] = rgba[idx+1] = rgba[idx+2] = 255; rgba[idx+3] = 255;
            }
        }
    }

    // --- 渲染点：时间→色相 (240°→0°: 蓝→青→绿→黄→红)，alpha 叠加 ---
    float tsRange = (maxTs > minTs) ? static_cast<float>(maxTs - minTs) : 1.0f;
    int pointRadius = 2;

    for (const auto& pt : points) {
        float t = static_cast<float>(pt.timestamp_ns - minTs) / tsRange;
        float hue = 240.0f * (1.0f - t);  // 240°(蓝) → 0°(红)

        uint8_t cr, cg, cb;
        hsl_to_rgb(hue, 1.0f, 0.5f, cr, cg, cb);

        int cxPx, cyPx;
        world_to_pixel(pt.x, pt.y, cxPx, cyPx);

        for (int dy = -pointRadius; dy <= pointRadius; ++dy) {
            for (int dx = -pointRadius; dx <= pointRadius; ++dx) {
                if (dx*dx + dy*dy > pointRadius*pointRadius) continue;
                int px = cxPx + dx;
                int py = cyPx + dy;
                if (px < 0 || px >= imgW || py < 0 || py >= imgH) continue;
                size_t idx = (static_cast<size_t>(py) * imgW + px) * 4;
                // Alpha 叠加：新颜色与现有颜色混合
                uint8_t& er = rgba[idx];
                uint8_t& eg = rgba[idx+1];
                uint8_t& eb = rgba[idx+2];
                uint8_t& ea = rgba[idx+3];
                float alpha = 0.3f;  // 每点透明度，累积形成热点
                er = static_cast<uint8_t>(er * (1.0f - alpha) + cr * alpha);
                eg = static_cast<uint8_t>(eg * (1.0f - alpha) + cg * alpha);
                eb = static_cast<uint8_t>(eb * (1.0f - alpha) + cb * alpha);
                ea = 255;
            }
        }
    }
}

bool NVMeDataManager::export_lidar_heatmap_png(uint64_t trigger_timestamp_ns,
                                                const std::string& output_path,
                                                double time_window_sec) {
    uint64_t window_ns = static_cast<uint64_t>(time_window_sec * 1'000'000'000.0);
    uint64_t start_ns = (trigger_timestamp_ns > window_ns)
                      ? (trigger_timestamp_ns - window_ns) : 0;
    uint64_t end_ns = trigger_timestamp_ns;

    // 0. 将内存缓冲区中的残留雷达数据刷入磁盘
    flush();

    // 1. 扫描 NVMe 收集 LiDAR 记录
    int read_fd = open(nvme_device_path_.c_str(), O_RDONLY);
    if (read_fd < 0) {
        fprintf(stderr, "[NVMeDataManager] heatmap: open failed: %s\n", strerror(errno));
        return false;
    }

    std::vector<LidarPointRecord> allPoints;
    off_t offset = 0;
    Header header;

    while (true) {
        ssize_t n = pread(read_fd, &header, sizeof(Header), offset);
        if (n != static_cast<ssize_t>(sizeof(Header))) break;
        if (header.magic_number != MAGIC_NUMBER) break;

        size_t record_payload = sizeof(Header) + header.data_size;
        size_t padding = (HEADER_ALIGNMENT - (record_payload % HEADER_ALIGNMENT)) % HEADER_ALIGNMENT;
        size_t record_size = record_payload + padding;

        if (header.data_type == static_cast<uint8_t>(DataType::LIDAR) &&
            // flush() 块 header ts=0，允许通过；每帧的真实 ts 在帧头中
            (header.timestamp_ns == 0 ||
             (header.timestamp_ns >= start_ns && header.timestamp_ns <= end_ns))) {

            std::vector<uint8_t> buf(header.data_size);
            n = pread(read_fd, buf.data(), header.data_size, offset + sizeof(Header));
            if (n == static_cast<ssize_t>(header.data_size)) {
                // 解析帧头: [pointsCount:u32][timestampNs:u64][LidarPointDisk * N] ...
                size_t pos = 0;
                while (pos + 12 <= static_cast<size_t>(header.data_size)) {
                    uint32_t frameCount;
                    uint64_t frameTs;
                    std::memcpy(&frameCount, buf.data() + pos, sizeof(frameCount));
                    std::memcpy(&frameTs, buf.data() + pos + sizeof(frameCount),
                               sizeof(frameTs));
                    pos += 12;  // 帧头: 4 + 8

                    size_t framePointsSize = static_cast<size_t>(frameCount) * sizeof(LidarPointDisk);
                    if (frameCount == 0 || pos + framePointsSize > static_cast<size_t>(header.data_size)) {
                        break;  // 帧头损坏或结束
                    }

                    // 时间窗口过滤
                    if (frameTs >= start_ns && frameTs <= end_ns) {
                        const LidarPointDisk* pts =
                            reinterpret_cast<const LidarPointDisk*>(buf.data() + pos);
                        for (uint32_t i = 0; i < frameCount; ++i) {
                            LidarPointRecord r;
                            r.x = pts[i].x;
                            r.y = pts[i].y;
                            r.timestamp_ns = frameTs;
                            allPoints.push_back(r);
                        }
                    }

                    pos += framePointsSize;
                }
            }
        }

        offset += static_cast<off_t>(record_size);
    }

    close(read_fd);

    if (allPoints.empty()) {
        fprintf(stderr, "[NVMeDataManager] heatmap: no LiDAR points in window\n");
        return false;
    }

    // 2. 渲染 RGBA 像素缓冲
    const int kImgSize = 1200;
    std::vector<uint8_t> rgba(static_cast<size_t>(kImgSize) * kImgSize * 4, 0);
    render_heatmap_pixels_(allPoints, kImgSize, kImgSize, rgba);

    // 3. 编码 PNG 并写盘
    unsigned error = lodepng::encode(output_path, rgba, kImgSize, kImgSize);
    if (error) {
        fprintf(stderr, "[NVMeDataManager] heatmap: lodepng encode error: %s\n",
                lodepng_error_text(error));
        return false;
    }

    size_t frameCount = 0;
    if (!allPoints.empty()) {
        uint64_t lastTs = allPoints[0].timestamp_ns;
        frameCount = 1;
        for (const auto& p : allPoints) {
            if (p.timestamp_ns != lastTs) {
                frameCount++;
                lastTs = p.timestamp_ns;
            }
        }
    }

    fprintf(stderr, "[NVMeDataManager] heatmap saved: %s (%zu points, %zu frames)\n",
            output_path.c_str(), allPoints.size(), frameCount);
    return true;
}

DataBlock* NVMeDataManager::acquire_pool_block() {
    std::lock_guard<std::mutex> lock(camera_pool_mutex_);
    for (int i = 0; i < CAMERA_POOL_SIZE; i++) {
        if (!camera_pool_used_[i]) {
            camera_pool_used_[i] = true;
            return &camera_pool_[i];
        }
    }
    return nullptr;  // 池耗尽
}

void NVMeDataManager::release_pool_block(DataBlock* block) {
    std::lock_guard<std::mutex> lock(camera_pool_mutex_);
    for (int i = 0; i < CAMERA_POOL_SIZE; i++) {
        if (&camera_pool_[i] == block) {
            camera_pool_used_[i] = false;
            return;
        }
    }
    // 非池块（回退分配），不做任何事
}

bool NVMeDataManager::write_video_frame_to_disk(const uint8_t* frame_data, size_t frame_size,
                                           uint64_t timestamp_ns, bool is_front_camera) {
    // 计算对齐填充（无需加锁）
    size_t padding = calc_padding(sizeof(Header) + frame_size);

    std::shared_ptr<DataBlock> block;

    // 优先从缓冲池取（避免运行时分配 6MB 堆内存）
    DataBlock* pool_block = acquire_pool_block();
    if (pool_block) {
        pool_block->data.assign(frame_data, frame_data + frame_size);
        prepare_header(pool_block->header,
                      is_front_camera ? DataType::VIDEO_FRONT : DataType::VIDEO_REAR,
                      timestamp_ns, static_cast<uint32_t>(frame_size));
        pool_block->padding_size = padding;

        // 自定义 deleter：将池块归还
        block = std::shared_ptr<DataBlock>(pool_block, [this](DataBlock* b) {
            this->release_pool_block(b);
        });
    } else {
        // 池耗尽，回退到普通分配（极少发生）
        auto fb = std::make_shared<DataBlock>();
        prepare_header(fb->header,
                      is_front_camera ? DataType::VIDEO_FRONT : DataType::VIDEO_REAR,
                      timestamp_ns, static_cast<uint32_t>(frame_size));
        fb->data.assign(frame_data, frame_data + frame_size);
        fb->padding_size = padding;
        block = std::move(fb);
    }

    // 仅入队时持锁，减小锁粒度
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        data_queue_.push(std::move(block));
    }
    queue_cv_.notify_one();

    return true;
}

bool NVMeDataManager::write_lidar_points_to_disk(const uint8_t* points_data, size_t points_size,
                                          uint64_t timestamp_ns) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 每帧前加 12 字节帧头: [pointsCount:u32][timestampNs:u64]
    static constexpr size_t kFrameHeaderSize = sizeof(uint32_t) + sizeof(uint64_t);
    size_t totalFrameSize = kFrameHeaderSize + points_size;

    // 检查缓冲池空间（含帧头）
    if (lidar_buffer_pos_ + totalFrameSize > lidar_buffer_.size()) {
        // 缓冲池不够放当前帧，先刷已有数据到队列
        if (lidar_buffer_pos_ > 0) {
            auto block = std::make_shared<DataBlock>();
            prepare_header(block->header, DataType::LIDAR, timestamp_ns,
                          static_cast<uint32_t>(lidar_buffer_pos_));
            block->data.assign(lidar_buffer_.data(),
                              lidar_buffer_.data() + lidar_buffer_pos_);
            block->padding_size = calc_padding(sizeof(Header) + lidar_buffer_pos_);

            data_queue_.push(std::move(block));
            queue_cv_.notify_one();

            // 重置缓冲池位置
            lidar_buffer_pos_ = 0;
        }

        // 单帧超过整个缓冲池大小则拒绝
        if (totalFrameSize > lidar_buffer_.size()) {
            std::cerr << "Lidar frame too large for buffer: " << totalFrameSize << " bytes"
                      << std::endl;
            return false;
        }
    }

    // 写入帧头 + 点数据
    uint32_t count = static_cast<uint32_t>(points_size / sizeof(LidarPointDisk));
    std::memcpy(lidar_buffer_.data() + lidar_buffer_pos_, &count, sizeof(count));
    std::memcpy(lidar_buffer_.data() + lidar_buffer_pos_ + sizeof(count),
               &timestamp_ns, sizeof(timestamp_ns));
    std::memcpy(lidar_buffer_.data() + lidar_buffer_pos_ + kFrameHeaderSize,
               points_data, points_size);
    lidar_buffer_pos_ += totalFrameSize;
    return true;
}

bool NVMeDataManager::write_imu_data_to_disk(const uint8_t* imu_data, size_t imu_size,
                                       uint64_t timestamp_ns) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 检查缓冲池空间
    if (imu_buffer_pos_ + imu_size > imu_buffer_.size()) {
        // 缓冲池满了，需要刷入队列
        if (!imu_buffer_pos_) {
            std::cerr << "IMU buffer is empty but still full, possible logic error" << std::endl;
            return false;
        }

        // 直接创建 DataBlock（一次拷贝，无包头拼接）
        auto block = std::make_shared<DataBlock>();
        prepare_header(block->header, DataType::IMU, timestamp_ns,
                      static_cast<uint32_t>(imu_buffer_pos_));
        block->data.assign(imu_buffer_.data(),
                          imu_buffer_.data() + imu_buffer_pos_);
        block->padding_size = calc_padding(sizeof(Header) + imu_buffer_pos_);

        data_queue_.push(std::move(block));
        queue_cv_.notify_one();

        // 重置缓冲池位置
        imu_buffer_pos_ = 0;
    }

    // 写入新数据到缓冲池
    if (imu_buffer_pos_ + imu_size <= imu_buffer_.size()) {
        std::memcpy(imu_buffer_.data() + imu_buffer_pos_, imu_data, imu_size);
        imu_buffer_pos_ += imu_size;
        return true;
    } else {
        std::cerr << "IMU data too large for buffer" << std::endl;
        return false;
    }
}

bool NVMeDataManager::read_video_frame_from_disk(uint64_t target_timestamp, float time_interval,
                                                  std::vector<uint8_t>& out_frame_data) {
    // 打开NVMe设备用于读取
    int read_fd = open(nvme_device_path_.c_str(), O_RDONLY);
    if (read_fd < 0) {
        std::cerr << "Failed to open NVMe device for reading: " << strerror(errno) << std::endl;
        return false;
    }

    off_t offset = 0;
    Header header;
    bool found = false;

    // 从起始位置依次扫描每条记录
    while (true) {
        // 读取包头
        ssize_t n = pread(read_fd, &header, sizeof(Header), offset);
        if (n != (ssize_t)sizeof(Header)) {
            break;  // 到达有效数据末尾或读取错误
        }

        // 检查魔数，确认是有效记录
        if (header.magic_number != MAGIC_NUMBER) {
            break;  // 非有效记录，停止扫描
        }

        // 计算本记录的总大小（512字节对齐）
        size_t record_payload = sizeof(Header) + header.data_size;
        size_t padding = (HEADER_ALIGNMENT - (record_payload % HEADER_ALIGNMENT)) % HEADER_ALIGNMENT;
        size_t record_size = record_payload + padding;

        // 检查是否是视频帧（前视或后视）
        bool is_video = (header.data_type == static_cast<uint8_t>(DataType::VIDEO_FRONT) ||
                        header.data_type == static_cast<uint8_t>(DataType::VIDEO_REAR));

        if (is_video) {
            // 计算时间差（绝对值）
            uint64_t time_diff = (target_timestamp > header.timestamp_ns)
                ? (target_timestamp - header.timestamp_ns)
                : (header.timestamp_ns - target_timestamp);

            uint64_t interval_ns = static_cast<uint64_t>(time_interval * 1000000000.0);

            if (time_diff <= interval_ns) {
                // 找到匹配的帧，读取帧数据
                out_frame_data.resize(header.data_size);
                n = pread(read_fd, out_frame_data.data(), header.data_size,
                         offset + sizeof(Header));
                if (n == (ssize_t)header.data_size) {
                    found = true;
                } else {
                    std::cerr << "Failed to read frame data at offset "
                              << (offset + sizeof(Header)) << std::endl;
                }
                break;  // 找到即停止（返回第一个匹配帧）
            }
        }

        // 移动到下一条记录
        offset += record_size;
    }

    close(read_fd);

    if (found) {
        std::cout << "Read video frame: type=" << (int)header.data_type
                  << ", size=" << header.data_size
                  << ", timestamp=" << header.timestamp_ns << std::endl;
    }

    return found;
}

bool NVMeDataManager::export_trigger_video_clip(uint64_t trigger_timestamp_ns,
                                                const std::string& output_path,
                                                double time_window_sec,
                                                int fps,
                                                int frame_width,
                                                int frame_height,
                                                int camera_id,
                                                bool input_is_nv12) {
    // 计算时间窗口（纳秒）：仅回溯 time_window_sec 秒
    uint64_t window_ns = static_cast<uint64_t>(time_window_sec * 1'000'000'000.0);
    uint64_t start_ns = (trigger_timestamp_ns > window_ns)
                      ? (trigger_timestamp_ns - window_ns) : 0;
    uint64_t end_ns = trigger_timestamp_ns;  // 不包含未来数据
    const size_t expected_frame_size = input_is_nv12
        ? static_cast<size_t>(frame_width) * frame_height * 3 / 2
        : static_cast<size_t>(frame_width) * frame_height * 3;

    // 打开 NVMe 设备用于读取
    int read_fd = open(nvme_device_path_.c_str(), O_RDONLY);
    if (read_fd < 0) {
        std::cerr << "Failed to open NVMe device for reading: " << strerror(errno) << std::endl;
        return false;
    }

    // 收集时间窗口内的视频帧
    struct FrameRecord {
        uint64_t timestamp_ns;
        std::vector<uint8_t> data;
    };
    std::vector<FrameRecord> frames;

    off_t offset = 0;
    Header header;

    while (true) {
        ssize_t n = pread(read_fd, &header, sizeof(Header), offset);
        if (n != static_cast<ssize_t>(sizeof(Header))) break;
        if (header.magic_number != MAGIC_NUMBER) break;

        size_t record_payload = sizeof(Header) + header.data_size;
        size_t padding = (HEADER_ALIGNMENT - (record_payload % HEADER_ALIGNMENT)) % HEADER_ALIGNMENT;
        size_t record_size = record_payload + padding;

        bool is_video = false;
        if (camera_id == -1) {
            // -1: 匹配所有视频类型（支持后续扩展更多摄像头）
            is_video = (header.data_type == static_cast<uint8_t>(DataType::VIDEO_FRONT) ||
                        header.data_type == static_cast<uint8_t>(DataType::VIDEO_REAR));
        } else {
            // 精确匹配指定摄像头ID（data_type 即摄像头序列号）
            is_video = (header.data_type == static_cast<uint8_t>(camera_id));
        }

        if (is_video && header.timestamp_ns >= start_ns && header.timestamp_ns <= end_ns) {
            if (header.data_size == expected_frame_size) {
                FrameRecord fr;
                fr.timestamp_ns = header.timestamp_ns;
                fr.data.resize(header.data_size);
                n = pread(read_fd, fr.data.data(), header.data_size, offset + sizeof(Header));
                if (n == static_cast<ssize_t>(header.data_size)) {
                    frames.push_back(std::move(fr));
                }
            }
        }

        offset += record_size;
    }

    close(read_fd);

    if (frames.empty()) {
        std::cerr << "No video frames found in time window ["
                  << start_ns << ", " << end_ns << "]" << std::endl;
        return false;
    }

    // 按时间戳排序保证播放顺序
    std::sort(frames.begin(), frames.end(), [](const FrameRecord& a, const FrameRecord& b) {
        return a.timestamp_ns < b.timestamp_ns;
    });

    // ================================================================
    //  MPP 硬件编码 + FFmpeg MP4 复用（参照 sentinel-streamer/mpp_encoder）
    //  数据流: RGB888(内存) → swscale → NV12 → h264_rkmpp → MP4
    // ================================================================

    // Step A: 打开 MPP H.264 硬件编码器
    const AVCodec* codec = avcodec_find_encoder_by_name("h264_rkmpp");
    if (!codec) {
        std::cerr << "[NVMeExport] h264_rkmpp encoder not found" << std::endl;
        return false;
    }

    AVCodecContext* encCtx = avcodec_alloc_context3(codec);
    if (!encCtx) {
        std::cerr << "[NVMeExport] avcodec_alloc_context3 failed" << std::endl;
        return false;
    }

    encCtx->width       = frame_width;
    encCtx->height      = frame_height;
    encCtx->time_base   = AVRational{1, 90000};
    encCtx->framerate   = AVRational{fps, 1};
    encCtx->gop_size    = fps;
    encCtx->bit_rate    = 8000000;   // 8 Mbps, 1080p 回溯录像质量
    encCtx->max_b_frames = 0;
    encCtx->pix_fmt     = AV_PIX_FMT_NV12;

    encCtx->color_range     = AVCOL_RANGE_JPEG;
    encCtx->color_primaries = AVCOL_PRI_BT709;
    encCtx->color_trc       = AVCOL_TRC_BT709;
    encCtx->colorspace      = AVCOL_SPC_BT709;

    av_opt_set_int(encCtx->priv_data, "rc_mode", 0, 0);  // VBR
    av_opt_set_int(encCtx->priv_data, "delay",  0, 0);

    if (avcodec_open2(encCtx, codec, nullptr) < 0) {
        std::cerr << "[NVMeExport] avcodec_open2 failed" << std::endl;
        avcodec_free_context(&encCtx);
        return false;
    }

    // Step B: 打开 MP4 复用器
    AVFormatContext* fmtCtx = nullptr;
    if (avformat_alloc_output_context2(&fmtCtx, nullptr, "mp4",
                                       output_path.c_str()) < 0 || !fmtCtx) {
        std::cerr << "[NVMeExport] avformat_alloc_output_context2 failed" << std::endl;
        avcodec_free_context(&encCtx);
        return false;
    }

    AVStream* stream = avformat_new_stream(fmtCtx, nullptr);
    if (!stream) {
        std::cerr << "[NVMeExport] avformat_new_stream failed" << std::endl;
        avformat_free_context(fmtCtx);
        avcodec_free_context(&encCtx);
        return false;
    }

    if (avcodec_parameters_from_context(stream->codecpar, encCtx) < 0) {
        std::cerr << "[NVMeExport] avcodec_parameters_from_context failed" << std::endl;
        avformat_free_context(fmtCtx);
        avcodec_free_context(&encCtx);
        return false;
    }
    stream->time_base = encCtx->time_base;

    if (avio_open(&fmtCtx->pb, output_path.c_str(), AVIO_FLAG_WRITE) < 0) {
        std::cerr << "[NVMeExport] avio_open failed: " << output_path << std::endl;
        avformat_free_context(fmtCtx);
        avcodec_free_context(&encCtx);
        return false;
    }

    if (avformat_write_header(fmtCtx, nullptr) < 0) {
        std::cerr << "[NVMeExport] avformat_write_header failed" << std::endl;
        avio_closep(&fmtCtx->pb);
        avformat_free_context(fmtCtx);
        avcodec_free_context(&encCtx);
        return false;
    }

    // Step C: 创建 swscale 上下文 (仅 RGB888→NV12 时需要)
    SwsContext* swsCtx = nullptr;
    if (!input_is_nv12) {
        swsCtx = sws_getContext(
            frame_width, frame_height, AV_PIX_FMT_RGB24,
            frame_width, frame_height, AV_PIX_FMT_NV12,
            SWS_BILINEAR, nullptr, nullptr, nullptr);
        if (!swsCtx) {
            std::cerr << "[NVMeExport] sws_getContext failed" << std::endl;
            av_write_trailer(fmtCtx);
            avio_closep(&fmtCtx->pb);
            avformat_free_context(fmtCtx);
            avcodec_free_context(&encCtx);
            return false;
        }
    }

    // Step D: 逐帧 → NV12 → MPP 编码 → MP4 写盘
    uint64_t baseNs = frames.empty() ? 0 : frames[0].timestamp_ns;
    int y_size = frame_width * frame_height;

    for (const auto& fr : frames) {
        // D1: 创建 NV12 AVFrame
        AVFrame* nv12 = av_frame_alloc();
        if (!nv12) continue;
        nv12->format = AV_PIX_FMT_NV12;
        nv12->width  = frame_width;
        nv12->height = frame_height;
        nv12->pts    = static_cast<int64_t>((fr.timestamp_ns - baseNs) * 90000 / 1000'000'000);
        av_frame_get_buffer(nv12, 0);

        // D2: 填充 NV12 数据
        if (input_is_nv12) {
            memcpy(nv12->data[0], fr.data.data(), y_size);
            memcpy(nv12->data[1], fr.data.data() + y_size, y_size / 2);
        } else {
            const uint8_t* srcData[1] = { fr.data.data() };
            int srcStride[1] = { frame_width * 3 };
            sws_scale(swsCtx, srcData, srcStride, 0, frame_height,
                      nv12->data, nv12->linesize);
        }

        // D3: 送编码器
        avcodec_send_frame(encCtx, nv12);
        av_frame_free(&nv12);

        // D4: 收编码包 → 写 MP4
        AVPacket* pkt = av_packet_alloc();
        if (pkt) {
            while (avcodec_receive_packet(encCtx, pkt) == 0) {
                pkt->stream_index = 0;
                av_write_frame(fmtCtx, pkt);
                av_packet_unref(pkt);
            }
            av_packet_free(&pkt);
        }
    }

    // Step E: 排空编码器残余帧
    avcodec_send_frame(encCtx, nullptr);
    AVPacket* drainPkt = av_packet_alloc();
    if (drainPkt) {
        while (avcodec_receive_packet(encCtx, drainPkt) == 0) {
            drainPkt->stream_index = 0;
            av_write_frame(fmtCtx, drainPkt);
            av_packet_unref(drainPkt);
        }
        av_packet_free(&drainPkt);
    }

    // Step F: 清理
    if (swsCtx) sws_freeContext(swsCtx);
    av_write_trailer(fmtCtx);
    avio_closep(&fmtCtx->pb);
    avformat_free_context(fmtCtx);
    avcodec_free_context(&encCtx);

    std::cout << "Exported " << frames.size() << " frames ("
              << (static_cast<double>(frames.size()) / fps) << "s) to "
              << output_path << std::endl;

    return true;
}

size_t NVMeDataManager::get_queue_size() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return data_queue_.size();
}

size_t NVMeDataManager::get_buffer_usage() const {
    return lidar_buffer_pos_ + imu_buffer_pos_;
}

void NVMeDataManager::flush() {
    // 将残留的 LiDAR/IMU 缓冲区数据推入队列
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);

        if (lidar_buffer_pos_ > 0) {
            auto block = std::make_shared<DataBlock>();
            // 记录头时间戳用 0（语义：flush 导出，非真实帧时间戳）
            prepare_header(block->header, DataType::LIDAR, 0,
                          static_cast<uint32_t>(lidar_buffer_pos_));
            block->data.assign(lidar_buffer_.data(),
                              lidar_buffer_.data() + lidar_buffer_pos_);
            block->padding_size = calc_padding(sizeof(Header) + lidar_buffer_pos_);
            data_queue_.push(std::move(block));
            lidar_buffer_pos_ = 0;
        }

        if (imu_buffer_pos_ > 0) {
            auto block = std::make_shared<DataBlock>();
            prepare_header(block->header, DataType::IMU, 0,
                          static_cast<uint32_t>(imu_buffer_pos_));
            block->data.assign(imu_buffer_.data(),
                              imu_buffer_.data() + imu_buffer_pos_);
            block->padding_size = calc_padding(sizeof(Header) + imu_buffer_pos_);
            data_queue_.push(std::move(block));
            imu_buffer_pos_ = 0;
        }
    }
    queue_cv_.notify_one();

    // 等待 writer 线程将队列排空（数据安全落盘后返回）
    while (true) {
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            if (data_queue_.empty()) break;
        }
        usleep(1000);  // 1ms
    }
}

void NVMeDataManager::writer_thread() {
    while (running_) {
        std::shared_ptr<DataBlock> block;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);

            // 等待队列中有数据或关闭信号
            queue_cv_.wait(lock, [this] {
                return !data_queue_.empty() || !running_;
            });

            if (!running_) break;
            if (data_queue_.empty()) continue;

            // 出队（仅移动 shared_ptr，不拷贝数据）
            block = std::move(data_queue_.front());
            data_queue_.pop();
        }
        // IO 期间不持锁

        // 通过 O_DIRECT 页对齐缓冲区写入：header + payload + zero-padding
        if (!write_to_nvme(&block->header,
                          block->data.data(),
                          block->data.size(),
                          block->padding_size)) {
            std::cerr << "Writer thread: writev failed for "
                      << block->data.size() << " bytes" << std::endl;
        }
    }
}