#include "NVMeDataManager.h"
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <stdexcept>
#include <chrono>
#include <iostream>

NVMeDataManager::NVMeDataManager()
    : running_(false)
    , lidar_buffer_pos_(0)
    , imu_buffer_pos_(0)
    , nvme_fd_(-1) {
}

NVMeDataManager::~NVMeDataManager() {
    shutdown();
}

bool NVMeDataManager::initialize() {
    // 打开NVMe设备（不使用O_DIRECT以避免缓冲区对齐要求）
    nvme_fd_ = open("/dev/nvme0n1", O_WRONLY);
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

    if (nvme_fd_ >= 0) {
        close(nvme_fd_);
        nvme_fd_ = -1;
    }

    // 释放缓冲池内存
    lidar_buffer_.clear();
    imu_buffer_.clear();
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

    // 静态512字节零填充缓冲区（writev 需要非 const iov_base）
    static uint8_t zero_buf[HEADER_ALIGNMENT] = {0};

    struct iovec iov[3];
    iov[0].iov_base = const_cast<Header*>(header);
    iov[0].iov_len  = sizeof(Header);
    iov[1].iov_base = const_cast<uint8_t*>(data);
    iov[1].iov_len  = data_size;
    iov[2].iov_base = zero_buf;
    iov[2].iov_len  = padding_size;

    ssize_t written = writev(nvme_fd_, iov, 3);
    if (written < 0) {
        std::cerr << "writev to NVMe failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool NVMeDataManager::write_video_frame_to_disk(const uint8_t* frame_data, size_t frame_size,
                                           uint64_t timestamp_ns, bool is_front_camera) {
    // 计算对齐填充（无需加锁）
    size_t padding = calc_padding(sizeof(Header) + frame_size);

    // 直接创建 DataBlock（只拷贝一次数据，无包头拼接）
    auto block = std::make_shared<DataBlock>();
    prepare_header(block->header,
                  is_front_camera ? DataType::VIDEO_FRONT : DataType::VIDEO_REAR,
                  timestamp_ns, static_cast<uint32_t>(frame_size));
    block->data.assign(frame_data, frame_data + frame_size);
    block->padding_size = padding;

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

    // 检查缓冲池空间
    if (lidar_buffer_pos_ + points_size > lidar_buffer_.size()) {
        // 缓冲池满了，需要刷入队列
        if (!lidar_buffer_pos_) {
            std::cerr << "Lidar buffer is empty but still full, possible logic error" << std::endl;
            return false;
        }

        // 直接创建 DataBlock（一次拷贝，无包头拼接）
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

    // 写入新数据到缓冲池
    if (lidar_buffer_pos_ + points_size <= lidar_buffer_.size()) {
        std::memcpy(lidar_buffer_.data() + lidar_buffer_pos_, points_data, points_size);
        lidar_buffer_pos_ += points_size;
        return true;
    } else {
        std::cerr << "Lidar data too large for buffer" << std::endl;
        return false;
    }
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
    int read_fd = open("/dev/nvme0n1", O_RDONLY);
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

size_t NVMeDataManager::get_queue_size() const {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    return data_queue_.size();
}

size_t NVMeDataManager::get_buffer_usage() const {
    return lidar_buffer_pos_ + imu_buffer_pos_;
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

        // 使用 writev 零拷贝写入：header + payload + zero-padding
        if (!write_to_nvme(&block->header,
                          block->data.data(),
                          block->data.size(),
                          block->padding_size)) {
            std::cerr << "Writer thread: writev failed for "
                      << block->data.size() << " bytes" << std::endl;
        }
    }
}