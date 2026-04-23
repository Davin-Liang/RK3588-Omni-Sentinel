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
    // 初始化数据块
    front_camera_block_.data.reserve(1024 * 1024);  // 预分配足够空间
    rear_camera_block_.data.reserve(1024 * 1024);
    lidar_block_.data.reserve(1024 * 1024);
}

NVMeDataManager::~NVMeDataManager() {
    shutdown();
}

bool NVMeDataManager::initialize() {
    // 打开NVMe设备
    nvme_fd_ = open("/dev/nvme0n1", O_WRONLY | O_DIRECT);
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

bool NVMeDataManager::write_to_nvme(const iovec* iov, int iovcnt) {
    if (nvme_fd_ < 0) {
        return false;
    }

    ssize_t bytes_written = writev(nvme_fd_, iov, iovcnt);
    if (bytes_written < 0) {
        std::cerr << "Failed to write to NVMe: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool NVMeDataManager::write_video_frame_to_disk(const uint8_t* frame_data, size_t frame_size,
                                           uint64_t timestamp_ns, bool is_front_camera) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 选择对应的数据块
    DataBlock* target_block = is_front_camera ? &front_camera_block_ : &rear_camera_block_;

    // 准备包头
    prepare_header(target_block->header,
                 is_front_camera ? DataType::VIDEO_FRONT : DataType::VIDEO_REAR,
                 timestamp_ns, static_cast<uint32_t>(frame_size));

    // 设置数据
    target_block->data.clear();
    target_block->data.insert(target_block->data.end(),
                           reinterpret_cast<const uint8_t*>(&target_block->header),
                           reinterpret_cast<const uint8_t*>(&target_block->header) + sizeof(Header));
    target_block->data.insert(target_block->data.end(), frame_data, frame_data + frame_size);

    // 创建共享指针并加入队列
    auto block_ptr = std::make_shared<DataBlock>(*target_block);
    data_queue_.push(block_ptr);
    queue_cv_.notify_one();

    return true;
}

bool NVMeDataManager::write_lidar_points_to_disk(const uint8_t* points_data, size_t points_size,
                                          uint64_t timestamp_ns) {
    std::lock_guard<std::mutex> lock(queue_mutex_);

    // 检查缓冲池空间
    if (lidar_buffer_pos_ + points_size > lidar_buffer_.size()) {
        // 缓冲池满了，需要写入
        if (!lidar_buffer_pos_) {
            std::cerr << "Lidar buffer is empty but still full, possible logic error" << std::endl;
            return false;
        }

        // 准备包头
        prepare_header(lidar_block_.header, DataType::LIDAR, timestamp_ns,
                     static_cast<uint32_t>(lidar_buffer_pos_));

        // 设置数据
        lidar_block_.data.clear();
        lidar_block_.data.insert(lidar_block_.data.end(),
                              reinterpret_cast<const uint8_t*>(&lidar_block_.header),
                              reinterpret_cast<const uint8_t*>(&lidar_block_.header) + sizeof(Header));
        lidar_block_.data.insert(lidar_block_.data.end(),
                              lidar_buffer_.data(), lidar_buffer_.data() + lidar_buffer_pos_);

        // 创建共享指针并加入队列
        auto block_ptr = std::make_shared<DataBlock>(lidar_block_);
        data_queue_.push(block_ptr);
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
        // 缓冲池满了，需要写入
        if (!imu_buffer_pos_) {
            std::cerr << "IMU buffer is empty but still full, possible logic error" << std::endl;
            return false;
        }

        // 准备包头
        prepare_header(lidar_block_.header, DataType::IMU, timestamp_ns,
                     static_cast<uint32_t>(imu_buffer_pos_));

        // 设置数据
        lidar_block_.data.clear();
        lidar_block_.data.insert(lidar_block_.data.end(),
                              reinterpret_cast<const uint8_t*>(&lidar_block_.header),
                              reinterpret_cast<const uint8_t*>(&lidar_block_.header) + sizeof(Header));
        lidar_block_.data.insert(lidar_block_.data.end(),
                              imu_buffer_.data(), imu_buffer_.data() + imu_buffer_pos_);

        // 创建共享指针并加入队列
        auto block_ptr = std::make_shared<DataBlock>(lidar_block_);
        data_queue_.push(block_ptr);
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

bool NVMeDataManager::read_video_frame_from_disk(uint64_t target_timestamp, float time_interval) {
    // TODO: 实现从NVMe SSD读取特定时间戳的视频流
    // 这里需要实现实际的读取逻辑
    std::cout << "Reading video frames around timestamp: " << target_timestamp
              << " with interval: " << time_interval << " seconds" << std::endl;
    return false;  // 暂时返回false，需要实现实际逻辑
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
        std::unique_lock<std::mutex> lock(queue_mutex_);

        // 等待队列中有数据或关闭信号
        queue_cv_.wait(lock, [this] {
            return !data_queue_.empty() || !running_;
        });

        if (!running_) {
            break;
        }

        if (data_queue_.empty()) {
            continue;
        }

        // 获取数据块
        auto block_ptr = data_queue_.front();
        data_queue_.pop();
        lock.unlock();

        // 准备iovec结构用于writev
        iovec iov[2];
        iov[0].iov_base = &block_ptr->header;
        iov[0].iov_len = sizeof(Header);
        iov[1].iov_base = block_ptr->data.data();
        iov[1].iov_len = block_ptr->data.size() - sizeof(Header);

        // 写入NVMe
        write_to_nvme(iov, 2);
    }
}