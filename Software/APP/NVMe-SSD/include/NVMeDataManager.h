#ifndef NVME_DATA_MANAGER_H
#define NVME_DATA_MANAGER_H

#include <cstdint>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <sys/uio.h>
#include <unistd.h>
#include <memory>

// 数据类型枚举
enum class DataType : uint8_t {
    VIDEO_FRONT = 0,
    VIDEO_REAR = 1,
    LIDAR = 2,
    IMU = 3
};

// 数据包头结构
#pragma pack(push, 1)
struct Header {
    uint32_t magic_number;      // 魔数，标识数据起点
    uint8_t data_type;          // 数据类型
    uint64_t timestamp_ns;      // 绝对系统纳秒时间戳
    uint32_t data_size;         // 真实数据大小
};
#pragma pack(pop)

// 数据块结构
struct DataBlock {
    Header header;             // 数据包头
    std::vector<uint8_t> data;  // 数据内容
};

class NVMeDataManager {
public:
    // 构造函数和析构函数
    NVMeDataManager();
    ~NVMeDataManager();

    // 禁止拷贝和赋值
    NVMeDataManager(const NVMeDataManager&) = delete;
    NVMeDataManager& operator=(const NVMeDataManager&) = delete;

    // 初始化和清理
    bool initialize();
    void shutdown();

    // 数据写入接口
    bool write_video_frame_to_disk(const uint8_t* frame_data, size_t frame_size,
                                uint64_t timestamp_ns, bool is_front_camera);
    bool write_lidar_points_to_disk(const uint8_t* points_data, size_t points_size,
                                 uint64_t timestamp_ns);
    bool write_imu_data_to_disk(const uint8_t* imu_data, size_t imu_size,
                             uint64_t timestamp_ns);

    // 数据读取接口
    bool read_video_frame_from_disk(uint64_t target_timestamp, float time_interval);

    // 获取统计信息
    size_t get_queue_size() const;
    size_t get_buffer_usage() const;

private:
    // 线程函数
    void writer_thread();

    // 辅助函数
    void prepare_header(Header& header, DataType type, uint64_t timestamp, uint32_t data_size);
    bool write_to_nvme(const iovec* iov, int iovcnt);

    // 线程和同步
    std::thread writer_thread_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_;

    // 队列
    std::queue<std::shared_ptr<DataBlock>> data_queue_;

    // 缓冲池
    std::vector<uint8_t> lidar_buffer_;
    std::vector<uint8_t> imu_buffer_;
    size_t lidar_buffer_pos_;
    size_t imu_buffer_pos_;

    // 数据块
    DataBlock front_camera_block_;
    DataBlock rear_camera_block_;
    DataBlock lidar_block_;

    // NVMe设备文件描述符
    int nvme_fd_;

    // 配置参数
    static constexpr size_t BUFFER_SIZE = 1024 * 1024;      // 1MB缓冲池
    static constexpr size_t HEADER_ALIGNMENT = 512;        // 512B对齐
    static constexpr uint32_t MAGIC_NUMBER = 0xDEADBEEF;   // 魔数
};

#endif // NVME_DATA_MANAGER_H