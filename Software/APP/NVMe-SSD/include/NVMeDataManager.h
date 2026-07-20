#ifndef NVME_DATA_MANAGER_H
#define NVME_DATA_MANAGER_H

#include <cstdint>
#include <string>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <unistd.h>
#include <memory>
#include <sys/uio.h>

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
    std::vector<uint8_t> data;  // 数据内容（不含包头和填充）
    size_t padding_size;        // 512B对齐所需的填充字节数
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
    bool initialize(const char* device_path = "/dev/nvme0n1");
    void shutdown();

    // 数据写入接口
    bool write_video_frame_to_disk(const uint8_t* frame_data, size_t frame_size,
                                uint64_t timestamp_ns, bool is_front_camera);
    bool write_lidar_points_to_disk(const uint8_t* points_data, size_t points_size,
                                 uint64_t timestamp_ns);
    bool write_imu_data_to_disk(const uint8_t* imu_data, size_t imu_size,
                             uint64_t timestamp_ns);

    // 数据读取接口
    bool read_video_frame_from_disk(uint64_t target_timestamp, float time_interval,
                                    std::vector<uint8_t>& out_frame_data);

    // 导出触发时刻前 N 秒的视频片段（仅回溯，不含未来数据）
    // camera_id: -1=所有视频类型, N=仅匹配 data_type==N 的视频帧（方便扩展多摄像头）
    bool export_trigger_video_clip(uint64_t trigger_timestamp_ns,
                                   const std::string& output_path,
                                   double time_window_sec = 5.0,
                                   int fps = 15,
                                   int frame_width = 1920,
                                   int frame_height = 1080,
                                   int camera_id = -1,
                                   bool input_is_nv12 = false);

    // 导出雷达热力图 PNG（回溯窗口内所有 LiDAR 点按时间着色叠加）
    bool export_lidar_heatmap_png(uint64_t trigger_timestamp_ns,
                                   const std::string& output_path,
                                   double time_window_sec = 5.0);

    // 获取统计信息
    size_t get_queue_size() const;
    size_t get_buffer_usage() const;

private:
    // 热力图渲染辅助
    struct LidarPointRecord {
        float x, y;
        uint64_t timestamp_ns;
    };
    void render_heatmap_pixels_(const std::vector<LidarPointRecord>& points,
                                 int imgW, int imgH,
                                 std::vector<uint8_t>& rgba);

    // 线程函数
    void writer_thread();

    // 辅助函数
    void prepare_header(Header& header, DataType type, uint64_t timestamp, uint32_t data_size);
    bool write_to_nvme(const Header* header, const uint8_t* data,
                       size_t data_size, size_t padding_size);

    // 摄像头帧缓冲池管理
    DataBlock* acquire_pool_block();
    void release_pool_block(DataBlock* block);

    // 计算512B对齐填充
    static size_t calc_padding(size_t total_size) {
        size_t mod = total_size % HEADER_ALIGNMENT;
        return mod ? (HEADER_ALIGNMENT - mod) : 0;
    }

    // 线程和同步
    std::thread writer_thread_;
    mutable std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> running_;

    // 摄像头帧缓冲池（必须声明在 data_queue_ 之前，确保析构时池晚于队列销毁）
    static constexpr int CAMERA_POOL_SIZE = 4;
    static constexpr size_t CAMERA_MAX_SIZE = 1920 * 1080 * 3;  // 6,220,800
    DataBlock camera_pool_[CAMERA_POOL_SIZE];
    bool camera_pool_used_[CAMERA_POOL_SIZE];
    mutable std::mutex camera_pool_mutex_;

    // 队列
    std::queue<std::shared_ptr<DataBlock>> data_queue_;

    // 缓冲池
    std::vector<uint8_t> lidar_buffer_;
    std::vector<uint8_t> imu_buffer_;
    size_t lidar_buffer_pos_;
    size_t imu_buffer_pos_;

    // 页对齐的写入缓冲区（O_DIRECT 要求）
    uint8_t* write_buffer_;
    size_t write_buffer_size_;

    // NVMe设备文件描述符
    int nvme_fd_;

    // 设备路径
    std::string nvme_device_path_;

    // 配置参数
    static constexpr size_t BUFFER_SIZE = 1024 * 1024;      // 1MB缓冲池
    static constexpr size_t HEADER_ALIGNMENT = 512;        // 512B对齐
    static constexpr uint32_t MAGIC_NUMBER = 0xDEADBEEF;   // 魔数
};

#endif // NVME_DATA_MANAGER_H