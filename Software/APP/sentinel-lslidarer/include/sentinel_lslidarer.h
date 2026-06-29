#ifndef SENTINEL_LSLIDARER_H
#define SENTINEL_LSLIDARER_H

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

/**
 * @struct LidarPoint
 * @brief 单线雷达的一个点（2D 平面坐标 + 强度）
 */
struct LidarPoint {
    float x;         ///< X 坐标（米）
    float y;         ///< Y 坐标（米）
    float intensity; ///< 反射强度 (0-255)
};

/**
 * @struct LidarFrame
 * @brief 一帧完整的雷达点云
 *
 * 调用 get_closest_frame() 前，需将 points 指向预分配的缓冲区，
 * 缓冲区大小至少为 max_points_per_frame() 返回值的 LidarPoint 数组。
 */
struct LidarFrame {
    uint64_t timestampNs; ///< 帧时间戳（CLOCK_MONOTONIC，纳秒）
    uint32_t pointsCount; ///< 帧内有效点数
    LidarPoint* points;   ///< 点数据指针（调用者预分配）
};

/**
 * @struct LidarConfig
 * @brief 雷达配置参数
 *
 * 默认值对应 N10Plus 单线雷达通过串口连接。
 * 协议常量（kPacketLength 等）与型号绑定，由 lidarModel 选择。
 */
struct LidarConfig {
    /// 雷达型号: "N10Plus" (当前仅支持此型号)
    std::string lidarModel = "N10Plus";

    /// 串口设备路径
    std::string serialPort = "/dev/sentinel_lidar";

    /// 串口波特率 (N10Plus = 460800)
    int baudRate = 460800;

    /// N10Plus 电机频率 (Hz, 默认 10)
    int n10PlusHz = 10;

    /// N10Plus: 数据包固定长度（字节）
    static constexpr int kPacketLength = 108;

    /// N10Plus: 起始方位角在包中的字节偏移
    static constexpr int kAngleBitsStart = 5;

    /// N10Plus: 结束方位角在包中的字节偏移
    static constexpr int kEndAngleBitsStart = 105;

    /// N10Plus: 点数据在包中的起始字节偏移
    static constexpr int kDataBitsStart = 7;

    /// N10Plus: 每包角度组数（16 个角度 × 2 回波 = 32 点每包）
    static constexpr int kPacketPointsMax = 16;

    /// N10Plus: 每圈理论最大点数 = ceil(5400 Hz / 10 Hz) = 540
    static constexpr int kPointsPerSweep = 540;

    /// 距离分辨率（米/计数值）
    static constexpr float kDistanceResolution = 0.001f;

    /// 最小有效距离（米）
    float minRange = 0.15f;

    /// 最大有效距离（米）
    float maxRange = 50.0f;

    /// 角度屏蔽起始（0.01° 单位，默认 9000 = 90.00°）
    int angleDisableMin = 9000;

    /// 角度屏蔽结束（0.01° 单位，默认 24000 = 240.00°）
    int angleDisableMax = 24000;

    /// 环形缓冲区存储的帧数
    uint32_t ringBufferSize = 10;
};

// ============================================================================
// 内部实现：环形缓冲区 + 串口操作
// ============================================================================

/**
 * @class RingBuffer
 * @brief 单写单读（SWCR）无锁环形缓冲区
 *
 * 写者（读取线程）通过 begin_write/commit_write 写入帧；
 * 读者（应用线程）通过 write_index + copy_slot 读取帧。
 * memory_order_release/acquire 保证写者写完槽数据后读者可见。
 */
class RingBuffer {
public:
    /**
     * @brief 构造环形缓冲区并预分配内存池
     * @param capacity        最大帧数
     * @param maxPointsPerSlot 每帧最大点数
     */
    RingBuffer(uint32_t capacity, uint32_t maxPointsPerSlot);

    ~RingBuffer();

    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    /**
     * @brief 获取当前写槽位的点缓冲区指针（仅写者调用）
     * @return 指向 pointsPool_ 中当前槽位的 LidarPoint 数组指针
     */
    LidarPoint* begin_write();

    /**
     * @brief 提交当前帧到环形缓冲区（仅写者调用）
     * @param timestampNs 帧时间戳
     * @param pointsCount 帧内有效点数
     */
    void commit_write(uint64_t timestampNs, uint32_t pointsCount);

    /**
     * @brief 原子读取写索引快照（仅读者调用）
     * @return 当前写索引值
     */
    uint32_t write_index() const;

    /**
     * @brief 将指定槽的帧数据拷贝到 outFrame（仅读者调用）
     * @param slotIdx  槽索引，需保证在 [0, min(writeIndex, capacity)) 范围内
     * @param outFrame 输出帧
     */
    void copy_slot(uint32_t slotIdx, LidarFrame& outFrame);

    /// 返回环形缓冲区容量
    uint32_t capacity() const { return capacity_; }

    /// 返回每帧最大点数
    uint32_t max_points_per_slot() const { return maxPointsPerSlot_; }

private:
    struct Slot {
        std::atomic<uint32_t> sequence{0}; ///< 写入序号（全局 writeIndex + 1），用于内存屏障和有效性判断
        uint64_t timestampNs{0};           ///< 帧时间戳
        uint32_t pointsCount{0};           ///< 帧内点数
    };

    uint32_t capacity_;
    uint32_t maxPointsPerSlot_;
    Slot* slots_;
    LidarPoint* pointsPool_;
    std::atomic<uint32_t> writeIndex_{0};
};

/**
 * @class SerialPort
 * @brief Linux POSIX 串口操作封装
 */
class SerialPort {
public:
    SerialPort();
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    /**
     * @brief 打开并配置串口
     * @param port     设备路径（如 /dev/ttyUSB0）
     * @param baudRate 波特率
     * @return true 成功，false 失败
     */
    bool open(const std::string& port, int baudRate);

    /**
     * @brief 关闭串口
     */
    void close();

    /**
     * @brief 读取一个 N10Plus 数据包（阻塞，带超时）
     * @param buffer    输出缓冲区（至少 kPacketLength 字节）
     * @param packetLen 输出实际包长度
     * @return >0 成功返回包长度，0 超时/同步丢失，<0 错误
     */
    int read_packet(uint8_t* buffer, int& packetLen);

    /// 返回文件描述符
    int fd() const { return fd_; }

    /// 是否已打开
    bool is_open() const { return fd_ >= 0; }

private:
    int fd_;
};

// ============================================================================
// SentinelLslidarer
// ============================================================================

/**
 * @class SentinelLslidarer
 * @brief 脱离 ROS 的镭神 N10Plus 单线雷达驱动
 *
 * 独立线程读取串口数据，解析并累积为完整圈（帧），
 * 存入环形缓冲区。应用层通过时间戳查找最接近的一帧点云。
 *
 * 典型用法：
 * @code
 *   SentinelLslidarer lidar;
 *   LidarConfig cfg;
 *   lidar.load_config(cfg);
 *   lidar.start();
 *
 *   LidarFrame frame;
 *   frame.points = new LidarPoint[lidar.max_points_per_frame()];
 *   if (lidar.get_closest_frame(cameraTimestamp, frame)) {
 *       // 使用 frame.points[0..frame.pointsCount-1]
 *   }
 *   delete[] frame.points;
 *   lidar.stop();
 * @endcode
 */
class SentinelLslidarer {
public:
    SentinelLslidarer();
    ~SentinelLslidarer();

    SentinelLslidarer(const SentinelLslidarer&) = delete;
    SentinelLslidarer& operator=(const SentinelLslidarer&) = delete;

    // ---- 生命周期 ----

    /**
     * @brief 从 LidarConfig 结构体加载配置
     * @param config 雷达配置参数
     * @return true 配置有效，false 参数非法
     */
    bool load_config(const LidarConfig& config);

    /**
     * @brief 启动雷达：打开串口、初始化 LUT、分配环形缓冲区、启动读取线程
     * @return true 启动成功，false 串口打开失败或内存不足
     */
    bool start();

    /**
     * @brief 停止雷达：通知线程退出、等待线程结束、关闭串口、释放内存
     */
    void stop();

    /**
     * @brief 查询雷达是否正在运行
     * @return true 正在运行
     */
    bool is_running() const;

    // ---- 帧查询 ----

    /**
     * @brief 获取与给定时间戳最接近的一帧点云
     * @param cameraTsNs 图像帧的时间戳（CLOCK_MONOTONIC，单位纳秒）
     * @param outFrame   输出帧，调用者需预分配 outFrame.points 缓冲区
     *                   （大小至少为 max_points_per_frame()）
     * @return true 查找成功，false 环形缓冲区为空（尚无帧数据）
     */
    bool get_closest_frame(uint64_t cameraTsNs, LidarFrame& outFrame);

    /**
     * @brief 获取最新的帧（无需时间戳匹配）
     * @param outFrame 输出帧，调用者需预分配 outFrame.points 缓冲区
     * @return true 成功，false 环形缓冲区为空
     */
    bool get_latest_frame(LidarFrame& outFrame);

    /**
     * @brief 返回环形缓冲区中当前可用的帧数
     * @return 可用帧数量（0 ~ ringBufferSize）
     */
    uint32_t available_frames() const;

    // ---- 配置查询 ----

    /**
     * @brief 返回每帧最大点数
     * @return 最大点数（即 outFrame.points 所需的最小预分配元素数）
     */
    uint32_t max_points_per_frame() const;

    /**
     * @brief 获取当前配置的只读引用
     * @return 配置结构体常量引用
     */
    const LidarConfig& config() const;

private:
    // ---- 解码中间结构（2 回波点对）----
    struct DecodedPoint {
        uint16_t azimuth;  // 0.01° 单位 [0, 36000)
        float distance;    // 米
        float intensity;   // 反射强度 (0-255, N10Plus 有效)
    };

    // ---- 读取线程 ----
    void reader_loop_();

    // ---- 包处理（实现在 m10p_protocol.cpp）----
    void build_lut_();
    bool check_packet_validity_(const uint8_t* data, int packetLen) const;
    bool is_point_valid_(float distance, int azimuth) const;

    // 解码一个 N10Plus 包，返回解码出的点数量（含双回波）
    int decode_packet_(const uint8_t* data, int packetLen,
                       DecodedPoint* decoded, int maxPoints);

    // ---- 配置 ----
    LidarConfig config_;

    // ---- 线程状态 ----
    std::atomic<bool> running_{false};
    std::thread readerThread_;

    // ---- 串口 ----
    std::unique_ptr<SerialPort> serialPort_;

    // ---- LUT ----
    float sinLut_[36000]{};
    float cosLut_[36000]{};

    // ---- 环形缓冲区 ----
    std::unique_ptr<RingBuffer> ringBuffer_;
};

#endif  // SENTINEL_LSLIDARER_H
