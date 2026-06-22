#ifndef SENTINEL_STREAMER_H
#define SENTINEL_STREAMER_H

#include <cstdint>
#include <cstddef>
#include <functional>
#include <vector>

// 前置声明，避免循环依赖
struct DmaBuffer_t;
class SentinelVisioner;
struct StreamerContext;  // 内部实现，参见 sentinel_streamer.cpp

/**
 * @brief 录像分辨率
 */
enum class RecordResolution {
    RES_1080P = 1080,
    RES_720P  = 720
};

/**
 * @brief 推流 OSD 模式
 */
enum class StreamOsdMode {
    WITHOUT_OSD = 0,  ///< 无 OSD 叠加 720p（已实现）
    WITH_OSD    = 1   ///< 有 OSD 叠加 720p（预留，暂不实现）
};

/**
 * @brief OSD 叠加用的检测框（精简版，仅含绘制所需字段）
 */
struct StreamOsdBBox {
    uint32_t x1, y1, x2, y2;
    uint32_t classId;
    float    confidence;
};

/**
 * @brief OSD 检测结果提供者回调
 * @param camNum     摄像头编号
 * @param out        输出检测框列表
 * @param timeoutMs  超时毫秒数
 * @return true 成功获取，false 超时
 */
using StreamOsdProvider = std::function<bool(int camNum, std::vector<StreamOsdBBox>& out, int timeoutMs)>;

/**
 * @brief 推流 LiDAR OSD 模式
 */
enum class StreamLidarOsdMode {
    WITHOUT_LIDAR_OSD = 0,  ///< 无 LiDAR 点叠加
    WITH_LIDAR_OSD    = 1   ///< 叠加 LiDAR 点 + 距离标签
};

/**
 * @brief LiDAR OSD 叠加数据（单框）
 */
struct StreamLidarOsdBBox {
    uint32_t x1, y1, x2, y2;       ///< NPU 640x640 空间 bbox（距离标签定位用）
    float    distanceMeters;        ///< 框内 LiDAR 点平均距离
    std::vector<float> pointsU;     ///< 投影 U 坐标（原始图像空间）
    std::vector<float> pointsV;     ///< 投影 V 坐标（原始图像空间）
    uint32_t pointCount;            ///< = pointsU.size()
};

/**
 * @brief LiDAR OSD 数据提供者回调
 * @param camNum     摄像头编号
 * @param out        输出 LiDAR OSD bbox 列表
 * @param timeoutMs  超时毫秒数
 * @return true 成功获取，false 超时
 */
using StreamLidarOsdProvider = std::function<bool(int camNum, std::vector<StreamLidarOsdBBox>& out, int timeoutMs)>;

/**
 * @brief 状态回调事件类型
 */
enum class StreamerEvent {
    STREAM_STARTED  = 0,  ///< 推流已启动
    STREAM_STOPPED  = 1,  ///< 推流已停止
    RECORD_STARTED  = 2,  ///< 录像已启动
    RECORD_STOPPED  = 3,  ///< 录像已停止
    ERROR           = 4,  ///< 错误（detail 为错误描述）
};

/**
 * @brief 状态回调函数类型
 * @param camNum  摄像头编号
 * @param event   事件类型
 * @param detail  附加信息（启动时为 URL/路径，错误时为错误描述，可为 nullptr）
 */
using StreamerCallback = void (*)(int camNum, StreamerEvent event, const char* detail);

/**
 * @class SentinelStreamer
 * @brief 推流与录像组件。
 *
 * 作为 SentinelVisioner 的下游消费者，从 processTaskQueue 拉取 1080p NV12 原始帧，
 * 经 RGA 缩放为 720p 后通过 MPP 硬件编码为 H.264，再复用为 RTSP 推流和/或 MP4 录像。
 *
 * 每路摄像头拥有独立的推流线程，支持同时推流和录像。
 *
 * 典型用法:
 * @code
 *   SentinelVisioner visioner;
 *   visioner.add_camera("/dev/video11", 1920, 1080, 8, 0);
 *   visioner.camera_stream_ctrl(0, true);
 *
 *   SentinelStreamer streamer;
 *   streamer.add_camera(0, &visioner);
 *   streamer.start_stream(0, "rtsp://192.168.1.100:8554/live/cam0");
 *   streamer.start_record(0, "/tmp/rec.mp4", RecordResolution::RES_1080P);
 *   // ... 运行 ...
 *   streamer.stop_record(0);
 *   streamer.stop_stream(0);
 *   streamer.remove_camera(0);
 * @endcode
 */
class SentinelStreamer {
public:
    SentinelStreamer();
    ~SentinelStreamer();

    // 禁止拷贝
    SentinelStreamer(const SentinelStreamer&) = delete;
    SentinelStreamer& operator=(const SentinelStreamer&) = delete;

    // ================================================================
    // 生命周期
    // ================================================================

    /**
     * @brief 注册一路摄像头，初始化缩放池
     * @param camNum    摄像头逻辑编号（与 SentinelVisioner::add_camera 的 camNum 对应）
     * @param visioner  已初始化并开启视频流的 SentinelVisioner 实例指针
     * @param poolSize  720p 缩放缓冲池大小（默认 4，MPP 偶发慢时可加大）
     * @return true 成功 / false 失败
     */
    bool add_camera(int camNum, SentinelVisioner* visioner, int poolSize = 4);

    /**
     * @brief 注销一路摄像头，停止推流/录像并释放所有资源
     * @param camNum 摄像头编号
     * @return true 成功 / false 失败
     */
    bool remove_camera(int camNum);

    // ================================================================
    // 推流
    // ================================================================

    /**
     * @brief 启动 720p RTSP 推流
     * @param camNum  摄像头编号
     * @param rtspUrl 推流目标 URL (如 "rtsp://192.168.1.100:8554/live/cam0")
     * @return true 成功 / false 失败
     */
    bool start_stream(int camNum, const char* rtspUrl);

    /**
     * @brief 停止 RTSP 推流
     * @param camNum 摄像头编号
     * @return true 成功 / false 失败
     */
    bool stop_stream(int camNum);

    /**
     * @brief 查询是否正在推流
     * @param camNum 摄像头编号
     * @return true 推流中 / false 未推流
     */
    bool is_streaming(int camNum) const;

    // ================================================================
    // 推流 OSD 模式
    // ================================================================

    /**
     * @brief 设置推流 OSD 模式（支持运行时切换）
     * @param camNum 摄像头编号
     * @param mode   WITHOUT_OSD: 无 OSD 720p 推流
     *               WITH_OSD:    有 OSD 720p 推流
     * @return true 成功 / false 失败
     */
    bool set_stream_osd_mode(int camNum, StreamOsdMode mode);

    /**
     * @brief 设置 OSD 检测结果提供者
     * @param provider 回调函数，streamer 推流线程每帧轮询获取检测框
     */
    void set_osd_provider(StreamOsdProvider provider);

    /**
     * @brief 设置推流 LiDAR OSD 模式（支持运行时切换）
     * @param camNum 摄像头编号
     * @param mode   WITHOUT_LIDAR_OSD / WITH_LIDAR_OSD
     * @return true 成功 / false 失败
     */
    bool set_stream_lidar_osd_mode(int camNum, StreamLidarOsdMode mode);

    /**
     * @brief 设置 LiDAR OSD 数据提供者
     * @param provider 回调函数，streamer 推流线程每帧轮询获取 LiDAR 点数据
     */
    void set_lidar_osd_provider(StreamLidarOsdProvider provider);

    // ================================================================
    // 录像
    // ================================================================

    /**
     * @brief 启动 MP4 录像
     * @param camNum     摄像头编号
     * @param filePath   输出 MP4 文件路径
     * @param resolution 录像分辨率 (RES_1080P 或 RES_720P)
     * @return true 成功 / false 失败
     */
    bool start_record(int camNum, const char* filePath, RecordResolution resolution);

    /**
     * @brief 停止 MP4 录像
     * @param camNum 摄像头编号
     * @return true 成功 / false 失败
     */
    bool stop_record(int camNum);

    /**
     * @brief 查询是否正在录像
     * @param camNum 摄像头编号
     * @return true 录像中 / false 未录像
     */
    bool is_recording(int camNum) const;

    // ================================================================
    // 状态回调
    // ================================================================

    /**
     * @brief 设置状态回调（推流/录像启停、错误时回调）
     *
     * 回调在 SentinelStreamer 内部线程调用，Qt 用户需通过信号槽跨线程传递。
     * 传 nullptr 取消回调。
     */
    void set_callback(StreamerCallback cb);

    // ================================================================
    // 录像帧缓冲（供磁盘写入线程消费）
    // ================================================================

    bool init_record_buffer(int camNum, int slotCount, int width, int height);

    bool try_get_record_frame(int camNum, uint8_t** outData, size_t* outSize,
                              uint64_t* outTimestampUs);

    void release_record_frame(int camNum, uint8_t* data);

private:
    StreamerContext* contexts_[2];  ///< 最多支持 2 路摄像头，不透明实现
};

#endif  // SENTINEL_STREAMER_H
