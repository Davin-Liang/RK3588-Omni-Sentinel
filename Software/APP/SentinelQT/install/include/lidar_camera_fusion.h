#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

#include "sentinel_lslidarer.h"
#include "lidar_tracking_types.h"

/**
 * @struct YoloBBox
 * @brief  YOLO 目标检测输出的单个 2D 边界框。
 * @note   定义在此处是因为项目中 YOLO 模块尚未实现；
 *         当 YOLO 组件就绪后，此结构体可提取到共享头文件中。
 */
#ifndef YOLO_BBOX_DEFINED
#define YOLO_BBOX_DEFINED
struct YoloBBox {
    uint32_t x1, y1;       ///< 左上角像素坐标（包含）
    uint32_t x2, y2;       ///< 右下角像素坐标（不包含），宽度 = x2 - x1
    uint32_t classId;      ///< 类别 ID（COCO 格式）
    float    confidence;   ///< 置信度 [0.0, 1.0]
    uint64_t timestampNs;  ///< 该检测框对应的图像帧时间戳（CLOCK_MONOTONIC, ns）
};
#endif

/**
 * @struct CameraConfig
 * @brief  单个相机的内参 + 外参 + 图像尺寸。
 *         作为 fuse_data() 的参数传入，不同相机使用不同的 CameraConfig。
 */
struct CameraConfig {
    float    fx, fy, cx, cy;       ///< 相机内参（4 参数针孔模型）
    float    tLidarToCam[16];      ///< 雷达到该相机的 4x4 外参矩阵（行主序）
    uint32_t imgWidth;             ///< 图像宽度（像素）
    uint32_t imgHeight;            ///< 图像高度（像素）
    float    yoloConfThreshold = 0.60f; ///< YOLO 检测置信度阈值（仅 person classId=0）
};

/**
 * @struct FusionResult
 * @brief  融合输出（只读，内存由 LidarCameraFusion 管理）。
 *         内容在下一次 reset() 调用时被覆盖。
 *
 * bbox 按 fuse_data() 调用顺序排列：
 *   首次调用 fuse_data(A, ...) → bbox 0 .. nA-1
 *   二次调用 fuse_data(B, ...) → bbox nA .. nA+nB-1
 */
struct FusionResult {
    uint64_t imageTimestampNs;          ///< 图像帧时间戳（CLOCK_MONOTONIC, ns）
    uint64_t lidarTimestampNs;          ///< 雷达帧时间戳（CLOCK_MONOTONIC, ns）

    const uint32_t* bboxPointIndices;   ///< 展平的点索引数组，按 bbox 分段
    const uint32_t* bboxPointCounts;    ///< 数组，bboxPointCounts[i] = 第 i 个 bbox 的点数量
    const float*    bboxPointU;         ///< 投影像素 U 坐标（原始图像空间），与 bboxPointIndices 同布局
    const float*    bboxPointV;         ///< 投影像素 V 坐标（原始图像空间），与 bboxPointIndices 同布局
    uint32_t bboxCount;                 ///< 已累积的 bbox 总数
};

using DetectionProvider = std::function<bool(int camNum, std::vector<YoloBBox>& out, int timeoutMs)>;

/**
 * @struct PerCameraLidarOsd
 * @brief  单路相机的 LiDAR OSD 快照数据（深拷贝，独立于融合内部缓冲区）。
 */
struct PerCameraLidarOsd {
    int      camNum;
    uint32_t imgWidth;
    uint32_t imgHeight;
    std::vector<float>    bboxPointU;
    std::vector<float>    bboxPointV;
    std::vector<uint32_t> bboxPointCounts;
    uint32_t              bboxCount;
    std::vector<uint32_t> bboxX1;
    std::vector<uint32_t> bboxY1;
    std::vector<uint32_t> bboxX2;
    std::vector<uint32_t> bboxY2;
    std::vector<float>    lidarPointX;
    std::vector<float>    lidarPointY;
    std::vector<uint32_t> bboxPointIndices;
    std::vector<float>    bboxClusterDistMeters;
};

/**
 * @struct LidarOsdSnapshot
 * @brief  多路相机 LiDAR OSD 快照，由融合线程写入、streamer 侧读取。
 */
struct LidarOsdSnapshot {
    uint64_t          timestampNs;
    PerCameraLidarOsd cameras[2];
    uint32_t          camCount;
};

class SentinelLslidarer;
class LidarTargetTracker;

/**
 * @class LidarCameraFusion
 * @brief 视觉-雷达数据融合（支持单相机 / 双相机累积融合，支持内部线程模式）。
 *
 * 手动模式（无内部线程）：
 * @code
 * LidarCameraFusion fusion;
 * fusion.reset();
 * fusion.fuse_data(detections, imageTs, lidarFrame, cameraCfg);
 * const FusionResult& r = fusion.result();
 * @endcode
 *
 * 线程模式：
 * @code
 * LidarCameraFusion fusion;
 * CameraConfig cfgs[2] = { frontCfg, rearCfg };
 * fusion.start(&lidar, cfgs, 2);
 * // ... 运行中，线程持续融合 ...
 * fusion.stop();
 * const FusionResult& r = fusion.result();
 * @endcode
 */
class LidarCameraFusion {
public:
    static constexpr uint32_t kMaxLidarPoints = 1200;  ///< N10Plus 单圈最大点数
    static constexpr uint32_t kMaxDetections  = 100;   ///< 累积最大 bbox 数
    static constexpr uint32_t kMaxCameras     = 2;     ///< 最大相机数量

    LidarCameraFusion();
    ~LidarCameraFusion();

    // 禁止拷贝
    LidarCameraFusion(const LidarCameraFusion&) = delete;
    LidarCameraFusion& operator=(const LidarCameraFusion&) = delete;

    // ---- 手动模式 API ----

    void reset();

    bool fuse_data(const std::vector<YoloBBox>& detections,
                   uint64_t imageTimestampNs,
                   const LidarFrame& lidarFrame,
                   const CameraConfig& cameraCfg);

    const FusionResult& result() const;
    uint32_t behind_camera_count() const;
    uint32_t out_of_image_count() const;

    // ---- 线程模式 API ----

    /**
     * @brief  启动内部融合线程。
     *         线程循环：获取 YOLO 结果 → 取雷达帧 → 累积融合 → 输出结果。
     *         目前 YOLO 结果使用虚构测试数据（待推理类就绪后替换）。
     * @param  lidar       雷达驱动实例指针（非拥有，生命周期由调用者管理）
     * @param  camConfigs   相机配置数组
     * @param  camCount     相机数量（1 或 2）
     * @return true 成功，false 参数非法或已在运行
     */
    bool start(SentinelLslidarer* lidar,
               const CameraConfig* camConfigs,
               uint32_t camCount);

    /**
     * @brief  设置外部检测提供者（替换内部假检测）。
     *         设置后融合线程将从此回调获取 YOLO 检测结果。
     * @param  provider  回调函数；返回 false 表示超时无数据
     */
    void set_detection_provider(DetectionProvider provider);

    /**
     * @brief  获取最新 LiDAR OSD 快照（非阻塞，线程安全）。
     * @param  out       输出快照（深拷贝，调用者无需释放）
     * @param  timeoutMs 保留参数，当前实现忽略
     * @return true 有数据，false 尚无快照
     */
    bool try_get_lidar_osd_snapshot(LidarOsdSnapshot& out, int timeoutMs);

    /**
     * @brief  停止融合线程并等待退出。
     */
    void stop();

    /**
     * @brief  查询融合线程是否在运行。
     */
    bool is_running() const;

    // ---- 运行时配置查询/更新 ----

    /**
     * @brief  获取当前跟踪器配置的只读引用。
     * @return TrackerConfig 常量引用
     */
    const TrackerConfig& get_tracker_config() const;

    /**
     * @brief  获取指定相机的配置（运行时查询）。
     * @param  camIndex 相机索引 [0, camCount-1]
     * @param  outCfg   输出配置（调用者分配）
     * @return true 索引合法，false 失败
     */
    bool get_camera_config(uint32_t camIndex, CameraConfig& outCfg) const;

    /**
     * @brief  运行时更新相机内参（保留外参矩阵不变）。
     *         内部先读取当前 camConfigs_[camIndex] 的 tLidarToCam，
     *         仅覆盖 fx/fy/cx/cy/imgWidth/imgHeight 字段。
     * @param  camIndex  相机索引 [0, camCount-1]
     * @param  fx, fy, cx, cy  针孔模型内参
     * @param  imgWidth, imgHeight  图像尺寸（像素）
     * @return true 索引合法，false 失败
     */
    bool update_camera_intrinsics(uint32_t camIndex,
                                  float fx, float fy, float cx, float cy,
                                  uint32_t imgWidth, uint32_t imgHeight);

    /**
     * @brief  获取当前相机数量。
     * @return 相机数量（0 = 未启动或仅构造）
     */
    uint32_t get_cam_count() const;

    // ---- 目标跟踪 API ----

    /**
     * @brief  配置跟踪器参数（需在 enable_tracking 之前调用）。
     * @param  config 跟踪器配置，内部做合法性校验
     * @return true 配置有效并已应用，false 参数非法
     */
    bool configure_tracker(const TrackerConfig& config);

    /**
     * @brief  启用/禁用目标跟踪。
     * @param  enable true 启用，false 禁用
     * @return true 成功
     */
    bool enable_tracking(bool enable);

    /**
     * @brief  重置跟踪器状态（清零所有航迹，ID 重置）。
     */
    void reset_tracking();

    /**
     * @brief  执行一次跟踪更新（手动模式，在 fuse_data() 完成后调用）。
     * @param  fusionResult fuse_data() 输出的融合结果
     * @param  lidarPoints   原始雷达点云数组（与 fuse_data() 使用的相同）
     * @param  pointCount    雷达点云有效点数
     * @param  bboxes        YOLO 检测框数组（仅用于 classId/confidence，顺序须与 fuse_data 调用顺序一致）
     * @param  bboxCount     bbox 数量
     * @param  timestampNs   当前帧时间戳（CLOCK_MONOTONIC, ns）
     * @return true 成功，false 参数非法或 tracker 未分配
     */
    bool update_tracking(const FusionResult& fusionResult,
                         const LidarPoint* lidarPoints,
                         uint32_t pointCount,
                         const YoloBBox* bboxes,
                         uint32_t bboxCount,
                         uint64_t timestampNs,
                         const uint32_t* bboxCamIdx = nullptr);

    /**
     * @brief  注册距离告警回调。
     * @param  cb       回调函数指针
     * @param  userData 用户数据指针（透传到回调）
     */
    void register_warning_callback(TrackingCallback cb, void* userData);

    /**
     * @brief  拷贝当前跟踪目标快照（线程安全）。
     * @param  out      输出缓冲区（调用者预分配）
     * @param  maxCount 缓冲区容量
     * @param  outCount 输出实际拷贝数量
     * @return true 成功
     */
    bool copy_tracked_targets(TrackedTarget* out, uint32_t maxCount,
                              uint32_t* outCount) const;

    bool copy_cluster_vis(ClusterVisData* out, uint32_t maxCount,
                          uint32_t* outCount) const;

private:
    // ---- 数学辅助 ----
    void transform_point_(float lx, float ly, const float* T,
                          float& cx, float& cy, float& cz) const;
    void project_point_(float cx, float cy, float cz,
                        const CameraConfig& cameraCfg,
                        float& u, float& v) const;

    // ---- 线程 ----
    void fusion_thread_();

    /**
     * @brief 生成虚构 YOLO 检测结果（测试用，推理类就绪后删除）。
     *        为每帧生成一个覆盖图像中心区域的检测框。
     */
    void generate_fake_detections_(uint32_t camIndex);

    // 预分配缓冲区（构造时分配，析构时释放）
    uint32_t* candidatePointBuf;
    int32_t*  pointToBbox;
    uint32_t* bboxPointCountsBuf;
    uint32_t* bboxOffsets;
    uint32_t* writeCursor;
    float*    bboxPointUBuf_;
    float*    bboxPointVBuf_;
    float*    tempU_;
    float*    tempV_;
    float*    lidarPointXBuf_;
    float*    lidarPointYBuf_;

    FusionResult result_;

    uint32_t totalBboxCount;
    uint32_t totalCandidateCount;
    uint32_t behindCameraCount;
    uint32_t outOfImageCount;

    // 线程相关
    std::atomic<bool> running_{false};
    std::thread        fusionThread_;
    SentinelLslidarer* lidar_;
    CameraConfig       camConfigs_[kMaxCameras];
    uint32_t           camCount_;
    LidarPoint*        lidarPointsBuf_;
    DetectionProvider   detectionProvider_;
    std::vector<YoloBBox> fakeDetections_[kMaxCameras];  ///< 虚构测试数据，检测提供者未设置时使用

    // ---- LiDAR OSD 快照 ----
    std::mutex          osdSnapshotMutex_;
    LidarOsdSnapshot    latestOsdSnapshot_;

    // ---- 目标跟踪 ----
    bool               trackingEnabled_{false};
    TrackerConfig      trackerConfig_{};
    LidarTargetTracker* tracker_{nullptr};
};

#endif // LIDAR_CAMERA_FUSION_H
