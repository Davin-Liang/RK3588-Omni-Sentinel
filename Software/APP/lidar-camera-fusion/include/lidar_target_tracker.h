#ifndef LIDAR_TARGET_TRACKER_H
#define LIDAR_TARGET_TRACKER_H

#include <cstdint>
#include <mutex>

#include "lidar_tracking_types.h"
#include "lidar_camera_fusion.h"

/**
 * @class LidarTargetTracker
 * @brief 多目标跟踪器（内部辅助类，由 LidarCameraFusion 持有）。
 *
 * 流水线：聚类 → Alpha-Beta 预测 → 贪心最近邻关联 → 校正 → 生命周期管理 → 告警检查。
 * 所有缓冲区在构造时预分配，运行时无堆分配。
 *
 * 线程安全：内部双缓冲（workingTracks_ + snapshotTracks_），
 *          update() 结束后加锁拷贝到 snapshotTracks_，
 *          copy_snapshot() 加锁从 snapshotTracks_ 读取。
 */
class LidarTargetTracker {
public:
    static constexpr uint32_t kMaxTracks      = 50;
    static constexpr uint32_t kMaxDetections  = 200;
    static constexpr uint32_t kMaxLidarPoints = 540;

    LidarTargetTracker();
    ~LidarTargetTracker();

    LidarTargetTracker(const LidarTargetTracker&) = delete;
    LidarTargetTracker& operator=(const LidarTargetTracker&) = delete;

    /**
     * @brief 配置跟踪器参数。
     * @return true 参数合法并已应用，false 参数非法（保留旧配置）
     */
    bool configure(const TrackerConfig& config);

    /**
     * @brief 重置所有跟踪状态（航迹清零，ID 重置）。
     */
    void reset();

    /**
     * @brief 注册告警回调。
     */
    void register_callback(TrackingCallback cb, void* userData);

    /**
     * @brief 执行一次跟踪更新。
     */
    bool update(const FusionResult& fusionResult,
                const LidarPoint* lidarPoints,
                uint32_t pointCount,
                const YoloBBox* bboxes,
                uint32_t bboxCount,
                uint64_t timestampNs);

    /**
     * @brief 拷贝当前跟踪目标快照（线程安全）。
     * @param outCount 输出实际拷贝数量
     * @return true 成功
     */
    bool copy_snapshot(TrackedTarget* out, uint32_t maxCount,
                       uint32_t* outCount) const;

private:
    // ---- 内部结构 ----
    struct DetectionCandidate {
        float    x, y;
        uint32_t classId;
        float    confidence;
        float    avgIntensity;
        uint32_t pointCount;
        uint32_t bboxIdx;
        bool     isOrphan;    ///< 来自 LiDAR 孤儿点聚类（非 bbox 归属）
    };

    struct ClusterInfo {
        uint32_t startIdx;
        uint32_t count;
        float    sumX, sumY, sumI;
    };

    // ---- 配置与回调 ----
    TrackerConfig   config_;
    TrackingCallback warningCb_{nullptr};
    void*           warningUserData_{nullptr};

    // ---- 航迹存储（双缓冲） ----
    TrackedTarget workingTracks_[kMaxTracks];
    TrackedTarget snapshotTracks_[kMaxTracks];
    uint32_t      activeTrackCount_{0};
    uint32_t      snapshotTrackCount_{0};
    uint32_t      nextTrackId_{1};
    mutable std::mutex snapshotMutex_;

    // ---- 预测状态 ----
    float predX_[kMaxTracks];
    float predY_[kMaxTracks];
    float predVX_[kMaxTracks];
    float predVY_[kMaxTracks];

    // ---- 本帧检测缓存 ----
    DetectionCandidate detections_[kMaxDetections];
    uint32_t           detectionCount_{0};

    // ---- 聚类缓冲区 ----
    float    clusterPointsX_[kMaxLidarPoints];
    float    clusterPointsY_[kMaxLidarPoints];
    uint32_t clusterPointIndices_[kMaxLidarPoints];
    int32_t  clusterAssignments_[kMaxLidarPoints];
    bool     pointAssigned_[kMaxLidarPoints];     // 标记已被 bbox 认领的点

    // ---- 关联缓冲区 ----
    int32_t trackMatches_[kMaxTracks];
    int32_t detMatches_[kMaxDetections];
    bool    trackAssigned_[kMaxTracks];
    bool    detAssigned_[kMaxDetections];

    // ---- 配置校验 ----
    bool validate_config_(const TrackerConfig& cfg) const;

    // ---- 聚类 ----
    void cluster_bbox_points_(const FusionResult& fusionResult,
                              const LidarPoint* lidarPoints,
                              uint32_t pointCount,
                              const YoloBBox* bboxes,
                              uint32_t bboxCount);

    void cluster_orphan_points_(const FusionResult& fusionResult,
                                 const LidarPoint* lidarPoints,
                                 uint32_t pointCount);

    // ---- 预测 ----
    void predict_tracks_(uint64_t timestampNs);

    // ---- 关联 ----
    void associate_(uint64_t timestampNs, const YoloBBox* bboxes);

    // ---- 校正 ----
    void apply_correction_(uint32_t trackIdx, const DetectionCandidate& det,
                           float dt);

    // ---- 生命周期 ----
    void manage_lifecycle_();

    // ---- 告警 ----
    void check_warnings_(uint64_t nowNs);

    // ---- 快照 ----
    void update_snapshot_();
};

#endif // LIDAR_TARGET_TRACKER_H
