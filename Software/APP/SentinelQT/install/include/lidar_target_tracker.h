#ifndef LIDAR_TARGET_TRACKER_H
#define LIDAR_TARGET_TRACKER_H

#include <cstdint>
#include <mutex>

#include "lidar_tracking_types.h"
#include "lidar_camera_fusion.h"

class LidarTargetTracker {
public:
    static constexpr uint32_t kMaxTracks      = 50;
    static constexpr uint32_t kMaxDetections  = 200;
    static constexpr uint32_t kMaxLidarPoints = 540;
    static constexpr uint32_t kMaxClusters    = 32;

    LidarTargetTracker();
    ~LidarTargetTracker();

    LidarTargetTracker(const LidarTargetTracker&) = delete;
    LidarTargetTracker& operator=(const LidarTargetTracker&) = delete;

    bool configure(const TrackerConfig& config);
    void reset();
    void register_callback(TrackingCallback cb, void* userData);

    bool update(const FusionResult& fusionResult,
                const LidarPoint* lidarPoints,
                uint32_t pointCount,
                const YoloBBox* bboxes,
                uint32_t bboxCount,
                uint64_t timestampNs);

    bool copy_snapshot(TrackedTarget* out, uint32_t maxCount,
                       uint32_t* outCount) const;

    /** @brief 拷贝聚类可视化数据（线程安全） */
    bool copy_cluster_vis(ClusterVisData* out, uint32_t maxCount,
                          uint32_t* outCount) const;

    uint32_t get_detection_count() const { return detectionCount_; }

    bool get_bbox_detection_centroid(uint32_t globalBboxIdx,
                                     float& outX, float& outY) const;

    void set_camera_configs(const CameraConfig* configs, uint32_t count);

private:
    // ---- 内部结构 ----
    struct DetectionCandidate {
        float    x, y;
        uint32_t classId;
        float    confidence;
        float    avgIntensity;
        uint32_t pointCount;
        uint32_t bboxIdx;
        bool     isOrphan;
    };

    struct ClusterInfo {
        uint32_t startIdx;
        uint32_t count;
        float    sumX, sumY, sumI;
    };

    struct ClusterRecord {
        float    cx, cy, radius;
        uint32_t pointCount;
        uint32_t consecutiveFrames;
        uint64_t lastLidarTs;
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

    // ---- DBSCAN 聚类缓冲区 ----
    float    clusterPointsX_[kMaxLidarPoints];
    float    clusterPointsY_[kMaxLidarPoints];
    uint32_t clusterPointIndices_[kMaxLidarPoints];
    int32_t  clusterAssignments_[kMaxLidarPoints];
    int32_t  seedStack_[kMaxLidarPoints];

    // ---- 聚类中间数据 ----
    float    clusterCentroidX_[kMaxClusters];
    float    clusterCentroidY_[kMaxClusters];
    float    clusterRadii_[kMaxClusters];
    uint32_t clusterPointCounts_[kMaxClusters];
    int32_t  clusterBboxMatch_[kMaxClusters];
    float    clusterScore_[kMaxClusters];
    bool     claimedByBbox_[kMaxClusters];
    uint32_t bboxCluster_[50];
    uint32_t rawClusterCount_{0};

    // ---- 时间证据累积 ----
    ClusterRecord clusterHistory_[kMaxClusters];
    uint32_t      clusterHistoryCount_{0};

    // ---- 聚类可视化 ----
    ClusterVisData clusterVisBuf_[kMaxClusters];
    uint32_t       clusterVisCount_{0};
    mutable std::mutex clusterVisMutex_;

    // ---- LiDAR 去重 ----
    uint64_t lastLidarTimestampNs_{0};

    // ---- 关联缓冲区 ----
    int32_t trackMatches_[kMaxTracks];
    int32_t detMatches_[kMaxDetections];
    bool    trackAssigned_[kMaxTracks];
    bool    detAssigned_[kMaxDetections];

    // ---- 相机配置 ----
    CameraConfig camCfg_[2];
    uint32_t     camCfgCount_{0};

    // ---- 配置校验 ----
    bool validate_config_(const TrackerConfig& cfg) const;

    // ---- DBSCAN 聚类 ----
    void dbscan_cluster_(const LidarPoint* lidarPoints,
                         uint32_t pointCount,
                         const YoloBBox* bboxes,
                         uint32_t bboxCount);

    // ---- 时间证据累积 ----
    void persist_clusters_();

    // ---- Bbox 认领评分 ----
    void bbox_claim_(const YoloBBox* bboxes, uint32_t bboxCount, uint32_t nc,
                     ClusterInfo* clusters);

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
