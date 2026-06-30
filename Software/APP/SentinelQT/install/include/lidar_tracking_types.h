#ifndef LIDAR_TRACKING_TYPES_H
#define LIDAR_TRACKING_TYPES_H

#include <cstdint>

/**
 * @enum  TrackState
 * @brief 航迹生命周期状态（5-状态模型）。
 */
enum class TrackState : uint8_t {
    Tentative       = 0,  ///< 待确认，新track试探期
    FusionTracking  = 1,  ///< 视觉雷达融合跟踪（YOLO+LiDAR）
    PureRadarTracking = 2, ///< 纯雷达跟踪（YOLO丢失，LiDAR续命）
    Lost            = 3,  ///< 跟踪丢失中，仅靠Alpha-Beta外推
    Deleted         = 4   ///< 已删除，槽位可回收
};

/**
 * @struct TrackedTarget
 * @brief  单个跟踪目标的完整状态（只读快照，供 Qt UI 和告警回调使用）。
 */
struct TrackedTarget {
    uint32_t id;                ///< 唯一航迹 ID（单调递增）
    uint32_t classId;           ///< COCO 类别 ID
    TrackState state;           ///< 生命周期状态
    float    posX;              ///< 滤波后位置 X（米，LiDAR 坐标系）
    float    posY;              ///< 滤波后位置 Y（米，LiDAR 坐标系）
    float    velX;              ///< 滤波后速度 X（m/s）
    float    velY;              ///< 滤波后速度 Y（m/s）
    float    distanceMeters;    ///< 到雷达原点的欧氏距离（预计算）
    uint64_t lastUpdateNs;      ///< 最近一次观测更新时间戳（CLOCK_MONOTONIC, ns）
    uint64_t firstSeenNs;       ///< 航迹首次创建时间戳
    uint32_t age;               ///< 航迹存在的总帧数
    uint32_t consecutiveHits;   ///< 连续命中帧数
    uint32_t consecutiveMisses; ///< 连续丢失帧数
    float    confidence;        ///< 最近一次检测置信度 [0.0, 1.0]
    float    avgIntensity;      ///< 关联点云的平均反射强度 (0-255)
    uint32_t pointCount;        ///< 关联点云的点数
    uint32_t bboxIdx;           ///< 创建该航迹的 bbox 索引（0xFFFFFFFF=孤儿）
    uint64_t lastWarningNs;     ///< 上次触发告警的时间戳
    bool     warningActive;     ///< 当前是否处于告警状态
};

/**
 * @struct ClusterVisData
 * @brief  聚类可视化数据（供 Web Canvas 渲染）。
 */
struct ClusterVisData {
    float    cx, cy;       ///< 质心（LiDAR 坐标系，米）
    float    radius;       ///< 簇内点到质心的最大距离（米）
    uint32_t pointCount;   ///< 簇内点数
    uint32_t bboxIdx;      ///< 认领该簇的 bbox 索引（0xFFFFFFFF=孤儿）
    bool     isOrphan;     ///< 是否为孤儿簇
};

/**
 * @struct TrackerConfig
 * @brief  跟踪器全部可调参数。
 *
 * 调用 configure_tracker() 时进行合法性校验，不满足则返回 false。
 */
struct TrackerConfig {
    // ---- DBSCAN 聚类 ----
    float    dbscanEpsMeters           = 0.5f;
    uint32_t dbscanMinPoints           = 5;
    float    maxPointDistanceMeters    = 30.0f;
    float    maxClusterDistanceMeters  = 10.0f;
    uint32_t clusterPersistenceFrames  = 2;

    // ---- Alpha-Beta 滤波 ----
    float    alpha                      = 0.45f;
    float    beta                       = 0.2f;
    float    minDtSec                   = 0.001f;
    float    maxDtSec                   = 1.0f;
    float    defaultDtSec               = 0.1f;
    uint32_t minHitsForVelocity         = 2;

    // ---- 过滤 ----
    float    minTrackDistanceMeters      = 0.3f;  // 小于此距离的检测不创建 track（过滤原点噪声）

    // ---- 关联 ----
    float    bboxAssocMaxDistMeters     = 0.75f;
    float    orphanAssocMaxDistMeters   = 0.5f;

    // ---- Bbox 认领 ----
    float    bboxClaimMaxPixelDist      = 100.0f;
    uint32_t minBboxClaimPoints         = 10;     // bbox 只认领点数 >= 此值的簇（过滤噪声）

    // ---- 生命周期 ----
    uint32_t minHitsToConfirm           = 3;
    uint32_t maxTentativeMisses         = 1;
    uint32_t maxFusionMisses            = 2;   // FusionTracking/PureRadar 连续丢失多少帧才→Lost
    uint32_t maxLostFrames              = 20;
    uint32_t maxTracks                  = 50;

    // ---- 告警（迟滞） ----
    float    warningEnterDistMeters     = 3.0f;
    float    warningExitDistMeters      = 3.5f;
    uint32_t minConfirmedAgeForWarning  = 2;
    uint64_t warningCooldownNs          = 2000000000ULL;

    // ---- 可视化 ----
    float    clusterVisOpacity          = 0.30f;
    float    radarRangeMeters           = 10.0f;  // 俯视图显示范围
};

/**
 * @brief 告警回调类型。
 * @param target   触发告警的目标完整状态
 * @param userData 注册时传入的用户数据指针
 */
using TrackingCallback = void (*)(const TrackedTarget& target, void* userData);

#endif // LIDAR_TRACKING_TYPES_H
