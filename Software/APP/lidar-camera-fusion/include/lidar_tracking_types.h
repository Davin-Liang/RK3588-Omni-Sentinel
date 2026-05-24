#ifndef LIDAR_TRACKING_TYPES_H
#define LIDAR_TRACKING_TYPES_H

#include <cstdint>

/**
 * @enum  TrackState
 * @brief 航迹生命周期状态。
 */
enum class TrackState : uint8_t {
    Tentative = 0,  ///< 新建未确认，连续命中足够帧数后升级
    Confirmed = 1,  ///< 已确认，正常跟踪中
    Coasting  = 2,  ///< 短暂丢失，仅靠预测外推
    Deleted   = 3   ///< 已删除，槽位可回收
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
    uint64_t lastWarningNs;     ///< 上次触发告警的时间戳
    bool     warningActive;     ///< 当前是否处于告警状态
};

/**
 * @struct TrackerConfig
 * @brief  跟踪器全部可调参数。
 *
 * 调用 configure_tracker() 时进行合法性校验，不满足则返回 false。
 */
struct TrackerConfig {
    // ---- 聚类 ----
    float    clusterEpsMeters           = 0.5f;
    uint32_t minClusterPoints           = 3;

    // ---- Alpha-Beta 滤波 ----
    float    alpha                      = 0.7f;
    float    beta                       = 0.3f;
    float    minDtSec                   = 0.001f;
    float    maxDtSec                   = 1.0f;
    float    defaultDtSec               = 0.1f;
    uint32_t minHitsForVelocity         = 2;

    // ---- 关联 ----
    float    maxAssociationDistMeters   = 2.0f;
    bool     requireClassIdMatch        = true;

    // ---- 生命周期 ----
    uint32_t minHitsToConfirm           = 3;
    uint32_t maxTentativeMisses         = 1;
    uint32_t maxCoastingFrames          = 5;
    uint32_t maxStaleCoastingFrames     = 2;
    uint32_t maxTracks                  = 50;

    // ---- 告警（迟滞） ----
    float    warningEnterDistMeters     = 3.0f;
    float    warningExitDistMeters      = 3.5f;
    uint32_t minConfirmedAgeForWarning  = 2;
    uint64_t warningCooldownNs          = 2000000000ULL;
};

/**
 * @brief 告警回调类型。
 * @param target   触发告警的目标完整状态
 * @param userData 注册时传入的用户数据指针
 */
using TrackingCallback = void (*)(const TrackedTarget& target, void* userData);

#endif // LIDAR_TRACKING_TYPES_H
