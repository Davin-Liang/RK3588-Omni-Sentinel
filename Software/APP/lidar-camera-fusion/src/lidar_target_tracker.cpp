#include "lidar_target_tracker.h"

#include <cmath>
#include <cstdio>
#include <cstring>

// ============================================================================
// 构造 / 析构
// ============================================================================

LidarTargetTracker::LidarTargetTracker()
{
    std::memset(workingTracks_, 0, sizeof(workingTracks_));
    std::memset(snapshotTracks_, 0, sizeof(snapshotTracks_));
    std::memset(predX_, 0, sizeof(predX_));
    std::memset(predY_, 0, sizeof(predY_));
    std::memset(predVX_, 0, sizeof(predVX_));
    std::memset(predVY_, 0, sizeof(predVY_));
    std::memset(detections_, 0, sizeof(detections_));
    std::memset(clusterPointsX_, 0, sizeof(clusterPointsX_));
    std::memset(clusterPointsY_, 0, sizeof(clusterPointsY_));
    std::memset(clusterPointIndices_, 0, sizeof(clusterPointIndices_));
    std::memset(clusterAssignments_, 0, sizeof(clusterAssignments_));
}

LidarTargetTracker::~LidarTargetTracker()
{
    // 所有缓冲区为栈数组，无需手动释放
}

// ============================================================================
// 配置与回调
// ============================================================================

bool LidarTargetTracker::configure(const TrackerConfig& config)
{
    if (!validate_config_(config)) {
        fprintf(stderr, "[LidarTargetTracker] configure: invalid config\n");
        return false;
    }
    config_ = config;
    return true;
}

void LidarTargetTracker::reset()
{
    std::memset(workingTracks_, 0, sizeof(workingTracks_));
    std::memset(snapshotTracks_, 0, sizeof(snapshotTracks_));
    std::memset(predX_, 0, sizeof(predX_));
    std::memset(predY_, 0, sizeof(predY_));
    std::memset(predVX_, 0, sizeof(predVX_));
    std::memset(predVY_, 0, sizeof(predVY_));
    activeTrackCount_   = 0;
    snapshotTrackCount_ = 0;
    nextTrackId_        = 1;
    detectionCount_     = 0;
}

void LidarTargetTracker::register_callback(TrackingCallback cb, void* userData)
{
    warningCb_       = cb;
    warningUserData_ = userData;
}

// ============================================================================
// 主入口
// ============================================================================

bool LidarTargetTracker::update(const FusionResult& fusionResult,
                                 const LidarPoint* lidarPoints,
                                 uint32_t pointCount,
                                 const YoloBBox* bboxes,
                                 uint32_t bboxCount,
                                 uint64_t timestampNs)
{
    if ((!lidarPoints && pointCount > 0) || (!bboxes && bboxCount > 0)) {
        fprintf(stderr, "[LidarTargetTracker] update: null input\n");
        return false;
    }
    if (pointCount == 0 && bboxCount == 0) {
        return true;  // 空输入，静默成功
    }

    // 数量不一致时以 FusionResult 为准
    if (bboxCount != fusionResult.bboxCount) {
        fprintf(stderr, "[LidarTargetTracker] WARNING: bboxCount(%u) != "
                "fusionResult.bboxCount(%u)\n", bboxCount, fusionResult.bboxCount);
    }
    uint32_t effectiveBboxCount = fusionResult.bboxCount;
    if (effectiveBboxCount > bboxCount) {
        effectiveBboxCount = bboxCount;
    }

    // 1. 聚类：bbox 内点 → 检测
    cluster_bbox_points_(fusionResult, lidarPoints, pointCount,
                         bboxes, bboxCount);

    // 2. 聚类：孤儿点（未被 bbox 认领的 LiDAR 点）→ 补充检测
    cluster_orphan_points_(fusionResult, lidarPoints, pointCount);

    // 4. 预测：所有活跃航迹向前预测
    predict_tracks_(timestampNs);

    // 5. 关联：检测与航迹匹配
    associate_(timestampNs, bboxes);

    // 6. 生命周期管理
    manage_lifecycle_();

    // 7. 告警检查
    check_warnings_(timestampNs);

    // 8. 更新快照（线程安全）
    update_snapshot_();

    return true;
}

// ============================================================================
// 快照拷贝
// ============================================================================

bool LidarTargetTracker::copy_snapshot(TrackedTarget* out, uint32_t maxCount,
                                        uint32_t* outCount) const
{
    if (!out || !outCount) {
        return false;
    }

    std::lock_guard<std::mutex> lock(snapshotMutex_);
    uint32_t count = snapshotTrackCount_;
    if (count > maxCount) {
        count = maxCount;
    }
    std::memcpy(out, snapshotTracks_, count * sizeof(TrackedTarget));
    *outCount = count;
    return true;
}

// ============================================================================
// 配置校验
// ============================================================================

bool LidarTargetTracker::validate_config_(const TrackerConfig& cfg) const
{
    if (cfg.alpha < 0.0f || cfg.alpha > 1.0f)       return false;
    if (cfg.beta  < 0.0f || cfg.beta  > 1.0f)       return false;
    if (cfg.minDtSec     <= 0.0f)                    return false;
    if (cfg.defaultDtSec <  cfg.minDtSec)            return false;
    if (cfg.maxDtSec     <= cfg.minDtSec)            return false;
    if (cfg.clusterEpsMeters <= 0.0f)                return false;
    if (cfg.minClusterPoints < 1)                    return false;
    if (cfg.maxAssociationDistMeters <= 0.0f)        return false;
    if (cfg.maxTracks > kMaxTracks)                  return false;
    if (cfg.warningEnterDistMeters <= 0.0f)          return false;
    if (cfg.warningExitDistMeters <= cfg.warningEnterDistMeters) return false;
    if (cfg.warningCooldownNs == 0)                  return false;
    if (cfg.minHitsToConfirm < 1)                    return false;
    if (cfg.maxCoastingFrames < 1)                   return false;
    if (cfg.maxStaleCoastingFrames > cfg.maxCoastingFrames) return false;
    return true;
}

// ============================================================================
// 1. 聚类：扫描顺序 CDC + wrap-around + 评分选簇
// ============================================================================

void LidarTargetTracker::cluster_bbox_points_(
    const FusionResult& fusionResult,
    const LidarPoint* lidarPoints,
    uint32_t pointCount,
    const YoloBBox* bboxes,
    uint32_t bboxCount)
{
    detectionCount_ = 0;

    uint32_t offset = 0;
    for (uint32_t b = 0; b < fusionResult.bboxCount; ++b) {
        uint32_t count = fusionResult.bboxPointCounts[b];
        if (count < config_.minClusterPoints) {
            offset += count;
            continue;
        }

        // 提取点坐标（保持扫描索引顺序），同时做边界检查
        uint32_t validCount = 0;
        for (uint32_t j = 0; j < count; ++j) {
            uint32_t idx = fusionResult.bboxPointIndices[offset + j];
            if (idx >= pointCount) {
                fprintf(stderr, "[LidarTargetTracker] point index %u out of "
                        "range (max %u)\n", idx, pointCount);
                continue;
            }
            clusterPointIndices_[validCount] = idx;
            clusterPointsX_[validCount]      = lidarPoints[idx].x;
            clusterPointsY_[validCount]      = lidarPoints[idx].y;
            clusterAssignments_[validCount]  = -1;
            ++validCount;
        }
        offset += count;

        if (validCount < config_.minClusterPoints) {
            continue;
        }

        // 线性扫描聚类（已按扫描顺序排列，无需排序）
        uint32_t    clusterCount = 0;
        ClusterInfo clusters[32]; // 一个 bbox 最多 32 个簇

        {
            uint32_t ci = 0;
            clusters[ci].startIdx = 0;
            clusters[ci].count    = 1;
            clusters[ci].sumX     = clusterPointsX_[0];
            clusters[ci].sumY     = clusterPointsY_[0];
            clusters[ci].sumI     = 0.0f;

            for (uint32_t i = 1; i < validCount; ++i) {
                float dx = clusterPointsX_[i] - clusterPointsX_[i - 1];
                float dy = clusterPointsY_[i] - clusterPointsY_[i - 1];
                float dist2 = dx * dx + dy * dy;

                if (dist2 < config_.clusterEpsMeters * config_.clusterEpsMeters) {
                    clusters[ci].count++;
                    clusters[ci].sumX += clusterPointsX_[i];
                    clusters[ci].sumY += clusterPointsY_[i];
                    clusterAssignments_[i] = static_cast<int32_t>(ci);
                } else {
                    ++ci;
                    if (ci >= 32) break;
                    clusters[ci].startIdx = i;
                    clusters[ci].count    = 1;
                    clusters[ci].sumX     = clusterPointsX_[i];
                    clusters[ci].sumY     = clusterPointsY_[i];
                    clusters[ci].sumI     = 0.0f;
                }
            }
            clusterCount = ci + 1;
            if (clusterCount > 32) clusterCount = 32;
        }

        // Wrap-around 检查：用最后一个簇的最后一个点 与 第一个簇的第一个点 的距离判断
        if (clusterCount >= 2) {
            uint32_t lastIdx  = clusters[clusterCount - 1].startIdx
                                + clusters[clusterCount - 1].count - 1;
            uint32_t firstIdx = clusters[0].startIdx;
            float dx = clusterPointsX_[lastIdx] - clusterPointsX_[firstIdx];
            float dy = clusterPointsY_[lastIdx] - clusterPointsY_[firstIdx];
            float d2 = dx * dx + dy * dy;
            if (d2 < config_.clusterEpsMeters * config_.clusterEpsMeters) {
                // 合并首尾簇
                clusters[0].startIdx = clusters[clusterCount - 1].startIdx;
                clusters[0].count   += clusters[clusterCount - 1].count;
                clusters[0].sumX    += clusters[clusterCount - 1].sumX;
                clusters[0].sumY    += clusters[clusterCount - 1].sumY;
                --clusterCount;
            }
        }

        // 过滤小簇 + 评分选最优
        int32_t bestCluster = -1;
        float   bestScore   = -1.0f;
        int32_t largestCluster = -1;
        uint32_t largestCount   = 0;

        for (uint32_t ci = 0; ci < clusterCount; ++ci) {
            if (clusters[ci].count < config_.minClusterPoints) continue;

            if (clusters[ci].count > largestCount) {
                largestCount   = clusters[ci].count;
                largestCluster = static_cast<int32_t>(ci);
            }

            float cx = clusters[ci].sumX / static_cast<float>(clusters[ci].count);
            float cy = clusters[ci].sumY / static_cast<float>(clusters[ci].count);
            float dist = std::sqrt(cx * cx + cy * cy);
            if (dist > 2.5f) continue;   // 忽略 2.5m 外的簇（排除远处工位/墙壁）
            float distanceBonus = 1.0f - (dist / 50.0f);
            if (distanceBonus < 0.0f) distanceBonus = 0.0f;
            if (distanceBonus > 1.0f) distanceBonus = 1.0f;

            float score = static_cast<float>(clusters[ci].count) * 1.0f
                         + distanceBonus * 1.0f;

            if (score > bestScore) {
                bestScore   = score;
                bestCluster = static_cast<int32_t>(ci);
            }
        }

        // 小簇过滤：最高分簇点数明显少于最大簇 → 选最大簇
        if (bestCluster >= 0 && largestCluster >= 0
            && bestCluster != largestCluster
            && static_cast<float>(clusters[bestCluster].count)
               < static_cast<float>(largestCount) * 0.6f)
        {
            bestCluster = largestCluster;
        }

        if (bestCluster < 0) continue;

        // 输出观测
        if (detectionCount_ < kMaxDetections) {
            DetectionCandidate& det = detections_[detectionCount_];
            float cx = clusters[bestCluster].sumX
                      / static_cast<float>(clusters[bestCluster].count);
            float cy = clusters[bestCluster].sumY
                      / static_cast<float>(clusters[bestCluster].count);
            det.x           = cx;
            det.y           = cy;
            det.classId     = (b < bboxCount) ? bboxes[b].classId : 0;
            det.confidence  = (b < bboxCount) ? bboxes[b].confidence : 0.0f;
            det.avgIntensity = 0.0f;
            det.pointCount  = clusters[bestCluster].count;
            det.bboxIdx     = b;
            det.isOrphan    = false;
            ++detectionCount_;
        }
    }
}

// ============================================================================
// 1b. 孤儿点聚类：未被 bbox 认领的 LiDAR 点 → 补充检测
// ============================================================================

void LidarTargetTracker::cluster_orphan_points_(
    const FusionResult& fusionResult,
    const LidarPoint* lidarPoints,
    uint32_t pointCount)
{
    // 标记所有已被 bbox 认领的点
    std::memset(pointAssigned_, 0, sizeof(pointAssigned_));
    uint32_t totalAssigned = 0;
    for (uint32_t b = 0; b < fusionResult.bboxCount; ++b) {
        uint32_t count = fusionResult.bboxPointCounts[b];
        // offset 需要累加，但 FusionResult 不提供 per-bbox offset
        // bboxPointIndices 是展平数组，从第 0 个点开始
        // 无法直接获取每个 bbox 的起始偏移，需要重新累加
        // 实际上 cluster_bbox_points_ 已经循环过一次了，
        // 这里我们用更简单的方法：标记所有出现在 bboxPointIndices 中的点
    }
    // 重新累加 offset 来标记
    uint32_t offset = 0;
    for (uint32_t b = 0; b < fusionResult.bboxCount; ++b) {
        uint32_t count = fusionResult.bboxPointCounts[b];
        for (uint32_t j = 0; j < count; ++j) {
            uint32_t idx = fusionResult.bboxPointIndices[offset + j];
            if (idx < pointCount) {
                pointAssigned_[idx] = true;
                ++totalAssigned;
            }
        }
        offset += count;
    }

    if (pointCount <= totalAssigned) return; // 没有孤儿点

    // 收集未认领的有效点
    uint32_t orphanCount = 0;
    for (uint32_t i = 0; i < pointCount && i < kMaxLidarPoints; ++i) {
        if (pointAssigned_[i]) continue;
        if (lidarPoints[i].x == 0.0f && lidarPoints[i].y == 0.0f) continue;
        // 距离过滤：忽略太远的点（> 30m）
        float d2 = lidarPoints[i].x * lidarPoints[i].x
                  + lidarPoints[i].y * lidarPoints[i].y;
        if (d2 > 900.0f) continue; // 30m

        clusterPointIndices_[orphanCount] = i;
        clusterPointsX_[orphanCount]      = lidarPoints[i].x;
        clusterPointsY_[orphanCount]      = lidarPoints[i].y;
        clusterAssignments_[orphanCount]  = -1;
        ++orphanCount;
    }

    if (orphanCount < config_.minClusterPoints) return;

    // 按角度排序（孤儿点可能不连续，需要排序）
    // 使用简单的插入排序（点数量少）
    for (uint32_t i = 1; i < orphanCount; ++i) {
        float keyX = clusterPointsX_[i];
        float keyY = clusterPointsY_[i];
        uint32_t keyIdx = clusterPointIndices_[i];
        float keyAngle = std::atan2(keyY, keyX);
        int32_t j = static_cast<int32_t>(i) - 1;
        while (j >= 0 && std::atan2(clusterPointsY_[j], clusterPointsX_[j]) > keyAngle) {
            clusterPointsX_[j + 1]      = clusterPointsX_[j];
            clusterPointsY_[j + 1]      = clusterPointsY_[j];
            clusterPointIndices_[j + 1] = clusterPointIndices_[j];
            --j;
        }
        clusterPointsX_[j + 1]      = keyX;
        clusterPointsY_[j + 1]      = keyY;
        clusterPointIndices_[j + 1] = keyIdx;
    }

    // CDC 聚类（与 bbox 聚类相同的算法）
    ClusterInfo clusters[32];
    uint32_t clusterCount = 0;
    {
        uint32_t ci = 0;
        clusters[ci].startIdx = 0;
        clusters[ci].count    = 1;
        clusters[ci].sumX     = clusterPointsX_[0];
        clusters[ci].sumY     = clusterPointsY_[0];
        clusters[ci].sumI     = 0.0f;

        for (uint32_t i = 1; i < orphanCount && ci < 32; ++i) {
            float dx = clusterPointsX_[i] - clusterPointsX_[i - 1];
            float dy = clusterPointsY_[i] - clusterPointsY_[i - 1];
            float d2 = dx * dx + dy * dy;

            if (d2 < config_.clusterEpsMeters * config_.clusterEpsMeters) {
                clusters[ci].count++;
                clusters[ci].sumX += clusterPointsX_[i];
                clusters[ci].sumY += clusterPointsY_[i];
            } else {
                ++ci;
                if (ci >= 32) break;
                clusters[ci].startIdx = i;
                clusters[ci].count    = 1;
                clusters[ci].sumX     = clusterPointsX_[i];
                clusters[ci].sumY     = clusterPointsY_[i];
                clusters[ci].sumI     = 0.0f;
            }
        }
        clusterCount = ci + 1;
        if (clusterCount > 32) clusterCount = 32;
    }

    // Wrap-around 检查
    if (clusterCount >= 2) {
        uint32_t lastIdx  = clusters[clusterCount - 1].startIdx
                            + clusters[clusterCount - 1].count - 1;
        uint32_t firstIdx = clusters[0].startIdx;
        float dx = clusterPointsX_[lastIdx] - clusterPointsX_[firstIdx];
        float dy = clusterPointsY_[lastIdx] - clusterPointsY_[firstIdx];
        if ((dx * dx + dy * dy) < config_.clusterEpsMeters * config_.clusterEpsMeters) {
            clusters[0].startIdx = clusters[clusterCount - 1].startIdx;
            clusters[0].count   += clusters[clusterCount - 1].count;
            clusters[0].sumX    += clusters[clusterCount - 1].sumX;
            clusters[0].sumY    += clusters[clusterCount - 1].sumY;
            --clusterCount;
        }
    }

    // 输出有效簇作为孤儿检测
    for (uint32_t ci = 0; ci < clusterCount; ++ci) {
        if (clusters[ci].count < config_.minClusterPoints) continue;
        if (detectionCount_ >= kMaxDetections) break;

        float cx = clusters[ci].sumX / static_cast<float>(clusters[ci].count);
        float cy = clusters[ci].sumY / static_cast<float>(clusters[ci].count);
        if (std::sqrt(cx * cx + cy * cy) > 2.5f) continue;   // 忽略远处簇

        DetectionCandidate& det = detections_[detectionCount_];
        det.x           = cx;
        det.y           = cy;
        det.classId     = 0;   // 孤儿点无类别信息
        det.confidence  = 0.5f;
        det.avgIntensity = 0.0f;
        det.pointCount  = clusters[ci].count;
        det.bboxIdx     = 0xFFFFFFFF;
        det.isOrphan    = true;
        ++detectionCount_;
    }
}

// ============================================================================
// 2. Alpha-Beta 预测
// ============================================================================

void LidarTargetTracker::predict_tracks_(uint64_t timestampNs)
{
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        if (workingTracks_[i].state == TrackState::Deleted) continue;

        float dt = static_cast<float>(timestampNs - workingTracks_[i].lastUpdateNs)
                   * 1e-9f;

        if (dt <= config_.minDtSec) {
            dt = config_.defaultDtSec;
        }
        if (dt > config_.maxDtSec) {
            dt = 0.0f;
        }

        if (dt > 0.0f && workingTracks_[i].age > 0) {
            predX_[i]  = workingTracks_[i].posX + workingTracks_[i].velX * dt;
            predY_[i]  = workingTracks_[i].posY + workingTracks_[i].velY * dt;
            predVX_[i] = workingTracks_[i].velX;
            predVY_[i] = workingTracks_[i].velY;
        } else {
            predX_[i]  = workingTracks_[i].posX;
            predY_[i]  = workingTracks_[i].posY;
            predVX_[i] = workingTracks_[i].velX;
            predVY_[i] = workingTracks_[i].velY;
        }
    }
}

// ============================================================================
// 3. 贪心最近邻关联（classId 门控）
// ============================================================================

void LidarTargetTracker::associate_(uint64_t timestampNs, const YoloBBox* /*bboxes*/)
{
    // 重置关联状态
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        trackMatches_[i]  = -1;
        trackAssigned_[i] = false;
    }
    for (uint32_t j = 0; j < detectionCount_; ++j) {
        detMatches_[j]  = -1;
        detAssigned_[j] = false;
    }

    // 对每个检测，找最近的未匹配活跃航迹
    float gateDist2 = config_.maxAssociationDistMeters * config_.maxAssociationDistMeters;

    for (uint32_t j = 0; j < detectionCount_; ++j) {
        const DetectionCandidate& det = detections_[j];
        int32_t bestTrack = -1;
        float   bestDist2 = gateDist2;

        for (uint32_t i = 0; i < activeTrackCount_; ++i) {
            TrackedTarget& track = workingTracks_[i];
            if (track.state == TrackState::Deleted) continue;
            if (trackAssigned_[i]) continue;

            // classId 门控
            if (config_.requireClassIdMatch && track.classId != det.classId) {
                continue;
            }

            // 孤儿检测（纯 LiDAR）只能匹配 coasting 的 Confirmed 航迹
            if (det.isOrphan) {
                if (track.state != TrackState::Coasting
                    || track.consecutiveHits < config_.minHitsToConfirm) {
                    continue;
                }
            }

            float dx = det.x - predX_[i];
            float dy = det.y - predY_[i];
            float d2 = dx * dx + dy * dy;

            // 孤儿检测使用 1m 门限 (d2 = 1.0)
            if (det.isOrphan && d2 > 1.0f) {
                continue;
            }

            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestTrack = static_cast<int32_t>(i);
            }
        }

        if (bestTrack >= 0) {
            trackMatches_[bestTrack] = static_cast<int32_t>(j);
            detMatches_[j]           = bestTrack;
            trackAssigned_[bestTrack] = true;
            detAssigned_[j]           = true;
        }
    }

    // 匹配成功 → 校正
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state == TrackState::Deleted) continue;

        if (trackAssigned_[i] && trackMatches_[i] >= 0) {
            uint32_t j = static_cast<uint32_t>(trackMatches_[i]);
            const DetectionCandidate& det = detections_[j];

            float dt = static_cast<float>(timestampNs - track.lastUpdateNs)
                       * 1e-9f;
            if (dt <= config_.minDtSec)  dt = config_.defaultDtSec;
            if (dt >  config_.maxDtSec)  dt = 0.0f;

            apply_correction_(i, det, dt);
            track.consecutiveMisses = 0;
            track.age++;
            // consecutiveHits 在 apply_correction_ 中递增
            track.lastUpdateNs = timestampNs;
            track.classId      = det.classId;
            track.confidence   = det.confidence;
            track.avgIntensity = det.avgIntensity;
            track.pointCount   = det.pointCount;
            track.distanceMeters = std::sqrt(track.posX * track.posX
                                            + track.posY * track.posY);

            if (track.state == TrackState::Coasting) {
                track.state = TrackState::Confirmed;
            }
        } else {
            // 未匹配：递增丢失计数
            track.consecutiveMisses++;
            track.age++;
        }
    }

    // 未匹配检测 → 创建新 Tentative 航迹（孤儿检测不创建航迹）
    for (uint32_t j = 0; j < detectionCount_; ++j) {
        if (detAssigned_[j]) continue;
        if (detections_[j].isOrphan) continue;   // 孤儿检测只续命，不新建

        const DetectionCandidate& det = detections_[j];

        // 线性扫描找空闲槽位
        int32_t slot = -1;
        for (uint32_t i = 0; i < activeTrackCount_; ++i) {
            if (workingTracks_[i].state == TrackState::Deleted) {
                slot = static_cast<int32_t>(i);
                break;
            }
        }
        if (slot < 0 && activeTrackCount_ < config_.maxTracks) {
            slot = static_cast<int32_t>(activeTrackCount_++);
        }
        if (slot < 0) {
            fprintf(stderr, "[LidarTargetTracker] track slots exhausted\n");
            continue;
        }

        uint32_t si = static_cast<uint32_t>(slot);
        std::memset(&workingTracks_[si], 0, sizeof(TrackedTarget));
        workingTracks_[si].id          = nextTrackId_++;
        workingTracks_[si].classId     = det.classId;
        workingTracks_[si].state       = TrackState::Tentative;
        workingTracks_[si].posX        = det.x;
        workingTracks_[si].posY        = det.y;
        workingTracks_[si].velX        = 0.0f;
        workingTracks_[si].velY        = 0.0f;
        workingTracks_[si].distanceMeters = std::sqrt(det.x * det.x + det.y * det.y);
        workingTracks_[si].lastUpdateNs   = timestampNs;
        workingTracks_[si].firstSeenNs    = timestampNs;
        workingTracks_[si].age            = 1;
        workingTracks_[si].consecutiveHits   = 1;
        workingTracks_[si].consecutiveMisses = 0;
        workingTracks_[si].confidence    = det.confidence;
        workingTracks_[si].avgIntensity  = det.avgIntensity;
        workingTracks_[si].pointCount    = det.pointCount;
        workingTracks_[si].warningActive = false;
        workingTracks_[si].lastWarningNs = 0;

        predX_[si]  = det.x;
        predY_[si]  = det.y;
        predVX_[si] = 0.0f;
        predVY_[si] = 0.0f;
    }
}

// ============================================================================
// 校正：Alpha-Beta 位置 + 速度更新
// ============================================================================

void LidarTargetTracker::apply_correction_(uint32_t trackIdx,
                                            const DetectionCandidate& det,
                                            float dt)
{
    TrackedTarget& track = workingTracks_[trackIdx];

    track.consecutiveHits++;  // 先递增，再判断速度更新条件

    float rx = det.x - predX_[trackIdx];
    float ry = det.y - predY_[trackIdx];

    track.posX = predX_[trackIdx] + config_.alpha * rx;
    track.posY = predY_[trackIdx] + config_.alpha * ry;

    if (dt > config_.minDtSec
        && track.consecutiveHits >= config_.minHitsForVelocity) {
        track.velX = predVX_[trackIdx] + (config_.beta / dt) * rx;
        track.velY = predVY_[trackIdx] + (config_.beta / dt) * ry;
    }
    // 否则速度保持为 0（新航迹不急于估计速度）
}

// ============================================================================
// 4. 生命周期管理
// ============================================================================

void LidarTargetTracker::manage_lifecycle_()
{
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state == TrackState::Deleted) continue;

        switch (track.state) {
        case TrackState::Tentative:
            if (track.consecutiveMisses > config_.maxTentativeMisses) {
                track.state = TrackState::Deleted;
            } else if (track.consecutiveHits >= config_.minHitsToConfirm) {
                track.state = TrackState::Confirmed;
            }
            break;

        case TrackState::Confirmed:
            if (track.consecutiveMisses > 0) {
                track.state = TrackState::Coasting;
            }
            break;

        case TrackState::Coasting:
            if (track.consecutiveMisses >= config_.maxCoastingFrames) {
                track.state = TrackState::Deleted;
            }
            if (track.consecutiveMisses >= config_.maxStaleCoastingFrames
                && track.age > config_.minHitsToConfirm) {
                track.state = TrackState::Deleted;
            }
            break;

        case TrackState::Deleted:
        default:
            break;
        }
    }
}

// ============================================================================
// 5. 告警检查（迟滞 + 冷却）
// ============================================================================

void LidarTargetTracker::check_warnings_(uint64_t nowNs)
{
    if (!warningCb_) return;

    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state != TrackState::Confirmed) continue;
        if (track.age < config_.minConfirmedAgeForWarning) continue;

        float dist = track.distanceMeters;

        if (!track.warningActive) {
            if (dist < config_.warningEnterDistMeters) {
                track.warningActive = true;
                track.lastWarningNs = nowNs;
                warningCb_(track, warningUserData_);
            }
        } else {
            if (dist > config_.warningExitDistMeters) {
                track.warningActive = false;
            } else if ((nowNs - track.lastWarningNs) > config_.warningCooldownNs) {
                track.lastWarningNs = nowNs;
                warningCb_(track, warningUserData_);
            }
        }
    }
}

// ============================================================================
// 6. 快照拷贝（加锁）
// ============================================================================

void LidarTargetTracker::update_snapshot_()
{
    std::lock_guard<std::mutex> lock(snapshotMutex_);
    uint32_t count = 0;
    for (uint32_t i = 0; i < activeTrackCount_ && count < kMaxTracks; ++i) {
        if (workingTracks_[i].state != TrackState::Deleted) {
            snapshotTracks_[count] = workingTracks_[i];
            ++count;
        }
    }
    snapshotTrackCount_ = count;
}
