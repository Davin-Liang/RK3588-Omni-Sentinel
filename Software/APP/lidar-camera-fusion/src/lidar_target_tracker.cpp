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

bool LidarTargetTracker::update(const FusionResult& /*fusionResult*/,
                                 const LidarPoint* lidarPoints,
                                 uint32_t pointCount,
                                 const YoloBBox* bboxes,
                                 uint32_t bboxCount,
                                 uint64_t timestampNs)
{
    if (!lidarPoints && pointCount > 0) {
        fprintf(stderr, "[LidarTargetTracker] update: null input\n");
        return false;
    }

    // 1. 全局聚类：所有 LiDAR 点 → 簇 → 匹配 bbox 或标记孤儿
    cluster_all_points_(lidarPoints, pointCount, bboxes, bboxCount);

    // 2. 预测：所有活跃航迹向前预测
    predict_tracks_(timestampNs);

    // 3. 关联：检测与航迹匹配
    associate_(timestampNs, bboxes);

    // 4. 生命周期管理
    manage_lifecycle_();

    // 5. 告警检查
    check_warnings_(timestampNs);

    // 6. 更新快照（线程安全）
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

bool LidarTargetTracker::get_bbox_detection_centroid(uint32_t globalBboxIdx,
                                                       float& outX, float& outY) const
{
    for (uint32_t i = 0; i < detectionCount_; ++i) {
        if (detections_[i].bboxIdx == globalBboxIdx && !detections_[i].isOrphan) {
            outX = detections_[i].x;
            outY = detections_[i].y;
            return true;
        }
    }
    return false;
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
// set_camera_configs
// ============================================================================

void LidarTargetTracker::set_camera_configs(const CameraConfig* configs, uint32_t count)
{
    camCfgCount_ = (count > 2) ? 2 : count;
    for (uint32_t i = 0; i < camCfgCount_; ++i)
        camCfg_[i] = configs[i];
}

// ============================================================================
// 1. 全局聚类：全部 LiDAR 点 → CDC → 簇质心投影匹配 bbox
// ============================================================================

void LidarTargetTracker::cluster_all_points_(
    const LidarPoint* lidarPoints,
    uint32_t pointCount,
    const YoloBBox* bboxes,
    uint32_t bboxCount)
{
    detectionCount_ = 0;

    // ---- 1a. 收集有效点 ----
    uint32_t validCount = 0;
    for (uint32_t i = 0; i < pointCount && i < kMaxLidarPoints; ++i) {
        if (lidarPoints[i].x == 0.0f && lidarPoints[i].y == 0.0f) continue;
        float d2 = lidarPoints[i].x * lidarPoints[i].x
                  + lidarPoints[i].y * lidarPoints[i].y;
        if (d2 > 900.0f) continue;
        clusterPointIndices_[validCount] = i;
        clusterPointsX_[validCount]      = lidarPoints[i].x;
        clusterPointsY_[validCount]      = lidarPoints[i].y;
        clusterAssignments_[validCount]  = -1;
        ++validCount;
    }

    if (validCount < config_.minClusterPoints) return;

    // ---- 1b. 按角度排序 ----
    for (uint32_t i = 1; i < validCount; ++i) {
        float kx = clusterPointsX_[i], ky = clusterPointsY_[i];
        uint32_t ki = clusterPointIndices_[i];
        float ka = std::atan2(ky, kx);
        int32_t j = static_cast<int32_t>(i) - 1;
        while (j >= 0 && std::atan2(clusterPointsY_[j], clusterPointsX_[j]) > ka) {
            clusterPointsX_[j + 1]      = clusterPointsX_[j];
            clusterPointsY_[j + 1]      = clusterPointsY_[j];
            clusterPointIndices_[j + 1] = clusterPointIndices_[j];
            --j;
        }
        clusterPointsX_[j + 1]      = kx;
        clusterPointsY_[j + 1]      = ky;
        clusterPointIndices_[j + 1] = ki;
    }

    // ---- 1c. CDC 聚类 ----
    ClusterInfo clusters[32];
    uint32_t nc = 0;
    {
        uint32_t ci = 0;
        clusters[ci].startIdx = 0; clusters[ci].count = 1;
        clusters[ci].sumX = clusterPointsX_[0];
        clusters[ci].sumY = clusterPointsY_[0]; clusters[ci].sumI = 0.0f;
        for (uint32_t i = 1; i < validCount && ci < 32; ++i) {
            float dx = clusterPointsX_[i] - clusterPointsX_[i - 1];
            float dy = clusterPointsY_[i] - clusterPointsY_[i - 1];
            float d2 = dx * dx + dy * dy;
            if (d2 < config_.clusterEpsMeters * config_.clusterEpsMeters) {
                clusters[ci].count++;
                clusters[ci].sumX += clusterPointsX_[i];
                clusters[ci].sumY += clusterPointsY_[i];
            } else {
                ++ci; if (ci >= 32) break;
                clusters[ci].startIdx = i; clusters[ci].count = 1;
                clusters[ci].sumX = clusterPointsX_[i];
                clusters[ci].sumY = clusterPointsY_[i]; clusters[ci].sumI = 0.0f;
            }
        }
        nc = ci + 1; if (nc > 32) nc = 32;
    }

    // Wrap-around
    if (nc >= 2) {
        uint32_t li = clusters[nc - 1].startIdx + clusters[nc - 1].count - 1;
        float dx = clusterPointsX_[li] - clusterPointsX_[0];
        float dy = clusterPointsY_[li] - clusterPointsY_[0];
        if ((dx * dx + dy * dy) < config_.clusterEpsMeters * config_.clusterEpsMeters) {
            clusters[0].startIdx = clusters[nc - 1].startIdx;
            clusters[0].count   += clusters[nc - 1].count;
            clusters[0].sumX    += clusters[nc - 1].sumX;
            clusters[0].sumY    += clusters[nc - 1].sumY;
            --nc;
        }
    }

    // ---- 1d. 选簇 + 投影匹配 bbox（每 bbox 选评分最高簇） ----
    struct BboxCandidate { uint32_t ci; float score; };
    BboxCandidate bboxBest[50];
    // 非孤儿簇计数：<= bboxCount

    for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb) {
        bboxBest[bb].ci = 0xFFFFFFFF;
        bboxBest[bb].score = -1.0f;
    }

    for (uint32_t ci = 0; ci < nc; ++ci) {
        if (clusters[ci].count < config_.minClusterPoints) continue;

        float cx = clusters[ci].sumX / static_cast<float>(clusters[ci].count);
        float cy = clusters[ci].sumY / static_cast<float>(clusters[ci].count);
        float dist = std::sqrt(cx * cx + cy * cy);
        if (dist > 2.5f) continue;

        float score = static_cast<float>(clusters[ci].count) * (2.5f / dist);

        // 投影质心到各相机，匹配 bbox
        int32_t  bestBb = -1;
        float    bestBbD2 = 1e9f;

        for (uint32_t cc = 0; cc < camCfgCount_; ++cc) {
            const CameraConfig& cfg = camCfg_[cc];
            float cX = cfg.tLidarToCam[0] * cx + cfg.tLidarToCam[1] * cy
                     + cfg.tLidarToCam[3];
            float cZ = cfg.tLidarToCam[8] * cx + cfg.tLidarToCam[9] * cy
                     + cfg.tLidarToCam[11];
            if (cZ <= 0.0f) continue;

            float u = cfg.fx * cX / cZ + cfg.cx;
            float v = cfg.fy * cfg.tLidarToCam[7] / cZ + cfg.cy;

            for (uint32_t bb = 0; bb < bboxCount; ++bb) {
                if (u < static_cast<float>(bboxes[bb].x1)) continue;
                if (u >= static_cast<float>(bboxes[bb].x2)) continue;
                if (v < static_cast<float>(bboxes[bb].y1)) continue;
                if (v >= static_cast<float>(bboxes[bb].y2)) continue;

                float bcx = (bboxes[bb].x1 + bboxes[bb].x2) * 0.5f;
                float bcy = (bboxes[bb].y1 + bboxes[bb].y2) * 0.5f;
                float d2 = (u - bcx) * (u - bcx) + (v - bcy) * (v - bcy);
                if (d2 < bestBbD2) {
                    bestBbD2 = d2;
                    bestBb = static_cast<int32_t>(bb);
                }
            }
        }

        if (bestBb >= 0 && score > bboxBest[bestBb].score) {
            bboxBest[bestBb].ci = ci;
            bboxBest[bestBb].score = score;
        }
    }

    // 输出 bbox 检测 + 孤儿检测
    bool ciUsed[32] = {};
    for (uint32_t bb = 0; bb < bboxCount; ++bb) {
        if (bboxBest[bb].ci == 0xFFFFFFFF) continue;
        if (detectionCount_ >= kMaxDetections) break;
        uint32_t ci = bboxBest[bb].ci;
        ciUsed[ci] = true;

        float cx = clusters[ci].sumX / static_cast<float>(clusters[ci].count);
        float cy = clusters[ci].sumY / static_cast<float>(clusters[ci].count);

        DetectionCandidate& det = detections_[detectionCount_];
        det.x = cx; det.y = cy;
        det.classId     = bboxes[bb].classId;
        det.confidence  = bboxes[bb].confidence;
        det.avgIntensity = 0.0f;
        det.pointCount  = clusters[ci].count;
        det.bboxIdx     = bb;
        det.isOrphan    = false;
        ++detectionCount_;
    }

    // 未被 bbox 选中的簇 → 孤儿
    for (uint32_t ci = 0; ci < nc; ++ci) {
        if (ciUsed[ci]) continue;
        if (clusters[ci].count < config_.minClusterPoints) continue;
        if (detectionCount_ >= kMaxDetections) break;

        float cx = clusters[ci].sumX / static_cast<float>(clusters[ci].count);
        float cy = clusters[ci].sumY / static_cast<float>(clusters[ci].count);
        if (std::sqrt(cx * cx + cy * cy) > 2.5f) continue;

        DetectionCandidate& det = detections_[detectionCount_];
        det.x = cx; det.y = cy;
        det.classId     = 0;
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

            // 孤儿检测（纯 LiDAR）只能匹配 Coasting 的已确认航迹
            if (det.isOrphan) {
                if (track.state != TrackState::Coasting
                    || track.consecutiveHits < config_.minHitsToConfirm) {
                    continue;
                }
            }

            float dx = det.x - predX_[i];
            float dy = det.y - predY_[i];
            float d2 = dx * dx + dy * dy;

            // bbox 检测：跨 bbox 匹配时门限缩到 25%
            if (!det.isOrphan && track.bboxIdx != 0xFFFFFFFF
                && track.bboxIdx != det.bboxIdx) {
                if (d2 > gateDist2 * 0.25f) continue;
            }

            // 孤儿检测使用独立门限
            if (det.isOrphan) {
                float g2 = config_.maxOrphanAssocDistMeters
                         * config_.maxOrphanAssocDistMeters;
                if (d2 > g2) continue;
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
            track.bboxIdx      = det.bboxIdx;
            track.confidence   = det.confidence;
            track.avgIntensity = det.avgIntensity;
            track.pointCount   = det.pointCount;
            track.distanceMeters = std::sqrt(track.posX * track.posX
                                            + track.posY * track.posY);

            // 仅 bbox 检测能恢复 Coasting → Confirmed，孤儿检测保持 Coasting
            if (track.state == TrackState::Coasting && !det.isOrphan) {
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
        if (detections_[j].isOrphan) continue;   // 孤儿检测不创建航迹

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
        workingTracks_[si].bboxIdx         = det.bboxIdx;
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
