#include "lidar_target_tracker.h"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>

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
    std::memset(seedStack_, 0, sizeof(seedStack_));
    std::memset(clusterCentroidX_, 0, sizeof(clusterCentroidX_));
    std::memset(clusterCentroidY_, 0, sizeof(clusterCentroidY_));
    std::memset(clusterRadii_, 0, sizeof(clusterRadii_));
    std::memset(clusterPointCounts_, 0, sizeof(clusterPointCounts_));
    std::memset(clusterBboxMatch_, 0, sizeof(clusterBboxMatch_));
    std::memset(clusterScore_, 0, sizeof(clusterScore_));
    std::memset(claimedByBbox_, 0, sizeof(claimedByBbox_));
    std::memset(bboxCluster_, 0xFF, sizeof(bboxCluster_));
    std::memset(clusterHistory_, 0, sizeof(clusterHistory_));
    std::memset(clusterVisBuf_, 0, sizeof(clusterVisBuf_));
}

LidarTargetTracker::~LidarTargetTracker() {}

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
    std::memset(clusterHistory_, 0, sizeof(clusterHistory_));
    activeTrackCount_   = 0;
    snapshotTrackCount_ = 0;
    nextTrackId_        = 1;
    detectionCount_     = 0;
    clusterHistoryCount_ = 0;
    rawClusterCount_    = 0;
    lastLidarTimestampNs_ = 0;
}

void LidarTargetTracker::register_callback(TrackingCallback cb, void* userData)
{
    warningCb_       = cb;
    warningUserData_ = userData;
}

bool LidarTargetTracker::update(const FusionResult&,
                                 const LidarPoint* lidarPoints,
                                 uint32_t pointCount,
                                 const YoloBBox* bboxes,
                                 uint32_t bboxCount,
                                 uint64_t timestampNs)
{
    if (!lidarPoints && pointCount > 0) return false;

    if (timestampNs == lastLidarTimestampNs_ && lastLidarTimestampNs_ != 0)
        return true;
    lastLidarTimestampNs_ = timestampNs;

    struct timespec t0, t1;
    clock_gettime(CLOCK_MONOTONIC, &t0);

    dbscan_cluster_(lidarPoints, pointCount, bboxes, bboxCount);
    persist_clusters_();
    bbox_claim_(bboxes, bboxCount, 0, nullptr);
    predict_tracks_(timestampNs);
    associate_(timestampNs, bboxes);
    manage_lifecycle_();
    check_warnings_(timestampNs);
    update_snapshot_();

    clock_gettime(CLOCK_MONOTONIC, &t1);
    static uint32_t timeLogCnt = 0;
    int64_t us = (t1.tv_sec - t0.tv_sec) * 1000000
               + (t1.tv_nsec - t0.tv_nsec) / 1000;
    fprintf(stderr, "[TrackerTime] %ld us\n", (long)us);

    return true;
}

bool LidarTargetTracker::copy_snapshot(TrackedTarget* out, uint32_t maxCount,
                                        uint32_t* outCount) const
{
    if (!out || !outCount) return false;
    std::lock_guard<std::mutex> lock(snapshotMutex_);
    uint32_t count = snapshotTrackCount_;
    if (count > maxCount) count = maxCount;
    std::memcpy(out, snapshotTracks_, count * sizeof(TrackedTarget));
    *outCount = count;
    return true;
}

bool LidarTargetTracker::copy_cluster_vis(ClusterVisData* out, uint32_t maxCount,
                                           uint32_t* outCount) const
{
    if (!out || !outCount) return false;
    std::lock_guard<std::mutex> lock(clusterVisMutex_);
    uint32_t count = clusterVisCount_;
    if (count > maxCount) count = maxCount;
    std::memcpy(out, clusterVisBuf_, count * sizeof(ClusterVisData));
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
    if (cfg.beta  > cfg.alpha)                       return false;
    if (cfg.minDtSec     <= 0.0f)                    return false;
    if (cfg.defaultDtSec <  cfg.minDtSec)            return false;
    if (cfg.maxDtSec     <= cfg.minDtSec)            return false;
    if (cfg.dbscanEpsMeters <= 0.0f)                 return false;
    if (cfg.dbscanMinPoints < 1)                     return false;
    if (cfg.maxPointDistanceMeters <= 0.0f)          return false;
    if (cfg.maxClusterDistanceMeters <= 0.0f)        return false;
    if (cfg.clusterPersistenceFrames < 1)            return false;
    if (cfg.bboxAssocMaxDistMeters <= 0.0f)          return false;
    if (cfg.orphanAssocMaxDistMeters <= 0.0f)        return false;
    if (cfg.bboxClaimMaxPixelDist <= 0.0f)           return false;
    if (cfg.maxTracks > kMaxTracks)                  return false;
    if (cfg.warningEnterDistMeters <= 0.0f)          return false;
    if (cfg.warningExitDistMeters <= cfg.warningEnterDistMeters) return false;
    if (cfg.warningCooldownNs == 0)                  return false;
    if (cfg.minHitsToConfirm < 1)                    return false;
    if (cfg.maxLostFrames < 1)                       return false;
    return true;
}

void LidarTargetTracker::set_camera_configs(const CameraConfig* configs, uint32_t count)
{
    camCfgCount_ = (count > 2) ? 2 : count;
    for (uint32_t i = 0; i < camCfgCount_; ++i)
        camCfg_[i] = configs[i];
}

// ============================================================================
// 1. DBSCAN 聚类
// ============================================================================

void LidarTargetTracker::dbscan_cluster_(const LidarPoint* lidarPoints,
                                          uint32_t pointCount,
                                          const YoloBBox*, uint32_t)
{
    detectionCount_ = 0;

    float maxDist2 = config_.maxPointDistanceMeters * config_.maxPointDistanceMeters;
    uint32_t validCount = 0;
    for (uint32_t i = 0; i < pointCount && i < kMaxLidarPoints; ++i) {
        if (lidarPoints[i].x == 0.0f && lidarPoints[i].y == 0.0f) continue;
        float d2 = lidarPoints[i].x * lidarPoints[i].x
                  + lidarPoints[i].y * lidarPoints[i].y;
        if (d2 > maxDist2) continue;
        clusterPointIndices_[validCount] = i;
        clusterPointsX_[validCount]      = lidarPoints[i].x;
        clusterPointsY_[validCount]      = lidarPoints[i].y;
        clusterAssignments_[validCount]  = -1;
        ++validCount;
    }

    rawClusterCount_ = 0;
    if (validCount < config_.dbscanMinPoints) return;

    float eps2 = config_.dbscanEpsMeters * config_.dbscanEpsMeters;

    for (uint32_t i = 0; i < validCount && rawClusterCount_ < kMaxClusters; ++i) {
        if (clusterAssignments_[i] != -1) continue;

        uint32_t nCount = 0;
        for (uint32_t j = 0; j < validCount; ++j) {
            if (j == i) continue;
            float dx = clusterPointsX_[i] - clusterPointsX_[j];
            float dy = clusterPointsY_[i] - clusterPointsY_[j];
            if (dx * dx + dy * dy < eps2) ++nCount;
        }

        if (nCount < config_.dbscanMinPoints) {
            clusterAssignments_[i] = -2;
            continue;
        }

        clusterAssignments_[i] = static_cast<int32_t>(rawClusterCount_);
        float sumX = clusterPointsX_[i];
        float sumY = clusterPointsY_[i];
        uint32_t cnt = 1;

        int32_t stackTop = 0;
        seedStack_[0] = static_cast<int32_t>(i);

        while (stackTop >= 0) {
            int32_t p = seedStack_[stackTop--];

            for (uint32_t j = 0; j < validCount; ++j) {
                if (clusterAssignments_[j] != -1) continue;

                float dx = clusterPointsX_[p] - clusterPointsX_[j];
                float dy = clusterPointsY_[p] - clusterPointsY_[j];
                if (dx * dx + dy * dy >= eps2) continue;

                clusterAssignments_[j] = static_cast<int32_t>(rawClusterCount_);
                sumX += clusterPointsX_[j];
                sumY += clusterPointsY_[j];
                ++cnt;

                uint32_t jnCount = 0;
                for (uint32_t k = 0; k < validCount; ++k) {
                    if (k == j) continue;
                    float dx2 = clusterPointsX_[j] - clusterPointsX_[k];
                    float dy2 = clusterPointsY_[j] - clusterPointsY_[k];
                    if (dx2 * dx2 + dy2 * dy2 < eps2) ++jnCount;
                    if (jnCount >= config_.dbscanMinPoints) break;
                }
                if (jnCount >= config_.dbscanMinPoints)
                    seedStack_[++stackTop] = static_cast<int32_t>(j);
            }
        }

        clusterCentroidX_[rawClusterCount_] = sumX / static_cast<float>(cnt);
        clusterCentroidY_[rawClusterCount_] = sumY / static_cast<float>(cnt);
        clusterPointCounts_[rawClusterCount_] = cnt;

        float maxR2 = 0.0f;
        float cx = clusterCentroidX_[rawClusterCount_];
        float cy = clusterCentroidY_[rawClusterCount_];
        for (uint32_t j = 0; j < validCount; ++j) {
            if (static_cast<uint32_t>(clusterAssignments_[j]) == rawClusterCount_) {
                float dx = clusterPointsX_[j] - cx;
                float dy = clusterPointsY_[j] - cy;
                float r2 = dx * dx + dy * dy;
                if (r2 > maxR2) maxR2 = r2;
            }
        }
        clusterRadii_[rawClusterCount_] = std::sqrt(maxR2);
        clusterBboxMatch_[rawClusterCount_] = -1;
        claimedByBbox_[rawClusterCount_] = false;

        ++rawClusterCount_;
    }
}

// ============================================================================
// 2. 时间证据累积
// ============================================================================

void LidarTargetTracker::persist_clusters_()
{
    uint32_t nc = rawClusterCount_;

    for (uint32_t ci = 0; ci < nc; ++ci) {
        float cx = clusterCentroidX_[ci];
        float cy = clusterCentroidY_[ci];

        // 历史匹配门限 > DBSCAN 邻域半径：cluster 质心帧间跳动可达 0.5m+（尤其远处稀疏点）
        float matchDist = config_.dbscanEpsMeters * 2.5f;
        if (matchDist < 1.0f) matchDist = 1.0f;
        float bestDist2 = matchDist * matchDist;
        int32_t bestMatch = -1;

        for (uint32_t hi = 0; hi < clusterHistoryCount_; ++hi) {
            if (clusterHistory_[hi].consecutiveFrames == 0) continue;
            float dx = cx - clusterHistory_[hi].cx;
            float dy = cy - clusterHistory_[hi].cy;
            float d2 = dx * dx + dy * dy;
            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestMatch = static_cast<int32_t>(hi);
            }
        }

        uint32_t hi;
        if (bestMatch >= 0) {
            hi = static_cast<uint32_t>(bestMatch);
            clusterHistory_[hi].cx = cx;
            clusterHistory_[hi].cy = cy;
            clusterHistory_[hi].radius = clusterRadii_[ci];
            clusterHistory_[hi].pointCount = clusterPointCounts_[ci];
            clusterHistory_[hi].consecutiveFrames++;
            clusterHistory_[hi].lastLidarTs = lastLidarTimestampNs_;
        } else {
            // 优先复用已失效的槽位，避免 history 只增不减
            int32_t reuseSlot = -1;
            for (uint32_t s = 0; s < clusterHistoryCount_; ++s) {
                if (clusterHistory_[s].consecutiveFrames == 0) {
                    reuseSlot = static_cast<int32_t>(s);
                    break;
                }
            }
            if (reuseSlot >= 0) {
                hi = static_cast<uint32_t>(reuseSlot);
            } else if (clusterHistoryCount_ < kMaxClusters) {
                hi = clusterHistoryCount_++;
            } else {
                // 全满：淘汰最老的记录（lastLidarTs 最小）
                uint64_t oldestTs = UINT64_MAX;
                int32_t oldestSlot = -1;
                for (uint32_t s = 0; s < clusterHistoryCount_; ++s) {
                    if (clusterHistory_[s].lastLidarTs < oldestTs) {
                        oldestTs = clusterHistory_[s].lastLidarTs;
                        oldestSlot = static_cast<int32_t>(s);
                    }
                }
                if (oldestSlot < 0) continue;
                hi = static_cast<uint32_t>(oldestSlot);
            }
            clusterHistory_[hi].cx = cx;
            clusterHistory_[hi].cy = cy;
            clusterHistory_[hi].radius = clusterRadii_[ci];
            clusterHistory_[hi].pointCount = clusterPointCounts_[ci];
            clusterHistory_[hi].consecutiveFrames = 1;
            clusterHistory_[hi].lastLidarTs = lastLidarTimestampNs_;
        }

        // 将 history 索引存入 clusterBboxMatch_（复用字段传持久化状态）
        clusterBboxMatch_[ci] = static_cast<int32_t>(hi);
    }

    // 清理过期 history
    for (uint32_t hi = 0; hi < clusterHistoryCount_; ++hi) {
        if (clusterHistory_[hi].consecutiveFrames == 0) continue;
        if (clusterHistory_[hi].lastLidarTs != lastLidarTimestampNs_)
            clusterHistory_[hi].consecutiveFrames = 0;
    }
}

// ============================================================================
// 3. Bbox 认领评分 → detection
// ============================================================================

void LidarTargetTracker::bbox_claim_(const YoloBBox* bboxes, uint32_t bboxCount,
                                      uint32_t /*nc*/, ClusterInfo* /*clusters*/)
{
    detectionCount_ = 0;
    uint32_t nc = rawClusterCount_;

    // 筛选 confirmed 簇
    bool confirmed[kMaxClusters] = {};
    for (uint32_t ci = 0; ci < nc; ++ci) {
        int32_t hi = clusterBboxMatch_[ci];
        if (hi < 0) continue;
        uint32_t hu = static_cast<uint32_t>(hi);
        if (hu >= kMaxClusters) continue;
        if (clusterHistory_[hu].consecutiveFrames >= config_.clusterPersistenceFrames) {
            float dist = std::sqrt(clusterCentroidX_[ci] * clusterCentroidX_[ci]
                                 + clusterCentroidY_[ci] * clusterCentroidY_[ci]);
            if (dist <= config_.maxClusterDistanceMeters)
                confirmed[ci] = true;
        }
    }

    static uint32_t claimLogCnt = 0; ++claimLogCnt;
    uint32_t confCnt = 0;
    for (uint32_t ci = 0; ci < nc; ++ci) if (confirmed[ci]) ++confCnt;
    fprintf(stderr, "[BboxClaim] nc=%u confirmed=%u bboxes=%u\n",
            nc, confCnt, bboxCount);

    // 每 bbox 选最佳簇
    for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb)
        bboxCluster_[bb] = 0xFFFFFFFF;

    float bboxBestScore[50];
    for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb)
        bboxBestScore[bb] = -1.0f;

    for (uint32_t ci = 0; ci < nc; ++ci) {
        if (!confirmed[ci]) continue;  // 只有持久化确认的簇才参与 bbox 认领
        if (clusterPointCounts_[ci] < config_.minBboxClaimPoints) continue;  // 点数太少不参与
        float cx = clusterCentroidX_[ci];
        float cy = clusterCentroidY_[ci];
        float centroidDist = std::sqrt(cx * cx + cy * cy);
        if (centroidDist > config_.maxClusterDistanceMeters) continue;
        if (centroidDist < config_.minTrackDistanceMeters) continue;

        for (uint32_t cc = 0; cc < camCfgCount_; ++cc) {
            const CameraConfig& cfg = camCfg_[cc];

            float cX = cfg.tLidarToCam[0] * cx + cfg.tLidarToCam[1] * cy
                     + cfg.tLidarToCam[3];
            float cY = cfg.tLidarToCam[4] * cx + cfg.tLidarToCam[5] * cy
                     + cfg.tLidarToCam[7];
            float cZ = cfg.tLidarToCam[8] * cx + cfg.tLidarToCam[9] * cy
                     + cfg.tLidarToCam[11];
            if (cZ <= 0.0f) continue;

            float u = cfg.fx * cX / cZ + cfg.cx;
            float v = cfg.fy * cY / cZ + cfg.cy;

            for (uint32_t bb = 0; bb < bboxCount; ++bb) {
                float pixelDist = 0.0f;

                if (u >= static_cast<float>(bboxes[bb].x1) &&
                    u <  static_cast<float>(bboxes[bb].x2) &&
                    v >= static_cast<float>(bboxes[bb].y1) &&
                    v <  static_cast<float>(bboxes[bb].y2)) {
                    pixelDist = 0.0f;
                } else {
                    float dx = 0.0f, dy = 0.0f;
                    if (u < static_cast<float>(bboxes[bb].x1))
                        dx = static_cast<float>(bboxes[bb].x1) - u;
                    else if (u >= static_cast<float>(bboxes[bb].x2))
                        dx = u - (static_cast<float>(bboxes[bb].x2) - 1.0f);
                    if (v < static_cast<float>(bboxes[bb].y1))
                        dy = static_cast<float>(bboxes[bb].y1) - v;
                    else if (v >= static_cast<float>(bboxes[bb].y2))
                        dy = v - (static_cast<float>(bboxes[bb].y2) - 1.0f);
                    pixelDist = std::sqrt(dx * dx + dy * dy);
                }

                float proximityFactor = 1.0f - pixelDist / config_.bboxClaimMaxPixelDist;
                if (proximityFactor <= 0.0f) continue;

                float score = 1.0f / (1.0f + centroidDist) * proximityFactor;

                if (score > bboxBestScore[bb]) {
                    bboxBestScore[bb] = score;
                    bboxCluster_[bb] = ci;
                }
            }
        }
    }

    // ---- 冲突解决：一个 cluster 不能被多个 bbox 认领 ----
    for (uint32_t ci = 0; ci < nc; ++ci)
        claimedByBbox_[ci] = false;

    // 找出被多个 bbox 选中的 cluster
    uint32_t bboxCountForCluster[kMaxClusters] = {};
    for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb) {
        uint32_t ci = bboxCluster_[bb];
        if (ci != 0xFFFFFFFF && ci < nc)
            bboxCountForCluster[ci]++;
    }

    // 对每个冲突 cluster，只保留得分最高的 bbox
    for (uint32_t ci = 0; ci < nc; ++ci) {
        if (bboxCountForCluster[ci] <= 1) continue;
        // 找到得分最高的 bbox
        float bestSc = -1.0f;
        uint32_t bestBb = 0xFFFFFFFF;
        for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb) {
            if (bboxCluster_[bb] == ci && bboxBestScore[bb] > bestSc) {
                bestSc = bboxBestScore[bb];
                bestBb = bb;
            }
        }
        // 取消其他 bbox 对此 cluster 的认领
        for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb) {
            if (bboxCluster_[bb] == ci && bb != bestBb)
                bboxCluster_[bb] = 0xFFFFFFFF;
        }
    }

    // 标记被认领的簇（冲突解决后的赢家）
    for (uint32_t ci = 0; ci < nc; ++ci)
        claimedByBbox_[ci] = false;
    for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb) {
        uint32_t ci = bboxCluster_[bb];
        if (ci != 0xFFFFFFFF && ci < nc)
            claimedByBbox_[ci] = true;
    }

    // 冲突后补选：空出的 bbox 从未认领的簇中重新选择
    for (uint32_t bb = 0; bb < bboxCount && bb < 50; ++bb) {
        if (bboxCluster_[bb] != 0xFFFFFFFF) continue;  // 已有簇
        float bestSc = -1.0f;
        uint32_t bestCi = 0xFFFFFFFF;
        for (uint32_t ci = 0; ci < nc; ++ci) {
            if (claimedByBbox_[ci]) continue;  // 已被认领
            float cx = clusterCentroidX_[ci];
            float cy = clusterCentroidY_[ci];
            float cDist = std::sqrt(cx * cx + cy * cy);
            if (cDist > config_.maxClusterDistanceMeters || cDist < config_.minTrackDistanceMeters) continue;
            // 重新计算对 bb 的 score
            float sc = -1.0f;
            for (uint32_t cc = 0; cc < camCfgCount_; ++cc) {
                const CameraConfig& cfg = camCfg_[cc];
                float cX = cfg.tLidarToCam[0]*cx + cfg.tLidarToCam[1]*cy + cfg.tLidarToCam[3];
                float cY = cfg.tLidarToCam[4]*cx + cfg.tLidarToCam[5]*cy + cfg.tLidarToCam[7];
                float cZ = cfg.tLidarToCam[8]*cx + cfg.tLidarToCam[9]*cy + cfg.tLidarToCam[11];
                if (cZ <= 0.0f) continue;
                float u = cfg.fx * cX / cZ + cfg.cx;
                float v = cfg.fy * cY / cZ + cfg.cy;
                float pixDist = 0.0f;
                if (u < static_cast<float>(bboxes[bb].x1)) pixDist = static_cast<float>(bboxes[bb].x1) - u;
                else if (u >= static_cast<float>(bboxes[bb].x2)) pixDist = u - (static_cast<float>(bboxes[bb].x2) - 1.0f);
                else if (v < static_cast<float>(bboxes[bb].y1)) pixDist = static_cast<float>(bboxes[bb].y1) - v;
                else if (v >= static_cast<float>(bboxes[bb].y2)) pixDist = v - (static_cast<float>(bboxes[bb].y2) - 1.0f);
                float pf = 1.0f - pixDist / config_.bboxClaimMaxPixelDist;
                if (pf <= 0.0f) continue;
                float s = 1.0f / (1.0f + cDist) * pf;
                if (s > sc) sc = s;
            }
            if (sc > bestSc) { bestSc = sc; bestCi = ci; }
        }
        if (bestCi != 0xFFFFFFFF) {
            bboxCluster_[bb] = bestCi;
            claimedByBbox_[bestCi] = true;
        }
    }

    // 输出 bbox detection
    for (uint32_t bb = 0; bb < bboxCount; ++bb) {
        uint32_t ci = bboxCluster_[bb];
        if (ci == 0xFFFFFFFF || ci >= nc) {
            fprintf(stderr, "[BboxClaim] bbox[%u] -> NONE\n", bb);
            continue;
        }
        if (detectionCount_ >= kMaxDetections) break;

        DetectionCandidate& det = detections_[detectionCount_];
        det.x = clusterCentroidX_[ci];
        det.y = clusterCentroidY_[ci];
        det.classId     = bboxes[bb].classId;
        det.confidence  = bboxes[bb].confidence;
        det.avgIntensity = 0.0f;
        det.pointCount  = clusterPointCounts_[ci];
        det.bboxIdx     = bb;
        det.isOrphan    = false;
        ++detectionCount_;

        float d = std::sqrt(det.x * det.x + det.y * det.y);
        fprintf(stderr, "[BboxClaim] bbox[%u] -> ci=%u dist=%.2fm pts=%u score=%.3f\n",
                bb, ci, d, det.pointCount, bboxBestScore[bb]);
    }

    // 输出 orphan detection
    for (uint32_t ci = 0; ci < nc; ++ci) {
        if (!confirmed[ci]) continue;
        if (claimedByBbox_[ci]) continue;
        float d = std::sqrt(clusterCentroidX_[ci] * clusterCentroidX_[ci]
                          + clusterCentroidY_[ci] * clusterCentroidY_[ci]);
        if (d < config_.minTrackDistanceMeters) continue;  // 过滤原点噪声
        if (detectionCount_ >= kMaxDetections) break;

        DetectionCandidate& det = detections_[detectionCount_];
        det.x = clusterCentroidX_[ci];
        det.y = clusterCentroidY_[ci];
        det.classId     = 0;
        det.confidence  = 0.5f;
        det.avgIntensity = 0.0f;
        det.pointCount  = clusterPointCounts_[ci];
        det.bboxIdx     = 0xFFFFFFFF;
        det.isOrphan    = true;
        ++detectionCount_;
    }
}

// ============================================================================
// 4. Alpha-Beta 预测
// ============================================================================

void LidarTargetTracker::predict_tracks_(uint64_t timestampNs)
{
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        if (workingTracks_[i].state == TrackState::Deleted) continue;

        float dt = static_cast<float>(timestampNs - workingTracks_[i].lastUpdateNs)
                   * 1e-9f;

        if (dt <= config_.minDtSec)  dt = config_.defaultDtSec;
        if (dt >  config_.maxDtSec)  dt = 0.0f;

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
// 5. 评分制贪心最近邻关联
// ============================================================================

void LidarTargetTracker::associate_(uint64_t timestampNs, const YoloBBox*)
{
    std::memset(detAssigned_, 0, sizeof(detAssigned_));
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        trackMatches_[i]  = -1;
        trackAssigned_[i] = false;
    }

    for (uint32_t j = 0; j < detectionCount_; ++j) {
        const DetectionCandidate& det = detections_[j];

        float gateDist = det.isOrphan
            ? config_.orphanAssocMaxDistMeters
            : config_.bboxAssocMaxDistMeters;
        float gateDist2 = gateDist * gateDist;

        int32_t bestTrack = -1;
        float   bestScore = -1.0f;

        for (uint32_t i = 0; i < activeTrackCount_; ++i) {
            TrackedTarget& track = workingTracks_[i];
            if (track.state == TrackState::Deleted) continue;
            if (trackAssigned_[i]) continue;
            if (det.isOrphan && track.state == TrackState::Tentative) continue;

            float dx = det.x - predX_[i];
            float dy = det.y - predY_[i];
            float d2 = dx * dx + dy * dy;
            if (d2 > gateDist2) continue;

            float score = 1.0f / (1.0f + d2);
            if (score > bestScore) {
                bestScore = score;
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

    // 匹配校正
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state == TrackState::Deleted) continue;

        if (trackAssigned_[i] && trackMatches_[i] >= 0) {
            uint32_t j = static_cast<uint32_t>(trackMatches_[i]);
            const DetectionCandidate& det = detections_[j];

            float dt = static_cast<float>(timestampNs - track.lastUpdateNs) * 1e-9f;
            if (dt <= config_.minDtSec) dt = config_.defaultDtSec;
            if (dt >  config_.maxDtSec) dt = 0.0f;

            apply_correction_(i, det, dt);
            track.consecutiveMisses = 0;
            track.age++;
            track.lastUpdateNs = timestampNs;
            track.classId      = det.classId;
            track.bboxIdx      = det.bboxIdx;
            track.confidence   = det.confidence;
            track.avgIntensity = det.avgIntensity;
            track.pointCount   = det.pointCount;
            track.distanceMeters = std::sqrt(track.posX * track.posX
                                           + track.posY * track.posY);
        } else {
            track.consecutiveMisses++;
            track.age++;
        }
    }

    // 未匹配 bbox 检测 → 新 Tentative
    for (uint32_t j = 0; j < detectionCount_; ++j) {
        if (detAssigned_[j]) continue;
        if (detections_[j].isOrphan) continue;

        const DetectionCandidate& det = detections_[j];

        int32_t slot = -1;
        for (uint32_t i = 0; i < activeTrackCount_; ++i) {
            if (workingTracks_[i].state == TrackState::Deleted) {
                slot = static_cast<int32_t>(i);
                break;
            }
        }
        if (slot < 0 && activeTrackCount_ < config_.maxTracks)
            slot = static_cast<int32_t>(activeTrackCount_++);

        // maxTracks 满：淘汰最老 Lost
        if (slot < 0) {
            uint32_t oldestIdx = 0xFFFFFFFF, oldestAge = 0;
            for (uint32_t i = 0; i < activeTrackCount_; ++i) {
                if (workingTracks_[i].state == TrackState::Lost
                    && workingTracks_[i].age > oldestAge) {
                    oldestAge = workingTracks_[i].age;
                    oldestIdx = i;
                }
            }
            if (oldestIdx != 0xFFFFFFFF) {
                workingTracks_[oldestIdx].state = TrackState::Deleted;
                slot = static_cast<int32_t>(oldestIdx);
            }
        }
        if (slot < 0) continue;

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

void LidarTargetTracker::apply_correction_(uint32_t trackIdx,
                                            const DetectionCandidate& det,
                                            float dt)
{
    TrackedTarget& track = workingTracks_[trackIdx];
    track.consecutiveHits++;

    float rx = det.x - predX_[trackIdx];
    float ry = det.y - predY_[trackIdx];

    track.posX = predX_[trackIdx] + config_.alpha * rx;
    track.posY = predY_[trackIdx] + config_.alpha * ry;

    if (dt > config_.minDtSec
        && track.consecutiveHits >= config_.minHitsForVelocity) {
        track.velX = predVX_[trackIdx] + (config_.beta / dt) * rx;
        track.velY = predVY_[trackIdx] + (config_.beta / dt) * ry;
    }
}

// ============================================================================
// 6. 5 状态生命周期
// ============================================================================

void LidarTargetTracker::manage_lifecycle_()
{
    // Phase 1: miss-based transitions
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state == TrackState::Deleted) continue;

        switch (track.state) {
        case TrackState::Tentative:
            if (track.consecutiveMisses > config_.maxTentativeMisses)
                track.state = TrackState::Deleted;
            else if (track.consecutiveHits >= config_.minHitsToConfirm)
                track.state = TrackState::FusionTracking;
            break;
        case TrackState::FusionTracking:
        case TrackState::PureRadarTracking:
            if (track.consecutiveMisses > config_.maxFusionMisses)
                track.state = TrackState::Lost;
            break;
        case TrackState::Lost:
            if (track.consecutiveMisses >= config_.maxLostFrames)
                track.state = TrackState::Deleted;
            break;
        default:
            break;
        }
    }

    // Phase 2: match-based transitions
    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state == TrackState::Deleted) continue;
        if (track.consecutiveMisses > 0) continue;

        bool matchedOrphan = (track.bboxIdx == 0xFFFFFFFF);

        if (track.state == TrackState::Lost) {
            track.state = matchedOrphan ? TrackState::PureRadarTracking
                                        : TrackState::FusionTracking;
        } else if (track.state == TrackState::FusionTracking && matchedOrphan) {
            track.state = TrackState::PureRadarTracking;
        } else if (track.state == TrackState::PureRadarTracking && !matchedOrphan) {
            track.state = TrackState::FusionTracking;
        }
    }
}

// ============================================================================
// 7. 告警
// ============================================================================

void LidarTargetTracker::check_warnings_(uint64_t nowNs)
{
    if (!warningCb_) return;

    for (uint32_t i = 0; i < activeTrackCount_; ++i) {
        TrackedTarget& track = workingTracks_[i];
        if (track.state != TrackState::FusionTracking
            && track.state != TrackState::PureRadarTracking) continue;
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
// 8. 快照
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

    // 聚类可视化快照
    {
        std::lock_guard<std::mutex> vlock(clusterVisMutex_);
        uint32_t vc = 0;
        static uint32_t cvLogCounter = 0;
        if (++cvLogCounter % 50 == 0)
            fprintf(stderr, "[TrackerVIS] historyCount=%u rawClusters=%u\n",
                    clusterHistoryCount_, rawClusterCount_);
        for (uint32_t hi = 0; hi < clusterHistoryCount_ && vc < kMaxClusters; ++hi) {
            if (clusterHistory_[hi].consecutiveFrames == 0) continue;
            ClusterVisData& vis = clusterVisBuf_[vc];
            vis.cx         = clusterHistory_[hi].cx;
            vis.cy         = clusterHistory_[hi].cy;
            vis.radius     = clusterHistory_[hi].radius;
            vis.pointCount = clusterHistory_[hi].pointCount;

            bool orphan = true;
            uint32_t bbIdx = 0xFFFFFFFF;
            for (uint32_t bb = 0; bb < 50; ++bb) {
                if (bboxCluster_[bb] != 0xFFFFFFFF) {
                    float dx = vis.cx - clusterCentroidX_[bboxCluster_[bb]];
                    float dy = vis.cy - clusterCentroidY_[bboxCluster_[bb]];
                    if (dx * dx + dy * dy < 0.01f) {
                        orphan = false;
                        bbIdx = bb;
                        break;
                    }
                }
            }
            vis.bboxIdx  = bbIdx;
            vis.isOrphan = orphan;
            ++vc;
        }
        clusterVisCount_ = vc;
    }
}
