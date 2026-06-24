#include "lidar_camera_fusion.h"
#include "lidar_target_tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

// ============================================================================
// 线程模式（依赖 SentinelLslidarer，与实际部署代码一同编译）
// ============================================================================

bool LidarCameraFusion::start(SentinelLslidarer* lidar,
                               const CameraConfig* camConfigs,
                               uint32_t camCount)
{
    if (!candidatePointBuf || !lidarPointsBuf_) {
        fprintf(stderr, "[LidarCameraFusion] start: buffers not allocated\n");
        return false;
    }
    if (!lidar || !camConfigs || camCount == 0 || camCount > kMaxCameras) {
        fprintf(stderr, "[LidarCameraFusion] start: invalid parameters\n");
        return false;
    }
    if (running_) {
        fprintf(stderr, "[LidarCameraFusion] start: already running\n");
        return false;
    }

    lidar_   = lidar;
    camCount_ = camCount;
    for (uint32_t i = 0; i < camCount; ++i) {
        camConfigs_[i] = camConfigs[i];
    }

    running_ = true;
    fusionThread_ = std::thread(&LidarCameraFusion::fusion_thread_, this);

    return true;
}

bool LidarCameraFusion::is_running() const
{
    return running_;
}

void LidarCameraFusion::set_detection_provider(DetectionProvider provider)
{
    detectionProvider_ = std::move(provider);
}

void LidarCameraFusion::fusion_thread_()
{
    printf("[LidarCameraFusion] fusion thread started, %u camera(s)\n", camCount_);

    uint64_t iterationCount = 0;

    while (running_) {
        // ---- 步骤 1：获取 YOLO 检测结果 ----
        for (uint32_t c = 0; c < camCount_; ++c) {
            if (detectionProvider_) {
                if (!detectionProvider_(c, fakeDetections_[c], 33))
                    fakeDetections_[c].clear();
            } else {
                generate_fake_detections_(c);
            }
            // 过滤：只保留 person (classId=0) 且置信度 >= 0.60
            auto& dets = fakeDetections_[c];
            dets.erase(std::remove_if(dets.begin(), dets.end(),
                [](const YoloBBox& b) { return b.classId != 0 || b.confidence < 0.60f; }),
                dets.end());
        }

        // ---- 步骤 2：从检测结果中提取时间戳 ----
        // 取第一个有数据的相机的首帧时间戳
        uint64_t tsNs = 0;
        bool gotTs = false;
        for (uint32_t c = 0; c < camCount_; ++c) {
            if (!fakeDetections_[c].empty()) {
                tsNs = fakeDetections_[c][0].timestampNs;
                gotTs = true;
                break;
            }
        }
        if (!gotTs) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // ---- 步骤 3：取最近雷达帧 ----
        LidarFrame frame;
        frame.points = lidarPointsBuf_;
        if (!lidar_->get_closest_frame(tsNs, frame)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        // [LidarCalib] 已注释

        // ---- 步骤 4：累积融合 + 同步记录 bbox ----
        reset();
        YoloBBox allBboxes[kMaxDetections];
        uint32_t totalBboxes = 0;
        for (uint32_t c = 0; c < camCount_; ++c) {
            if (!fakeDetections_[c].empty()) {
                fuse_data(fakeDetections_[c],
                          fakeDetections_[c][0].timestampNs,
                          frame, camConfigs_[c]);
                // 同步 append bbox，保证与 FusionResult 顺序 100% 一致
                for (const auto& b : fakeDetections_[c]) {
                    if (totalBboxes < kMaxDetections) {
                        allBboxes[totalBboxes++] = b;
                    }
                }
            }
        }

        // ---- 步骤 4.5：目标跟踪 ----
        if (trackingEnabled_ && tracker_) {
            tracker_->update(result_, lidarPointsBuf_, frame.pointsCount,
                             allBboxes, totalBboxes, frame.timestampNs);
        }

        // ---- 步骤 5：构建 LiDAR OSD 快照（含 tracker 聚类距离） ----
        for (uint32_t i = 0; i < frame.pointsCount; ++i) {
            lidarPointXBuf_[i] = lidarPointsBuf_[i].x;
            lidarPointYBuf_[i] = lidarPointsBuf_[i].y;
        }

        LidarOsdSnapshot snap;
        snap.timestampNs = frame.timestampNs;
        snap.camCount = camCount_;

        uint32_t globalBboxIdx = 0;
        for (uint32_t c = 0; c < camCount_; ++c) {
            auto& cam = snap.cameras[c];
            cam.camNum = c;
            cam.imgWidth  = camConfigs_[c].imgWidth;
            cam.imgHeight = camConfigs_[c].imgHeight;

            uint32_t camBboxCount = static_cast<uint32_t>(fakeDetections_[c].size());
            cam.bboxCount = camBboxCount;
            if (camBboxCount == 0) continue;

            cam.bboxX1.resize(camBboxCount);
            cam.bboxY1.resize(camBboxCount);
            cam.bboxX2.resize(camBboxCount);
            cam.bboxY2.resize(camBboxCount);
            for (uint32_t b = 0; b < camBboxCount; ++b) {
                cam.bboxX1[b] = fakeDetections_[c][b].x1;
                cam.bboxY1[b] = fakeDetections_[c][b].y1;
                cam.bboxX2[b] = fakeDetections_[c][b].x2;
                cam.bboxY2[b] = fakeDetections_[c][b].y2;
            }

            cam.bboxPointCounts.assign(&bboxPointCountsBuf[globalBboxIdx],
                                        &bboxPointCountsBuf[globalBboxIdx + camBboxCount]);

            uint32_t pointStart = bboxOffsets[globalBboxIdx];
            uint32_t pointEnd   = (globalBboxIdx + camBboxCount < result_.bboxCount)
                                    ? bboxOffsets[globalBboxIdx + camBboxCount]
                                    : totalCandidateCount;
            uint32_t camPointCount = pointEnd - pointStart;

            cam.bboxPointU.assign(&bboxPointUBuf_[pointStart],
                                   &bboxPointUBuf_[pointStart + camPointCount]);
            cam.bboxPointV.assign(&bboxPointVBuf_[pointStart],
                                   &bboxPointVBuf_[pointStart + camPointCount]);
            cam.bboxPointIndices.assign(&candidatePointBuf[pointStart],
                                         &candidatePointBuf[pointStart + camPointCount]);

            cam.lidarPointX.assign(lidarPointXBuf_, lidarPointXBuf_ + frame.pointsCount);
            cam.lidarPointY.assign(lidarPointYBuf_, lidarPointYBuf_ + frame.pointsCount);

            // 复用 tracker 聚类质心距离
            cam.bboxClusterDistMeters.resize(camBboxCount, 0.0f);
            for (uint32_t b = 0; b < camBboxCount; ++b) {
                float cx = 0, cy = 0;
                bool ok = (trackingEnabled_ && tracker_
                    && tracker_->get_bbox_detection_centroid(globalBboxIdx + b, cx, cy));
                if (ok) {
                    cam.bboxClusterDistMeters[b] = std::sqrt(cx * cx + cy * cy);
                }
                if (iterationCount % 10 == 0) {
                    fprintf(stderr, "[LidarOSD] cam%u bbox[%u] track_cx=%.3f track_cy=%.3f dist=%.2fm %s\n",
                            c, b, cx, cy, cam.bboxClusterDistMeters[b],
                            ok ? "" : "NO_TRACK");
                }
            }

            globalBboxIdx += camBboxCount;
        }

        {
            std::lock_guard<std::mutex> lock(osdSnapshotMutex_);
            latestOsdSnapshot_ = std::move(snap);
        }

        // ---- 步骤 6：输出结果 ----
        ++iterationCount;
        if (iterationCount % 10 == 0 && iterationCount % 100 != 0) {
            uint32_t tcnt = 0;
            if (trackingEnabled_ && tracker_) {
                TrackedTarget tmp[1];
                tracker_->copy_snapshot(tmp, 1, &tcnt);
            }
            printf("[LidarCameraFusion] #%lu bboxes:%u matched:%u tracks:%u\n",
                   (unsigned long)iterationCount, result_.bboxCount,
                   totalCandidateCount, tcnt);
        }
        if (iterationCount % 100 == 0) {
            uint32_t frameTotal = totalCandidateCount
                                  + behindCameraCount
                                  + outOfImageCount;
            printf("[LidarCameraFusion] %lu iters | bboxes:%u matched:%u "
                   "behind:%u out:%u frame:%u",
                   (unsigned long)iterationCount,
                   result_.bboxCount,
                   totalCandidateCount,
                   behindCameraCount,
                   outOfImageCount,
                   frameTotal);
            if (trackingEnabled_ && tracker_) {
                TrackedTarget snapshot[10];
                uint32_t tcount = 0;
                tracker_->copy_snapshot(snapshot, 10, &tcount);
                printf(" tracks:%u\n", tcount);
                for (uint32_t ti = 0; ti < tcount; ++ti) {
                    const char* stateStr = "?";
                    switch (snapshot[ti].state) {
                    case TrackState::Tentative: stateStr = "Tentative"; break;
                    case TrackState::Confirmed: stateStr = "Confirmed"; break;
                    case TrackState::Coasting:  stateStr = "Coasting";  break;
                    case TrackState::Deleted:   stateStr = "Deleted";   break;
                    default: break;
                    }
                    printf("  #%u | pos=(%.2f,%.2f) vel=(%.2f,%.2f) "
                           "dist=%.2fm %s\n",
                           snapshot[ti].id,
                           snapshot[ti].posX, snapshot[ti].posY,
                           snapshot[ti].velX, snapshot[ti].velY,
                           snapshot[ti].distanceMeters,
                           stateStr);
                }
            } else {
                printf("\n");
            }

            // 打印所有匹配点的雷达坐标（仅未启用跟踪时）
            if (!trackingEnabled_) {
                for (uint32_t j = 0; j < totalCandidateCount; ++j) {
                    uint32_t pi = result_.bboxPointIndices[j];
                    printf("  pt[%u]: lidar(%.3f, %.3f) u=%.0f v=%.0f\n", pi,
                           lidarPointsBuf_[pi].x, lidarPointsBuf_[pi].y,
                           camConfigs_[0].fx * lidarPointsBuf_[pi].y / (-lidarPointsBuf_[pi].x) + camConfigs_[0].cx,
                           camConfigs_[0].cy);
                }
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    printf("[LidarCameraFusion] fusion thread stopped after %lu iterations\n",
           (unsigned long)iterationCount);
}

void LidarCameraFusion::generate_fake_detections_(uint32_t camIndex)
{
    std::vector<YoloBBox>& dets = fakeDetections_[camIndex];
    dets.clear();

    const CameraConfig& cfg = camConfigs_[camIndex];

    // 虚构递增时间戳，模拟相机帧到达（~30fps）
    // 同一帧的所有 bbox 共享同一个时间戳
    static uint64_t fakeNs = 0;
    fakeNs += 33333333ULL;

    // bbox 覆盖雷达正后方区域，距离过滤交由 tracker 处理
    YoloBBox bbox;
    bbox.x1          = 280;
    bbox.y1          = 200;
    bbox.x2          = 460;
    bbox.y2          = 280;
    bbox.classId     = 0;
    bbox.confidence  = 0.9f;
    bbox.timestampNs = fakeNs;
    dets.push_back(bbox);

    // 示例：同一帧的第二个 bbox（共享相同 timestampNs）
    // bbox.x1 = ...;  bbox.timestampNs = fakeNs;  dets.push_back(bbox);
}
