#include "lidar_camera_fusion.h"
#include "lidar_target_tracker.h"

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

void LidarCameraFusion::fusion_thread_()
{
    printf("[LidarCameraFusion] fusion thread started, %u camera(s)\n", camCount_);

    uint64_t iterationCount = 0;

    while (running_) {
        // ---- 步骤 1：获取 YOLO 检测结果 ----
        // TODO: 替换为从推理类队列获取真实数据
        //   YoloResult result;
        //   if (!yoloQueue_->try_pop(result, 33)) { ... }
        for (uint32_t c = 0; c < camCount_; ++c) {
            generate_fake_detections_(c);
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

        // ---- 步骤 5：目标跟踪 ----
        if (trackingEnabled_ && tracker_) {
            tracker_->update(result_, lidarPointsBuf_, frame.pointsCount,
                             allBboxes, totalBboxes, frame.timestampNs);
        }

        // ---- 步骤 6：输出结果 ----
        // TODO: 推入下游队列
        ++iterationCount;
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
