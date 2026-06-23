#include "lidar_camera_fusion.h"
#include "lidar_target_tracker.h"

#include <cstdio>
#include <cstring>
#include <new>

// ============================================================================
// 构造 / 析构
// ============================================================================

LidarCameraFusion::LidarCameraFusion()
    : candidatePointBuf(nullptr)
    , pointToBbox(nullptr)
    , bboxPointCountsBuf(nullptr)
    , bboxOffsets(nullptr)
    , writeCursor(nullptr)
    , bboxPointUBuf_(nullptr)
    , bboxPointVBuf_(nullptr)
    , tempU_(nullptr)
    , tempV_(nullptr)
    , lidarPointXBuf_(nullptr)
    , lidarPointYBuf_(nullptr)
    , totalBboxCount(0)
    , totalCandidateCount(0)
    , behindCameraCount(0)
    , outOfImageCount(0)
    , lidar_(nullptr)
    , camCount_(0)
    , lidarPointsBuf_(nullptr)
{
    candidatePointBuf  = new (std::nothrow) uint32_t[kMaxLidarPoints];
    pointToBbox        = new (std::nothrow) int32_t[kMaxLidarPoints];
    bboxPointCountsBuf = new (std::nothrow) uint32_t[kMaxDetections];
    bboxOffsets        = new (std::nothrow) uint32_t[kMaxDetections];
    writeCursor        = new (std::nothrow) uint32_t[kMaxDetections];
    bboxPointUBuf_     = new (std::nothrow) float[kMaxLidarPoints];
    bboxPointVBuf_     = new (std::nothrow) float[kMaxLidarPoints];
    tempU_             = new (std::nothrow) float[kMaxLidarPoints];
    tempV_             = new (std::nothrow) float[kMaxLidarPoints];
    lidarPointXBuf_    = new (std::nothrow) float[kMaxLidarPoints];
    lidarPointYBuf_    = new (std::nothrow) float[kMaxLidarPoints];
    lidarPointsBuf_    = new (std::nothrow) LidarPoint[kMaxLidarPoints];
    tracker_           = new (std::nothrow) LidarTargetTracker();

    if (!candidatePointBuf || !pointToBbox || !bboxPointCountsBuf ||
        !bboxOffsets || !writeCursor || !bboxPointUBuf_ || !bboxPointVBuf_ ||
        !tempU_ || !tempV_ || !lidarPointXBuf_ || !lidarPointYBuf_ ||
        !lidarPointsBuf_ || !tracker_) {
        fprintf(stderr, "[LidarCameraFusion] buffer allocation failed in constructor\n");
        delete[] candidatePointBuf;   candidatePointBuf  = nullptr;
        delete[] pointToBbox;         pointToBbox        = nullptr;
        delete[] bboxPointCountsBuf;  bboxPointCountsBuf = nullptr;
        delete[] bboxOffsets;         bboxOffsets        = nullptr;
        delete[] writeCursor;         writeCursor        = nullptr;
        delete[] bboxPointUBuf_;      bboxPointUBuf_     = nullptr;
        delete[] bboxPointVBuf_;      bboxPointVBuf_     = nullptr;
        delete[] tempU_;              tempU_             = nullptr;
        delete[] tempV_;              tempV_             = nullptr;
        delete[] lidarPointXBuf_;     lidarPointXBuf_    = nullptr;
        delete[] lidarPointYBuf_;     lidarPointYBuf_    = nullptr;
        delete[] lidarPointsBuf_;     lidarPointsBuf_    = nullptr;
        delete tracker_;              tracker_           = nullptr;
    }

    std::memset(&result_, 0, sizeof(result_));
}

void LidarCameraFusion::stop()
{
    if (running_) {
        running_ = false;
        if (fusionThread_.joinable()) {
            fusionThread_.join();
        }
    }
}

LidarCameraFusion::~LidarCameraFusion()
{
    stop();

    delete[] candidatePointBuf;
    delete[] pointToBbox;
    delete[] bboxPointCountsBuf;
    delete[] bboxOffsets;
    delete[] writeCursor;
    delete[] bboxPointUBuf_;
    delete[] bboxPointVBuf_;
    delete[] tempU_;
    delete[] tempV_;
    delete[] lidarPointXBuf_;
    delete[] lidarPointYBuf_;
    delete[] lidarPointsBuf_;
    delete tracker_;
}

// ============================================================================
// 状态管理
// ============================================================================

void LidarCameraFusion::reset()
{
    totalBboxCount      = 0;
    totalCandidateCount = 0;
    behindCameraCount   = 0;
    outOfImageCount     = 0;

    std::memset(&result_, 0, sizeof(result_));

    for (uint32_t i = 0; i < kMaxLidarPoints; ++i) {
        pointToBbox[i] = -1;
        tempU_[i] = 0.0f;
        tempV_[i] = 0.0f;
    }
    for (uint32_t b = 0; b < kMaxDetections; ++b) {
        bboxPointCountsBuf[b] = 0;
        bboxOffsets[b]        = 0;
        writeCursor[b]        = 0;
    }
}

// ============================================================================
// 融合（累积模式）
// ============================================================================

bool LidarCameraFusion::fuse_data(const std::vector<YoloBBox>& detections,
                                   uint64_t imageTimestampNs,
                                   const LidarFrame& lidarFrame,
                                   const CameraConfig& cameraCfg)
{
    if (!candidatePointBuf || !pointToBbox) {
        fprintf(stderr, "[LidarCameraFusion] fuse_data: buffers not allocated\n");
        return false;
    }
    if (cameraCfg.imgWidth == 0 || cameraCfg.imgHeight == 0 ||
        cameraCfg.fx == 0.0f || cameraCfg.fy == 0.0f) {
        fprintf(stderr, "[LidarCameraFusion] fuse_data: invalid CameraConfig\n");
        return false;
    }

    uint32_t nPoints = lidarFrame.pointsCount;
    if (nPoints > kMaxLidarPoints) {
        nPoints = kMaxLidarPoints;
    }

    uint32_t nBboxes = static_cast<uint32_t>(detections.size());
    if (nBboxes > kMaxDetections - totalBboxCount) {
        nBboxes = kMaxDetections - totalBboxCount;
    }

    if (nBboxes == 0) {
        return true;
    }

    // ---- 第一趟：变换 + 投影 + 分类 ----
    for (uint32_t b = 0; b < nBboxes; ++b) {
        uint32_t gb = totalBboxCount + b;
        bboxPointCountsBuf[gb] = 0;
    }

    for (uint32_t i = 0; i < nPoints; ++i) {
        if (pointToBbox[i] >= 0) {
            continue;
        }

        float lx = lidarFrame.points[i].x;
        float ly = lidarFrame.points[i].y;

        float cx, cy, cz;
        transform_point_(lx, ly, cameraCfg.tLidarToCam, cx, cy, cz);

        if (cz <= 0.0f) {
            ++behindCameraCount;
            continue;
        }

        float u, v;
        project_point_(cx, cy, cz, cameraCfg, u, v);

        float fImgWidth  = static_cast<float>(cameraCfg.imgWidth);
        float fImgHeight = static_cast<float>(cameraCfg.imgHeight);
        if (u < 0.0f || u >= fImgWidth || v < 0.0f || v >= fImgHeight) {
            ++outOfImageCount;
            continue;
        }

        uint32_t ui = static_cast<uint32_t>(u);
        uint32_t vi = static_cast<uint32_t>(v);
        int32_t bboxIdx = -1;
        for (uint32_t b = 0; b < nBboxes; ++b) {
            const YoloBBox& bb = detections[b];
            if (ui >= bb.x1 && ui < bb.x2 && vi >= bb.y1 && vi < bb.y2) {
                bboxIdx = static_cast<int32_t>(b);
                break;
            }
        }

        tempU_[i] = u;
        tempV_[i] = v;

        if (bboxIdx >= 0) {
            uint32_t gb = totalBboxCount + static_cast<uint32_t>(bboxIdx);
            pointToBbox[i] = static_cast<int32_t>(gb);
            ++bboxPointCountsBuf[gb];
        }
    }

    totalBboxCount += nBboxes;

    // ---- 第二趟：计数排序写出 ----
    uint32_t cumOffset = 0;
    for (uint32_t gb = 0; gb < totalBboxCount; ++gb) {
        bboxOffsets[gb] = cumOffset;
        writeCursor[gb]  = 0;
        cumOffset += bboxPointCountsBuf[gb];
    }
    totalCandidateCount = cumOffset;

    for (uint32_t i = 0; i < nPoints; ++i) {
        int32_t gb = pointToBbox[i];
        if (gb >= 0) {
            uint32_t gbu = static_cast<uint32_t>(gb);
            uint32_t pos = bboxOffsets[gbu] + writeCursor[gbu];
            candidatePointBuf[pos] = i;
            bboxPointUBuf_[pos]   = tempU_[i];
            bboxPointVBuf_[pos]   = tempV_[i];
            ++writeCursor[gbu];
        }
    }

    result_.imageTimestampNs = imageTimestampNs;
    result_.lidarTimestampNs = lidarFrame.timestampNs;
    result_.bboxPointIndices = candidatePointBuf;
    result_.bboxPointCounts  = bboxPointCountsBuf;
    result_.bboxPointU       = bboxPointUBuf_;
    result_.bboxPointV       = bboxPointVBuf_;
    result_.bboxCount        = totalBboxCount;

    return true;
}

const FusionResult& LidarCameraFusion::result() const
{
    return result_;
}

uint32_t LidarCameraFusion::behind_camera_count() const
{
    return behindCameraCount;
}

uint32_t LidarCameraFusion::out_of_image_count() const
{
    return outOfImageCount;
}

// ============================================================================
// 数学辅助方法
// ============================================================================

void LidarCameraFusion::transform_point_(float lx, float ly, const float* T,
                                          float& cx, float& cy, float& cz) const
{
    cx = T[0] * lx + T[1] * ly + T[3];
    cy = T[4] * lx + T[5] * ly + T[7];
    cz = T[8] * lx + T[9] * ly + T[11];
}

void LidarCameraFusion::project_point_(float cx, float cy, float cz,
                                        const CameraConfig& cameraCfg,
                                        float& u, float& v) const
{
    float invZ = 1.0f / cz;
    u = cameraCfg.fx * cx * invZ + cameraCfg.cx;
    v = cameraCfg.fy * cy * invZ + cameraCfg.cy;
}

// ============================================================================
// 目标跟踪（委托给 LidarTargetTracker）
// ============================================================================

bool LidarCameraFusion::configure_tracker(const TrackerConfig& config)
{
    if (!tracker_) {
        fprintf(stderr, "[LidarCameraFusion] configure_tracker: "
                "tracker not allocated\n");
        return false;
    }
    if (!tracker_->configure(config)) {
        return false;
    }
    trackerConfig_ = config;
    return true;
}

// ============================================================================
// 运行时配置查询/更新
// ============================================================================

const TrackerConfig& LidarCameraFusion::get_tracker_config() const
{
    return trackerConfig_;
}

bool LidarCameraFusion::get_camera_config(uint32_t camIndex, CameraConfig& outCfg) const
{
    if (camIndex >= camCount_) {
        fprintf(stderr, "[LidarCameraFusion] get_camera_config: "
                "camIndex %u out of range (camCount=%u)\n", camIndex, camCount_);
        return false;
    }
    outCfg = camConfigs_[camIndex];
    return true;
}

bool LidarCameraFusion::update_camera_intrinsics(uint32_t camIndex,
                                                  float fx, float fy,
                                                  float cx, float cy,
                                                  uint32_t imgWidth,
                                                  uint32_t imgHeight)
{
    if (camIndex >= camCount_) {
        fprintf(stderr, "[LidarCameraFusion] update_camera_intrinsics: "
                "camIndex %u out of range (camCount=%u)\n", camIndex, camCount_);
        return false;
    }
    camConfigs_[camIndex].fx        = fx;
    camConfigs_[camIndex].fy        = fy;
    camConfigs_[camIndex].cx        = cx;
    camConfigs_[camIndex].cy        = cy;
    camConfigs_[camIndex].imgWidth  = imgWidth;
    camConfigs_[camIndex].imgHeight = imgHeight;
    return true;
}

uint32_t LidarCameraFusion::get_cam_count() const
{
    return camCount_;
}

bool LidarCameraFusion::enable_tracking(bool enable)
{
    if (enable && !tracker_) {
        fprintf(stderr, "[LidarCameraFusion] enable_tracking: "
                "tracker not allocated\n");
        return false;
    }
    trackingEnabled_ = enable;
    return true;
}

void LidarCameraFusion::reset_tracking()
{
    if (tracker_) {
        tracker_->reset();
    }
}

bool LidarCameraFusion::update_tracking(const FusionResult& fusionResult,
                                         const LidarPoint* lidarPoints,
                                         uint32_t pointCount,
                                         const YoloBBox* bboxes,
                                         uint32_t bboxCount,
                                         uint64_t timestampNs)
{
    if (!trackingEnabled_) {
        return true;
    }
    if (!tracker_) {
        return false;
    }
    return tracker_->update(fusionResult, lidarPoints, pointCount,
                            bboxes, bboxCount, timestampNs);
}

void LidarCameraFusion::register_warning_callback(TrackingCallback cb,
                                                    void* userData)
{
    if (tracker_) {
        tracker_->register_callback(cb, userData);
    }
}

bool LidarCameraFusion::copy_tracked_targets(TrackedTarget* out,
                                              uint32_t maxCount,
                                              uint32_t* outCount) const
{
    if (!tracker_) {
        if (outCount) *outCount = 0;
        return false;
    }
    return tracker_->copy_snapshot(out, maxCount, outCount);
}

bool LidarCameraFusion::try_get_lidar_osd_snapshot(LidarOsdSnapshot& out, int timeoutMs)
{
    (void)timeoutMs;
    std::lock_guard<std::mutex> lock(osdSnapshotMutex_);
    if (latestOsdSnapshot_.camCount == 0) return false;
    out = latestOsdSnapshot_;
    return true;
}
