#include "lidar_camera_fusion.h"

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
    , totalBboxCount(0)
    , totalCandidateCount(0)
    , behindCameraCount(0)
    , outOfImageCount(0)
{
    candidatePointBuf  = new (std::nothrow) uint32_t[kMaxLidarPoints];
    pointToBbox        = new (std::nothrow) int32_t[kMaxLidarPoints];
    bboxPointCountsBuf = new (std::nothrow) uint32_t[kMaxDetections];
    bboxOffsets        = new (std::nothrow) uint32_t[kMaxDetections];
    writeCursor        = new (std::nothrow) uint32_t[kMaxDetections];

    if (!candidatePointBuf || !pointToBbox || !bboxPointCountsBuf ||
        !bboxOffsets || !writeCursor) {
        fprintf(stderr, "[LidarCameraFusion] buffer allocation failed in constructor\n");
        delete[] candidatePointBuf;   candidatePointBuf  = nullptr;
        delete[] pointToBbox;         pointToBbox        = nullptr;
        delete[] bboxPointCountsBuf;  bboxPointCountsBuf = nullptr;
        delete[] bboxOffsets;         bboxOffsets        = nullptr;
        delete[] writeCursor;         writeCursor        = nullptr;
    }

    std::memset(&result_, 0, sizeof(result_));
}

LidarCameraFusion::~LidarCameraFusion()
{
    delete[] candidatePointBuf;
    delete[] pointToBbox;
    delete[] bboxPointCountsBuf;
    delete[] bboxOffsets;
    delete[] writeCursor;
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

    // pointToBbox: -1 = 未匹配
    for (uint32_t i = 0; i < kMaxLidarPoints; ++i) {
        pointToBbox[i] = -1;
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
        return true;  // 没有检测框，无需处理
    }

    // ---- 第一趟：变换 + 投影 + 分类 ----
    // 只处理尚未匹配的点（pointToBbox[i] == -1）
    for (uint32_t b = 0; b < nBboxes; ++b) {
        uint32_t gb = totalBboxCount + b;
        bboxPointCountsBuf[gb] = 0;
    }

    for (uint32_t i = 0; i < nPoints; ++i) {
        if (pointToBbox[i] >= 0) {
            continue;  // 已被前序调用匹配
        }

        float lx = lidarFrame.points[i].x;
        float ly = lidarFrame.points[i].y;

        // 外参变换
        float cx, cy, cz;
        transform_point_(lx, ly, cameraCfg.tLidarToCam, cx, cy, cz);

        // 相机后方判定
        if (cz <= 0.0f) {
            ++behindCameraCount;
            continue;
        }

        // 内参投影
        float u, v;
        project_point_(cx, cy, cz, cameraCfg, u, v);

        // 图像边界判定
        float fImgWidth  = static_cast<float>(cameraCfg.imgWidth);
        float fImgHeight = static_cast<float>(cameraCfg.imgHeight);
        if (u < 0.0f || u >= fImgWidth || v < 0.0f || v >= fImgHeight) {
            ++outOfImageCount;
            continue;
        }

        // bbox 归属判定（首次命中即跳出）
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

        if (bboxIdx >= 0) {
            uint32_t gb = totalBboxCount + static_cast<uint32_t>(bboxIdx);
            pointToBbox[i] = static_cast<int32_t>(gb);
            ++bboxPointCountsBuf[gb];
        }
    }

    // ---- 更新 totalBboxCount ----
    totalBboxCount += nBboxes;

    // ---- 第二趟：计数排序写出 ----
    // 计算所有 bbox 的偏移，从 totalCandidateCount 位置开始写入
    uint32_t cumOffset = 0;
    for (uint32_t gb = 0; gb < totalBboxCount; ++gb) {
        bboxOffsets[gb] = cumOffset;
        writeCursor[gb]  = 0;
        cumOffset += bboxPointCountsBuf[gb];
    }
    totalCandidateCount = cumOffset;

    // 写出所有已匹配的点（包括前序调用中已匹配的点，幂等操作）
    for (uint32_t i = 0; i < nPoints; ++i) {
        int32_t gb = pointToBbox[i];
        if (gb >= 0) {
            uint32_t gbu = static_cast<uint32_t>(gb);
            candidatePointBuf[bboxOffsets[gbu] + writeCursor[gbu]] = i;
            ++writeCursor[gbu];
        }
    }

    // ---- 填充结果 ----
    result_.imageTimestampNs = imageTimestampNs;
    result_.lidarTimestampNs = lidarFrame.timestampNs;
    result_.bboxPointIndices = candidatePointBuf;
    result_.bboxPointCounts  = bboxPointCountsBuf;
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
    // P_cam = T * (lx, ly, 0, 1)^T
    // T 为 4x4 行主序，省略 z=0 和 w=1 项
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
