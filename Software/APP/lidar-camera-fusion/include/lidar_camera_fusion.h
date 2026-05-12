#ifndef LIDAR_CAMERA_FUSION_H
#define LIDAR_CAMERA_FUSION_H

#include <cstdint>
#include <vector>

#include "sentinel_lslidarer.h"

/**
 * @struct YoloBBox
 * @brief  YOLO 目标检测输出的单个 2D 边界框。
 * @note   定义在此处是因为项目中 YOLO 模块尚未实现；
 *         当 YOLO 组件就绪后，此结构体可提取到共享头文件中。
 */
struct YoloBBox {
    uint32_t x1, y1;       ///< 左上角像素坐标（包含）
    uint32_t x2, y2;       ///< 右下角像素坐标（不包含），宽度 = x2 - x1
    uint32_t classId;      ///< 类别 ID（COCO 格式）
    float    confidence;   ///< 置信度 [0.0, 1.0]
};

/**
 * @struct CameraConfig
 * @brief  单个相机的内参 + 外参 + 图像尺寸。
 *         作为 fuse_data() 的参数传入，不同相机使用不同的 CameraConfig。
 */
struct CameraConfig {
    float    fx, fy, cx, cy;       ///< 相机内参（4 参数针孔模型）
    float    tLidarToCam[16];      ///< 雷达到该相机的 4x4 外参矩阵（行主序）
    uint32_t imgWidth;             ///< 图像宽度（像素）
    uint32_t imgHeight;            ///< 图像高度（像素）
};

/**
 * @struct FusionResult
 * @brief  融合输出（只读，内存由 LidarCameraFusion 管理）。
 *         内容在下一次 reset() 调用时被覆盖。
 *
 * bbox 按 fuse_data() 调用顺序排列：
 *   首次调用 fuse_data(A, ...) → bbox 0 .. nA-1
 *   二次调用 fuse_data(B, ...) → bbox nA .. nA+nB-1
 */
struct FusionResult {
    uint64_t imageTimestampNs;          ///< 图像帧时间戳（CLOCK_MONOTONIC, ns）
    uint64_t lidarTimestampNs;          ///< 雷达帧时间戳（CLOCK_MONOTONIC, ns）

    const uint32_t* bboxPointIndices;   ///< 展平的点索引数组，按 bbox 分段
    const uint32_t* bboxPointCounts;    ///< 数组，bboxPointCounts[i] = 第 i 个 bbox 的点数量
    uint32_t bboxCount;                 ///< 已累积的 bbox 总数
};

/**
 * @class LidarCameraFusion
 * @brief 视觉-雷达数据融合（支持单相机 / 多相机累积融合）。
 *
 * 单相机用法：
 * @code
 * LidarCameraFusion fusion;
 * fusion.reset();
 * fusion.fuse_data(detections, imageTs, lidarFrame, cameraCfg);
 * const FusionResult& r = fusion.result();
 * // r.bboxCount = detections.size()
 * @endcode
 *
 * 双相机用法：
 * @code
 * LidarCameraFusion fusion;
 * fusion.reset();
 * fusion.fuse_data(detectionsA, imageTs, lidarFrame, cameraCfgA);
 * fusion.fuse_data(detectionsB, imageTs, lidarFrame, cameraCfgB);
 * const FusionResult& r = fusion.result();
 * // r.bboxCount = detectionsA.size() + detectionsB.size()
 * @endcode
 */
class LidarCameraFusion {
public:
    static constexpr uint32_t kMaxLidarPoints = 540;   ///< N10Plus 单圈最大点数
    static constexpr uint32_t kMaxDetections  = 100;   ///< 累积最大 bbox 数

    LidarCameraFusion();
    ~LidarCameraFusion();

    // 禁止拷贝
    LidarCameraFusion(const LidarCameraFusion&) = delete;
    LidarCameraFusion& operator=(const LidarCameraFusion&) = delete;

    /**
     * @brief  重置内部状态，开始融合一帧新的雷达点云。
     *         必须在每次新的雷达帧融合前调用一次。
     *         单相机：reset() → fuse_data() → result()
     *         双相机：reset() → fuse_data(A) → fuse_data(B) → result()
     */
    void reset();

    /**
     * @brief  对一帧 YOLO 检测结果和一帧雷达点云执行数据融合（累积模式）。
     *         每次调用将当前相机的 bbox 追加到内部缓冲区。
     *         同一雷达帧的多相机调用之间不要调用 reset()。
     * @param  detections       YOLO 检测框列表
     * @param  imageTimestampNs 图像帧时间戳（CLOCK_MONOTONIC, ns）
     * @param  lidarFrame       雷达点云帧
     * @param  cameraCfg        该相机的内参 + 外参 + 图像尺寸
     * @return true 融合成功，false 缓冲区满或未调用 reset()
     */
    bool fuse_data(const std::vector<YoloBBox>& detections,
                   uint64_t imageTimestampNs,
                   const LidarFrame& lidarFrame,
                   const CameraConfig& cameraCfg);

    /**
     * @brief  获取累积融合结果（只读）。
     *         返回的引用在下一次 reset() 调用前有效。
     * @return FusionResult 的 const 引用
     */
    const FusionResult& result() const;

    /**
     * @brief  本轮累计的"相机后方"点数。
     */
    uint32_t behind_camera_count() const;

    /**
     * @brief  本轮累计的"图像平面外"点数。
     */
    uint32_t out_of_image_count() const;

private:
    /**
     * @brief 外参变换：P_cam = T * (lx, ly, 0, 1)^T
     */
    void transform_point_(float lx, float ly, const float* T,
                          float& cx, float& cy, float& cz) const;

    /**
     * @brief 内参投影：(u, v, 1)^T = K * (cx/cz, cy/cz, 1)^T
     */
    void project_point_(float cx, float cy, float cz,
                        const CameraConfig& cameraCfg,
                        float& u, float& v) const;

    // 预分配缓冲区（构造时分配，析构时释放）
    uint32_t* candidatePointBuf;     // kMaxLidarPoints，展平存储所有候选点索引
    int32_t*  pointToBbox;          // kMaxLidarPoints（-1 = 未匹配任何 bbox）
    uint32_t* bboxPointCountsBuf;   // kMaxDetections，每个 bbox 的命中点数
    uint32_t* bboxOffsets;          // kMaxDetections，每个 bbox 在 candidatePointBuf 中的偏移
    uint32_t* writeCursor;          // kMaxDetections，第二趟写入游标

    FusionResult result_;

    uint32_t totalBboxCount;        // 本轮已累积的 bbox 总数
    uint32_t totalCandidateCount;   // 本轮已累积的候选点总数
    uint32_t behindCameraCount;     // 本轮累计相机后方点数
    uint32_t outOfImageCount;       // 本轮累计图像外点数
};

#endif // LIDAR_CAMERA_FUSION_H
