/*
 * vision_eis.hpp - 视觉为主 + IMU辅助的电子防抖接口
 *
 * 设计目标：
 * 1. 视觉侧通过 LK 光流 + RANSAC 估计相邻帧全局运动 dx/dy/dtheta；
 * 2. 对累计运动轨迹做实时平滑，输出 offsetX / offsetY；
 * 3. IMU 不再直接决定 offset，而是提供 gyro_rms / vibration_level，辅助调节平滑强度；
 * 4. 该模块不依赖训练模型，只依赖 OpenCV 传统视觉算法。
 */

#ifndef __VISION_EIS_HPP__
#define __VISION_EIS_HPP__

#include <stdint.h>
#include <opencv2/opencv.hpp>

/* IMU 辅助状态：可由 ICM45686 读取线程或上层融合模块提供。
 * 注意：视觉 EIS 可以在 imuState == nullptr 时独立运行，此时使用中等震动参数。
 */
struct VisionImuAssistState {
    uint64_t timestampNs;

    float gyroX;
    float gyroY;
    float gyroZ;
    float accelX;
    float accelY;
    float accelZ;

    float gyroNorm;       /* 当前角速度模长，单位 rad/s */
    float accelNorm;      /* 当前加速度模长，单位 m/s^2 */
    float gyroRms;        /* 最近一段窗口内的角速度 RMS，单位 rad/s */
    int vibrationLevel;   /* 0:低震动 1:中震动 2:高震动 */

    VisionImuAssistState()
        : timestampNs(0),
          gyroX(0.0f), gyroY(0.0f), gyroZ(0.0f),
          accelX(0.0f), accelY(0.0f), accelZ(0.0f),
          gyroNorm(0.0f), accelNorm(0.0f), gyroRms(0.0f),
          vibrationLevel(1)
    {
    }
};

/* 每路相机一份视觉 EIS 配置。
 * 两个相机必须分别维护 VisionEisStabilizer，不能共用 prevFrame / 轨迹状态。
 */
struct VisionEisConfig {
    int camId;

    int inputWidth;          /* 原始图像宽度，例如 1920 */
    int inputHeight;         /* 原始图像高度，例如 1080 */
    int processWidth;        /* 视觉估计用缩放宽度，建议 480/640/960 */
    int processHeight;       /* 视觉估计用缩放高度，建议按原图比例设置 */

    int maxCorners;          /* goodFeaturesToTrack 最大角点数 */
    double qualityLevel;     /* 角点质量阈值 */
    double minDistance;      /* 角点最小间距 */
    int minTrackedPoints;    /* LK 跟踪后最少有效点数 */
    int minInliers;          /* RANSAC 最少内点数 */
    double ransacThreshold;  /* RANSAC 重投影阈值 */
    double maxOpticalFlow;   /* 单个特征点最大允许位移，过滤离群点 */

    int maxOffsetPixel;      /* 输出 offset 限幅，避免裁剪越界 */

    /*
     * 输出补偿方向与强度。
     * 视觉轨迹平滑得到的是“理想稳定轨迹 - 原始轨迹”的补偿量，
     * 但不同 RGA 裁剪/写入方式对 offset 正负号的解释可能不同。
     * 因此这里显式提供符号和增益配置，便于按实际链路标定。
     * 本项目当前 RGA crop 链路下，默认使用 -1/-1，避免把抖动同向放大。
     */
    int outputSignX;         /* 1 或 -1，控制最终 offsetX 方向 */
    int outputSignY;         /* 1 或 -1，控制最终 offsetY 方向 */
    float offsetGainX;       /* offsetX 增益，调试阶段建议 0.6~1.0 */
    float offsetGainY;       /* offsetY 增益，调试阶段建议 0.6~1.0 */
    int maxOffsetStepPixel;  /* 单帧 offset 最大变化量，<=0 表示不限制 */
    float minMotionPixel;    /* 小于该帧间运动时认为是噪声，不更新轨迹 */

    bool enableImuAdaptiveAlpha; /* 是否根据 IMU 震动等级调整 alpha */
    float alphaLowVibration;     /* 低震动 alpha。alpha 越大越跟手，防抖越弱 */
    float alphaMidVibration;     /* 中震动 alpha */
    float alphaHighVibration;    /* 高震动 alpha。实时链路建议不要过小 */

    bool enableRotationEstimate; /* 当前只记录 dtheta，默认不做旋转补偿 */

    VisionEisConfig()
        : camId(0),
          inputWidth(1920), inputHeight(1080),
          processWidth(640), processHeight(360),
          maxCorners(500), qualityLevel(0.01), minDistance(10.0),
          minTrackedPoints(30), minInliers(20), ransacThreshold(3.0),
          maxOpticalFlow(80.0), maxOffsetPixel(30),
          outputSignX(-1), outputSignY(-1),
          offsetGainX(1.0f), offsetGainY(1.0f),
          maxOffsetStepPixel(8), minMotionPixel(0.20f),
          enableImuAdaptiveAlpha(false),
          alphaLowVibration(0.45f),
          alphaMidVibration(0.45f),
          alphaHighVibration(0.45f),
          enableRotationEstimate(true)
    {
    }
};

struct VisionEisResult {
    uint64_t frameTimestampNs;

    bool success;           /* 本帧是否成功估计视觉运动 */
    bool visualReliable;    /* 视觉特征点和 RANSAC 内点是否足够可靠 */
    bool usedFallback;      /* 视觉失败时是否使用上一帧 offset 衰减回退 */

    float dx;               /* 当前帧相对上一帧的全局水平位移，原图像素 */
    float dy;               /* 当前帧相对上一帧的全局垂直位移，原图像素 */
    float dtheta;           /* 当前帧相对上一帧的旋转角，弧度 */

    float trajectoryX;      /* 累计原始运动轨迹 X */
    float trajectoryY;      /* 累计原始运动轨迹 Y */
    float trajectoryA;      /* 累计原始旋转轨迹 */

    float smoothX;          /* 平滑轨迹 X */
    float smoothY;          /* 平滑轨迹 Y */
    float smoothA;          /* 平滑轨迹 A */

    int offsetX;            /* 输出给 RGA 的水平补偿像素 */
    int offsetY;            /* 输出给 RGA 的垂直补偿像素 */

    int trackedPoints;      /* LK 跟踪后有效点数量 */
    int inliers;            /* RANSAC 内点数量 */
    float usedAlpha;        /* 本帧实际使用的轨迹平滑 alpha */
    double costMs;          /* 本帧视觉 EIS 计算耗时 */

    VisionEisResult()
        : frameTimestampNs(0),
          success(false), visualReliable(false), usedFallback(false),
          dx(0.0f), dy(0.0f), dtheta(0.0f),
          trajectoryX(0.0f), trajectoryY(0.0f), trajectoryA(0.0f),
          smoothX(0.0f), smoothY(0.0f), smoothA(0.0f),
          offsetX(0), offsetY(0), trackedPoints(0), inliers(0),
          usedAlpha(0.0f), costMs(0.0)
    {
    }
};

class VisionEisStabilizer {
public:
    explicit VisionEisStabilizer(const VisionEisConfig& config = VisionEisConfig());

    void reset();
    void setConfig(const VisionEisConfig& config);
    const VisionEisConfig& config() const;

    /*
     * 输入当前帧，输出本帧稳定所需 offset。
     * frame 可以是 BGR/RGB/GRAY，模块内部会转灰度并缩放。
     * imuState 可以为 nullptr，此时视觉 EIS 仍可独立运行。
     */
    bool processFrame(const cv::Mat& frame,
                      uint64_t timestampNs,
                      const VisionImuAssistState* imuState,
                      VisionEisResult& result);

private:
    cv::Mat preprocessFrame(const cv::Mat& frame) const;
    bool estimateGlobalMotion(const cv::Mat& prevGray,
                              const cv::Mat& currGray,
                              float& dx,
                              float& dy,
                              float& dtheta,
                              int& trackedPoints,
                              int& inliers) const;
    float chooseAlpha(const VisionImuAssistState* imuState) const;
    int clampOffset(int value) const;
    bool fallbackResult(uint64_t timestampNs, VisionEisResult& result);

private:
    VisionEisConfig config_;
    cv::Mat prevGray_;
    bool hasPrev_;
    bool smoothInitialized_;

    float trajectoryX_;
    float trajectoryY_;
    float trajectoryA_;
    float smoothX_;
    float smoothY_;
    float smoothA_;
    int lastOffsetX_;
    int lastOffsetY_;
};

#endif /* __VISION_EIS_HPP__ */

