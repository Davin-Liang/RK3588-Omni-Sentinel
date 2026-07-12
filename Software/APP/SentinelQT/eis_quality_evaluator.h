#ifndef EIS_QUALITY_EVALUATOR_H
#define EIS_QUALITY_EVALUATOR_H

#include <QImage>

#include <opencv2/core.hpp>

#include <deque>

/**
 * @brief IMU-only EIS 的图像效果评价结果。
 *
 * 注意：该类只分析防抖后的预览图像，不参与防抖控制，
 * 因此不会破坏“IMU-only 防抖”的设计。
 */
struct EisQualityMetrics {
    bool valid = false;
    bool baselineReady = false;

    // 开启防抖后，画面剩余的高频运动强度，单位：原始预览像素 RMS。
    float residualJitterRmsPx = 0.0f;

    // 相对关闭防抖时基线的抖动下降比例，单位：%。
    // 正数表示改善，负数表示防抖后反而更抖。
    float suppressionPercent = 0.0f;
};

/**
 * @brief 使用相邻预览帧的全局运动，实时评价 EIS 效果。
 *
 * 流程：角点 -> LK 光流 -> RANSAC 全局仿射 -> 去除低频运动趋势
 *      -> 计算高频残余运动 RMS。
 */
class EisQualityEvaluator
{
public:
    EisQualityEvaluator();

    void reset();
    void setEisEnabled(bool enabled);

    /**
     * @brief 处理一帧预览图。
     * @return 本次是否得到可信的全局运动估计。
     */
    bool processFrame(const QImage& image, EisQualityMetrics& metrics);

    EisQualityMetrics latestMetrics() const { return latestMetrics_; }

private:
    static float rms_(const std::deque<float>& values);
    static void pushFixed_(std::deque<float>& values, float value, std::size_t maxSize);

    cv::Mat previousGray_;

    bool eisEnabled_ = false;
    bool trendInitialized_ = false;
    float trendX_ = 0.0f;
    float trendY_ = 0.0f;

    // 防抖关闭时滚动采集的 3 秒基线；开启时冻结该基线。
    std::deque<float> baselineWindow_;

    // 防抖开启后的 2 秒残余抖动窗口。
    std::deque<float> activeWindow_;

    EisQualityMetrics latestMetrics_;
};

#endif // EIS_QUALITY_EVALUATOR_H
