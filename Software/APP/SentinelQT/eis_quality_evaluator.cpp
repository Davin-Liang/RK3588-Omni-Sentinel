#include "eis_quality_evaluator.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

namespace {
constexpr int kEvaluationWidth = 480;
constexpr std::size_t kBaselineWindowSize = 30; // 10 Hz 下约 3 秒
constexpr std::size_t kActiveWindowSize = 20;   // 10 Hz 下约 2 秒
constexpr std::size_t kMinBaselineSamples = 15;
constexpr std::size_t kMinActiveSamples = 8;
constexpr float kTrendAlpha = 0.18f;
}

EisQualityEvaluator::EisQualityEvaluator() = default;

void EisQualityEvaluator::reset()
{
    previousGray_.release();
    eisEnabled_ = false;
    trendInitialized_ = false;
    trendX_ = 0.0f;
    trendY_ = 0.0f;
    baselineWindow_.clear();
    activeWindow_.clear();
    latestMetrics_ = EisQualityMetrics();
}

void EisQualityEvaluator::setEisEnabled(bool enabled)
{
    if (eisEnabled_ == enabled) {
        return;
    }

    eisEnabled_ = enabled;

    // 切换开关时，前后两帧不属于同一处理状态，不能直接做光流比较。
    previousGray_.release();
    trendInitialized_ = false;
    trendX_ = 0.0f;
    trendY_ = 0.0f;
    activeWindow_.clear();

    latestMetrics_.valid = false;
    // 只要有效样本数足够就认为基线已采集完成。
    // 真正计算抑振率时再对过小的基线 RMS 做下限保护，
    // 避免静止场景中永远显示“采集基线中”。
    latestMetrics_.baselineReady =
        baselineWindow_.size() >= kMinBaselineSamples;
    latestMetrics_.residualJitterRmsPx = 0.0f;
    latestMetrics_.suppressionPercent = 0.0f;
}

float EisQualityEvaluator::rms_(const std::deque<float>& values)
{
    if (values.empty()) {
        return 0.0f;
    }

    double squareSum = 0.0;
    for (float value : values) {
        squareSum += static_cast<double>(value) * static_cast<double>(value);
    }
    return static_cast<float>(std::sqrt(squareSum / static_cast<double>(values.size())));
}

void EisQualityEvaluator::pushFixed_(std::deque<float>& values,
                                     float value,
                                     std::size_t maxSize)
{
    values.push_back(value);
    while (values.size() > maxSize) {
        values.pop_front();
    }
}

bool EisQualityEvaluator::processFrame(const QImage& image,
                                       EisQualityMetrics& metrics)
{
    if (image.isNull() || image.width() < 64 || image.height() < 64) {
        return false;
    }

    // QImage 转 RGB888，再转灰度，避免依赖输入帧原本的像素格式。
    const QImage rgbImage = image.convertToFormat(QImage::Format_RGB888);
    cv::Mat rgb(rgbImage.height(),
                rgbImage.width(),
                CV_8UC3,
                const_cast<uchar*>(rgbImage.constBits()),
                rgbImage.bytesPerLine());

    cv::Mat grayFull;
    cv::cvtColor(rgb, grayFull, cv::COLOR_RGB2GRAY);

    const int evalWidth = std::min(kEvaluationWidth, grayFull.cols);
    const int evalHeight = std::max(
        1,
        static_cast<int>(std::lround(
            static_cast<double>(grayFull.rows) * evalWidth / grayFull.cols)));

    cv::Mat gray;
    cv::resize(grayFull, gray, cv::Size(evalWidth, evalHeight), 0.0, 0.0, cv::INTER_AREA);

    if (previousGray_.empty()) {
        previousGray_ = gray.clone();
        return false;
    }

    std::vector<cv::Point2f> previousPoints;
    cv::goodFeaturesToTrack(previousGray_,
                            previousPoints,
                            250,
                            0.01,
                            7.0,
                            cv::Mat(),
                            3,
                            false,
                            0.04);

    if (previousPoints.size() < 30) {
        previousGray_ = gray.clone();
        return false;
    }

    std::vector<cv::Point2f> currentPoints;
    std::vector<uchar> forwardStatus;
    std::vector<float> forwardError;
    cv::calcOpticalFlowPyrLK(previousGray_,
                             gray,
                             previousPoints,
                             currentPoints,
                             forwardStatus,
                             forwardError,
                             cv::Size(21, 21),
                             3);

    // 前后向校验，降低移动人员、误匹配和模糊造成的评价波动。
    std::vector<cv::Point2f> backwardPoints;
    std::vector<uchar> backwardStatus;
    std::vector<float> backwardError;
    cv::calcOpticalFlowPyrLK(gray,
                             previousGray_,
                             currentPoints,
                             backwardPoints,
                             backwardStatus,
                             backwardError,
                             cv::Size(21, 21),
                             3);

    std::vector<cv::Point2f> validPrevious;
    std::vector<cv::Point2f> validCurrent;
    validPrevious.reserve(previousPoints.size());
    validCurrent.reserve(previousPoints.size());

    for (std::size_t i = 0; i < previousPoints.size(); ++i) {
        if (!forwardStatus[i] || !backwardStatus[i]) {
            continue;
        }
        if (forwardError[i] > 30.0f) {
            continue;
        }

        const float dx = backwardPoints[i].x - previousPoints[i].x;
        const float dy = backwardPoints[i].y - previousPoints[i].y;
        const float backError = std::sqrt(dx * dx + dy * dy);
        if (backError > 1.5f) {
            continue;
        }

        validPrevious.push_back(previousPoints[i]);
        validCurrent.push_back(currentPoints[i]);
    }

    if (validPrevious.size() < 20) {
        previousGray_ = gray.clone();
        return false;
    }

    cv::Mat inlierMask;
    const cv::Mat affine = cv::estimateAffinePartial2D(validPrevious,
                                                       validCurrent,
                                                       inlierMask,
                                                       cv::RANSAC,
                                                       2.0,
                                                       1000,
                                                       0.99,
                                                       10);

    if (affine.empty()) {
        previousGray_ = gray.clone();
        return false;
    }

    const int inlierCount = cv::countNonZero(inlierMask);
    const float inlierRatio = static_cast<float>(inlierCount) /
                              static_cast<float>(validPrevious.size());
    if (inlierCount < 15 || inlierRatio < 0.35f) {
        previousGray_ = gray.clone();
        return false;
    }

    // 仿射平移是在 480 宽评价图上的像素，换算回实际预览图像素。
    const float scaleX = static_cast<float>(rgbImage.width()) /
                         static_cast<float>(evalWidth);
    const float scaleY = static_cast<float>(rgbImage.height()) /
                         static_cast<float>(evalHeight);

    const float motionX = static_cast<float>(affine.at<double>(0, 2)) * scaleX;
    const float motionY = static_cast<float>(affine.at<double>(1, 2)) * scaleY;

    if (!std::isfinite(motionX) || !std::isfinite(motionY)) {
        previousGray_ = gray.clone();
        return false;
    }

    // EMA 估计慢速主动移动趋势，高频剩余量作为“抖动”。
    if (!trendInitialized_) {
        trendX_ = motionX;
        trendY_ = motionY;
        trendInitialized_ = true;
    } else {
        trendX_ += kTrendAlpha * (motionX - trendX_);
        trendY_ += kTrendAlpha * (motionY - trendY_);
    }

    const float jitterX = motionX - trendX_;
    const float jitterY = motionY - trendY_;
    const float jitterMagnitude = std::sqrt(jitterX * jitterX + jitterY * jitterY);

    if (!eisEnabled_) {
        pushFixed_(baselineWindow_, jitterMagnitude, kBaselineWindowSize);
    } else {
        pushFixed_(activeWindow_, jitterMagnitude, kActiveWindowSize);
    }

    const float baselineRms = rms_(baselineWindow_);
    const float activeRms = rms_(activeWindow_);

    latestMetrics_.baselineReady =
        baselineWindow_.size() >= kMinBaselineSamples;

    if (eisEnabled_) {
        latestMetrics_.valid = activeWindow_.size() >= kMinActiveSamples;
        latestMetrics_.residualJitterRmsPx = activeRms;

        if (latestMetrics_.valid && latestMetrics_.baselineReady) {
            // 对极小基线增加分母下限，保证数值稳定。
            // 调参时仍应在相似振动条件下完成“关闭基线”和“开启评价”。
            const float safeBaselineRms = std::max(baselineRms, 0.10f);
            float suppression = 100.0f * (1.0f - activeRms / safeBaselineRms);
            // 保留负值用于识别“防抖反而放大抖动”，同时避免异常值撑爆显示。
            suppression = std::max(-100.0f, std::min(100.0f, suppression));
            latestMetrics_.suppressionPercent = suppression;
        } else {
            latestMetrics_.suppressionPercent = 0.0f;
        }
    } else {
        latestMetrics_.valid = false;
        latestMetrics_.residualJitterRmsPx = 0.0f;
        latestMetrics_.suppressionPercent = 0.0f;
    }

    metrics = latestMetrics_;
    previousGray_ = gray.clone();
    return true;
}
