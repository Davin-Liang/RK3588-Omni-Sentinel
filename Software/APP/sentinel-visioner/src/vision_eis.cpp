/*
 * vision_eis.cpp - 视觉为主 + IMU辅助的电子防抖实现
 */

#include "vision_eis.hpp"
#include <algorithm>
#include <cmath>
#include <chrono>
#include <iostream>

namespace {
static int clamp_int(int v, int lo, int hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
}

VisionEisStabilizer::VisionEisStabilizer(const VisionEisConfig& config)
    : config_(config),
      hasPrev_(false),
      smoothInitialized_(false),
      trajectoryX_(0.0f), trajectoryY_(0.0f), trajectoryA_(0.0f),
      smoothX_(0.0f), smoothY_(0.0f), smoothA_(0.0f),
      lastOffsetX_(0), lastOffsetY_(0)
{
}

void VisionEisStabilizer::reset()
{
    prevGray_.release();
    hasPrev_ = false;
    smoothInitialized_ = false;
    trajectoryX_ = 0.0f;
    trajectoryY_ = 0.0f;
    trajectoryA_ = 0.0f;
    smoothX_ = 0.0f;
    smoothY_ = 0.0f;
    smoothA_ = 0.0f;
    lastOffsetX_ = 0;
    lastOffsetY_ = 0;
}

void VisionEisStabilizer::setConfig(const VisionEisConfig& config)
{
    config_ = config;
    reset();
}

const VisionEisConfig& VisionEisStabilizer::config() const
{
    return config_;
}

cv::Mat VisionEisStabilizer::preprocessFrame(const cv::Mat& frame) const
{
    cv::Mat gray;
    cv::Mat resized;

    if (frame.empty()) {
        return gray;
    }

    if (frame.channels() == 3) {
        /* RGB 和 BGR 转灰度只存在权重通道差异，防抖运动估计可接受。
         * 如果上层明确是 RGB，也可以改成 COLOR_RGB2GRAY。
         */
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else if (frame.channels() == 4) {
        cv::cvtColor(frame, gray, cv::COLOR_BGRA2GRAY);
    } else {
        gray = frame.clone();
    }

    if (config_.processWidth > 0 && config_.processHeight > 0 &&
        (gray.cols != config_.processWidth || gray.rows != config_.processHeight)) {
        cv::resize(gray, resized,
                   cv::Size(config_.processWidth, config_.processHeight),
                   0.0, 0.0, cv::INTER_AREA);
        return resized;
    }

    return gray;
}

bool VisionEisStabilizer::estimateGlobalMotion(const cv::Mat& prevGray,
                                               const cv::Mat& currGray,
                                               float& dx,
                                               float& dy,
                                               float& dtheta,
                                               int& trackedPoints,
                                               int& inliers) const
{
    std::vector<cv::Point2f> prevPts;
    std::vector<cv::Point2f> currPts;
    std::vector<uchar> status;
    std::vector<float> err;

    trackedPoints = 0;
    inliers = 0;
    dx = 0.0f;
    dy = 0.0f;
    dtheta = 0.0f;

    cv::goodFeaturesToTrack(prevGray,
                            prevPts,
                            config_.maxCorners,
                            config_.qualityLevel,
                            config_.minDistance);

    if ((int)prevPts.size() < config_.minTrackedPoints) {
        return false;
    }

    cv::calcOpticalFlowPyrLK(prevGray, currGray, prevPts, currPts, status, err);

    std::vector<cv::Point2f> goodPrev;
    std::vector<cv::Point2f> goodCurr;
    goodPrev.reserve(prevPts.size());
    goodCurr.reserve(prevPts.size());

    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i]) {
            continue;
        }

        double fx = currPts[i].x - prevPts[i].x;
        double fy = currPts[i].y - prevPts[i].y;
        double dist = std::sqrt(fx * fx + fy * fy);
        if (dist > config_.maxOpticalFlow) {
            continue;
        }

        goodPrev.push_back(prevPts[i]);
        goodCurr.push_back(currPts[i]);
    }

    trackedPoints = static_cast<int>(goodPrev.size());
    if (trackedPoints < config_.minTrackedPoints) {
        return false;
    }

    cv::Mat inlierMask;
    cv::Mat affine = cv::estimateAffinePartial2D(goodPrev,
                                                 goodCurr,
                                                 inlierMask,
                                                 cv::RANSAC,
                                                 config_.ransacThreshold);

    if (affine.empty() || affine.rows != 2 || affine.cols != 3) {
        return false;
    }

    for (int i = 0; i < inlierMask.rows; ++i) {
        if (inlierMask.at<uchar>(i, 0)) {
            inliers++;
        }
    }

    if (inliers < config_.minInliers) {
        return false;
    }

    double a = affine.at<double>(0, 0);
    double b = affine.at<double>(1, 0);
    double tx = affine.at<double>(0, 2);
    double ty = affine.at<double>(1, 2);

    float scaleX = 1.0f;
    float scaleY = 1.0f;
    if (config_.processWidth > 0 && config_.processHeight > 0) {
        scaleX = static_cast<float>(config_.inputWidth) / static_cast<float>(config_.processWidth);
        scaleY = static_cast<float>(config_.inputHeight) / static_cast<float>(config_.processHeight);
    }

    dx = static_cast<float>(tx) * scaleX;
    dy = static_cast<float>(ty) * scaleY;
    dtheta = static_cast<float>(std::atan2(b, a));

    return true;
}

float VisionEisStabilizer::chooseAlpha(const VisionImuAssistState* imuState) const
{
    if (!config_.enableImuAdaptiveAlpha || imuState == nullptr) {
        return config_.alphaMidVibration;
    }

    if (imuState->vibrationLevel <= 0) {
        return config_.alphaLowVibration;
    }
    if (imuState->vibrationLevel >= 2) {
        return config_.alphaHighVibration;
    }
    return config_.alphaMidVibration;
}

int VisionEisStabilizer::clampOffset(int value) const
{
    int maxOffset = config_.maxOffsetPixel;
    if (maxOffset < 0) {
        maxOffset = -maxOffset;
    }
    return clamp_int(value, -maxOffset, maxOffset);
}

bool VisionEisStabilizer::fallbackResult(uint64_t timestampNs, VisionEisResult& result)
{
    /* 视觉估计失败时，不直接输出突变 offset，而是让上一帧 offset 缓慢回零。
     * 这样可以避免低纹理/大面积动态目标导致画面突然跳动。
     */
    lastOffsetX_ = static_cast<int>(lastOffsetX_ * 0.90f);
    lastOffsetY_ = static_cast<int>(lastOffsetY_ * 0.90f);

    result = VisionEisResult();
    result.frameTimestampNs = timestampNs;
    result.success = false;
    result.visualReliable = false;
    result.usedFallback = true;
    result.offsetX = lastOffsetX_;
    result.offsetY = lastOffsetY_;
    return false;
}

bool VisionEisStabilizer::processFrame(const cv::Mat& frame,
                                       uint64_t timestampNs,
                                       const VisionImuAssistState* imuState,
                                       VisionEisResult& result)
{
    auto t0 = std::chrono::high_resolution_clock::now();

    result = VisionEisResult();
    result.frameTimestampNs = timestampNs;

    cv::Mat currGray = preprocessFrame(frame);
    if (currGray.empty()) {
        return fallbackResult(timestampNs, result);
    }

    if (!hasPrev_) {
        /* 第一帧没有前一帧可比，只建立基准，不输出补偿。 */
        prevGray_ = currGray.clone();
        hasPrev_ = true;
        result.success = false;
        result.visualReliable = false;
        result.offsetX = 0;
        result.offsetY = 0;
        return false;
    }

    float dx = 0.0f;
    float dy = 0.0f;
    float dtheta = 0.0f;
    int tracked = 0;
    int inliers = 0;

    bool ok = estimateGlobalMotion(prevGray_, currGray, dx, dy, dtheta, tracked, inliers);
    prevGray_ = currGray.clone();

    if (!ok) {
        return fallbackResult(timestampNs, result);
    }

    /*
     * very small dx/dy is usually feature tracking noise.
     * If we accumulate it forever, even a static camera can slowly drift and produce
     * a non-zero offset.  Here we ignore tiny frame-to-frame translations but still
     * keep the previous gray frame updated above.
     */
    const float motionMag = std::sqrt(dx * dx + dy * dy);
    if (motionMag >= config_.minMotionPixel) {
        trajectoryX_ += dx;
        trajectoryY_ += dy;
        trajectoryA_ += dtheta;
    }

    float alpha = chooseAlpha(imuState);
    if (alpha < 0.01f) alpha = 0.01f;
    if (alpha > 1.0f) alpha = 1.0f;

    if (!smoothInitialized_) {
        /* 第一次成功估计时，平滑轨迹直接对齐原始轨迹，避免起始跳变。 */
        smoothX_ = trajectoryX_;
        smoothY_ = trajectoryY_;
        smoothA_ = trajectoryA_;
        smoothInitialized_ = true;
    } else {
        smoothX_ = alpha * trajectoryX_ + (1.0f - alpha) * smoothX_;
        smoothY_ = alpha * trajectoryY_ + (1.0f - alpha) * smoothY_;
        smoothA_ = alpha * trajectoryA_ + (1.0f - alpha) * smoothA_;
    }

    /*
     * 原始补偿量 = 平滑轨迹 - 原始轨迹。
     * 由于后级 RGA 采用源图裁剪窗口平移，offset 的符号需要和实际裁剪语义匹配。
     * 这里用 outputSignX/Y 做显式配置；根据当前 cam0 评估结果，默认取 -1/-1，
     * 避免把画面运动同向放大。
     */
    float rawOffsetX = smoothX_ - trajectoryX_;
    float rawOffsetY = smoothY_ - trajectoryY_;

    int signX = (config_.outputSignX >= 0) ? 1 : -1;
    int signY = (config_.outputSignY >= 0) ? 1 : -1;
    int offsetX = static_cast<int>(std::round(signX * config_.offsetGainX * rawOffsetX));
    int offsetY = static_cast<int>(std::round(signY * config_.offsetGainY * rawOffsetY));

    /*
     * 实时链路存在一帧延迟，offset 突变会让画面被拉扯，表现为 jitter_eval 变差。
     * 因此限制单帧 offset 变化量，让补偿更平滑、保守。
     */
    if (config_.maxOffsetStepPixel > 0) {
        int step = config_.maxOffsetStepPixel;
        offsetX = clamp_int(offsetX, lastOffsetX_ - step, lastOffsetX_ + step);
        offsetY = clamp_int(offsetY, lastOffsetY_ - step, lastOffsetY_ + step);
    }

    offsetX = clampOffset(offsetX);
    offsetY = clampOffset(offsetY);

    lastOffsetX_ = offsetX;
    lastOffsetY_ = offsetY;

    auto t1 = std::chrono::high_resolution_clock::now();
    double costMs = std::chrono::duration<double, std::milli>(t1 - t0).count();

    result.success = true;
    result.visualReliable = true;
    result.usedFallback = false;
    result.dx = dx;
    result.dy = dy;
    result.dtheta = dtheta;
    result.trajectoryX = trajectoryX_;
    result.trajectoryY = trajectoryY_;
    result.trajectoryA = trajectoryA_;
    result.smoothX = smoothX_;
    result.smoothY = smoothY_;
    result.smoothA = smoothA_;
    result.offsetX = offsetX;
    result.offsetY = offsetY;
    result.trackedPoints = tracked;
    result.inliers = inliers;
    result.usedAlpha = alpha;
    result.costMs = costMs;

    return true;
}

