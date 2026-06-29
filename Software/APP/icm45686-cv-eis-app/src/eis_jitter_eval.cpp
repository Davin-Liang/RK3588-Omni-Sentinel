/*
 * eis_jitter_eval.cpp - EIS画面抖动RMS离线评估工具
 *
 * 功能说明：
 * 1. 输入原始视频 raw.mp4 和防抖后视频 eis.mp4；
 * 2. 通过相邻帧特征点跟踪估计全局运动；
 * 3. 统计每帧的 dx / dy / dtheta；
 * 4. 计算平移抖动 RMS、水平抖动标准差、垂直抖动标准差；
 * 5. 输出防抖前后对比和抖动降低率。
 *
 * 注意：
 * 该工具用于“画面级防抖效果评估”，不是 IMU 读取 Demo。
 * 最推荐的测试场景是相机对准静态标定板、墙面纹理或固定工业设备，
 * 然后人为施加轻微震动，分别录制 raw 和 eis 两路视频进行对比。
 *
 * 日期: 2026-06-08
 */

#include <opencv2/opencv.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

/* 用于保存相邻帧估计出来的全局运动 */
struct FrameMotion {
    double dx;        /* 水平方向平移，单位 pixel */
    double dy;        /* 垂直方向平移，单位 pixel */
    double thetaDeg;  /* 旋转角，单位 degree */
};

/* 用于保存一段视频的抖动统计结果 */
struct JitterMetrics {
    int totalPairs;       /* 相邻帧对总数 */
    int validPairs;       /* 成功估计运动的帧对数 */
    int failedPairs;      /* 估计失败的帧对数 */

    double rmsTrans;      /* 平移抖动RMS：sqrt(mean(dx^2 + dy^2)) */
    double stdDx;         /* 水平抖动标准差 */
    double stdDy;         /* 垂直抖动标准差 */
    double rmsThetaDeg;   /* 旋转抖动RMS，单位degree */

    JitterMetrics()
        : totalPairs(0),
          validPairs(0),
          failedPairs(0),
          rmsTrans(0.0),
          stdDx(0.0),
          stdDy(0.0),
          rmsThetaDeg(0.0)
    {
    }
};

/**************************实现函数********************************************
 *函数原型:     static void print_usage(const char* prog)
 *功    能:     打印使用说明
 *输入参数:     prog - 程序名称
 *输出参数:     无
 ******************************************************************************/
static void print_usage(const char* prog)
{
    std::cout << "Usage:\n";
    std::cout << "  " << prog << " <raw_video> <eis_video> [max_frames]\n\n";
    std::cout << "Example:\n";
    std::cout << "  " << prog << " raw.mp4 eis.mp4\n";
    std::cout << "  " << prog << " raw.mp4 eis.mp4 900\n\n";
    std::cout << "Meaning:\n";
    std::cout << "  raw_video   : 原始未防抖视频\n";
    std::cout << "  eis_video   : 防抖后视频\n";
    std::cout << "  max_frames  : 最多分析多少帧，默认分析整段视频\n\n";
    std::cout << "Recommended scene:\n";
    std::cout << "  1. Camera faces a static target, such as checkerboard, wall texture, AprilTag or fixed equipment.\n";
    std::cout << "  2. Record raw and EIS videos under the same vibration condition.\n";
    std::cout << "  3. Lower RMS means better stabilization.\n";
}

/**************************实现函数********************************************
 *函数原型:     static double calc_reduction(double rawValue, double eisValue)
 *功    能:     计算降低率
 *输入参数:     rawValue - 原始视频指标, eisValue - 防抖后视频指标
 *输出参数:     降低率百分比
 ******************************************************************************/
static double calc_reduction(double rawValue, double eisValue)
{
    if (rawValue <= 1e-9) {
        return 0.0;
    }

    return (rawValue - eisValue) / rawValue * 100.0;
}

/**************************实现函数********************************************
 *函数原型:     static cv::Mat preprocess_frame(const cv::Mat& frame)
 *功    能:     将视频帧转换为灰度图，并适当缩放以降低计算量
 *输入参数:     frame - 输入BGR帧
 *输出参数:     预处理后的灰度图
 ******************************************************************************/
static cv::Mat preprocess_frame(const cv::Mat& frame)
{
    cv::Mat gray;
    cv::Mat resized;

    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.clone();
    }

    /*
     * 为了让评估工具可以在普通PC或板端快速运行，这里对过大的图像做等比例缩小。
     * 注意：缩放会影响dx/dy的绝对像素值。
     * 如果你需要和原分辨率严格对应，可以删除这段缩放逻辑。
     */
    const int maxWidth = 960;
    if (gray.cols > maxWidth) {
        double scale = (double)maxWidth / (double)gray.cols;
        cv::resize(gray, resized, cv::Size(), scale, scale, cv::INTER_AREA);
        return resized;
    }

    return gray;
}

/**************************实现函数********************************************
 *函数原型:     static bool estimate_global_motion(const cv::Mat& prevGray,
 *                                                 const cv::Mat& currGray,
 *                                                 FrameMotion& motion)
 *功    能:     估计两帧之间的全局平移和旋转
 *输入参数:     prevGray - 前一帧灰度图, currGray - 当前帧灰度图
 *输出参数:     motion - 输出全局运动
 *返 回 值:     true估计成功，false估计失败
 ******************************************************************************/
static bool estimate_global_motion(const cv::Mat& prevGray,
                                   const cv::Mat& currGray,
                                   FrameMotion& motion)
{
    std::vector<cv::Point2f> prevPts;
    std::vector<cv::Point2f> currPts;
    std::vector<uchar> status;
    std::vector<float> err;

    /*
     * 1. 在前一帧中提取角点。
     *    maxCorners越大，鲁棒性越好，但计算量也越大。
     */
    cv::goodFeaturesToTrack(prevGray,
                            prevPts,
                            500,     /* maxCorners */
                            0.01,    /* qualityLevel */
                            10.0);   /* minDistance */

    if (prevPts.size() < 30) {
        return false;
    }

    /*
     * 2. 使用LK光流跟踪角点到当前帧。
     */
    cv::calcOpticalFlowPyrLK(prevGray,
                             currGray,
                             prevPts,
                             currPts,
                             status,
                             err);

    std::vector<cv::Point2f> goodPrev;
    std::vector<cv::Point2f> goodCurr;

    for (size_t i = 0; i < status.size(); ++i) {
        if (!status[i]) {
            continue;
        }

        /* 过滤明显异常的光流点，避免极端离群点影响全局运动估计 */
        double dx = currPts[i].x - prevPts[i].x;
        double dy = currPts[i].y - prevPts[i].y;
        double dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 80.0) {
            continue;
        }

        goodPrev.push_back(prevPts[i]);
        goodCurr.push_back(currPts[i]);
    }

    if (goodPrev.size() < 20) {
        return false;
    }

    /*
     * 3. 用RANSAC估计全局仿射变换。
     *    estimateAffinePartial2D 估计的是平移 + 旋转 + 等比例缩放，
     *    对相机小幅抖动场景足够稳定。
     */
    cv::Mat inlierMask;
    cv::Mat affine = cv::estimateAffinePartial2D(goodPrev,
                                                 goodCurr,
                                                 inlierMask,
                                                 cv::RANSAC,
                                                 3.0);

    if (affine.empty() || affine.rows != 2 || affine.cols != 3) {
        return false;
    }

    /*
     * affine =
     * [ a  -b  tx
     *   b   a  ty ]
     *
     * tx/ty 即平移量，atan2(b, a) 可得到旋转角。
     */
    double a = affine.at<double>(0, 0);
    double b = affine.at<double>(1, 0);
    double tx = affine.at<double>(0, 2);
    double ty = affine.at<double>(1, 2);

    motion.dx = tx;
    motion.dy = ty;
    motion.thetaDeg = std::atan2(b, a) * 180.0 / CV_PI;

    return true;
}

/**************************实现函数********************************************
 *函数原型:     static JitterMetrics compute_video_jitter(const std::string& path,
 *                                                        int maxFrames)
 *功    能:     计算单个视频的画面抖动指标
 *输入参数:     path - 视频路径, maxFrames - 最大分析帧数，<=0表示不限制
 *输出参数:     JitterMetrics统计结果
 ******************************************************************************/
static JitterMetrics compute_video_jitter(const std::string& path, int maxFrames)
{
    JitterMetrics metrics;
    std::vector<FrameMotion> motions;

    cv::VideoCapture cap(path);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open video: " << path << std::endl;
        return metrics;
    }

    cv::Mat prevFrame;
    cv::Mat currFrame;
    cv::Mat prevGray;
    cv::Mat currGray;

    if (!cap.read(prevFrame)) {
        std::cerr << "Failed to read first frame: " << path << std::endl;
        return metrics;
    }

    prevGray = preprocess_frame(prevFrame);

    int frameCount = 1;

    while (cap.read(currFrame)) {
        if (maxFrames > 0 && frameCount >= maxFrames) {
            break;
        }

        currGray = preprocess_frame(currFrame);

        FrameMotion motion;
        metrics.totalPairs++;

        if (estimate_global_motion(prevGray, currGray, motion)) {
            motions.push_back(motion);
            metrics.validPairs++;
        } else {
            metrics.failedPairs++;
        }

        prevGray = currGray;
        frameCount++;
    }

    if (motions.empty()) {
        return metrics;
    }

    double sumDx = 0.0;
    double sumDy = 0.0;
    double sumTrans2 = 0.0;
    double sumTheta2 = 0.0;

    for (size_t i = 0; i < motions.size(); ++i) {
        sumDx += motions[i].dx;
        sumDy += motions[i].dy;
        sumTrans2 += motions[i].dx * motions[i].dx + motions[i].dy * motions[i].dy;
        sumTheta2 += motions[i].thetaDeg * motions[i].thetaDeg;
    }

    double meanDx = sumDx / (double)motions.size();
    double meanDy = sumDy / (double)motions.size();

    double varDx = 0.0;
    double varDy = 0.0;

    for (size_t i = 0; i < motions.size(); ++i) {
        double ddx = motions[i].dx - meanDx;
        double ddy = motions[i].dy - meanDy;
        varDx += ddx * ddx;
        varDy += ddy * ddy;
    }

    metrics.rmsTrans = std::sqrt(sumTrans2 / (double)motions.size());
    metrics.stdDx = std::sqrt(varDx / (double)motions.size());
    metrics.stdDy = std::sqrt(varDy / (double)motions.size());
    metrics.rmsThetaDeg = std::sqrt(sumTheta2 / (double)motions.size());

    return metrics;
}

/**************************实现函数********************************************
 *函数原型:     static void print_metrics_table(const JitterMetrics& raw,
 *                                             const JitterMetrics& eis)
 *功    能:     打印防抖前后指标对比表
 *输入参数:     raw - 原始视频指标, eis - 防抖后视频指标
 *输出参数:     无
 ******************************************************************************/
static void print_metrics_table(const JitterMetrics& raw,
                                const JitterMetrics& eis)
{
    printf("\n================ EIS Jitter Evaluation ================\n");
    printf("Valid frame pairs: raw=%d/%d, eis=%d/%d\n",
           raw.validPairs, raw.totalPairs,
           eis.validPairs, eis.totalPairs);

    printf("\n%-28s %14s %14s %14s\n",
           "Metric", "Raw", "EIS", "Reduction");
    printf("--------------------------------------------------------------------------\n");

    printf("%-28s %13.3fpx %13.3fpx %12.2f%%\n",
           "Translation RMS",
           raw.rmsTrans,
           eis.rmsTrans,
           calc_reduction(raw.rmsTrans, eis.rmsTrans));

    printf("%-28s %13.3fpx %13.3fpx %12.2f%%\n",
           "Horizontal std",
           raw.stdDx,
           eis.stdDx,
           calc_reduction(raw.stdDx, eis.stdDx));

    printf("%-28s %13.3fpx %13.3fpx %12.2f%%\n",
           "Vertical std",
           raw.stdDy,
           eis.stdDy,
           calc_reduction(raw.stdDy, eis.stdDy));

    printf("%-28s %13.4fdeg %13.4fdeg %12.2f%%\n",
           "Rotation RMS",
           raw.rmsThetaDeg,
           eis.rmsThetaDeg,
           calc_reduction(raw.rmsThetaDeg, eis.rmsThetaDeg));

    printf("--------------------------------------------------------------------------\n");

    if (raw.validPairs < 10 || eis.validPairs < 10) {
        printf("Warning: too few valid frame pairs. Please use a video with richer texture.\n");
    }

    if (eis.rmsTrans < raw.rmsTrans) {
        printf("Result: EIS reduces frame-to-frame translation jitter.\n");
    } else {
        printf("Result: EIS does not reduce translation jitter in this test.\n");
        printf("Hint  : check offset sign, focal length, crop margin, timestamp alignment, or test scene.\n");
    }

    printf("========================================================\n");
}

/**************************实现函数********************************************
 *函数原型:     int main(int argc, char** argv)
 *功    能:     程序入口
 *输入参数:     argc - 参数数量, argv - 参数列表
 *输出参数:     0成功，负数失败
 ******************************************************************************/
int main(int argc, char** argv)
{
    if (argc < 3) {
        print_usage(argv[0]);
        return -1;
    }

    std::string rawVideo = argv[1];
    std::string eisVideo = argv[2];
    int maxFrames = 0;

    if (argc >= 4) {
        maxFrames = std::atoi(argv[3]);
    }

    std::cout << "Raw video : " << rawVideo << std::endl;
    std::cout << "EIS video : " << eisVideo << std::endl;
    if (maxFrames > 0) {
        std::cout << "Max frames: " << maxFrames << std::endl;
    } else {
        std::cout << "Max frames: all" << std::endl;
    }

    JitterMetrics rawMetrics = compute_video_jitter(rawVideo, maxFrames);
    JitterMetrics eisMetrics = compute_video_jitter(eisVideo, maxFrames);

    if (rawMetrics.validPairs == 0 || eisMetrics.validPairs == 0) {
        std::cerr << "Failed to compute jitter metrics. Please check video path or scene texture." << std::endl;
        return -1;
    }

    print_metrics_table(rawMetrics, eisMetrics);

    return 0;
}
