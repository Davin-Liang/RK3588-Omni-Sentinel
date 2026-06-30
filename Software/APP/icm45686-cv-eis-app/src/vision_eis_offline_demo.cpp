/*
 * vision_eis_offline_demo.cpp - 视觉为主 + IMU辅助 EIS 离线验证工具
 *
 * 功能：
 * 1. 读取 raw.mp4；
 * 2. 使用 LK 光流 + RANSAC 估计画面运动；
 * 3. 对运动轨迹做实时平滑，生成 eis_visual.mp4；
 * 4. 可选读取 IMU 震动等级 CSV，用于动态调整平滑 alpha；
 * 5. 输出逐帧日志 CSV，便于分析 tracked_points / inliers / gyro_rms / used_alpha / offset。
 *
 * 说明：
 * LK 光流和 ORB 都是传统视觉算法，不需要模型。本工具优先使用 LK 光流。
 */

#include "vision_eis.hpp"
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

struct CsvImuRow {
    float gyroRms;
    int vibrationLevel;

    CsvImuRow() : gyroRms(0.0f), vibrationLevel(1) {}
};

static void print_usage(const char* prog)
{
    std::cout << "Usage:\n";
    std::cout << "  " << prog << " <raw_video> <eis_output_video> [max_frames] [max_offset] [process_width] [alpha_low] [alpha_mid] [alpha_high] [imu_csv]\n\n";
    std::cout << "Example:\n";
    std::cout << "  " << prog << " raw.mp4 eis_visual.mp4\n";
    std::cout << "  " << prog << " raw.mp4 eis_visual.mp4 900 80 640 0.30 0.20 0.12\n\n";
    std::cout << "Optional imu_csv format:\n";
    std::cout << "  frame_id,gyro_rms,vibration_level\n";
    std::cout << "  1,0.02,0\n";
    std::cout << "  2,0.08,1\n";
    std::cout << "  3,0.20,2\n\n";
    std::cout << "Meaning:\n";
    std::cout << "  The tool is for PC-side offline verification of visual-main EIS.\n";
    std::cout << "  It does not require RK3588 or /dev/icm45686.\n";
}

static bool parse_imu_csv(const std::string& path, std::map<int, CsvImuRow>& rows)
{
    std::ifstream in(path.c_str());
    if (!in.is_open()) {
        std::cerr << "Failed to open imu csv: " << path << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        if (line.find("frame") != std::string::npos || line[0] == '#') {
            continue;
        }

        std::replace(line.begin(), line.end(), ',', ' ');
        std::stringstream ss(line);
        int frameId = 0;
        CsvImuRow row;
        if (ss >> frameId >> row.gyroRms >> row.vibrationLevel) {
            rows[frameId] = row;
        }
    }

    return true;
}

static void apply_translation(const cv::Mat& input,
                              cv::Mat& output,
                              int offsetX,
                              int offsetY)
{
    /*
     * 离线验证阶段直接用 warpAffine 平移整幅图。
     * 正式 RK3588 链路中，这一步应由 RGA 的平移/裁剪完成。
     */
    cv::Mat m = (cv::Mat_<double>(2, 3) << 1.0, 0.0, offsetX,
                                           0.0, 1.0, offsetY);
    cv::warpAffine(input, output, m, input.size(),
                   cv::INTER_LINEAR, cv::BORDER_REPLICATE);
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        print_usage(argv[0]);
        return -1;
    }

    std::string rawPath = argv[1];
    std::string outPath = argv[2];
    int maxFrames = (argc > 3) ? std::atoi(argv[3]) : 0;
    int maxOffset = (argc > 4) ? std::atoi(argv[4]) : 80;
    int processWidth = (argc > 5) ? std::atoi(argv[5]) : 640;
    float alphaLow = (argc > 6) ? static_cast<float>(std::atof(argv[6])) : 0.30f;
    float alphaMid = (argc > 7) ? static_cast<float>(std::atof(argv[7])) : 0.20f;
    float alphaHigh = (argc > 8) ? static_cast<float>(std::atof(argv[8])) : 0.12f;
    std::string imuCsv = (argc > 9) ? argv[9] : "";

    cv::VideoCapture cap(rawPath);
    if (!cap.isOpened()) {
        std::cerr << "Failed to open raw video: " << rawPath << std::endl;
        return -1;
    }

    int width = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
    int height = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    double fps = cap.get(cv::CAP_PROP_FPS);
    if (fps <= 1e-6) {
        fps = 30.0;
    }

    int processHeight = static_cast<int>((double)processWidth * (double)height / (double)width + 0.5);
    if (processHeight <= 0) {
        processHeight = height;
    }

    VisionEisConfig config;
    config.inputWidth = width;
    config.inputHeight = height;
    config.processWidth = processWidth;
    config.processHeight = processHeight;
    config.maxOffsetPixel = maxOffset;
    config.alphaLowVibration = alphaLow;
    config.alphaMidVibration = alphaMid;
    config.alphaHighVibration = alphaHigh;
    config.enableImuAdaptiveAlpha = !imuCsv.empty();

    std::map<int, CsvImuRow> imuRows;
    if (!imuCsv.empty()) {
        if (!parse_imu_csv(imuCsv, imuRows)) {
            std::cerr << "Warning: failed to parse imu csv. Continue without IMU adaptive alpha." << std::endl;
            config.enableImuAdaptiveAlpha = false;
        }
    }

    int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
    cv::VideoWriter writer(outPath, fourcc, fps, cv::Size(width, height));
    if (!writer.isOpened()) {
        std::cerr << "Failed to create output video: " << outPath << std::endl;
        return -1;
    }

    std::string logPath = outPath + ".csv";
    std::ofstream log(logPath.c_str());
    log << "frame_id,success,visual_reliable,used_fallback,dx,dy,dtheta,offset_x,offset_y,"
        << "tracked_points,inliers,gyro_rms,vibration_level,used_alpha,cost_ms\n";

    VisionEisStabilizer stabilizer(config);

    std::cout << "Raw video       : " << rawPath << std::endl;
    std::cout << "Output EIS video: " << outPath << std::endl;
    std::cout << "Frame size      : " << width << "x" << height << " @ " << fps << " FPS" << std::endl;
    std::cout << "Process size    : " << processWidth << "x" << processHeight << std::endl;
    std::cout << "Max offset      : " << maxOffset << " px" << std::endl;
    std::cout << "Alpha low/mid/high: " << alphaLow << " / " << alphaMid << " / " << alphaHigh << std::endl;
    std::cout << "Log csv         : " << logPath << std::endl;

    cv::Mat frame;
    int frameId = 0;
    int success = 0;
    int reliable = 0;

    while (cap.read(frame)) {
        frameId++;
        if (maxFrames > 0 && frameId > maxFrames) {
            break;
        }

        uint64_t tsNs = static_cast<uint64_t>((double)frameId * 1000000000.0 / fps);

        VisionImuAssistState imu;
        VisionImuAssistState* imuPtr = nullptr;
        std::map<int, CsvImuRow>::const_iterator it = imuRows.find(frameId);
        if (it != imuRows.end()) {
            imu.timestampNs = tsNs;
            imu.gyroRms = it->second.gyroRms;
            imu.vibrationLevel = it->second.vibrationLevel;
            imuPtr = &imu;
        }

        VisionEisResult result;
        stabilizer.processFrame(frame, tsNs, imuPtr, result);

        cv::Mat stabilized;
        apply_translation(frame, stabilized, result.offsetX, result.offsetY);
        writer.write(stabilized);

        if (result.success) success++;
        if (result.visualReliable) reliable++;

        float gyroRms = imuPtr ? imu.gyroRms : 0.0f;
        int vibrationLevel = imuPtr ? imu.vibrationLevel : -1;

        log << frameId << ','
            << result.success << ','
            << result.visualReliable << ','
            << result.usedFallback << ','
            << result.dx << ','
            << result.dy << ','
            << result.dtheta << ','
            << result.offsetX << ','
            << result.offsetY << ','
            << result.trackedPoints << ','
            << result.inliers << ','
            << gyroRms << ','
            << vibrationLevel << ','
            << result.usedAlpha << ','
            << result.costMs << '\n';
    }

    std::cout << "Processed frames: " << frameId << std::endl;
    std::cout << "Success frames  : " << success << std::endl;
    std::cout << "Reliable frames : " << reliable << std::endl;
    std::cout << "Done. You can evaluate with:\n";
    std::cout << "  ./icm45686_jitter_eval " << rawPath << " " << outPath << std::endl;

    return 0;
}
