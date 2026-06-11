#include "SentinelYoloInfer.h"

#include <csignal>
#include <iostream>
#include <string>

static volatile bool g_running = true;

static void on_signal(int) {
    g_running = false;
}

int main(int argc, char** argv) {
    if (argc < 5) {
        std::cout << "Usage: " << argv[0]
                  << " <model.rknn> <video_device> <cam_num> <ISP|USB> [width=1920] [height=1080] [buffer_count=6]\n";
        return 1;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string modelPath = argv[1];
    std::string deviceName = argv[2];
    int camNum = std::stoi(argv[3]);
    std::string type = argv[4];
    int width = argc > 5 ? std::stoi(argv[5]) : 1920;
    int height = argc > 6 ? std::stoi(argv[6]) : 1080;
    int bufferCount = argc > 7 ? std::stoi(argv[7]) : 6;

    CameraType camType = (type == "USB" || type == "usb") ? CameraType::USB_CAM : CameraType::ISP_CAM;

    SentinelVisioner visioner;
    if (!visioner.add_camera(deviceName, width, height, bufferCount, camNum, camType)) {
        std::cerr << "add_camera failed\n";
        return 2;
    }
    if (!visioner.camera_stream_ctrl(camNum, true)) {
        std::cerr << "camera_stream_ctrl open failed\n";
        return 3;
    }

    SentinelYoloInferConfig cfg;
    cfg.modelPath = modelPath;
    cfg.waitTimeoutMs = 200;
    cfg.pushEmptyResult = true;

    SentinelYoloInfer infer(&visioner, cfg);
    if (!infer.create_infer_thread(camNum)) {
        std::cerr << "create_infer_thread failed\n";
        visioner.camera_stream_ctrl(camNum, false);
        return 4;
    }

    while (g_running) {
        YoloBBoxList boxes;
        if (!infer.try_get_osd_result(camNum, boxes, 500)) {
            continue;
        }
        std::cout << "cam=" << camNum << " boxes=" << boxes.size() << std::endl;
        for (const auto& b : boxes) {
            std::cout << "  cls=" << b.classId
                      << " conf=" << b.confidence
                      << " box=(" << b.x1 << "," << b.y1 << "," << b.x2 << "," << b.y2 << ")"
                      << " ts=" << b.timestampNs << std::endl;
        }
    }

    infer.stop_infer_thread(camNum);
    visioner.camera_stream_ctrl(camNum, false);
    return 0;
}
