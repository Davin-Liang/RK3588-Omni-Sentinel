#include "SentinelYoloInfer.h"

#include <csignal>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

static volatile bool g_running = true;

static void on_signal(int) {
    g_running = false;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cout << "Usage: " << argv[0]
                  << " <model.rknn> <usb_video_device> <isp_video_device> "
                  << "[usb_width=640] [usb_height=480] [isp_width=1920] [isp_height=1080] [buffer_count=6]\n";
        return 1;
    }

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    std::string modelPath = argv[1];
    std::string usbDevice = argv[2];
    std::string ispDevice = argv[3];

    int usbWidth = argc > 4 ? std::stoi(argv[4]) : 640;
    int usbHeight = argc > 5 ? std::stoi(argv[5]) : 480;
    int ispWidth = argc > 6 ? std::stoi(argv[6]) : 1920;
    int ispHeight = argc > 7 ? std::stoi(argv[7]) : 1080;
    int bufferCount = argc > 8 ? std::stoi(argv[8]) : 6;

    const int usbCamNum = 0;
    const int ispCamNum = 1;

    SentinelVisioner visioner;

    if (!visioner.add_camera(usbDevice, usbWidth, usbHeight, bufferCount, usbCamNum, CameraType::USB_CAM)) {
        std::cerr << "add USB camera failed: " << usbDevice << std::endl;
        return 2;
    }

    if (!visioner.add_camera(ispDevice, ispWidth, ispHeight, bufferCount, ispCamNum, CameraType::ISP_CAM)) {
        std::cerr << "add ISP camera failed: " << ispDevice << std::endl;
        return 3;
    }

    if (!visioner.camera_stream_ctrl(usbCamNum, true)) {
        std::cerr << "open USB camera stream failed\n";
        return 4;
    }

    if (!visioner.camera_stream_ctrl(ispCamNum, true)) {
        std::cerr << "open ISP camera stream failed\n";
        visioner.camera_stream_ctrl(usbCamNum, false);
        return 5;
    }

    SentinelYoloInferConfig cfg;
    cfg.modelPath = modelPath;
    cfg.waitTimeoutMs = 200;
    cfg.pushEmptyResult = true;
    cfg.boxThreshold = 0.25f;
    cfg.nmsThreshold = 0.45f;

    SentinelYoloInfer infer(&visioner, cfg);

    if (!infer.create_infer_thread(usbCamNum)) {
        std::cerr << "create USB infer thread failed\n";
        visioner.camera_stream_ctrl(usbCamNum, false);
        visioner.camera_stream_ctrl(ispCamNum, false);
        return 6;
    }

    if (!infer.create_infer_thread(ispCamNum)) {
        std::cerr << "create ISP infer thread failed\n";
        infer.stop_infer_thread(usbCamNum);
        visioner.camera_stream_ctrl(usbCamNum, false);
        visioner.camera_stream_ctrl(ispCamNum, false);
        return 7;
    }

    std::cout << "[DualDemo] started. USB camNum=0, ISP camNum=1" << std::endl;

    while (g_running) {
        YoloBBoxList usbBoxes;
        if (infer.try_get_osd_result(usbCamNum, usbBoxes, 10)) {
            std::cout << "[USB] cam=" << usbCamNum
                      << " boxes=" << usbBoxes.size()
                      << std::endl;

            for (const auto& b : usbBoxes) {
                std::cout << "  cls=" << b.classId
                          << " conf=" << b.confidence
                          << " box=(" << b.x1 << "," << b.y1 << "," << b.x2 << "," << b.y2 << ")"
                          << " ts=" << b.timestampNs
                          << std::endl;
            }
        }

        YoloBBoxList ispBoxes;
        if (infer.try_get_osd_result(ispCamNum, ispBoxes, 10)) {
            std::cout << "[ISP] cam=" << ispCamNum
                      << " boxes=" << ispBoxes.size()
                      << std::endl;

            for (const auto& b : ispBoxes) {
                std::cout << "  cls=" << b.classId
                          << " conf=" << b.confidence
                          << " box=(" << b.x1 << "," << b.y1 << "," << b.x2 << "," << b.y2 << ")"
                          << " ts=" << b.timestampNs
                          << std::endl;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    infer.stop_infer_thread(usbCamNum);
    infer.stop_infer_thread(ispCamNum);

    visioner.camera_stream_ctrl(usbCamNum, false);
    visioner.camera_stream_ctrl(ispCamNum, false);

    std::cout << "[DualDemo] stopped." << std::endl;
    return 0;
}
