#include "lidar_camera_fusion.h"
#include "sentinel_lslidarer.h"

#include <cstdio>
#include <cstring>
#include <csignal>

static bool gRunning = true;

static void sig_handler(int)
{
    gRunning = false;
}

int main()
{
    signal(SIGINT,  sig_handler);
    signal(SIGTERM, sig_handler);

    // ---- 1. 配置雷达 ----
    SentinelLslidarer lidar;
    LidarConfig lidarCfg;
    lidarCfg.angleDisableMin = 0;   // 关闭盲区
    lidarCfg.angleDisableMax = 0;
    if (!lidar.load_config(lidarCfg)) {
        fprintf(stderr, "[DEMO] lidar load_config failed\n");
        return 1;
    }
    if (!lidar.start()) {
        fprintf(stderr, "[DEMO] lidar start failed\n");
        return 1;
    }
    printf("[DEMO] lidar started\n");

    // ---- 2. 配置相机 ----
    CameraConfig camCfg;
    camCfg.fx = 400.0f; camCfg.fy = 400.0f;
    camCfg.cx = 320.0f; camCfg.cy = 240.0f;
    camCfg.imgWidth  = 640;
    camCfg.imgHeight = 480;
    // 外参：相机朝 -X（雷达后方），cx = ly,  cz = -lx
    std::memset(camCfg.tLidarToCam, 0, sizeof(camCfg.tLidarToCam));
    camCfg.tLidarToCam[1]  =  1.0f;   // cx = ly（正后方点 y≈0.15, 投影后 u 偏右约 60px）
    camCfg.tLidarToCam[8]  = -1.0f;   // cz = -lx（正后方 lx<0, cz>0）
    camCfg.tLidarToCam[15] =  1.0f;

    // ---- 3. 启动融合线程 ----
    LidarCameraFusion fusion;
    if (!fusion.start(&lidar, &camCfg, 1)) {
        fprintf(stderr, "[DEMO] fusion start failed\n");
        lidar.stop();
        return 1;
    }
    printf("[DEMO] fusion thread started, running 10 seconds...\n");

    // ---- 4. 运行 10 秒 ----
    for (int i = 0; i < 10 && gRunning; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ---- 5. 停止并打印结果 ----
    printf("[DEMO] stopping...\n");
    fusion.stop();
    lidar.stop();

    const FusionResult& r = fusion.result();
    printf("[DEMO] last result: %u bboxes, %u candidate points\n",
           r.bboxCount, r.bboxPointCounts ? r.bboxPointCounts[0] : 0);
    printf("[DEMO] behind camera: %u, out of image: %u\n",
           fusion.behind_camera_count(),
           fusion.out_of_image_count());

    return 0;
}
