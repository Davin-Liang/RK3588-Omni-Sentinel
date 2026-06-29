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

    // ---- 3. 配置跟踪 ----
    LidarCameraFusion fusion;

    TrackerConfig trackerCfg;
    trackerCfg.bboxAssocMaxDistMeters = 3.0f; // 关联门限 3m
    trackerCfg.warningEnterDistMeters = 0.5f;   // 0.5 米内告警
    trackerCfg.warningExitDistMeters  = 0.8f;
    trackerCfg.warningCooldownNs      = 3000000000ULL; // 3 秒冷却
    fusion.configure_tracker(trackerCfg);

    fusion.register_warning_callback(
        [](const TrackedTarget& t, void*) {
            printf("[WARNING] target %u class=%u dist=%.2fm pos=(%.2f,%.2f)\n",
                   t.id, t.classId, t.distanceMeters, t.posX, t.posY);
        }, nullptr);

    fusion.enable_tracking(true);
    printf("[DEMO] tracking enabled (warning at %.1fm)\n",
           trackerCfg.warningEnterDistMeters);

    // ---- 4. 启动融合线程 ----
    if (!fusion.start(&lidar, &camCfg, 1)) {
        fprintf(stderr, "[DEMO] fusion start failed\n");
        lidar.stop();
        return 1;
    }
    printf("[DEMO] fusion thread started, running 30 seconds...\n");

    // ---- 5. 运行 30 秒 ----
    for (int i = 0; i < 30 && gRunning; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // ---- 6. 停止并打印结果 ----
    printf("[DEMO] stopping...\n");
    fusion.stop();
    lidar.stop();

    const FusionResult& r = fusion.result();
    printf("[DEMO] last fusion: %u bboxes, %u candidate points\n",
           r.bboxCount, r.bboxPointCounts ? r.bboxPointCounts[0] : 0);
    printf("[DEMO] behind camera: %u, out of image: %u\n",
           fusion.behind_camera_count(),
           fusion.out_of_image_count());

    // 打印跟踪结果
    TrackedTarget snapshot[10];
    uint32_t trackCount = 0;
    fusion.copy_tracked_targets(snapshot, 10, &trackCount);
    printf("[DEMO] tracked targets: %u\n", trackCount);
    for (uint32_t i = 0; i < trackCount; ++i) {
        const TrackedTarget& t = snapshot[i];
        const char* stateStr = "?";
        switch (t.state) {
        case TrackState::Tentative: stateStr = "Tentative"; break;
        case TrackState::FusionTracking:  stateStr = "FusionTracking"; break;
        case TrackState::PureRadarTracking: stateStr = "PureRadarTracking"; break;
        case TrackState::Lost:  stateStr = "Lost";  break;
        case TrackState::Deleted:   stateStr = "Deleted";   break;
        default: break;
        }
        printf("[DEMO]   #%u | pos=(%.2f,%.2f) vel=(%.2f,%.2f) "
               "dist=%.2fm %s\n",
               t.id, t.posX, t.posY, t.velX, t.velY,
               t.distanceMeters, stateStr);
    }

    return 0;
}
