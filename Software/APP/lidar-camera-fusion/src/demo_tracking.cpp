#include "lidar_camera_fusion.h"

#include <cmath>
#include <cstdio>
#include <cstring>

static int gPassed = 0;
static int gFailed = 0;

static void check(bool condition, const char* desc)
{
    if (condition) {
        ++gPassed;
        printf("  PASS: %s\n", desc);
    } else {
        ++gFailed;
        fprintf(stderr, "  FAIL: %s\n", desc);
    }
}

static void make_xy_to_xz_T(float* T)
{
    std::memset(T, 0, 16 * sizeof(float));
    T[0]  = 1.0f;
    T[9]  = 1.0f;
    T[15] = 1.0f;
}

// 告警回调计数器
static uint32_t gWarningCount = 0;
static void warning_callback(const TrackedTarget& t, void*)
{
    ++gWarningCount;
    printf("  [WARNING] target %u dist=%.2fm state=%d\n",
           t.id, t.distanceMeters, static_cast<int>(t.state));
}

// ---- 测试辅助：生成一条运动轨迹的 LiDAR 点 ----
static LidarPoint make_point(float x, float y)
{
    LidarPoint p;
    p.x = x; p.y = y; p.intensity = 100.0f;
    return p;
}

// ---- 测试 1：单静止目标 ----
static void test_stationary_target(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 1: Stationary target (10 frames) ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPoints = 20;
    const uint64_t baseTs = 1000000000000ULL;
    const uint64_t dtNs   = 100000000ULL; // 0.1s per frame

    for (uint32_t frame = 0; frame < 10; ++frame) {
        // 静止目标在 (2.0, 5.0) 处
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 10.0f);
            points[i].y = 5.0f + 0.03f * (static_cast<float>(i % 5) - 2.0f);
            points[i].intensity = 100.0f;
        }

        // bbox 覆盖该区域在图像上的投影
        YoloBBox bbox;
        bbox.x1 = 420; bbox.y1 = 200;
        bbox.x2 = 540; bbox.y2 = 280;
        bbox.classId = 0;
        bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;

        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, bbox.timestampNs, lf, cam);

        const FusionResult& r = fusion.result();
        check(r.bboxCount == 1, "fusion bboxCount == 1");

        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 快照查询
    TrackedTarget snapshot[10];
    uint32_t count = 0;
    fusion.copy_tracked_targets(snapshot, 10, &count);
    check(count == 1, "one tracked target");

    if (count >= 1) {
        const TrackedTarget& t = snapshot[0];
        check(t.state == TrackState::Confirmed, "target confirmed");
        check(std::fabs(t.posX - 2.0f) < 0.3f, "posX near 2.0");
        check(std::fabs(t.posY - 5.0f) < 0.3f, "posY near 5.0");
        check(std::fabs(t.velX) < 0.5f, "velX near 0");
        check(std::fabs(t.velY) < 0.5f, "velY near 0");
        printf("  pos=(%.3f, %.3f) vel=(%.3f, %.3f) dist=%.3f\n",
               t.posX, t.posY, t.velX, t.velY, t.distanceMeters);
    }
}

// ---- 测试 2：匀速运动目标 ----
static void test_constant_velocity(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 2: Constant velocity target (5 m/s) ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPoints = 15;
    const uint64_t baseTs = 2000000000000ULL;
    const uint64_t dtNs   = 100000000ULL;
    float cx = 2.0f, cy = 10.0f;
    const float vx = 0.0f, vy = -5.0f;

    for (uint32_t frame = 0; frame < 15; ++frame) {
        float t = static_cast<float>(frame) * 0.1f;
        float posX = cx + vx * t;
        float posY = cy + vy * t;

        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = posX + 0.05f * (static_cast<float>(i) - 7.0f);
            points[i].y = posY;
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 0;   bbox.y1 = 0;
        bbox.x2 = 640; bbox.y2 = 480;
        bbox.classId = 0;
        bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;

        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, bbox.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    TrackedTarget snapshot[10];
    uint32_t count = 0;
    fusion.copy_tracked_targets(snapshot, 10, &count);
    check(count == 1, "one tracked target");

    if (count >= 1) {
        const TrackedTarget& t = snapshot[0];
        check(t.state == TrackState::Confirmed, "target confirmed");
        check(std::fabs(t.velY - (-5.0f)) < 2.0f, "velY near -5.0 m/s");
        printf("  final pos=(%.3f, %.3f) vel=(%.3f, %.3f)\n",
               t.posX, t.posY, t.velX, t.velY);
    }
}

// ---- 测试 3：两目标交叉 ----
static void test_crossing_targets(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 3: Two crossing targets (watch for ID swap) ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPointsPerTarget = 10;
    const uint64_t baseTs = 3000000000000ULL;
    const uint64_t dtNs   = 100000000ULL;

    for (uint32_t frame = 0; frame < 20; ++frame) {
        float t = static_cast<float>(frame) * 0.1f;

        // 目标 A：从 (1, 10) 向右移动到 (3, 10)
        float ax = 1.0f + 1.0f * t; // vx=1 m/s
        float ay = 10.0f;

        // 目标 B：从 (3, 10) 向左移动到 (1, 10)
        float bx = 3.0f - 1.0f * t; // vx=-1 m/s
        float by = 10.0f;

        // 目标 A 的点
        for (uint32_t i = 0; i < nPointsPerTarget; ++i) {
            points[i].x = ax + 0.03f * static_cast<float>(static_cast<int>(i) - 5);
            points[i].y = ay;
            points[i].intensity = 100.0f;
        }
        // 目标 B 的点
        for (uint32_t i = 0; i < nPointsPerTarget; ++i) {
            points[nPointsPerTarget + i].x = bx + 0.03f * static_cast<float>(static_cast<int>(i) - 5);
            points[nPointsPerTarget + i].y = by;
            points[nPointsPerTarget + i].intensity = 100.0f;
        }

        YoloBBox bboxA, bboxB;
        bboxA.x1 = 250; bboxA.y1 = 180;
        bboxA.x2 = 400; bboxA.y2 = 300;
        bboxA.classId = 0; bboxA.confidence = 0.9f;
        bboxA.timestampNs = baseTs + frame * dtNs;

        bboxB.x1 = 400; bboxB.y1 = 180;
        bboxB.x2 = 550; bboxB.y2 = 300;
        bboxB.classId = 0; bboxB.confidence = 0.9f;
        bboxB.timestampNs = baseTs + frame * dtNs;

        std::vector<YoloBBox> dets;
        dets.push_back(bboxA);
        dets.push_back(bboxB);

        LidarFrame lf;
        lf.timestampNs = baseTs + frame * dtNs;
        lf.points = points;
        lf.pointsCount = nPointsPerTarget * 2;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, lf.pointsCount,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    TrackedTarget snapshot[10];
    uint32_t count = 0;
    fusion.copy_tracked_targets(snapshot, 10, &count);
    // 交叉后 ID 可能交换，仅验证有航迹输出
    check(count >= 1, "at least one track after crossing");
    printf("  track count after crossing: %u\n", count);
    for (uint32_t i = 0; i < count; ++i) {
        printf("  track[%u] id=%u pos=(%.2f,%.2f) vel=(%.2f,%.2f)\n",
               i, snapshot[i].id, snapshot[i].posX, snapshot[i].posY,
               snapshot[i].velX, snapshot[i].velY);
    }
}

// ---- 测试 4：目标出现和消失 ----
static void test_appear_disappear(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 4: Target appear and disappear ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPoints = 15;
    const uint64_t baseTs = 4000000000000ULL;
    const uint64_t dtNs   = 100000000ULL;

    // 帧 0-4：无目标
    for (uint32_t frame = 0; frame < 5; ++frame) {
        LidarFrame lf;
        lf.timestampNs = baseTs + frame * dtNs;
        lf.points = points;
        lf.pointsCount = nPoints;
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 20.0f; // 远距离背景点，不在 bbox 内
            points[i].y = 20.0f;
            points[i].intensity = 30.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 300; bbox.y1 = 200;
        bbox.x2 = 340; bbox.y2 = 280;
        bbox.classId = 0; bbox.confidence = 0.9f;
        bbox.timestampNs = lf.timestampNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 帧 5-9：出现目标在 (2, 5)
    for (uint32_t frame = 5; frame < 10; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
            points[i].y = 5.0f;
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 400; bbox.y1 = 200;
        bbox.x2 = 520; bbox.y2 = 280;
        bbox.classId = 0; bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 帧 10-14：目标消失
    for (uint32_t frame = 10; frame < 15; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 20.0f;
            points[i].y = 20.0f;
            points[i].intensity = 30.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 300; bbox.y1 = 200;
        bbox.x2 = 340; bbox.y2 = 280;
        bbox.classId = 0; bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    TrackedTarget snapshot[10];
    uint32_t count = 0;
    fusion.copy_tracked_targets(snapshot, 10, &count);
    // 目标消失一段时间后应被删除
    check(count == 0, "target deleted after disappearing");
}

// ---- 测试 5：近距离告警 ----
static void test_warning(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 5: Proximity warning ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    // 配置告警阈值
    TrackerConfig cfg;
    cfg.warningEnterDistMeters = 2.5f;
    cfg.warningExitDistMeters  = 3.0f;
    cfg.minConfirmedAgeForWarning = 2;
    cfg.warningCooldownNs = 2000000000ULL;
    fusion.configure_tracker(cfg);

    gWarningCount = 0;
    fusion.enable_tracking(true);

    const uint32_t nPoints = 15;
    const uint64_t baseTs = 5000000000000ULL;
    const uint64_t dtNs   = 100000000ULL;

    // 目标在 3.5m（安全距离）→ 不应告警
    for (uint32_t frame = 0; frame < 5; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 0.0f;
            points[i].y = 3.5f;
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 310; bbox.y1 = 230;
        bbox.x2 = 330; bbox.y2 = 250;
        bbox.classId = 0; bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    check(gWarningCount == 0, "no warning at safe distance");

    // 目标移动到 2.0m（危险距离）→ 应告警
    for (uint32_t frame = 5; frame < 8; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 0.0f;
            points[i].y = 2.0f;
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 315; bbox.y1 = 235;
        bbox.x2 = 325; bbox.y2 = 245;
        bbox.classId = 0; bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 等目标 confirmed + 年龄够了应该告警
    check(gWarningCount >= 1, "warning triggered at danger distance");
    printf("  warning count: %u\n", gWarningCount);
}

// ---- 测试 6：dt 异常 ----
static void test_dt_anomalies(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 6: dt anomalies ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPoints = 15;
    const uint64_t baseTs = 6000000000000ULL;

    YoloBBox bbox;
    bbox.x1 = 400; bbox.y1 = 200;
    bbox.x2 = 540; bbox.y2 = 280;
    bbox.classId = 0; bbox.confidence = 0.9f;
    bbox.timestampNs = baseTs;

    for (uint32_t i = 0; i < nPoints; ++i) {
        points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
        points[i].y = 5.0f;
        points[i].intensity = 100.0f;
    }

    // dt = 0：同一时间戳两次调用
    printf("  Test: dt = 0 (same timestamp twice)\n");
    {
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);
        LidarFrame lf;
        lf.timestampNs = baseTs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, baseTs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               baseTs);
        // 第二次相同时间戳
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               baseTs);
    }

    check(true, "dt=0 did not crash");

    // dt 很大：跳过 2 秒
    printf("  Test: dt = 2s (large gap)\n");
    {
        bbox.timestampNs = baseTs + 2000000000ULL;
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
            points[i].y = 5.0f;
            points[i].intensity = 100.0f;
        }

        std::vector<YoloBBox> dets;
        dets.push_back(bbox);
        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, bbox.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               bbox.timestampNs);
    }

    check(true, "dt=2s did not crash");
}

// ---- 测试 7：空输入 ----
static void test_empty_input(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 7: Empty input ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint64_t ts = 7000000000000ULL;
    std::vector<YoloBBox> dets; // 空

    LidarFrame lf;
    lf.timestampNs = ts;
    lf.points = points;
    lf.pointsCount = 0;

    fusion.reset();
    fusion.fuse_data(dets, ts, lf, cam);
    const FusionResult& r = fusion.result();
    bool ok = fusion.update_tracking(r, points, 0,
                                      nullptr, 0, ts);
    check(ok, "empty input: no crash");
}

// ---- 测试 8：bboxCount 不一致 + 越界点索引 ----
static void test_edge_cases(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 8: Edge cases (bboxCount mismatch, out-of-bounds index) ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPoints = 15;
    const uint64_t ts = 8000000000000ULL;

    for (uint32_t i = 0; i < nPoints; ++i) {
        points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
        points[i].y = 5.0f;
        points[i].intensity = 100.0f;
    }

    YoloBBox bbox;
    bbox.x1 = 400; bbox.y1 = 200;
    bbox.x2 = 540; bbox.y2 = 280;
    bbox.classId = 0; bbox.confidence = 0.9f;
    bbox.timestampNs = ts;
    std::vector<YoloBBox> dets;
    dets.push_back(bbox);

    LidarFrame lf;
    lf.timestampNs = ts;
    lf.points = points;
    lf.pointsCount = nPoints;

    fusion.reset();
    fusion.fuse_data(dets, ts, lf, cam);
    const FusionResult& r = fusion.result();

    // 故意传入不匹配的 bboxCount
    YoloBBox wrongBboxes[2] = { bbox, bbox };
    bool ok = fusion.update_tracking(r, points, nPoints,
                                      wrongBboxes, 2, // 声称 2 个但 FusionResult 只有 1 个
                                      ts);
    check(ok, "bboxCount mismatch: no crash");

    // 传入 pointCount=0 模拟越界索引
    ok = fusion.update_tracking(r, points, 0, // pointCount=0 但索引指向正常范围
                                 dets.data(), static_cast<uint32_t>(dets.size()),
                                 ts);
    check(ok, "out-of-bounds index: no crash");
}

// ---- 测试 9：人走出相机，LiDAR 孤儿点维持跟踪 ----
static void test_orphan_coasting(LidarCameraFusion& fusion, LidarPoint* points)
{
    fusion.reset_tracking();
    printf("\n=== Test 9: Orphan LiDAR keeps coasting track alive ===\n");

    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f; cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    const uint32_t nPoints = 15;
    const uint64_t baseTs = 9000000000000ULL;
    const uint64_t dtNs   = 100000000ULL;

    // Phase 1（帧 0-4）：人在相机视野内，bbox 覆盖 → 正常跟踪
    for (uint32_t frame = 0; frame < 5; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
            points[i].y = 5.0f;
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 400; bbox.y1 = 200;   // 覆盖人在 (2.0, 5.0) 的投影
        bbox.x2 = 520; bbox.y2 = 280;
        bbox.classId = 0;
        bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 确认航迹已创建并确认
    {
        TrackedTarget snapshot[10];
        uint32_t count = 0;
        fusion.copy_tracked_targets(snapshot, 10, &count);
        check(count >= 1, "phase 1: track created");
        if (count >= 1) {
            check(snapshot[0].state == TrackState::Confirmed,
                  "phase 1: track confirmed");
            printf("  phase 1 track: pos=(%.2f,%.2f) state=%d\n",
                   snapshot[0].posX, snapshot[0].posY,
                   static_cast<int>(snapshot[0].state));
        }
    }

    // Phase 2（帧 5-9）：人走出相机，bbox 不覆盖人的位置
    // LiDAR 点仍在原位，但 bbox 放在图像另一侧 → 点变成孤儿
    for (uint32_t frame = 5; frame < 10; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
            points[i].y = 5.0f;   // 人还在原位，LiDAR 仍能看到
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 0;   bbox.y1 = 0;     // bbox 在图像左上角
        bbox.x2 = 100; bbox.y2 = 100;   // 不覆盖人在 (2.0,5.0) 的投影 u≈480
        bbox.classId = 0;
        bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 确认航迹进入 Coasting 但仍存活（被孤儿检测续命）
    {
        TrackedTarget snapshot[10];
        uint32_t count = 0;
        fusion.copy_tracked_targets(snapshot, 10, &count);
        check(count >= 1, "phase 2: track still alive (orphan coasting)");
        if (count >= 1) {
            printf("  phase 2 track: pos=(%.2f,%.2f) state=%d\n",
                   snapshot[0].posX, snapshot[0].posY,
                   static_cast<int>(snapshot[0].state));
        }
    }

    // Phase 3（帧 10-14）：人走回相机，bbox 重新覆盖
    for (uint32_t frame = 10; frame < 15; ++frame) {
        for (uint32_t i = 0; i < nPoints; ++i) {
            points[i].x = 2.0f + 0.05f * (static_cast<float>(i) - 7.0f);
            points[i].y = 5.0f;
            points[i].intensity = 100.0f;
        }

        YoloBBox bbox;
        bbox.x1 = 400; bbox.y1 = 200;   // 重新覆盖人的投影
        bbox.x2 = 520; bbox.y2 = 280;
        bbox.classId = 0;
        bbox.confidence = 0.9f;
        bbox.timestampNs = baseTs + frame * dtNs;
        std::vector<YoloBBox> dets;
        dets.push_back(bbox);

        LidarFrame lf;
        lf.timestampNs = bbox.timestampNs;
        lf.points = points;
        lf.pointsCount = nPoints;

        fusion.reset();
        fusion.fuse_data(dets, lf.timestampNs, lf, cam);
        const FusionResult& r = fusion.result();
        fusion.update_tracking(r, points, nPoints,
                               dets.data(), static_cast<uint32_t>(dets.size()),
                               lf.timestampNs);
    }

    // 确认航迹恢复 Confirmed
    {
        TrackedTarget snapshot[10];
        uint32_t count = 0;
        fusion.copy_tracked_targets(snapshot, 10, &count);
        check(count >= 1, "phase 3: track recovered");
        if (count >= 1) {
            check(snapshot[0].state == TrackState::Confirmed,
                  "phase 3: track back to Confirmed");
            printf("  phase 3 track: pos=(%.2f,%.2f) state=%d\n",
                   snapshot[0].posX, snapshot[0].posY,
                   static_cast<int>(snapshot[0].state));
        }
    }
}

int main()
{
    printf("=== Lidar Camera Fusion — Tracking Demo ===\n");

    // 分配 LiDAR 点缓冲区
    const uint32_t kMaxPoints = 540;
    LidarPoint* points = new (std::nothrow) LidarPoint[kMaxPoints];
    if (!points) {
        fprintf(stderr, "Failed to allocate points\n");
        return 1;
    }

    // 创建融合实例
    LidarCameraFusion fusion;

    // 配置跟踪器
    TrackerConfig cfg;
    cfg.maxTracks = 10;
    bool cfgOk = fusion.configure_tracker(cfg);
    check(cfgOk, "configure_tracker succeeds");

    // 注册告警回调
    fusion.register_warning_callback(warning_callback, nullptr);

    // 启用跟踪
    bool enOk = fusion.enable_tracking(true);
    check(enOk, "enable_tracking succeeds");

    // 运行各测试
    test_stationary_target(fusion, points);
    test_constant_velocity(fusion, points);
    test_crossing_targets(fusion, points);
    test_appear_disappear(fusion, points);
    test_warning(fusion, points);
    test_dt_anomalies(fusion, points);
    test_empty_input(fusion, points);
    test_edge_cases(fusion, points);
    test_orphan_coasting(fusion, points);

    // 最终快照
    TrackedTarget snapshot[10];
    uint32_t finalCount = 0;
    fusion.copy_tracked_targets(snapshot, 10, &finalCount);
    printf("\nFinal snapshot: %u active track(s)\n", finalCount);

    // 检查可配置 at runtime
    bool disableOk = fusion.enable_tracking(false);
    check(disableOk, "disable_tracking succeeds");
    check(fusion.update_tracking(FusionResult(), nullptr, 0, nullptr, 0, 0),
          "update_tracking with tracking disabled returns true");

    delete[] points;

    printf("\n=== Results: %d passed, %d failed ===\n", gPassed, gFailed);
    return gFailed > 0 ? 1 : 0;
}
