#include "lidar_camera_fusion.h"

#include <cstdio>
#include <cstring>

/**
 * @brief 断言检查，失败时打印描述并返回 false。
 */
static bool check(bool condition, const char* desc)
{
    if (!condition) {
        fprintf(stderr, "[DEMO] FAIL: %s\n", desc);
        return false;
    }
    return true;
}

/**
 * @brief 构造 radar-XY → cam-XZ 旋转外参矩阵（4×4 行主序）。
 *        lidar x → cam x,  lidar y → cam z,  lidar z=0 → cam y=0
 *        使 N10Plus 水平扫描面的点投影后 z 坐标 > 0（在相机前方）。
 */
static void make_xy_to_xz_T(float* T)
{
    std::memset(T, 0, 16 * sizeof(float));
    T[0]  = 1.0f;   // cx = lx   (row0 col0 = lx 系数)
    T[9]  = 1.0f;   // cz = ly   (row2 col1 = ly 系数)
    T[15] = 1.0f;   // w  = 1    (row3 col3 = 齐次坐标)
    // cy 恒为 0：雷达 z=0, row1 全为零即可
}

int main()
{
    int passed = 0;
    int failed = 0;

    const uint32_t kTotalPoints = 100;
    const uint64_t imageTs = 1234567890000000ULL;

    // ---- 构造虚拟相机 ----
    // 内参：f=400 像素, 主点 (320, 240), 图像 640×480
    // 外参：将雷达水平扫描面旋转到相机前方（cz > 0）
    CameraConfig cam;
    cam.fx = 400.0f; cam.fy = 400.0f;
    cam.cx = 320.0f; cam.cy = 240.0f;
    cam.imgWidth = 640; cam.imgHeight = 480;
    make_xy_to_xz_T(cam.tLidarToCam);

    // ---- 合成雷达点 ----
    // 投影公式：u = fx * lx / ly + cx,  v = cy（单线雷达 cy ≡ 0, 故 v ≡ cy）
    //
    // 组 A（车辆，50 点）：lx ∈ [1.05, 1.95], ly ∈ [3.10, 3.90]
    //   → u ∈ [400×1.05/3.90+320=428, 400×1.95/3.10+320=572]  ⊂ bbox0 [410, 600)
    //
    // 组 B（行人，30 点）：lx ∈ [-0.95, -0.20], ly ∈ [3.10, 3.90]
    //   → u ∈ [400×(-0.95)/3.10+320=197, 400×(-0.20)/3.90+320=299]  ⊂ bbox1 [180, 335)
    //
    // 组 C（背景，20 点）：lx ∈ [0.32, 0.77], ly = 5.00~5.10
    //   → u ∈ [400×0.32/5.10+320=345, 400×0.77/5.00+320=382]  ∉ bbox0 ∪ bbox1
    LidarPoint* points = new (std::nothrow) LidarPoint[kTotalPoints];
    if (!points) { fprintf(stderr, "[DEMO] alloc failed\n"); return 1; }

    uint32_t idx = 0;
    for (int xi = 0; xi < 10; ++xi) {
        for (int yi = 0; yi < 5; ++yi) {
            points[idx].x = 1.05f + 0.10f * static_cast<float>(xi);
            points[idx].y = 3.10f + 0.20f * static_cast<float>(yi);
            points[idx].intensity = 100.0f; ++idx;
        }
    }
    for (int xi = 0; xi < 6; ++xi) {
        for (int yi = 0; yi < 5; ++yi) {
            points[idx].x = -0.95f + 0.15f * static_cast<float>(xi);
            points[idx].y =  3.10f + 0.20f * static_cast<float>(yi);
            points[idx].intensity = 80.0f; ++idx;
        }
    }
    for (int xi = 0; xi < 10; ++xi) {
        for (int yi = 0; yi < 2; ++yi) {
            points[idx].x = 0.32f + 0.05f * static_cast<float>(xi);
            points[idx].y = 5.00f + 0.10f * static_cast<float>(yi);
            points[idx].intensity = 50.0f; ++idx;
        }
    }

    LidarFrame frame;
    frame.timestampNs = 1234567890000000ULL;
    frame.pointsCount = kTotalPoints;
    frame.points      = points;

    // 检测框需完全覆盖对应组所有点的投影范围
    // bbox：左上角 (x1,y1) 包含，右下角 (x2,y2) 不包含
    std::vector<YoloBBox> detections;
    detections.push_back({410, 200, 600, 280, 2, 0.95f, imageTs});  // 车辆
    detections.push_back({180, 200, 335, 280, 0, 0.88f, imageTs});  // 行人

    // ---- 融合 ----
    LidarCameraFusion fusion;
    fusion.reset();
    bool ok = fusion.fuse_data(detections, imageTs, frame, cam);
    check(ok, "fuse_data == true") ? ++passed : ++failed;

    const FusionResult& r = fusion.result();

    // ---- 验证 bbox 数量与点数 ----
    ok = check(r.bboxCount == 2, "bboxCount == 2");
    ok ? ++passed : ++failed;

    ok = check(r.bboxPointCounts[0] == 50, "car bbox: 50 points");
    if (ok) ++passed; else { ++failed; printf("  actual: %u\n", r.bboxPointCounts[0]); }

    ok = check(r.bboxPointCounts[1] == 30, "person bbox: 30 points");
    if (ok) ++passed; else { ++failed; printf("  actual: %u\n", r.bboxPointCounts[1]); }

    // ---- 诊断计数器 ----
    ok = check(fusion.behind_camera_count() == 0, "behindCameraCount == 0");
    ok ? ++passed : ++failed;

    ok = check(fusion.out_of_image_count() == 0, "outOfImageCount == 0");
    ok ? ++passed : ++failed;

    // 匹配 + 后方 + 图像外 = 80，剩余 20 为背景点（不在任何 bbox 内）
    uint32_t classified = r.bboxPointCounts[0] + r.bboxPointCounts[1]
                          + fusion.behind_camera_count()
                          + fusion.out_of_image_count();
    ok = check(classified == 80, "match(50+30) == 80 (plus 20 background)");
    if (ok) ++passed; else { ++failed; printf("  actual: %u\n", classified); }

    // ---- 索引边界合法性 ----
    uint32_t totalIndices = r.bboxPointCounts[0] + r.bboxPointCounts[1];
    bool idxValid = true;
    for (uint32_t i = 0; i < totalIndices; ++i) {
        if (r.bboxPointIndices[i] >= kTotalPoints) { idxValid = false; break; }
    }
    ok = check(idxValid, "indices in [0,99]");
    ok ? ++passed : ++failed;

    // ---- 逆投影验证 ----
    // 对每个已分类的点，用手工重算投影坐标，确认落在对应的 bbox 内。
    // v 恒为 cy（因为外参中 cy ≡ 0），只需校验 u。
    bool classificationValid = true;
    for (uint32_t i = 0; i < r.bboxPointCounts[0]; ++i) {
        uint32_t pi = r.bboxPointIndices[i];
        float u = cam.fx * points[pi].x / points[pi].y + cam.cx;
        uint32_t ui = static_cast<uint32_t>(u);
        uint32_t vi = static_cast<uint32_t>(cam.cy);
        if (!(ui >= 410 && ui < 600 && vi >= 200 && vi < 280)) {
            classificationValid = false; break;
        }
    }
    uint32_t bbox1Offset = r.bboxPointCounts[0];
    for (uint32_t i = 0; i < r.bboxPointCounts[1]; ++i) {
        uint32_t pi = r.bboxPointIndices[bbox1Offset + i];
        float u = cam.fx * points[pi].x / points[pi].y + cam.cx;
        uint32_t ui = static_cast<uint32_t>(u);
        uint32_t vi = static_cast<uint32_t>(cam.cy);
        if (!(ui >= 180 && ui < 335 && vi >= 200 && vi < 280)) {
            classificationValid = false; break;
        }
    }
    ok = check(classificationValid, "reverse projection valid");
    ok ? ++passed : ++failed;

    printf("==========================================\n");
    printf("  Single camera: %d PASSED, %d FAILED\n", passed, failed);
    printf("==========================================\n");

    delete[] points;
    return (failed > 0) ? 1 : 0;
}
