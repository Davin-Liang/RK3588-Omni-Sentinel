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
 */
static void make_xy_to_xz_T(float* T)
{
    std::memset(T, 0, 16 * sizeof(float));
    T[0]  = 1.0f;   // cx = lx
    T[9]  = 1.0f;   // cz = ly
    T[15] = 1.0f;   // w  = 1
    // cy 恒为 0：雷达 z=0, row1 全为零
}

int main()
{
    int passed = 0;
    int failed = 0;

    const uint32_t kTotalPoints = 100;
    const uint64_t imageTs = 1234567890000000ULL;

    // ---- 合成雷达点 ----
    // 投影公式（与外参 + 内参相关）：u = fx * cx / cz + cx_principal
    // 使用 make_xy_to_xz_T: cx=lx, cz=ly, 故 u = fx * lx / ly + cx
    //
    // 组 A（相机 A 目标，40 点）：lx ∈ [1.05, 1.89], ly ∈ [3.10, 3.90]
    //   → u ∈ [400×1.05/3.90+320=428, 400×1.89/3.10+320=564]  ⊂ bboxA [410, 600)
    //
    // 组 B（相机 B 目标，40 点）：lx ∈ [-1.00, -0.30], ly ∈ [3.10, 3.90]
    //   → u ∈ [400×(-1.00)/3.10+320=191, 400×(-0.30)/3.90+320=289]  ⊂ bboxB [150, 300)
    //
    // 组 C（背景，20 点）：lx ∈ [0.32, 0.77], ly = 5.00~5.10
    //   → u ∈ [345, 382]，不在 bboxA 或 bboxB 内
    LidarPoint* points = new (std::nothrow) LidarPoint[kTotalPoints];
    if (!points) { fprintf(stderr, "[DEMO] alloc failed\n"); return 1; }

    uint32_t idx = 0;
    for (int xi = 0; xi < 8; ++xi) {
        for (int yi = 0; yi < 5; ++yi) {
            points[idx].x = 1.05f + 0.12f * static_cast<float>(xi);
            points[idx].y = 3.10f + 0.20f * static_cast<float>(yi);
            points[idx].intensity = 100.0f; ++idx;
        }
    }
    for (int xi = 0; xi < 8; ++xi) {
        for (int yi = 0; yi < 5; ++yi) {
            points[idx].x = -1.00f + 0.10f * static_cast<float>(xi);
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

    // ---- 两个相机 ----
    // 相同内参和图像尺寸，不同检测框覆盖互不重叠的 u 区间。
    // 实际场景中两个相机外参不同（FOV 不重叠），此处为简化测试共用外参。
    CameraConfig camA, camB;
    camA.fx = 400.0f; camA.fy = 400.0f; camA.cx = 320.0f; camA.cy = 240.0f;
    camA.imgWidth = 640; camA.imgHeight = 480;
    make_xy_to_xz_T(camA.tLidarToCam);
    camB = camA;

    std::vector<YoloBBox> detA, detB;
    detA.push_back({410, 200, 600, 280, 2, 0.9f});   // 相机 A
    detB.push_back({150, 200, 300, 280, 0, 0.8f});   // 相机 B

    // ---- 双相机累积融合 ----
    // reset() → fuse_data(A) → fuse_data(B) → result()
    // 同一雷达帧，两次调用追加累积，不覆盖。
    LidarCameraFusion fusion;
    fusion.reset();

    bool ok = fusion.fuse_data(detA, imageTs, frame, camA);
    check(ok, "camA fuse_data == true") ? ++passed : ++failed;

    ok = fusion.fuse_data(detB, imageTs, frame, camB);
    check(ok, "camB fuse_data == true") ? ++passed : ++failed;

    const FusionResult& r = fusion.result();

    // ---- bbox 总数 = 1 + 1 = 2 ----
    ok = check(r.bboxCount == 2, "bboxCount == 2");
    ok ? ++passed : ++failed;

    // ---- 各 bbox 点数 ----
    ok = check(r.bboxPointCounts[0] == 40, "camA bbox: 40 points (group A)");
    if (ok) ++passed; else { ++failed; printf("  actual: %u\n", r.bboxPointCounts[0]); }

    ok = check(r.bboxPointCounts[1] == 40, "camB bbox: 40 points (group B)");
    if (ok) ++passed; else { ++failed; printf("  actual: %u\n", r.bboxPointCounts[1]); }

    // ---- 无交叉泄漏 ----
    // 组 A 点索引在 [0, 39]，组 B 点索引在 [40, 79]。
    // 若某点被归入错误的 bbox，说明累积逻辑有 bug。
    bool noLeak = true;
    for (uint32_t i = 0; i < r.bboxPointCounts[0]; ++i) {
        if (r.bboxPointIndices[i] >= 40) { noLeak = false; break; }
    }
    uint32_t off1 = r.bboxPointCounts[0];  // bbox1 在展平数组中的起始偏移
    for (uint32_t i = 0; i < r.bboxPointCounts[1]; ++i) {
        if (r.bboxPointIndices[off1 + i] < 40 || r.bboxPointIndices[off1 + i] >= 80) {
            noLeak = false; break;
        }
    }
    ok = check(noLeak, "no cross-camera point leakage");
    ok ? ++passed : ++failed;

    // ---- 逆投影验证 ----
    // 对每个已分类的点手工重算投影坐标，确认落在对应 bbox 内。
    // v 恒为 cy（外参中 cy ≡ 0），只需校验 u 和 v 的范围。
    bool classificationValid = true;
    for (uint32_t i = 0; i < r.bboxPointCounts[0]; ++i) {
        uint32_t pi = r.bboxPointIndices[i];
        float u = camA.fx * points[pi].x / points[pi].y + camA.cx;
        uint32_t ui = static_cast<uint32_t>(u);
        uint32_t vi = static_cast<uint32_t>(camA.cy);
        if (!(ui >= 410 && ui < 600 && vi >= 200 && vi < 280)) {
            classificationValid = false; break;
        }
    }
    for (uint32_t i = 0; i < r.bboxPointCounts[1]; ++i) {
        uint32_t pi = r.bboxPointIndices[off1 + i];
        float u = camB.fx * points[pi].x / points[pi].y + camB.cx;
        uint32_t ui = static_cast<uint32_t>(u);
        uint32_t vi = static_cast<uint32_t>(camB.cy);
        if (!(ui >= 150 && ui < 300 && vi >= 200 && vi < 280)) {
            classificationValid = false; break;
        }
    }
    ok = check(classificationValid, "reverse projection valid");
    ok ? ++passed : ++failed;

    // ---- 时间戳传递 ----
    ok = check(r.imageTimestampNs == imageTs, "imageTimestampNs");
    ok ? ++passed : ++failed;

    ok = check(r.lidarTimestampNs == frame.timestampNs, "lidarTimestampNs");
    ok ? ++passed : ++failed;

    printf("==========================================\n");
    printf("  Dual camera: %d PASSED, %d FAILED\n", passed, failed);
    printf("==========================================\n");

    delete[] points;
    return (failed > 0) ? 1 : 0;
}
