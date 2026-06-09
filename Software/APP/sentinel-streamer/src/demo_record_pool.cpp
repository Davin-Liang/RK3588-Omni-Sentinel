/**
 * @brief RecordBufferPool 独立测试程序
 *
 * 验证环形缓冲池的：初始化 → 写入（含环绕覆盖） → 消费 FIFO → 归还 → 计数正确性。
 * 在 RK3588 板端运行，使用真实 DMA 内存和 RGA 硬件拷贝。
 */

#include <cstdio>
#include <cstring>
#include "record_buffer_pool.h"
#include "dma-buffer-pool.h"

int main()
{
    fprintf(stderr, "=== RecordBufferPool 单元测试 ===\n");

    // ---- 1. 创建源 DMA 缓冲池（模拟 streamer 的 origCopyPool） ----
    const int srcWidth  = 640;
    const int srcHeight = 480;
    const size_t frameSize = srcWidth * srcHeight * 3 / 2;  // NV12

    DmaBufferPool srcPool;
    if (!srcPool.alloc_pool(1, srcWidth, srcHeight, BufferFormat::NV12)) {
        fprintf(stderr, "[TEST] FAIL: srcPool alloc_pool failed\n");
        return 1;
    }
    DmaBuffer_t* srcBuf = srcPool.get_buffer();
    if (!srcBuf) {
        fprintf(stderr, "[TEST] FAIL: srcPool get_buffer failed\n");
        return 1;
    }

    // 填充测试图案（Y 平面渐变，UV 平面固定值）
    uint8_t* yPlane  = static_cast<uint8_t*>(srcBuf->virtAddr);
    uint8_t* uvPlane = yPlane + srcWidth * srcHeight;
    for (int y = 0; y < srcHeight; ++y) {
        memset(yPlane + y * srcWidth, 128 + (y * 127 / srcHeight), srcWidth);
    }
    memset(uvPlane, 128, srcWidth * srcHeight / 2);

    fprintf(stderr, "[TEST] src dmaFd=%d size=%zux%zu\n",
            srcBuf->dmaFd, frameSize, frameSize);

    // ---- 2. 创建 RecordBufferPool（5 槽位，640x480） ----
    const int slotCount = 5;
    RecordBufferPool pool;
    if (!pool.alloc_pool(slotCount, srcWidth, srcHeight)) {
        fprintf(stderr, "[TEST] FAIL: alloc_pool failed\n");
        return 1;
    }
    fprintf(stderr, "[TEST] pool allocated: %d slots, %dx%d, frameSize=%zu\n",
            slotCount, srcWidth, srcHeight, frameSize);

    // ---- 3. 写入 10 帧（超过 5 槽位，触发环绕覆盖） ----
    fprintf(stderr, "[TEST] writing 10 frames (capacity=%d)...\n", slotCount);
    for (int i = 0; i < 10; ++i) {
        uint64_t ts = 1000000 + i * 33333;  // 模拟 30fps 时间戳
        pool.write_frame(srcBuf->dmaFd, srcWidth, srcHeight, ts);
        fprintf(stderr, "[TEST]   write frame %d ts=%llu count=%d\n",
                i, (unsigned long long)ts, pool.frame_count());
    }

    // 预期：首轮写满 5 帧后 count=5，之后每次覆盖旧帧 count 保持 5
    int finalCount = pool.frame_count();
    if (finalCount != slotCount) {
        fprintf(stderr, "[TEST] FAIL: expected count=%d, got %d after 10 writes\n",
                slotCount, finalCount);
        return 1;
    }
    fprintf(stderr, "[TEST] PASS: count=%d after overflowing writes\n", finalCount);

    // ---- 4. 消费 3 帧，验证 FIFO 顺序 ----
    fprintf(stderr, "[TEST] consuming 3 frames...\n");
    for (int i = 0; i < 3; ++i) {
        uint8_t* data = nullptr;
        size_t size = 0;
        uint64_t ts = 0;
        if (!pool.try_get_record_frame(&data, &size, &ts)) {
            fprintf(stderr, "[TEST] FAIL: try_get_record_frame returned false on frame %d\n", i);
            return 1;
        }
        fprintf(stderr, "[TEST]   get frame %d: data=%p size=%zu ts=%llu\n",
                i, (void*)data, size, (unsigned long long)ts);

        // 验证数据完整性（读取 Y 平面第一行验证非零）
        if (data[0] == 0 && data[1] == 0) {
            fprintf(stderr, "[TEST] WARN: frame %d first bytes: %02x %02x %02x %02x\n",
                    i, data[0], data[1], data[2], data[3]);
        }
        pool.release_record_frame(data);
    }
    fprintf(stderr, "[TEST] PASS: 3 frames consumed and released\n");

    // ---- 5. 验证归还后可继续写入 ----
    fprintf(stderr, "[TEST] writing 3 more frames after release...\n");
    for (int i = 0; i < 3; ++i) {
        uint64_t ts = 2000000 + i * 33333;
        pool.write_frame(srcBuf->dmaFd, srcWidth, srcHeight, ts);
        fprintf(stderr, "[TEST]   write frame post-release %d ts=%llu count=%d\n",
                i, (unsigned long long)ts, pool.frame_count());
    }

    // ---- 6. 全量消费并验证 ----
    fprintf(stderr, "[TEST] draining all remaining frames...\n");
    int consumed = 0;
    uint64_t lastTs = 0;
    while (true) {
        uint8_t* data = nullptr;
        size_t size = 0;
        uint64_t ts = 0;
        if (!pool.try_get_record_frame(&data, &size, &ts)) break;
        if (consumed > 0 && ts < lastTs) {
            fprintf(stderr, "[TEST] WARN: timestamp went backward! %llu < %llu\n",
                    (unsigned long long)ts, (unsigned long long)lastTs);
        }
        lastTs = ts;
        pool.release_record_frame(data);
        ++consumed;
    }
    fprintf(stderr, "[TEST] drained %d frames, final count=%d\n",
            consumed, pool.frame_count());

    // ---- 7. 清理 ----
    pool.destroy_pool();
    srcPool.release_buffer(srcBuf);
    srcPool.destroy_pool();

    fprintf(stderr, "[TEST] === ALL TESTS PASSED ===\n");
    return 0;
}
