#include "sentinel_lslidarer.h"

#include <cstdio>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <unistd.h>

// ============================================================================
// SentinelLslidarer — 生命周期
// ============================================================================

SentinelLslidarer::SentinelLslidarer() = default;

SentinelLslidarer::~SentinelLslidarer() {
    stop();
}

bool SentinelLslidarer::load_config(const LidarConfig& config) {
    if (running_.load(std::memory_order_acquire)) {
        return false;  // 运行时不允许修改配置
    }
    config_ = config;
    return true;
}

bool SentinelLslidarer::start() {
    if (running_.load(std::memory_order_acquire)) {
        return false;  // 已运行
    }

    // 1. 打开串口
    serialPort_ = std::make_unique<SerialPort>();
    if (!serialPort_->open(config_.serialPort, config_.baudRate)) {
        serialPort_.reset();
        return false;
    }

    // 2. 预计算 sin/cos LUT
    build_lut_();

    // 3. 分配环形缓冲区
    ringBuffer_ = std::make_unique<RingBuffer>(config_.ringBufferSize,
                                               LidarConfig::kPointsPerSweep);

    // 4. 启动读取线程
    running_.store(true, std::memory_order_release);
    readerThread_ = std::thread(&SentinelLslidarer::reader_loop_, this);

    return true;
}

void SentinelLslidarer::stop() {
    running_.store(false, std::memory_order_release);

    if (readerThread_.joinable()) {
        readerThread_.join();
    }

    if (serialPort_) {
        serialPort_->close();
        serialPort_.reset();
    }

    ringBuffer_.reset();
}

bool SentinelLslidarer::is_running() const {
    return running_.load(std::memory_order_acquire);
}

// ============================================================================
// 帧查询
// ============================================================================

bool SentinelLslidarer::get_closest_frame(uint64_t cameraTsNs, LidarFrame& outFrame) {
    if (!ringBuffer_) return false;

    uint32_t writeIdx = ringBuffer_->write_index();
    if (writeIdx == 0) return false;

    uint32_t validCount = std::min(writeIdx, ringBuffer_->capacity());
    if (validCount == 0) return false;

    // 线性扫描找最小时间差
    uint32_t bestIdx = 0;
    uint64_t bestDelta = UINT64_MAX;

    for (uint32_t i = 0; i < validCount; ++i) {
        LidarFrame tmp;
        tmp.points = outFrame.points;  // 复用调用者缓冲区
        ringBuffer_->copy_slot(i, tmp);

        if (tmp.pointsCount == 0) continue;

        uint64_t delta;
        if (tmp.timestampNs > cameraTsNs) {
            delta = tmp.timestampNs - cameraTsNs;
        } else {
            delta = cameraTsNs - tmp.timestampNs;
        }

        if (delta < bestDelta) {
            bestDelta = delta;
            bestIdx   = i;
        }
    }

    ringBuffer_->copy_slot(bestIdx, outFrame);
    return true;
}

uint32_t SentinelLslidarer::available_frames() const {
    if (!ringBuffer_) return 0;
    uint32_t writeIdx = ringBuffer_->write_index();
    return std::min(writeIdx, ringBuffer_->capacity());
}

uint32_t SentinelLslidarer::max_points_per_frame() const {
    return LidarConfig::kPointsPerSweep;
}

const LidarConfig& SentinelLslidarer::config() const {
    return config_;
}

// ============================================================================
// 读取线程
// ============================================================================

void SentinelLslidarer::reader_loop_() {
    constexpr int kMaxDecodeBuffer = 128;
    constexpr int kDiagInterval     = 5000;  // 每 5000 次重试打印一次诊断

    LidarPoint* sweepBuf = ringBuffer_->begin_write();
    uint32_t sweepCount  = 0;

    uint16_t lastAzimuth = 0;
    bool isFirstSweep     = true;
    uint64_t lastPacketTimeNs = 0;

    uint8_t rawBuf[256];
    SentinelLslidarer::DecodedPoint decoded[kMaxDecodeBuffer];

    // 诊断计数
    uint64_t retryCount    = 0;
    uint64_t timeoutCount  = 0;
    uint64_t syncFailCount = 0;
    uint64_t validPackets  = 0;
    uint64_t invalidPackets = 0;
    uint64_t sweepCount_   = 0;

    while (running_.load(std::memory_order_acquire)) {
        // ---- 周期性诊断输出 ----
        if (retryCount > 0 && (retryCount % kDiagInterval) == 0) {
            std::fprintf(stderr,
                "[LidarDiag] retries=%lu, timeouts=%lu, syncFails=%lu, "
                "validPkts=%lu, invalidPkts=%lu, sweeps=%lu\n",
                static_cast<unsigned long>(retryCount),
                static_cast<unsigned long>(timeoutCount),
                static_cast<unsigned long>(syncFailCount),
                static_cast<unsigned long>(validPackets),
                static_cast<unsigned long>(invalidPackets),
                static_cast<unsigned long>(sweepCount_));
        }

        // 一次性警告：串口无任何数据
        if (timeoutCount == 5 && validPackets == 0) {
            std::fprintf(stderr,
                "[LidarDiag] WARNING: No data received after 1000 timeouts.\n"
                "  Check: 1) radar power & motor spinning\n"
                "         2) serial port exists & has correct permissions\n"
                "         3) baud rate matches radar model (N10Plus=460800)\n"
                "         4) run: cat <serial_port> | xxd | head\n");
        }

        // ---- 1. 读取一个 N10Plus 包 ----
        int pktLen = 0;
        int ret = serialPort_->read_packet(rawBuf, pktLen);
        if (ret <= 0) {
            if (ret < 0) {
                std::fprintf(stderr, "[LidarDiag] Serial read error, exiting.\n");
                break;
            }
            ++retryCount;
            ++timeoutCount;
            usleep(1000);  // 1ms 回退，避免 CPU 空转
            continue;
        }

        // ---- 2. 校验包头和包尾 ----
        if (!check_packet_validity_(rawBuf, pktLen)) {
            ++retryCount;
            ++syncFailCount;
            continue;
        }
        ++validPackets;

        // ---- 3. 记录包到达时间戳 ----
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        uint64_t packetTsNs = static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL
                            + static_cast<uint64_t>(ts.tv_nsec);

        // ---- 4. 解码 ----
        int decodedCount = decode_packet_(rawBuf, pktLen, decoded, kMaxDecodeBuffer);
        if (decodedCount == 0) {
            ++invalidPackets;
            continue;
        }

        // ---- 5. 遍历解码点，检测圈边界 ----
        int boundaryIdx = decodedCount;
        for (int i = 0; i < decodedCount; ++i) {
            int diff = static_cast<int>(decoded[i].azimuth) - static_cast<int>(lastAzimuth);
            if (std::abs(diff) > 35000) {
                boundaryIdx = i;
                break;
            }
            lastAzimuth = decoded[i].azimuth;
        }

        // ---- 6. 第一圈处理：跳过不完整数据 ----
        if (isFirstSweep) {
            if (boundaryIdx == decodedCount) {
                continue;  // 未找到圈边界，丢弃整个包
            }
            // 从圈边界开始累积（边界前的点为上一圈残余，丢弃）
            for (int i = boundaryIdx; i < decodedCount; ++i) {
                lastAzimuth = decoded[i].azimuth;
                if (!is_point_valid_(decoded[i].distance, static_cast<int>(decoded[i].azimuth))) {
                    continue;
                }
                int lutIdx = decoded[i].azimuth;
                sweepBuf[sweepCount].x         = decoded[i].distance * cosLut_[lutIdx];
                sweepBuf[sweepCount].y         = decoded[i].distance * sinLut_[lutIdx];
                sweepBuf[sweepCount].intensity = decoded[i].intensity;
                ++sweepCount;
            }
            isFirstSweep = false;
            lastPacketTimeNs = packetTsNs;
            continue;
        }

        // ---- 7. 正常模式：累积 boundaryIdx 之前的点到当前圈 ----

        for (int i = 0; i < boundaryIdx; ++i) {
            if (!is_point_valid_(decoded[i].distance, static_cast<int>(decoded[i].azimuth))) {
                continue;
            }
            int lutIdx = decoded[i].azimuth;
            sweepBuf[sweepCount].x         = decoded[i].distance * cosLut_[lutIdx];
            sweepBuf[sweepCount].y         = decoded[i].distance * sinLut_[lutIdx];
            sweepBuf[sweepCount].intensity = 0.0f;
            ++sweepCount;
        }

        // ---- 8. 圈边界处理 ----
        if (boundaryIdx < decodedCount) {
            // 插值圈结束时间戳
            uint64_t sweepEndNs = packetTsNs;
            if (lastPacketTimeNs > 0) {
                int remainingAfterBoundary = decodedCount - boundaryIdx;
                sweepEndNs = packetTsNs - (packetTsNs - lastPacketTimeNs)
                           * static_cast<uint64_t>(remainingAfterBoundary)
                           / static_cast<uint64_t>(decodedCount);
            }

            ringBuffer_->commit_write(sweepEndNs, sweepCount);
            ++sweepCount_;

            // 重置，开始新一圈
            sweepBuf   = ringBuffer_->begin_write();
            sweepCount = 0;

            // boundaryIdx 之后的点属于新一圈
            for (int i = boundaryIdx; i < decodedCount; ++i) {
                lastAzimuth = decoded[i].azimuth;
                if (!is_point_valid_(decoded[i].distance, static_cast<int>(decoded[i].azimuth))) {
                    continue;
                }
                int lutIdx = decoded[i].azimuth;
                sweepBuf[sweepCount].x         = decoded[i].distance * cosLut_[lutIdx];
                sweepBuf[sweepCount].y         = decoded[i].distance * sinLut_[lutIdx];
                sweepBuf[sweepCount].intensity = decoded[i].intensity;
                ++sweepCount;
            }
        }

        lastPacketTimeNs = packetTsNs;
    }

    running_.store(false, std::memory_order_release);
}

