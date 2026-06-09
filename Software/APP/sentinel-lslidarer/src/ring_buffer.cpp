#include "sentinel_lslidarer.h"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <stdlib.h>

// ============================================================================
// RingBuffer
// ============================================================================

RingBuffer::RingBuffer(uint32_t capacity, uint32_t maxPointsPerSlot)
    : capacity_(capacity)
    , maxPointsPerSlot_(maxPointsPerSlot)
    , slots_(nullptr)
    , pointsPool_(nullptr) {

    slots_ = new (std::nothrow) Slot[capacity_]();
    if (!slots_) return;

    size_t poolSize = static_cast<size_t>(capacity_) * maxPointsPerSlot_;
    posix_memalign(reinterpret_cast<void**>(&pointsPool_), 64,
                   poolSize * sizeof(LidarPoint));
}

RingBuffer::~RingBuffer() {
    delete[] slots_;
    free(pointsPool_);
}

LidarPoint* RingBuffer::begin_write() {
    uint32_t idx = writeIndex_.load(std::memory_order_relaxed) % capacity_;
    return pointsPool_ + static_cast<size_t>(idx) * maxPointsPerSlot_;
}

void RingBuffer::commit_write(uint64_t timestampNs, uint32_t pointsCount) {
    uint32_t idx = writeIndex_.load(std::memory_order_relaxed) % capacity_;
    uint32_t seq = writeIndex_.load(std::memory_order_relaxed);

    slots_[idx].pointsCount = pointsCount;
    slots_[idx].timestampNs = timestampNs;

    // Sequence store with release — 保证上述槽数据写入对读者可见
    slots_[idx].sequence.store(seq + 1, std::memory_order_release);

    writeIndex_.fetch_add(1, std::memory_order_release);
}

uint32_t RingBuffer::write_index() const {
    return writeIndex_.load(std::memory_order_acquire);
}

void RingBuffer::copy_slot(uint32_t slotIdx, LidarFrame& outFrame) {
    // Acquire 保证读到对应 sequence 对应的槽数据
    uint32_t seq = slots_[slotIdx].sequence.load(std::memory_order_acquire);
    (void)seq;  // 用于内存屏障，实际不需要使用值

    outFrame.timestampNs = slots_[slotIdx].timestampNs;
    uint32_t count = slots_[slotIdx].pointsCount;
    outFrame.pointsCount = std::min(count, maxPointsPerSlot_);

    size_t byteCount = static_cast<size_t>(outFrame.pointsCount) * sizeof(LidarPoint);
    std::memcpy(outFrame.points,
                pointsPool_ + static_cast<size_t>(slotIdx) * maxPointsPerSlot_,
                byteCount);
}

// C++14 要求 static constexpr 成员有类外定义（ODR-use）
constexpr int    LidarConfig::kPacketLength;
constexpr int    LidarConfig::kAngleBitsStart;
constexpr int    LidarConfig::kEndAngleBitsStart;
constexpr int    LidarConfig::kDataBitsStart;
constexpr int    LidarConfig::kPacketPointsMax;
constexpr int    LidarConfig::kPointsPerSweep;
constexpr float  LidarConfig::kDistanceResolution;
