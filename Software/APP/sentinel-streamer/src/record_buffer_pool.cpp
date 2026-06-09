#include "record_buffer_pool.h"
#include <cstdio>
#include <cstring>
#include <new>

RecordBufferPool::RecordBufferPool()
    : slots_(nullptr)
    , capacity_(0)
    , writeIdx_(0)
    , readIdx_(0)
    , count_(0)
    , frameSize_(0)
{}

RecordBufferPool::~RecordBufferPool()
{
    destroy_pool();
}

bool RecordBufferPool::alloc_pool(int slotCount, int width, int height)
{
    if (slotCount <= 0 || width <= 0 || height <= 0) {
        fprintf(stderr, "[RecordBufferPool] invalid params: slots=%d %dx%d\n",
                slotCount, width, height);
        return false;
    }

    if (!pool_.alloc_pool(slotCount, width, height, BufferFormat::NV12)) {
        fprintf(stderr, "[RecordBufferPool] DmaBufferPool alloc_pool failed\n");
        return false;
    }

    frameSize_ = static_cast<size_t>(width) * height * 3 / 2;
    capacity_ = slotCount;

    slots_ = new (std::nothrow) Slot[capacity_];
    if (!slots_) {
        fprintf(stderr, "[RecordBufferPool] alloc slots_ failed\n");
        pool_.destroy_pool();
        capacity_ = 0;
        frameSize_ = 0;
        return false;
    }

    for (int i = 0; i < capacity_; ++i) {
        slots_[i].buffer = pool_.get_buffer();
        if (!slots_[i].buffer) {
            fprintf(stderr, "[RecordBufferPool] get_buffer failed at slot %d\n", i);
            delete[] slots_;
            slots_ = nullptr;
            pool_.destroy_pool();
            capacity_ = 0;
            frameSize_ = 0;
            return false;
        }
        slots_[i].timestampUs = 0;
        slots_[i].written = false;
        slots_[i].checkedOut = false;
        addrToIdx_[slots_[i].buffer->virtAddr] = i;
    }

    writeIdx_ = 0;
    readIdx_ = 0;
    count_ = 0;

    fprintf(stderr, "[RecordBufferPool] allocated %d slots, %dx%d NV12 (%zu bytes each)\n",
            capacity_, width, height, frameSize_);
    return true;
}

void RecordBufferPool::destroy_pool()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (slots_) {
        delete[] slots_;
        slots_ = nullptr;
    }
    addrToIdx_.clear();
    pool_.destroy_pool();
    capacity_ = 0;
    count_ = 0;
    frameSize_ = 0;
}

// RGA NV12 DMA 硬件拷贝（定义在 rga_scaler.cpp）
extern bool rga_nv12_copy(int srcFd, int srcWidth, int srcHeight, int dstFd);

void RecordBufferPool::write_frame(int srcDmaFd, int srcWidth, int srcHeight,
                                   uint64_t timestampUs)
{
    std::lock_guard<std::mutex> lock(mutex_);

    int idx = writeIdx_;
    int attempts = 0;
    while (slots_[idx].checkedOut && attempts < capacity_) {
        idx = (idx + 1) % capacity_;
        ++attempts;
    }
    if (attempts >= capacity_) {
        fprintf(stderr, "[RecordBufferPool] all slots checkedOut, dropping frame\n");
        return;
    }

    if (!rga_nv12_copy(srcDmaFd, srcWidth, srcHeight, slots_[idx].buffer->dmaFd)) {
        fprintf(stderr, "[RecordBufferPool] RGA copy failed, dropping frame\n");
        return;
    }

    if (!slots_[idx].written && count_ < capacity_) ++count_;
    slots_[idx].timestampUs = timestampUs;
    slots_[idx].written = true;
    writeIdx_ = (idx + 1) % capacity_;
}

bool RecordBufferPool::try_get_record_frame(uint8_t** outData, size_t* outSize,
                                            uint64_t* outTimestampUs)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (count_ == 0) return false;

    int attempts = 0;
    while (attempts < capacity_) {
        int idx = readIdx_;
        if (slots_[idx].written && !slots_[idx].checkedOut) {
            slots_[idx].checkedOut = true;
            slots_[idx].written = false;
            --count_;
            readIdx_ = (readIdx_ + 1) % capacity_;
            *outData = static_cast<uint8_t*>(slots_[idx].buffer->virtAddr);
            *outSize = frameSize_;
            *outTimestampUs = slots_[idx].timestampUs;
            return true;
        }
        readIdx_ = (readIdx_ + 1) % capacity_;
        ++attempts;
    }
    return false;
}

void RecordBufferPool::release_record_frame(const uint8_t* data)
{
    std::lock_guard<std::mutex> lock(mutex_);

    auto it = addrToIdx_.find(data);
    if (it != addrToIdx_.end()) {
        slots_[it->second].checkedOut = false;
    }
}

int RecordBufferPool::frame_count() const
{
    return count_;
}
