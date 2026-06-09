#ifndef RECORD_BUFFER_POOL_H
#define RECORD_BUFFER_POOL_H

#include <cstdint>
#include <cstddef>
#include <mutex>
#include <unordered_map>
#include "dma-buffer-pool.h"

class RecordBufferPool {
public:
    RecordBufferPool();
    ~RecordBufferPool();

    RecordBufferPool(const RecordBufferPool&) = delete;
    RecordBufferPool& operator=(const RecordBufferPool&) = delete;

    bool alloc_pool(int slotCount, int width, int height);

    void destroy_pool();

    void write_frame(int srcDmaFd, int srcWidth, int srcHeight, uint64_t timestampUs);

    bool try_get_record_frame(uint8_t** outData, size_t* outSize,
                              uint64_t* outTimestampUs);

    void release_record_frame(const uint8_t* data);

    int frame_count() const;

private:
    DmaBufferPool pool_;
    struct Slot {
        DmaBuffer_t* buffer;
        uint64_t timestampUs;
        bool written;
        bool checkedOut;
    };
    Slot* slots_;
    int capacity_;
    int writeIdx_;
    int readIdx_;
    int count_;
    size_t frameSize_;
    std::mutex mutex_;
    std::unordered_map<const void*, int> addrToIdx_;
};

#endif // RECORD_BUFFER_POOL_H
