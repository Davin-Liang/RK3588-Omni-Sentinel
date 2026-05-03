#include "dma-buffer-pool.h"
#include <memory>  // 引入智能指针
#include <iostream>
#include "dma_alloc.h"

DmaBufferPool::DmaBufferPool() 
{
    head_ = nullptr;
}

DmaBufferPool::~DmaBufferPool() 
{
    destroy_pool();
}

bool DmaBufferPool::alloc_pool(int count, int width, int height, BufferFormat format)
{
    int bufferSize;
    
    /* param check */
    if (!count) {
        std::cerr << "[err]: Buffer number can't be 0!" << std::endl;
        return false;  
    }
    if (!width || !height) {
        std::cerr << "[err]: Buffer's length of side can't be 0!" << std::endl;
        return false;       
    }

    // 使用强类型枚举 (enum class) 的作用域解析
    if (format == BufferFormat::YUV422)
        bufferSize = width * height * 2;
    else if (format == BufferFormat::RGB888)
        bufferSize = width * height * 3;
    else if (format == BufferFormat::NV12)
        bufferSize = width * height * 3 / 2; // NV12 是 1.5 倍
    else
        bufferSize = width * height * 2; // 默认兜底
    
    for (int i = 0; i < count; ++i) {
        std::unique_ptr<DmaBuffer_t> newBuf = std::make_unique<DmaBuffer_t>();
        
        /* alloc DMA physical buffer */
        newBuf->bufferSize = bufferSize;
        newBuf->width = width;
        newBuf->height = height;
        
        int ret = dma_buf_alloc(DMA_HEAP_DMA32_UNCACHE_PATCH, newBuf->bufferSize, 
                                &(newBuf->dmaFd), &(newBuf->virtAddr));
        if (ret < 0 || newBuf->virtAddr == nullptr) {
            std::cerr << "[err]: Failed to allocate DMA buffer!" << std::endl;
            destroy_pool(); // 申请失败，安全撤退
            return false;
        }
        
        /* 成功分配 DMA 内存后，剥夺 unique_ptr 的控制权，交给原始链表 */
        DmaBuffer_t* rawBuf = newBuf.release();
        
        /* 登记到全量花名册，保证销毁时不漏一个 */
        allBuffers_.push_back(rawBuf);
        
        /* 头插法：将新节点挂在当前的 head_ 前面，然后自己成为新的 head_ */
        rawBuf->next = head_;
        head_ = rawBuf;
    }

    return true;
}

void DmaBufferPool::destroy_pool()
{
    std::lock_guard<std::mutex> lock(poolMutex_); 
    
    /* 遍历全量花名册，无差别击杀所有分配过的内存 */
    for (DmaBuffer_t* current : allBuffers_) {
        if (current != nullptr) {
            if (current->dmaFd >= 0 || current->virtAddr != nullptr) {
                dma_buf_free(current->bufferSize, &(current->dmaFd), &(current->virtAddr));
                std::cout << "DMA buffer freed. Fd: " << current->dmaFd << std::endl;
            }
            delete current; 
        }
    }
    
    allBuffers_.clear();
    head_ = nullptr;
    std::cout << "内存池已彻底销毁！" << std::endl;    
}

DmaBuffer_t* DmaBufferPool::get_buffer() 
{
    std::lock_guard<std::mutex> lock(poolMutex_);
    
    if (head_ == nullptr) return nullptr; 

    DmaBuffer_t* buf = head_;
    head_ = head_->next; 
    
    buf->ifUse.store(true); // 标记为已占用
    buf->next = nullptr;    // 断开它与链表的联系，防止外部乱指
    
    return buf;
}

void DmaBufferPool::release_buffer(DmaBuffer_t* buf) 
{
    if (buf == nullptr) return;
    
    buf->ifUse.store(false); // 恢复为空闲状态

    std::lock_guard<std::mutex> lock(poolMutex_);
        
    /* 头插法：将归还的节点重新放回链表头部 */
    buf->next = head_;
    head_ = buf;
}