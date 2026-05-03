#pragma once

#include <atomic>
#include <mutex>
#include <vector>

enum class BufferFormat {
    YUV422,
    RGB888,
    NV12
};

/**
 * @brief DMA 内存块节点结构体
 * @note 作为一个单向链表节点，同时管理底层 DMA 物理内存的元数据
 */
struct DmaBuffer_t {
    int dmaFd;               ///< DMA 文件描述符 (File Descriptor)
    void* virtAddr;          ///< CPU 映射的虚拟地址
    std::atomic<bool> ifUse; ///< 无锁状态标记，true 表示正在被业务占用
    int bufferSize;          ///< 内存块的实际字节大小
    int width;               ///< 图像宽度
    int height;              ///< 图像高度
    DmaBuffer_t* next;       ///< 指向下一个空闲节点的指针

    DmaBuffer_t() : dmaFd(-1), virtAddr(nullptr), ifUse(false), 
                    bufferSize(0), width(0), height(0), next(nullptr) {}
};

/**
 * @brief DMA 物理内存池管理类
 * @details 基于空闲链表 (Free List) 和全量花名册的高性能零拷贝内存池，
 * 专为边缘计算平台的异构硬件模块设计。
 */
class DmaBufferPool {
public:
    /**
     * @brief 默认构造函数
     */
    DmaBufferPool();

    /**
     * @brief 析构函数，自动调用 destroy_pool 释放所有硬件资源
     */
    ~DmaBufferPool();

    DmaBufferPool(const DmaBufferPool&) = delete;
    DmaBufferPool& operator=(const DmaBufferPool&) = delete;

    /**
     * @brief 分配并初始化底层 DMA 内存池
     * @param count  需要预分配的 DMA 内存块数量
     * @param width  图像宽度
     * @param height 图像高度
     * @param format 图像格式 (决定最终的物理内存分配大小)
     * @return true 分配成功 | false 分配失败
     */
    bool alloc_pool(int count, int width, int height, BufferFormat format);

    /**
     * @brief 彻底销毁内存池并释放所有底层 DRM/DMA 物理内存
     */
    void destroy_pool(); // C++ 规范 2：参数列表不需要写 void

    /**
     * @brief 从内存池中获取一块空闲的 DMA 内存 (出栈)
     * @return DmaBuffer_t* 成功返回可用内存块指针，池空则返回 nullptr
     */
    DmaBuffer_t* get_buffer();

    /**
     * @brief 将使用完毕的 DMA 内存块归还给内存池 (入栈)
     * @param buf 需要归还的内存块指针
     */
    void release_buffer(DmaBuffer_t* buf);

private:
    // C++ 规范 3：私有成员变量统一加上后缀下划线 `_`，以区分局部变量
    DmaBuffer_t* head_;                      ///< 空闲链表的头指针
    std::mutex poolMutex_;                   ///< 保护链表结构的出入栈互斥锁
    std::vector<DmaBuffer_t*> allBuffers_;   ///< 全量花名册，用于销毁时的绝对安全追踪
};