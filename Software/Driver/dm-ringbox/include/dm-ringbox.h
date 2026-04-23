/**
 * @file dm-ringbox.h
 * @brief RingBox Device Mapper 驱动头文件
 *
 * @details 定义驱动使用的宏、常量和数据结构。
 *          本头文件同时供内核驱动和用户空间程序使用。
 *          用户空间程序可通过此头文件访问 ioctl 接口。
 *
 * @author RK3568 Development Team
 * @version 1.0.0
 * @date 2026-04-15
 * @license GPL
 */

#ifndef _DM_RINGBOX_H
#define _DM_RINGBOX_H

#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
#include <stddef.h>
#include <sys/ioctl.h>
#endif

/*============================================================================
 * 宏定义与常量
 *============================================================================*/

/** @brief 驱动名称 */
#define RINGBOX_NAME            "ringbox"

/** @brief 驱动版本号 */
#define RINGBOX_VERSION_MAJOR   1
#define RINGBOX_VERSION_MINOR   0
#define RINGBOX_VERSION_PATCH   0

/** @brief 驱动版本字符串 */
#define RINGBOX_VERSION_STRING  "1.0.0"

/** @brief ioctl 命令魔数 */
#define RINGBOX_IOCTL_MAGIC     'R'

/*============================================================================
 * ioctl 命令定义
 *============================================================================*/

/**
 * @defgroup ioctl_commands ioctl 命令定义
 * @brief 用户空间与驱动交互的控制命令
 * @{
 */

/** @brief 获取环形缓冲区状态 */
#define RINGBOX_IOC_GET_STATUS  _IOR(RINGBOX_IOCTL_MAGIC, 0x01, struct ringbox_status)

/** @brief 重置写入指针和统计信息 */
#define RINGBOX_IOC_RESET_WRITE _IO(RINGBOX_IOCTL_MAGIC, 0x02)

/** @brief 设置物理起始地址 */
#define RINGBOX_IOC_SET_START   _IOW(RINGBOX_IOCTL_MAGIC, 0x03, uint64_t)

/** @brief 获取驱动版本信息 */
#define RINGBOX_IOC_GET_VERSION _IOR(RINGBOX_IOCTL_MAGIC, 0x04, struct ringbox_version)

/** @brief 获取设备容量信息 */
#define RINGBOX_IOC_GET_CAPACITY _IOR(RINGBOX_IOCTL_MAGIC, 0x05, struct ringbox_capacity)

/** @} */ /* end of ioctl_commands */

/*============================================================================
 * 数据结构定义
 *============================================================================*/

/**
 * @brief 驱动版本信息结构体
 * @details 用于向应用层返回驱动版本号
 */
struct ringbox_version {
    uint32_t major;         /**< 主版本号 */
    uint32_t minor;         /**< 次版本号 */
    uint32_t patch;         /**< 修订号 */
    char     version_str[16]; /**< 版本字符串 */
};

/**
 * @brief 环形缓冲区状态信息结构体
 * @details 用于向应用层返回当前驱动状态
 */
struct ringbox_status {
    uint64_t physical_start;        /**< 物理起始扇区 */
    uint64_t ring_capacity;         /**< 环形缓冲区总容量（扇区数） */
    uint64_t current_write_pos;     /**< 当前写入位置（扇区） */
    uint64_t total_write_ops;       /**< 累计写操作次数 */
    uint64_t total_read_ops;        /**< 累计读操作次数 */
    uint64_t total_sectors_written; /**< 累计写入扇区数 */
    uint32_t is_active;             /**< 设备激活状态标志 (1: 激活, 0: 非激活) */
    uint32_t reserved;              /**< 保留字段，用于对齐 */
};

/**
 * @brief 设备容量信息结构体
 * @details 用于向应用层返回设备容量配置
 */
struct ringbox_capacity {
    uint64_t physical_start;    /**< 物理起始扇区 */
    uint64_t ring_capacity;     /**< 环形缓冲区总容量（扇区数） */
    uint64_t ring_size_bytes;   /**< 环形缓冲区大小（字节） */
    uint64_t ring_size_mb;      /**< 环形缓冲区大小（MB） */
};

/*============================================================================
 * 内核空间专用定义
 *============================================================================*/

#ifdef __KERNEL__

#include <linux/spinlock.h>
#include <linux/atomic.h>
#include <linux/device-mapper.h>

/**
 * @brief 驱动私有数据结构
 * @details 依附于每个生成的 dm_target，存储设备配置和运行时状态
 */
struct ringbox_c {
    /* 设备配置 */
    struct dm_dev *target_dev;      /**< 底层物理设备指针 (如 /dev/nvme0n1) */
    sector_t physical_start;        /**< 在物理盘上的起始扇区偏移 */
    sector_t ring_capacity;         /**< 环形缓冲区的总物理扇区数 */

    /* 运行时状态 - 使用原子变量保证线程安全 */
    atomic64_t current_write_pos;   /**< 当前写入位置（环形，取模后） */
    atomic64_t total_write_ops;     /**< 累计写操作计数 */
    atomic64_t total_read_ops;      /**< 累计读操作计数 */
    atomic64_t total_sectors;       /**< 累计传输扇区数 */

    /* 同步保护 */
    spinlock_t config_lock;         /**< 配置修改自旋锁 */
    bool is_active;                 /**< 设备激活状态标志 */
};

/*============================================================================
 * 内核空间函数声明
 *============================================================================*/

/**
 * @brief  构造函数 (.ctr)
 * @details 由 dmsetup create 触发。负责解析参数，打开底层设备，分配内存。
 *
 * @param  ti   Device Mapper 目标结构体指针
 * @param  argc 参数个数
 * @param  argv 参数数组
 *
 * @return 0 成功
 * @return -EINVAL 参数无效
 * @return -ENOMEM 内存分配失败
 */
int ringbox_ctr(struct dm_target *ti, unsigned int argc, char **argv);

/**
 * @brief  析构函数 (.dtr)
 * @details 由 dmsetup remove 触发。负责释放设备句柄和申请的内存。
 *
 * @param  ti Device Mapper 目标结构体指针
 */
void ringbox_dtr(struct dm_target *ti);

/**
 * @brief  核心映射函数 (.map)
 * @details 拦截并篡改 struct bio，实现环形缓冲区逻辑。
 *
 * @param  ti  Device Mapper 目标结构体指针
 * @param  bio 描述一次块 I/O 操作的核心结构体
 *
 * @return DM_MAPIO_REMAPPED 已修改地址并交由底层处理
 * @return DM_MAPIO_KILL     请求无效，终止 I/O
 */
int ringbox_map(struct dm_target *ti, struct bio *bio);

/**
 * @brief  ioctl 控制接口
 * @details 提供用户空间与驱动的交互接口。
 *
 * @param  ti  Device Mapper 目标结构体指针
 * @param  cmd ioctl 命令字
 * @param  arg 用户空间传入的参数
 *
 * @return 0 成功
 * @return -ENOTTY 不支持的命令
 * @return -EFAULT 数据拷贝失败
 */
int ringbox_ioctl(struct dm_target *ti, unsigned int cmd, unsigned long arg);

/**
 * @brief  状态输出函数
 * @details 响应 dmsetup status 命令，输出当前设备状态。
 *
 * @param  ti            Device Mapper 目标结构体指针
 * @param  type          状态类型
 * @param  status_flags  状态标志
 * @param  result        输出缓冲区
 * @param  maxlen        缓冲区最大长度
 */
void ringbox_status(struct dm_target *ti, status_type_t type,
                    unsigned int status_flags, char *result, unsigned int maxlen);

#endif /* __KERNEL__ */

#endif /* _DM_RINGBOX_H */
