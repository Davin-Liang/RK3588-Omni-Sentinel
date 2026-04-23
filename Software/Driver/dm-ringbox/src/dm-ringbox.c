/**
 * @file dm-ringbox.c
 * @brief 自定义 Device Mapper 驱动，实现基于 NVMe SSD 的环形覆盖写入功能。
 *
 * @details 本驱动专为高性能日志/黑匣子存储设计，实现以下核心功能：
 *          1. 调用 dm_register_target() 注册名为 "ringbox" 的自定义映射目标
 *          2. 实现 .map 回调函数，拦截并篡改 struct bio 的目标设备和物理扇区地址
 *          3. 支持环形缓冲区循环写入，实现数据覆盖机制
 *
 * @design_principles 设计规范：
 *          - 数据零拷贝：仅修改 bio 元数据，不触碰实际数据载荷
 *          - 事件驱动：基于内核块层回调，无后台轮询线程
 *          - 硬件优先：让 NVMe 控制器和 DMA 处理数据传输
 *          - 内存安全：严格管理内存分配与释放
 *
 * @author RK3568 Development Team
 * @version 1.0.0
 * @date 2026-04-15
 * @license GPL
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/blkdev.h>
#include <linux/device-mapper.h>
#include <linux/bio.h>
#include <linux/uaccess.h>

#include "dm-ringbox.h"

/*============================================================================
 * Device Mapper 目标注册结构
 *============================================================================*/

/**
 * @brief ringbox 目标类型定义
 * @details 注册到内核 Device Mapper 框架的目标描述符
 */
static struct target_type ringbox_target = {
    .name           = RINGBOX_NAME,
    .version        = {RINGBOX_VERSION_MAJOR, RINGBOX_VERSION_MINOR, RINGBOX_VERSION_PATCH},
    .module         = THIS_MODULE,
    .ctr            = ringbox_ctr,
    .dtr            = ringbox_dtr,
    .map            = ringbox_map,
    .ioctl          = ringbox_ioctl,
    .status         = ringbox_status,
    .features       = DM_TARGET_SINGLETON,  /* 单例模式，确保设备唯一性 */
};

/*============================================================================
 * 构造函数实现
 *============================================================================*/

/**
 * @brief  构造函数 (.ctr)
 * @details 由 dmsetup create 触发。负责解析参数，打开底层设备，分配内存。
 *          实现严格的参数校验和错误处理，确保内存安全。
 *
 * @param  ti   Device Mapper 目标结构体指针
 * @param  argc 参数个数
 * @param  argv 参数数组
 *              - argv[0]: 底层设备路径 (如 /dev/nvme0n1)
 *              - argv[1]: 物理起始扇区
 *              - argv[2]: 环形缓冲区大小（扇区数）
 *
 * @return 0 成功
 * @return -EINVAL 参数无效
 * @return -ENOMEM 内存分配失败
 * @return 其他负值 设备获取失败
 *
 * @note 必须与 ringbox_dtr() 配对使用，确保资源正确释放
 */
int ringbox_ctr(struct dm_target *ti, unsigned int argc, char **argv)
{
    struct ringbox_c *ringbox_ctx = NULL;
    unsigned long long temp_start;
    unsigned long long temp_capacity;
    int ret = 0;

    /*------------------------------------------------------------
     * 1. 参数数量校验
     *------------------------------------------------------------*/
    if (argc != 3) {
        ti->error = "Invalid argument count. Expected: <device> <start_sector> <capacity_sectors>";
        return -EINVAL;
    }

    /*------------------------------------------------------------
     * 2. 内存分配 (使用 kzalloc 确保零初始化，防止脏数据)
     *------------------------------------------------------------*/
    ringbox_ctx = kzalloc(sizeof(struct ringbox_c), GFP_KERNEL);
    if (ringbox_ctx == NULL) {
        ti->error = "Failed to allocate memory for ringbox context";
        return -ENOMEM;
    }

    /*------------------------------------------------------------
     * 3. 解析物理扇区参数
     *------------------------------------------------------------*/
    if (kstrtoull(argv[1], 10, &temp_start) != 0) {
        ti->error = "Invalid physical start sector format";
        ret = -EINVAL;
        goto error_free_ctx;
    }

    if (kstrtoull(argv[2], 10, &temp_capacity) != 0 || temp_capacity == 0) {
        ti->error = "Invalid ring capacity (must be > 0)";
        ret = -EINVAL;
        goto error_free_ctx;
    }

    ringbox_ctx->physical_start = (sector_t)temp_start;
    ringbox_ctx->ring_capacity = (sector_t)temp_capacity;

    /*------------------------------------------------------------
     * 4. 获取底层块设备引用
     *------------------------------------------------------------*/
    ret = dm_get_device(ti, argv[0], dm_table_get_mode(ti->table), &ringbox_ctx->target_dev);
    if (ret != 0) {
        ti->error = "Failed to acquire target block device";
        goto error_free_ctx;
    }

    /*------------------------------------------------------------
     * 5. 初始化运行时状态
     *------------------------------------------------------------*/
    spin_lock_init(&ringbox_ctx->config_lock);
    atomic64_set(&ringbox_ctx->current_write_pos, 0);
    atomic64_set(&ringbox_ctx->total_write_ops, 0);
    atomic64_set(&ringbox_ctx->total_read_ops, 0);
    atomic64_set(&ringbox_ctx->total_sectors, 0);
    ringbox_ctx->is_active = true;

    /*------------------------------------------------------------
     * 6. 设置设备最小/最大 I/O 限制
     *------------------------------------------------------------*/
    ti->private = ringbox_ctx;

    /* 禁用 flush 优化，确保数据一致性 */
    ti->num_flush_bios = 1;
    ti->num_discard_bios = 1;
    ti->num_write_zeroes_bios = 1;

    pr_info("%s: Device constructed successfully\n"
            "  - Device: %s\n"
            "  - Physical Start: %llu sectors\n"
            "  - Ring Capacity: %llu sectors (%llu MB)\n",
            RINGBOX_NAME, argv[0],
            (unsigned long long)ringbox_ctx->physical_start,
            (unsigned long long)ringbox_ctx->ring_capacity,
            (unsigned long long)(ringbox_ctx->ring_capacity >> 11));  /* 扇区转MB */

    return 0;  /* 成功返回 0 */

/*------------------------------------------------------------
 * 错误处理路径 (遵循内核 goto 错误处理范式)
 *------------------------------------------------------------*/
error_free_ctx:
    kfree(ringbox_ctx);
    return ret;
}

/*============================================================================
 * 析构函数实现
 *============================================================================*/

/**
 * @brief  析构函数 (.dtr)
 * @details 由 dmsetup remove 触发。负责释放设备句柄和申请的内存。
 *          严格遵循资源释放顺序，避免内存泄漏和设备引用计数错误。
 *
 * @param  ti Device Mapper 目标结构体指针
 *
 * @note 资源释放顺序：先释放设备引用，再释放内存
 */
void ringbox_dtr(struct dm_target *ti)
{
    struct ringbox_c *ringbox_ctx = (struct ringbox_c *)ti->private;

    if (ringbox_ctx == NULL) {
        pr_warn("%s: NULL context in destructor\n", RINGBOX_NAME);
        return;
    }

    pr_info("%s: Destroying device\n"
            "  - Total Write Ops: %llu\n"
            "  - Total Read Ops: %llu\n"
            "  - Total Sectors: %llu\n",
            RINGBOX_NAME,
            (unsigned long long)atomic64_read(&ringbox_ctx->total_write_ops),
            (unsigned long long)atomic64_read(&ringbox_ctx->total_read_ops),
            (unsigned long long)atomic64_read(&ringbox_ctx->total_sectors));

    /* 标记设备为非活动状态 */
    ringbox_ctx->is_active = false;
    smp_wmb();  /* 写内存屏障，确保状态更新可见 */

    /* 释放底层设备引用 */
    if (ringbox_ctx->target_dev != NULL) {
        dm_put_device(ti, ringbox_ctx->target_dev);
        ringbox_ctx->target_dev = NULL;
    }

    /* 释放私有数据结构内存 */
    kfree(ringbox_ctx);
    ti->private = NULL;

    pr_info("%s: Device destroyed successfully\n", RINGBOX_NAME);
}

/*============================================================================
 * 核心映射函数实现
 *============================================================================*/

/**
 * @brief  核心映射函数 (.map)
 * @details 拦截并篡改 struct bio，实现环形缓冲区逻辑。
 *
 *          【零拷贝设计原则】
 *          本函数仅修改 bio 结构中的元数据（目标设备和扇区地址），
 *          完全不触碰 bio->bi_io_vec 中的实际数据缓冲区。
 *          真实数据传输由底层 NVMe 驱动通过 DMA 直接完成，
 *          CPU 不参与数据搬运，实现真正的零拷贝。
 *
 *          【事件驱动模型】
 *          本函数仅在内核块层调度 I/O 时被调用，无后台线程轮询。
 *          空转时不消耗任何 CPU 资源。
 *
 * @param  ti  Device Mapper 目标结构体指针
 * @param  bio 描述一次块 I/O 操作的核心结构体
 *
 * @return DM_MAPIO_REMAPPED 已修改地址并交由底层处理
 * @return DM_MAPIO_KILL     请求无效，终止 I/O
 *
 * @note 该函数运行在软中断上下文，不能睡眠
 */
int ringbox_map(struct dm_target *ti, struct bio *bio)
{
    struct ringbox_c *ringbox_ctx = (struct ringbox_c *)ti->private;
    sector_t logic_sector;
    sector_t mapped_sector;
    sector_t ring_offset;
    unsigned int nr_sectors;

    /*------------------------------------------------------------
     * 安全检查
     *------------------------------------------------------------*/
    if (ringbox_ctx == NULL || !ringbox_ctx->is_active) {
        return DM_MAPIO_KILL;
    }

    /*------------------------------------------------------------
     * 1. 将目标设备重定向到底层 NVMe 设备
     *    【零拷贝】仅修改设备指针，数据页不拷贝
     *------------------------------------------------------------*/
    bio_set_dev(bio, ringbox_ctx->target_dev->bdev);

    /*------------------------------------------------------------
     * 2. 计算逻辑扇区偏移
     *    dm_target_offset 返回当前 bio 相对于 dm 设备起始的偏移
     *------------------------------------------------------------*/
    logic_sector = dm_target_offset(ti, bio->bi_iter.bi_sector);
    nr_sectors = bio_sectors(bio);

    /*------------------------------------------------------------
     * 3. 环形缓冲区地址映射
     *    将逻辑地址映射到物理环形缓冲区地址
     *
     *    【环形算法】
     *    ring_offset = logic_sector % ring_capacity
     *    mapped_sector = physical_start + ring_offset
     *------------------------------------------------------------*/
    ring_offset = do_div(logic_sector, ringbox_ctx->ring_capacity);
    mapped_sector = ringbox_ctx->physical_start + ring_offset;

    /*------------------------------------------------------------
     * 4. 篡改目的地物理扇区地址
     *    【零拷贝】仅修改扇区号，数据页不拷贝
     *------------------------------------------------------------*/
    bio->bi_iter.bi_sector = mapped_sector;

    /*------------------------------------------------------------
     * 5. 更新统计信息 (原子操作，无锁)
     *------------------------------------------------------------*/
    if (bio_data_dir(bio) == WRITE) {
        atomic64_inc(&ringbox_ctx->total_write_ops);
        atomic64_add(nr_sectors, &ringbox_ctx->total_sectors);

        /* 更新当前写入位置（用于状态查询） */
        atomic64_set(&ringbox_ctx->current_write_pos,
                     (ring_offset + nr_sectors) % ringbox_ctx->ring_capacity);
    } else {
        atomic64_inc(&ringbox_ctx->total_read_ops);
    }

    /*------------------------------------------------------------
     * 6. 返回映射完成状态
     *    DM_MAPIO_REMAPPED 告诉内核：地址已修改，继续向下提交
     *------------------------------------------------------------*/
    return DM_MAPIO_REMAPPED;
}

/*============================================================================
 * ioctl 接口实现
 *============================================================================*/

/**
 * @brief  ioctl 控制接口
 * @details 提供用户空间与驱动的交互接口，支持状态查询和控制操作。
 *
 * @param  ti  Device Mapper 目标结构体指针
 * @param  cmd ioctl 命令字
 * @param  arg 用户空间传入的参数
 *
 * @return 0 成功
 * @return -ENOTTY 不支持的命令
 * @return -EFAULT 数据拷贝失败
 * @return -EPERM 设备未激活
 */
int ringbox_ioctl(struct dm_target *ti, unsigned int cmd, unsigned long arg)
{
    struct ringbox_c *ringbox_ctx = (struct ringbox_c *)ti->private;
    struct ringbox_status status;
    struct ringbox_version version;
    struct ringbox_capacity capacity;
    void __user *user_ptr = (void __user *)arg;
    sector_t temp_sector;

    /* 设备状态检查 */
    if (ringbox_ctx == NULL || !ringbox_ctx->is_active) {
        return -EPERM;
    }

    switch (cmd) {
    case RINGBOX_IOC_GET_STATUS:
        /* 获取当前状态信息 */
        status.physical_start = (uint64_t)ringbox_ctx->physical_start;
        status.ring_capacity = (uint64_t)ringbox_ctx->ring_capacity;
        status.current_write_pos = (uint64_t)atomic64_read(&ringbox_ctx->current_write_pos);
        status.total_write_ops = (uint64_t)atomic64_read(&ringbox_ctx->total_write_ops);
        status.total_read_ops = (uint64_t)atomic64_read(&ringbox_ctx->total_read_ops);
        status.total_sectors_written = (uint64_t)atomic64_read(&ringbox_ctx->total_sectors);
        status.is_active = ringbox_ctx->is_active ? 1 : 0;
        status.reserved = 0;

        if (copy_to_user(user_ptr, &status, sizeof(status)) != 0) {
            return -EFAULT;
        }
        break;

    case RINGBOX_IOC_RESET_WRITE:
        /* 重置写入指针和统计信息 */
        atomic64_set(&ringbox_ctx->current_write_pos, 0);
        atomic64_set(&ringbox_ctx->total_write_ops, 0);
        atomic64_set(&ringbox_ctx->total_read_ops, 0);
        atomic64_set(&ringbox_ctx->total_sectors, 0);
        pr_info("%s: Statistics reset by ioctl\n", RINGBOX_NAME);
        break;

    case RINGBOX_IOC_SET_START:
        /* 设置新的物理起始地址（需要持有锁） */
        if (copy_from_user(&temp_sector, user_ptr, sizeof(uint64_t)) != 0) {
            return -EFAULT;
        }

        spin_lock(&ringbox_ctx->config_lock);
        ringbox_ctx->physical_start = (sector_t)temp_sector;
        spin_unlock(&ringbox_ctx->config_lock);

        pr_info("%s: Physical start updated to %llu\n",
                RINGBOX_NAME, (unsigned long long)temp_sector);
        break;

    case RINGBOX_IOC_GET_VERSION:
        /* 获取驱动版本信息 */
        version.major = RINGBOX_VERSION_MAJOR;
        version.minor = RINGBOX_VERSION_MINOR;
        version.patch = RINGBOX_VERSION_PATCH;
        snprintf(version.version_str, sizeof(version.version_str), "%s", RINGBOX_VERSION_STRING);

        if (copy_to_user(user_ptr, &version, sizeof(version)) != 0) {
            return -EFAULT;
        }
        break;

    case RINGBOX_IOC_GET_CAPACITY:
        /* 获取设备容量信息 */
        capacity.physical_start = (uint64_t)ringbox_ctx->physical_start;
        capacity.ring_capacity = (uint64_t)ringbox_ctx->ring_capacity;
        capacity.ring_size_bytes = (uint64_t)ringbox_ctx->ring_capacity * 512;  /* 扇区转字节 */
        capacity.ring_size_mb = capacity.ring_size_bytes >> 20;  /* 字节转MB */

        if (copy_to_user(user_ptr, &capacity, sizeof(capacity)) != 0) {
            return -EFAULT;
        }
        break;

    default:
        pr_warn("%s: Unknown ioctl command: 0x%08x\n", RINGBOX_NAME, cmd);
        return -ENOTTY;
    }

    return 0;
}

/*============================================================================
 * 状态输出函数实现
 *============================================================================*/

/**
 * @brief  状态输出函数
 * @details 响应 dmsetup status 命令，输出当前设备状态。
 *
 * @param  ti            Device Mapper 目标结构体指针
 * @param  type          状态类型 (STATUSTYPE_INFO / STATUSTYPE_TABLE)
 * @param  status_flags  状态标志
 * @param  result        输出缓冲区
 * @param  maxlen        缓冲区最大长度
 */
void ringbox_status(struct dm_target *ti, status_type_t type,
                    unsigned int status_flags, char *result, unsigned int maxlen)
{
    struct ringbox_c *ringbox_ctx = (struct ringbox_c *)ti->private;

    if (ringbox_ctx == NULL) {
        scnprintf(result, maxlen, "Error: NULL context");
        return;
    }

    switch (type) {
    case STATUSTYPE_INFO:
        /* 输出运行时状态信息 */
        scnprintf(result, maxlen,
                  "device:%s start:%llu capacity:%llu "
                  "write_pos:%llu write_ops:%llu read_ops:%llu sectors:%llu active:%d",
                  ringbox_ctx->target_dev->name,
                  (unsigned long long)ringbox_ctx->physical_start,
                  (unsigned long long)ringbox_ctx->ring_capacity,
                  (unsigned long long)atomic64_read(&ringbox_ctx->current_write_pos),
                  (unsigned long long)atomic64_read(&ringbox_ctx->total_write_ops),
                  (unsigned long long)atomic64_read(&ringbox_ctx->total_read_ops),
                  (unsigned long long)atomic64_read(&ringbox_ctx->total_sectors),
                  ringbox_ctx->is_active ? 1 : 0);
        break;

    case STATUSTYPE_TABLE:
        /* 输出设备表配置 */
        scnprintf(result, maxlen, "%s %llu %llu",
                  ringbox_ctx->target_dev->name,
                  (unsigned long long)ringbox_ctx->physical_start,
                  (unsigned long long)ringbox_ctx->ring_capacity);
        break;

    default:
        result[0] = '\0';
        break;
    }
}

/*============================================================================
 * 模块初始化与卸载
 *============================================================================*/

/**
 * @brief  模块初始化入口
 * @details 注册 "ringbox" 目标到内核 Device Mapper 框架。
 * @return 0 成功
 * @return 负值 注册失败
 */
static int __init dm_ringbox_init(void)
{
    int ret;

    pr_info("%s: Initializing module v%d.%d.%d\n",
            RINGBOX_NAME, RINGBOX_VERSION_MAJOR,
            RINGBOX_VERSION_MINOR, RINGBOX_VERSION_PATCH);

    ret = dm_register_target(&ringbox_target);
    if (ret < 0) {
        pr_err("%s: Failed to register target (error: %d)\n", RINGBOX_NAME, ret);
        return ret;
    }

    pr_info("%s: Module loaded successfully\n", RINGBOX_NAME);
    return 0;
}

/**
 * @brief  模块卸载入口
 * @details 从内核 Device Mapper 框架注销 "ringbox" 目标。
 */
static void __exit dm_ringbox_exit(void)
{
    dm_unregister_target(&ringbox_target);
    pr_info("%s: Module unloaded\n", RINGBOX_NAME);
}

/*============================================================================
 * 模块注册
 *============================================================================*/

module_init(dm_ringbox_init);
module_exit(dm_ringbox_exit);

MODULE_AUTHOR("RK3568 Development Team");
MODULE_DESCRIPTION("RingBox Device Mapper Target for High-Performance NVMe SSD Logging");
MODULE_LICENSE("GPL");
MODULE_VERSION(RINGBOX_VERSION_STRING);
