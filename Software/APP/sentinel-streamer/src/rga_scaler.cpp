/**
 * @brief RGA 硬件缩放：1080p NV12 → 720p NV12
 *
 * 封装瑞芯微 RGA im2d API，将 1920×1080 NV12 DMA-BUF 缩放为 1280×720 NV12 DMA-BUF。
 * 完全参照 sentinel-visioner 的 rga_scale_nv12_to_nv12_() 实现。
 */

#include <cstdio>
#include <cstring>

#include "im2d.h"
#include "drmrga.h"

bool rga_scale_nv12_1080p_to_720p(int srcFd, int dstFd)
{
    if (srcFd <= 0 || dstFd <= 0) {
        fprintf(stderr, "[RgaScaler] invalid dmaFd\n");
        return false;
    }

    int fmt = RK_FORMAT_YCrCb_420_SP;

    // 导入源 DMA-BUF
    im_handle_param_t srcParam = {1920, 1080, fmt};
    rga_buffer_handle_t srcHandle = importbuffer_fd(srcFd, &srcParam);
    if (srcHandle <= 0) {
        fprintf(stderr, "[RgaScaler] importbuffer_fd src failed\n");
        return false;
    }

    // 导入目标 DMA-BUF
    im_handle_param_t dstParam = {1280, 720, fmt};
    rga_buffer_handle_t dstHandle = importbuffer_fd(dstFd, &dstParam);
    if (dstHandle <= 0) {
        fprintf(stderr, "[RgaScaler] importbuffer_fd dst failed\n");
        releasebuffer_handle(srcHandle);
        return false;
    }

    // 包装 RGA buffer
    rga_buffer_t srcBuf = wrapbuffer_handle(srcHandle, 1920, 1080, fmt, 1920, 1080);
    rga_buffer_t dstBuf = wrapbuffer_handle(dstHandle, 1280, 720,  fmt, 1280, 720);

    // 全图缩放
    im_rect srect = {0, 0, 1920, 1080};
    im_rect drect = {0, 0, 1280, 720};

    rga_buffer_t pat;   memset(&pat,   0, sizeof(rga_buffer_t));
    im_rect      prect; memset(&prect, 0, sizeof(im_rect));

    IM_STATUS ret = improcess(srcBuf, dstBuf, pat, srect, drect, prect, 0);

    releasebuffer_handle(srcHandle);
    releasebuffer_handle(dstHandle);

    if (ret <= 0) {
        fprintf(stderr, "[RgaScaler] improcess failed\n");
        return false;
    }

    return true;
}
