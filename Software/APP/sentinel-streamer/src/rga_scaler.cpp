/**
 * @brief RGA 硬件缩放/拷贝：任意分辨率 NV12 → 720p NV12
 *
 * 封装瑞芯微 RGA im2d API，将任意分辨率 NV12 DMA-BUF 缩放为 1280×720 NV12 DMA-BUF。
 * 当源分辨率已是 1280×720 时，使用 imcopy 做 1:1 拷贝。
 */

#include <cstdio>
#include <cstring>

#include "im2d.h"
#include "drmrga.h"

bool rga_scale_nv12_to_720p(int srcFd, int srcWidth, int srcHeight, int dstFd)
{
    if (srcFd <= 0 || dstFd <= 0) {
        fprintf(stderr, "[RgaScaler] invalid dmaFd\n");
        return false;
    }

    int fmt = RK_FORMAT_YCrCb_420_SP;

    im_handle_param_t srcParam = {srcWidth, srcHeight, fmt};
    rga_buffer_handle_t srcHandle = importbuffer_fd(srcFd, &srcParam);
    if (srcHandle <= 0) {
        fprintf(stderr, "[RgaScaler] importbuffer_fd src fd=%d (%dx%d) failed\n",
                srcFd, srcWidth, srcHeight);
        return false;
    }

    im_handle_param_t dstParam = {1280, 720, fmt};
    rga_buffer_handle_t dstHandle = importbuffer_fd(dstFd, &dstParam);
    if (dstHandle <= 0) {
        fprintf(stderr, "[RgaScaler] importbuffer_fd dst failed\n");
        releasebuffer_handle(srcHandle);
        return false;
    }

    rga_buffer_t srcBuf = wrapbuffer_handle(srcHandle, srcWidth, srcHeight, fmt, srcWidth, srcHeight);
    rga_buffer_t dstBuf = wrapbuffer_handle(dstHandle, 1280, 720, fmt, 1280, 720);

    im_rect srect = {0, 0, srcWidth, srcHeight};
    im_rect drect = {0, 0, 1280, 720};

    rga_buffer_t pat;   memset(&pat,   0, sizeof(rga_buffer_t));
    im_rect      prect; memset(&prect, 0, sizeof(im_rect));

    IM_STATUS ret;
    if (srcWidth == 1280 && srcHeight == 720) {
        ret = imcopy(srcBuf, dstBuf);
    } else {
        ret = improcess(srcBuf, dstBuf, pat, srect, drect, prect, 0);
    }

    releasebuffer_handle(srcHandle);
    releasebuffer_handle(dstHandle);

    if (ret <= 0) {
        fprintf(stderr, "[RgaScaler] RGA %s failed (ret=%d)\n",
                (srcWidth == 1280 && srcHeight == 720) ? "imcopy" : "improcess", (int)ret);
        return false;
    }

    return true;
}
