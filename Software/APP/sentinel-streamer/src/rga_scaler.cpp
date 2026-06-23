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

bool rga_scale_nv12_to_720p(int srcFd, int srcWidth, int srcHeight, int dstFd,
                             int eisOffsetX, int eisOffsetY, bool eisActive,
                             int eisMargin)
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

    IM_STATUS ret;
    if (!eisActive) {
        if (srcWidth == 1280 && srcHeight == 720) {
            ret = imcopy(srcBuf, dstBuf);
        } else {
            im_rect srect = {0, 0, srcWidth, srcHeight};
            im_rect drect = {0, 0, 1280, 720};

            rga_buffer_t pat;   memset(&pat,   0, sizeof(rga_buffer_t));
            im_rect      prect; memset(&prect, 0, sizeof(im_rect));

            ret = improcess(srcBuf, dstBuf, pat, srect, drect, prect, 0);
        }
    } else {
        const int marginX = eisMargin;
        const int marginY = eisMargin * srcHeight / srcWidth;

        int cropX = marginX + eisOffsetX;
        int cropY = marginY + eisOffsetY;
        int cropW = srcWidth  - 2 * marginX;
        int cropH = srcHeight - 2 * marginY;

        cropX &= ~1; cropY &= ~1;
        cropW &= ~1; cropH &= ~1;

        if (cropX < 0) cropX = 0;
        if (cropY < 0) cropY = 0;
        if (cropX + cropW > srcWidth)  cropX = srcWidth  - cropW;
        if (cropY + cropH > srcHeight) cropY = srcHeight - cropH;
        cropX &= ~1; cropY &= ~1;

        im_rect srect = {cropX, cropY, cropW, cropH};
        im_rect drect = {0, 0, 1280, 720};

        rga_buffer_t pat;   memset(&pat,   0, sizeof(rga_buffer_t));
        im_rect      prect; memset(&prect, 0, sizeof(im_rect));

        ret = improcess(srcBuf, dstBuf, pat, srect, drect, prect, IM_SYNC);
    }

    releasebuffer_handle(srcHandle);
    releasebuffer_handle(dstHandle);

    if (ret <= 0) {
        fprintf(stderr, "[RgaScaler] RGA %s failed (ret=%d)\n",
                (!eisActive && srcWidth == 1280 && srcHeight == 720)
                    ? "imcopy" : "improcess", (int)ret);
        return false;
    }

    return true;
}

bool rga_nv12_copy(int srcFd, int srcWidth, int srcHeight, int dstFd)
{
    if (srcFd <= 0 || dstFd <= 0) {
        fprintf(stderr, "[RgaScaler] rga_nv12_copy: invalid dmaFd\n");
        return false;
    }

    int fmt = RK_FORMAT_YCrCb_420_SP;

    im_handle_param_t srcParam = {srcWidth, srcHeight, fmt};
    rga_buffer_handle_t srcHandle = importbuffer_fd(srcFd, &srcParam);
    if (srcHandle <= 0) {
        fprintf(stderr, "[RgaScaler] rga_nv12_copy: import src fd=%d failed\n", srcFd);
        return false;
    }

    im_handle_param_t dstParam = {srcWidth, srcHeight, fmt};
    rga_buffer_handle_t dstHandle = importbuffer_fd(dstFd, &dstParam);
    if (dstHandle <= 0) {
        fprintf(stderr, "[RgaScaler] rga_nv12_copy: import dst fd=%d failed\n", dstFd);
        releasebuffer_handle(srcHandle);
        return false;
    }

    rga_buffer_t srcBuf = wrapbuffer_handle(srcHandle, srcWidth, srcHeight, fmt, srcWidth, srcHeight);
    rga_buffer_t dstBuf = wrapbuffer_handle(dstHandle, srcWidth, srcHeight, fmt, srcWidth, srcHeight);

    IM_STATUS ret = imcopy(srcBuf, dstBuf);

    releasebuffer_handle(srcHandle);
    releasebuffer_handle(dstHandle);

    if (ret != IM_STATUS_SUCCESS) {
        fprintf(stderr, "[RgaScaler] rga_nv12_copy: imcopy failed ret=%d\n", (int)ret);
        return false;
    }

    return true;
}
