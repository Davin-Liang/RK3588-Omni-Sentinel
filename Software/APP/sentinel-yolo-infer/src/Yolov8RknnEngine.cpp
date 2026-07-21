// Adapted from Rockchip official RKNN YOLOv8 demo zero-copy implementation.
// Original demo files are kept under official_yolov8_reference/ for comparison.

#include "Yolov8RknnEngine.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <linux/dma-buf.h>
#include <numeric>
#include <set>
#include <sys/ioctl.h>
#include <unistd.h>

namespace {

constexpr int OBJ_CLASS_NUM = 80;
constexpr int OBJ_NUMB_MAX_SIZE = 128;

static inline int clamp_int(float val, int minVal, int maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return static_cast<int>(val);
}

static bool read_file(const std::string& path, std::vector<uint8_t>& data) {
    std::ifstream ifs(path, std::ios::binary | std::ios::ate);
    if (!ifs) {
        std::cerr << "[Yolov8RknnEngine] open model failed: " << path << std::endl;
        return false;
    }
    std::streamsize size = ifs.tellg();
    if (size <= 0) {
        std::cerr << "[Yolov8RknnEngine] empty model file: " << path << std::endl;
        return false;
    }
    ifs.seekg(0, std::ios::beg);
    data.resize(static_cast<size_t>(size));
    if (!ifs.read(reinterpret_cast<char*>(data.data()), size)) {
        std::cerr << "[Yolov8RknnEngine] read model failed: " << path << std::endl;
        return false;
    }
    return true;
}

static void sync_dma_buf(int fd, uint64_t flags) {
    if (fd < 0) return;
    struct dma_buf_sync sync = {};
    sync.flags = flags;
    if (ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync) < 0) {
        // 不是所有内核/驱动都要求显式 sync，失败不直接中断推理。
        std::cerr << "[Yolov8RknnEngine] DMA_BUF_IOCTL_SYNC failed: " << strerror(errno) << std::endl;
    }
}

static float calculate_overlap(float xmin0, float ymin0, float xmax0, float ymax0,
                               float xmin1, float ymin1, float xmax1, float ymax1) {
    float w = std::max(0.0f, std::min(xmax0, xmax1) - std::max(xmin0, xmin1) + 1.0f);
    float h = std::max(0.0f, std::min(ymax0, ymax1) - std::max(ymin0, ymin1) + 1.0f);
    float inter = w * h;
    float area0 = (xmax0 - xmin0 + 1.0f) * (ymax0 - ymin0 + 1.0f);
    float area1 = (xmax1 - xmin1 + 1.0f) * (ymax1 - ymin1 + 1.0f);
    float uni = area0 + area1 - inter;
    return uni <= 0.0f ? 0.0f : inter / uni;
}

static int nms(int validCount, const std::vector<float>& outputLocations,
               const std::vector<int>& classIds, std::vector<int>& order,
               int filterId, float threshold) {
    for (int i = 0; i < validCount; ++i) {
        int n = order[i];
        if (n == -1 || classIds[n] != filterId) continue;
        for (int j = i + 1; j < validCount; ++j) {
            int m = order[j];
            if (m == -1 || classIds[m] != filterId) continue;

            float xmin0 = outputLocations[n * 4 + 0];
            float ymin0 = outputLocations[n * 4 + 1];
            float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
            float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];

            float xmin1 = outputLocations[m * 4 + 0];
            float ymin1 = outputLocations[m * 4 + 1];
            float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
            float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];

            if (calculate_overlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1) > threshold) {
                order[j] = -1;
            }
        }
    }
    return 0;
}

static int quick_sort_indice_inverse(std::vector<float>& input, int left, int right, std::vector<int>& indices) {
    float key;
    int keyIndex;
    int low = left;
    int high = right;
    if (left < right) {
        keyIndex = indices[left];
        key = input[left];
        while (low < high) {
            while (low < high && input[high] <= key) high--;
            input[low] = input[high];
            indices[low] = indices[high];
            while (low < high && input[low] >= key) low++;
            input[high] = input[low];
            indices[high] = indices[low];
        }
        input[low] = key;
        indices[low] = keyIndex;
        quick_sort_indice_inverse(input, left, low - 1, indices);
        quick_sort_indice_inverse(input, low + 1, right, indices);
    }
    return low;
}

static inline int32_t clip_f32(float val, float minVal, float maxVal) {
    float f = val <= minVal ? minVal : (val >= maxVal ? maxVal : val);
    return static_cast<int32_t>(f);
}

static int8_t qnt_f32_to_affine(float f32, int32_t zp, float scale) {
    float dst = (f32 / scale) + zp;
    return static_cast<int8_t>(clip_f32(dst, -128.0f, 127.0f));
}

static float deqnt_affine_to_f32(int8_t qnt, int32_t zp, float scale) {
    return (static_cast<float>(qnt) - static_cast<float>(zp)) * scale;
}

static void compute_dfl(float* tensor, int dflLen, float* box) {
    for (int b = 0; b < 4; b++) {
        std::vector<float> expTensor(dflLen);
        float expSum = 0.0f;
        float accSum = 0.0f;
        for (int i = 0; i < dflLen; i++) {
            expTensor[i] = std::exp(tensor[i + b * dflLen]);
            expSum += expTensor[i];
        }
        for (int i = 0; i < dflLen; i++) {
            accSum += expTensor[i] / expSum * i;
        }
        box[b] = accSum;
    }
}

static int process_i8(int8_t* boxTensor, int32_t boxZp, float boxScale,
                      int8_t* scoreTensor, int32_t scoreZp, float scoreScale,
                      int8_t* scoreSumTensor, int32_t scoreSumZp, float scoreSumScale,
                      int gridH, int gridW, int stride, int dflLen,
                      std::vector<float>& boxes,
                      std::vector<float>& objProbs,
                      std::vector<int>& classId,
                      float threshold) {
    int validCount = 0;
    int gridLen = gridH * gridW;
    int8_t scoreThreshold = qnt_f32_to_affine(threshold, scoreZp, scoreScale);
    int8_t scoreSumThreshold = qnt_f32_to_affine(threshold, scoreSumZp, scoreSumScale);

    for (int i = 0; i < gridH; i++) {
        for (int j = 0; j < gridW; j++) {
            int offset = i * gridW + j;
            int maxClassId = -1;

            if (scoreSumTensor != nullptr && scoreSumTensor[offset] < scoreSumThreshold) {
                continue;
            }

            int8_t maxScore = static_cast<int8_t>(-scoreZp);
            for (int c = 0; c < OBJ_CLASS_NUM; c++) {
                if ((scoreTensor[offset] > scoreThreshold) && (scoreTensor[offset] > maxScore)) {
                    maxScore = scoreTensor[offset];
                    maxClassId = c;
                }
                offset += gridLen;
            }

            if (maxScore > scoreThreshold) {
                offset = i * gridW + j;
                float box[4];
                std::vector<float> beforeDfl(dflLen * 4);
                for (int k = 0; k < dflLen * 4; k++) {
                    beforeDfl[k] = deqnt_affine_to_f32(boxTensor[offset], boxZp, boxScale);
                    offset += gridLen;
                }
                compute_dfl(beforeDfl.data(), dflLen, box);

                float x1 = (-box[0] + j + 0.5f) * stride;
                float y1 = (-box[1] + i + 0.5f) * stride;
                float x2 = ( box[2] + j + 0.5f) * stride;
                float y2 = ( box[3] + i + 0.5f) * stride;
                boxes.push_back(x1);
                boxes.push_back(y1);
                boxes.push_back(x2 - x1);
                boxes.push_back(y2 - y1);
                objProbs.push_back(deqnt_affine_to_f32(maxScore, scoreZp, scoreScale));
                classId.push_back(maxClassId);
                validCount++;
            }
        }
    }
    return validCount;
}

static int NC1HWC2_i8_to_NCHW_i8(const int8_t* src, int8_t* dst, int* dims,
                                 int channel, int h, int w, int /*zp*/, float /*scale*/) {
    int batch = dims[0];
    int C1 = dims[1];
    int C2 = dims[4];
    int hwSrc = dims[2] * dims[3];
    int hwDst = h * w;
    for (int b = 0; b < batch; b++) {
        const int8_t* srcB = src + b * C1 * hwSrc * C2;
        int8_t* dstB = dst + b * channel * hwDst;
        for (int c = 0; c < channel; ++c) {
            int plane = c / C2;
            const int8_t* srcBC = plane * hwSrc * C2 + srcB;
            int offset = c % C2;
            for (int curH = 0; curH < h; ++curH) {
                for (int curW = 0; curW < w; ++curW) {
                    int curHW = curH * w + curW;
                    dstB[c * hwDst + curHW] = srcBC[C2 * curHW + offset];
                }
            }
        }
    }
    return 0;
}

} // namespace

Yolov8RknnEngine::Yolov8RknnEngine() = default;

Yolov8RknnEngine::~Yolov8RknnEngine() {
    release();
}

bool Yolov8RknnEngine::init(const std::string& modelPath, float boxThreshold, float nmsThreshold,
                            int npuCoreMask) {
    release();
    boxThreshold_ = boxThreshold;
    nmsThreshold_ = nmsThreshold;
    npuCoreMask_  = npuCoreMask;

    std::vector<uint8_t> model;
    if (!read_file(modelPath, model)) return false;

    int ret = rknn_init(&rknnCtx_, model.data(), static_cast<uint32_t>(model.size()), 0, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "[Yolov8RknnEngine] rknn_init failed, ret=" << ret << std::endl;
        rknnCtx_ = 0;
        return false;
    }

    // 绑定 NPU 核心：YOLO 独占 Core 2，Core 0/1 留给 DeepSeek LLM
    ret = rknn_set_core_mask(rknnCtx_, static_cast<rknn_core_mask>(npuCoreMask_));
    if (ret != RKNN_SUCC) {
        std::cerr << "[Yolov8RknnEngine] rknn_set_core_mask failed, ret=" << ret
                  << " (mask=" << npuCoreMask_ << "), continuing anyway" << std::endl;
        // 不致命，继续使用驱动默认分配
    } else {
        std::cout << "[Yolov8RknnEngine] NPU core mask set to " << npuCoreMask_ << std::endl;
    }

    if (!queryModelInfo_()) {
        release();
        return false;
    }

    if (!createOutputMems_()) {
        release();
        return false;
    }

    return true;
}

void Yolov8RknnEngine::release() {
    destroyOutputMems_();

    if (rknnCtx_ != 0) {
        rknn_destroy(rknnCtx_);
        rknnCtx_ = 0;
    }

    inputAttrs_.clear();
    outputAttrs_.clear();
    inputNativeAttrs_.clear();
    outputNativeAttrs_.clear();
    modelWidth_ = 0;
    modelHeight_ = 0;
    modelChannel_ = 0;
    isQuant_ = false;
}

bool Yolov8RknnEngine::queryModelInfo_() {
    int ret = rknn_query(rknnCtx_, RKNN_QUERY_IN_OUT_NUM, &ioNum_, sizeof(ioNum_));
    if (ret != RKNN_SUCC) {
        std::cerr << "[Yolov8RknnEngine] RKNN_QUERY_IN_OUT_NUM failed, ret=" << ret << std::endl;
        return false;
    }

    inputAttrs_.assign(ioNum_.n_input, rknn_tensor_attr{});
    outputAttrs_.assign(ioNum_.n_output, rknn_tensor_attr{});
    inputNativeAttrs_.assign(ioNum_.n_input, rknn_tensor_attr{});
    outputNativeAttrs_.assign(ioNum_.n_output, rknn_tensor_attr{});

    for (uint32_t i = 0; i < ioNum_.n_input; ++i) {
        inputAttrs_[i].index = i;
        ret = rknn_query(rknnCtx_, RKNN_QUERY_INPUT_ATTR, &inputAttrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "[Yolov8RknnEngine] RKNN_QUERY_INPUT_ATTR failed, ret=" << ret << std::endl;
            return false;
        }

        inputNativeAttrs_[i].index = i;
        ret = rknn_query(rknnCtx_, RKNN_QUERY_NATIVE_INPUT_ATTR, &inputNativeAttrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "[Yolov8RknnEngine] RKNN_QUERY_NATIVE_INPUT_ATTR failed, ret=" << ret << std::endl;
            return false;
        }
    }

    for (uint32_t i = 0; i < ioNum_.n_output; ++i) {
        outputAttrs_[i].index = i;
        ret = rknn_query(rknnCtx_, RKNN_QUERY_OUTPUT_ATTR, &outputAttrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "[Yolov8RknnEngine] RKNN_QUERY_OUTPUT_ATTR failed, ret=" << ret << std::endl;
            return false;
        }

        outputNativeAttrs_[i].index = i;
        ret = rknn_query(rknnCtx_, RKNN_QUERY_NATIVE_OUTPUT_ATTR, &outputNativeAttrs_[i], sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC) {
            std::cerr << "[Yolov8RknnEngine] RKNN_QUERY_NATIVE_OUTPUT_ATTR failed, ret=" << ret << std::endl;
            return false;
        }
    }

    if (inputAttrs_.empty()) {
        std::cerr << "[Yolov8RknnEngine] no input tensor" << std::endl;
        return false;
    }

    if (inputAttrs_[0].fmt == RKNN_TENSOR_NCHW) {
        modelChannel_ = inputAttrs_[0].dims[1];
        modelHeight_ = inputAttrs_[0].dims[2];
        modelWidth_ = inputAttrs_[0].dims[3];
    } else {
        modelHeight_ = inputAttrs_[0].dims[1];
        modelWidth_ = inputAttrs_[0].dims[2];
        modelChannel_ = inputAttrs_[0].dims[3];
    }

    isQuant_ = (!outputNativeAttrs_.empty() &&
                outputNativeAttrs_[0].qnt_type == RKNN_TENSOR_QNT_AFFINE_ASYMMETRIC &&
                outputNativeAttrs_[0].type == RKNN_TENSOR_INT8);

    std::cout << "[Yolov8RknnEngine] model input: " << modelWidth_ << "x" << modelHeight_
              << "x" << modelChannel_ << ", outputs=" << ioNum_.n_output
              << ", quant=" << (isQuant_ ? "true" : "false") << std::endl;

    return true;
}

bool Yolov8RknnEngine::createOutputMems_() {
    outputMems_.assign(ioNum_.n_output, nullptr);
    for (uint32_t i = 0; i < ioNum_.n_output; ++i) {
        outputMems_[i] = rknn_create_mem(rknnCtx_, outputNativeAttrs_[i].size_with_stride);
        if (!outputMems_[i]) {
            std::cerr << "[Yolov8RknnEngine] rknn_create_mem output failed, index=" << i << std::endl;
            return false;
        }
        int ret = rknn_set_io_mem(rknnCtx_, outputMems_[i], &outputNativeAttrs_[i]);
        if (ret != RKNN_SUCC) {
            std::cerr << "[Yolov8RknnEngine] rknn_set_io_mem output failed, index=" << i << ", ret=" << ret << std::endl;
            return false;
        }
    }
    return true;
}

void Yolov8RknnEngine::destroyOutputMems_() {
    if (rknnCtx_ == 0) {
        outputMems_.clear();
        return;
    }
    for (auto* mem : outputMems_) {
        if (mem) {
            rknn_destroy_mem(rknnCtx_, mem);
        }
    }
    outputMems_.clear();
}

bool Yolov8RknnEngine::inferFromDmaBuffer(int dmaFd,
                                          void* virtAddr,
                                          int bufferSize,
                                          int width,
                                          int height,
                                          uint64_t timestampNs,
                                          std::vector<YoloBBox>& out) {
    out.clear();
    if (rknnCtx_ == 0 || dmaFd < 0 || bufferSize <= 0) {
        return false;
    }
    if (width != modelWidth_ || height != modelHeight_ || modelChannel_ != 3) {
        std::cerr << "[Yolov8RknnEngine] NPU buffer size mismatch. got "
                  << width << "x" << height << ", model "
                  << modelWidth_ << "x" << modelHeight_ << std::endl;
        return false;
    }

    const int inputBytes = modelWidth_ * modelHeight_ * modelChannel_;
    const int requiredInputBytes = inputNativeAttrs_[0].size_with_stride > 0
                                       ? static_cast<int>(inputNativeAttrs_[0].size_with_stride)
                                       : inputBytes;
    if (bufferSize < requiredInputBytes) {
        std::cerr << "[Yolov8RknnEngine] dma buffer too small. bufferSize=" << bufferSize
                  << ", need=" << requiredInputBytes << std::endl;
        return false;
    }

    // 与官方 zero-copy demo 保持一致：使用 native input attr，仅将输入类型改为 UINT8。
    // 这样归一化/量化可由 RKNN runtime 融合到 NPU 侧执行。
    rknn_tensor_attr inputAttr = inputNativeAttrs_[0];
    inputAttr.index = 0;
    inputAttr.type = RKNN_TENSOR_UINT8;

    sync_dma_buf(dmaFd, DMA_BUF_SYNC_START | DMA_BUF_SYNC_RW);

    // RK3588/RKNPU2 zero-copy input: import SentinelVisioner NPU small image DMA-BUF directly.
    // Depending on RKNN runtime version, rknn_set_io_mem may internally keep only the fd/addr metadata,
    // so the imported mem must stay alive until rknn_run returns.
    rknn_tensor_mem* inputMem = rknn_create_mem_from_fd(rknnCtx_, dmaFd, virtAddr,
                                                       static_cast<uint32_t>(bufferSize), 0);
    if (!inputMem) {
        std::cerr << "[Yolov8RknnEngine] rknn_create_mem_from_fd failed" << std::endl;
        sync_dma_buf(dmaFd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
        return false;
    }

    int ret = rknn_set_io_mem(rknnCtx_, inputMem, &inputAttr);
    if (ret != RKNN_SUCC) {
        std::cerr << "[Yolov8RknnEngine] rknn_set_io_mem input failed, ret=" << ret << std::endl;
        rknn_destroy_mem(rknnCtx_, inputMem);
        sync_dma_buf(dmaFd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
        return false;
    }

    ret = rknn_run(rknnCtx_, nullptr);
    if (ret != RKNN_SUCC) {
        std::cerr << "[Yolov8RknnEngine] rknn_run failed, ret=" << ret << std::endl;
        rknn_destroy_mem(rknnCtx_, inputMem);
        sync_dma_buf(dmaFd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
        return false;
    }

    std::vector<rknn_output> outputs;
    bool ok = collectOutputs_(outputs);
    if (ok) {
        postProcess_(outputs, timestampNs, out);
        freeCollectedOutputs_(outputs);
    }

    rknn_destroy_mem(rknnCtx_, inputMem);
    sync_dma_buf(dmaFd, DMA_BUF_SYNC_END | DMA_BUF_SYNC_RW);
    return ok;
}

bool Yolov8RknnEngine::collectOutputs_(std::vector<rknn_output>& outputs) {
    outputs.assign(ioNum_.n_output, rknn_output{});

    if (!isQuant_) {
        std::cerr << "[Yolov8RknnEngine] zero-copy path currently supports int8 quantized YOLOv8 model only" << std::endl;
        return false;
    }

    for (uint32_t i = 0; i < ioNum_.n_output; ++i) {
        int channel = outputAttrs_[i].dims[1];
        int h = outputAttrs_[i].n_dims > 2 ? outputAttrs_[i].dims[2] : 1;
        int w = outputAttrs_[i].n_dims > 3 ? outputAttrs_[i].dims[3] : 1;

        outputs[i].index = i;
        outputs[i].want_float = 0;
        outputs[i].size = outputNativeAttrs_[i].n_elems * sizeof(int8_t);
        outputs[i].buf = std::malloc(outputs[i].size);
        if (!outputs[i].buf) {
            std::cerr << "[Yolov8RknnEngine] malloc output failed, index=" << i << std::endl;
            freeCollectedOutputs_(outputs);
            return false;
        }

        if (outputNativeAttrs_[i].fmt == RKNN_TENSOR_NC1HWC2) {
            NC1HWC2_i8_to_NCHW_i8(static_cast<int8_t*>(outputMems_[i]->virt_addr),
                                  static_cast<int8_t*>(outputs[i].buf),
                                  reinterpret_cast<int*>(outputNativeAttrs_[i].dims),
                                  channel, h, w,
                                  outputNativeAttrs_[i].zp,
                                  outputNativeAttrs_[i].scale);
        } else {
            std::memcpy(outputs[i].buf, outputMems_[i]->virt_addr, outputs[i].size);
        }
    }
    return true;
}

void Yolov8RknnEngine::freeCollectedOutputs_(std::vector<rknn_output>& outputs) {
    for (auto& out : outputs) {
        if (out.buf) {
            std::free(out.buf);
            out.buf = nullptr;
        }
    }
}

void Yolov8RknnEngine::postProcess_(const std::vector<rknn_output>& outputs,
                                    uint64_t timestampNs,
                                    std::vector<YoloBBox>& out) {
    out.clear();
    if (outputs.empty() || outputAttrs_.empty()) return;

    std::vector<float> filterBoxes;
    std::vector<float> objProbs;
    std::vector<int> classIds;
    int validCount = 0;

    // RKNPU2 YOLOv8 默认为 3 个 branch，每个 branch 有 box/score/(optional score_sum)。
    int dflLen = outputAttrs_[0].dims[1] / 4;
    int outputPerBranch = ioNum_.n_output / 3;
    if (outputPerBranch < 2 || dflLen <= 0) {
        std::cerr << "[Yolov8RknnEngine] unsupported YOLOv8 output layout, n_output=" << ioNum_.n_output << std::endl;
        return;
    }

    for (int i = 0; i < 3; i++) {
        void* scoreSum = nullptr;
        int32_t scoreSumZp = 0;
        float scoreSumScale = 1.0f;
        if (outputPerBranch == 3) {
            scoreSum = outputs[i * outputPerBranch + 2].buf;
            scoreSumZp = outputAttrs_[i * outputPerBranch + 2].zp;
            scoreSumScale = outputAttrs_[i * outputPerBranch + 2].scale;
        }

        int boxIdx = i * outputPerBranch;
        int scoreIdx = i * outputPerBranch + 1;
        int gridH = outputAttrs_[boxIdx].dims[2];
        int gridW = outputAttrs_[boxIdx].dims[3];
        int stride = modelHeight_ / gridH;

        validCount += process_i8(static_cast<int8_t*>(outputs[boxIdx].buf),
                                 outputAttrs_[boxIdx].zp,
                                 outputAttrs_[boxIdx].scale,
                                 static_cast<int8_t*>(outputs[scoreIdx].buf),
                                 outputAttrs_[scoreIdx].zp,
                                 outputAttrs_[scoreIdx].scale,
                                 static_cast<int8_t*>(scoreSum),
                                 scoreSumZp,
                                 scoreSumScale,
                                 gridH,
                                 gridW,
                                 stride,
                                 dflLen,
                                 filterBoxes,
                                 objProbs,
                                 classIds,
                                 boxThreshold_);
    }

    if (validCount <= 0) return;

    std::vector<int> indexArray(validCount);
    std::iota(indexArray.begin(), indexArray.end(), 0);
    quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

    std::set<int> classSet(classIds.begin(), classIds.end());
    for (int c : classSet) {
        nms(validCount, filterBoxes, classIds, indexArray, c, nmsThreshold_);
    }

    out.reserve(std::min(validCount, OBJ_NUMB_MAX_SIZE));
    int count = 0;
    for (int i = 0; i < validCount && count < OBJ_NUMB_MAX_SIZE; ++i) {
        int n = indexArray[i];
        if (n == -1) continue;

        float x1f = filterBoxes[n * 4 + 0];
        float y1f = filterBoxes[n * 4 + 1];
        float x2f = x1f + filterBoxes[n * 4 + 2];
        float y2f = y1f + filterBoxes[n * 4 + 3];

        int x1 = clamp_int(std::floor(x1f), 0, modelWidth_);
        int y1 = clamp_int(std::floor(y1f), 0, modelHeight_);
        int x2 = clamp_int(std::ceil(x2f), 0, modelWidth_);
        int y2 = clamp_int(std::ceil(y2f), 0, modelHeight_);

        if (x2 <= x1 || y2 <= y1) continue;

        YoloBBox bbox{};
        bbox.x1 = static_cast<uint32_t>(x1);
        bbox.y1 = static_cast<uint32_t>(y1);
        bbox.x2 = static_cast<uint32_t>(x2);
        bbox.y2 = static_cast<uint32_t>(y2);
        bbox.classId = static_cast<uint32_t>(classIds[n]);
        bbox.confidence = objProbs[i];
        bbox.timestampNs = timestampNs;
        out.push_back(bbox);
        count++;
    }
}
