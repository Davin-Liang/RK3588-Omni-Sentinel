#include "nvme_worker.h"
#include "sentinel_streamer.h"
#include "sentinel_lslidarer.h"
#include "NVMeDataManager.h"

#include <QThread>
#include <cstdio>

NvmeWorker::NvmeWorker(SentinelStreamer* streamer,
                       NVMeDataManager* nvme,
                       SentinelLslidarer** lidarPtr,
                       int numCameras,
                       QObject* parent)
    : QObject(parent)
    , streamer_(streamer)
    , nvme_(nvme)
    , lidarPtr_(lidarPtr)
    , numCameras_(numCameras)
    , lastLidarTs_(0)
    , frameSkipCount_{0, 0}
{
}

void NvmeWorker::start()
{
    running_ = true;
    // 每 N 帧写入一次，降低 NVMe 写压力（全帧率 45MB/s → 15MB/s）
    static constexpr int kWriteInterval = 4;
    fprintf(stderr, "[NvmeWorker] started for %d cameras, write every %d frames\n",
            numCameras_, kWriteInterval);

    while (running_) {
        bool gotAny = false;

        for (int cam = 0; cam < numCameras_; ++cam) {
            uint8_t* data = nullptr;
            size_t size = 0;
            uint64_t timestampUs = 0;

            if (streamer_->try_get_record_frame(cam, &data, &size, &timestampUs)) {
                ++frameSkipCount_[cam];
                if (frameSkipCount_[cam] >= kWriteInterval) {
                    nvme_->write_video_frame_to_disk(
                        data, size,
                        timestampUs * 1000,
                        cam == 0);
                    frameSkipCount_[cam] = 0;
                }

                streamer_->release_record_frame(cam, data);
                gotAny = true;
            }
        }

        // 雷达数据写入（与视频帧同步存储）
        // lidarPtr_ 指向 Widget::lidar_，每次解引用获取最新指针值
        SentinelLslidarer* lidar = (lidarPtr_ != nullptr) ? *lidarPtr_ : nullptr;
        if (lidar != nullptr && lidar->is_running()) {
            LidarFrame frame;
            frame.points = lidarPointsBuf_;
            if (lidar->get_latest_frame(frame)) {
                if (frame.timestampNs != lastLidarTs_) {
                    nvme_->write_lidar_points_to_disk(
                        reinterpret_cast<const uint8_t*>(frame.points),
                        frame.pointsCount * sizeof(LidarPoint),
                        frame.timestampNs);
                    lastLidarTs_ = frame.timestampNs;
                }
                gotAny = true;
            }
        }

        if (!gotAny) {
            QThread::msleep(5);
        }
    }

    fprintf(stderr, "[NvmeWorker] stopped\n");
}

void NvmeWorker::stop()
{
    running_ = false;
}
