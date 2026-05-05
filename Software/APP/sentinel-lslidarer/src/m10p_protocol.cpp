#include "sentinel_lslidarer.h"

#include <cmath>

// ============================================================================
// SentinelLslidarer 私有方法：协议相关（改编自 lslidar_x10_driver.cpp）
// ============================================================================

void SentinelLslidarer::build_lut_() {
    static constexpr float kDegToRad = static_cast<float>(M_PI / 180.0);

    for (int i = 0; i < 36000; ++i) {
        float rad = static_cast<float>(i) * 0.01f * kDegToRad;
        sinLut_[i] = std::sin(rad);
        cosLut_[i] = std::cos(rad);
    }
}

bool SentinelLslidarer::check_packet_validity_(const uint8_t* data, int packetLen) const {
    // 包头校验
    if (data[0] != 0xA5 || data[1] != 0x5A) {
        return false;
    }

    // N10Plus / N10: CRC 字节累加和校验
    uint8_t crc = 0;
    for (int i = 0; i < packetLen - 1; ++i) {
        crc += data[i];
    }

    return crc == data[packetLen - 1];
}

int SentinelLslidarer::decode_packet_(const uint8_t* data, int packetLen,
                                      DecodedPoint* decoded, int maxPoints) {

    // N10Plus: 读取起止方位角
    int startAngle = ((data[LidarConfig::kAngleBitsStart] << 8)
                    + data[LidarConfig::kAngleBitsStart + 1]) % 36000;
    int endAngle   = ((data[LidarConfig::kEndAngleBitsStart] << 8)
                    + data[LidarConfig::kEndAngleBitsStart + 1]) % 36000;

    int angleInterval;
    if (startAngle > endAngle) {
        angleInterval = endAngle + 36000 - startAngle;
    } else {
        angleInterval = endAngle - startAngle;
    }

    int angleGroups     = LidarConfig::kPacketPointsMax;   // 16
    float angleIncrement = static_cast<float>(angleInterval) / (angleGroups - 1);

    int totalPoints = angleGroups * 2;  // 双回波 = 32
    if (totalPoints > maxPoints) totalPoints = maxPoints;

    int pointCount = 0;
    for (int group = 0; group < angleGroups; ++group) {
        int currentAngle = (startAngle + static_cast<int>(angleIncrement * group)) % 36000;

        for (int echo = 0; echo < 2; ++echo) {
            if (pointCount >= totalPoints) break;

            int dataOff = LidarConfig::kDataBitsStart + group * 6 + echo * 3;

            uint16_t rawDist = (static_cast<uint16_t>(data[dataOff]) << 8)
                             | data[dataOff + 1];
            float dist = static_cast<float>(rawDist) * LidarConfig::kDistanceResolution;
            float intensity = static_cast<float>(data[dataOff + 2]);

            decoded[pointCount].azimuth   = static_cast<uint16_t>(currentAngle);
            decoded[pointCount].distance  = dist;
            decoded[pointCount].intensity = intensity;
            ++pointCount;
        }
    }

    return pointCount;
}

bool SentinelLslidarer::is_point_valid_(float distance, int azimuth) const {
    if (distance < config_.minRange || distance > config_.maxRange) return false;
    if (azimuth >= config_.angleDisableMin && azimuth < config_.angleDisableMax) return false;
    return true;
}
