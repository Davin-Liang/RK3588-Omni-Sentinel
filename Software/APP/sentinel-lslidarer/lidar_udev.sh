#!/bin/bash
#===============================================================================
# 镭神 单线雷达专用 udev 规则安装脚本
#
# 根据雷达串口转换芯片类型，自动创建符号链接 /dev/sentinel_lidar
# 支持以下三种芯片（雷达端串口号固定为 0001）：
#   - CP2102  (Silicon Labs)  vendor=10c4  product=ea60
#   - CH9102  (WCH, 含驱动)   vendor=1a86  product=55d4  设备名 ttyCH343USB*
#   - CH9102  (WCH, 无驱动)   vendor=1a86  product=55d4  设备名 ttyACM*
#
# 用法: sudo bash lidar_udev.sh
#===============================================================================

set -e

RULES_DIR=/etc/udev/rules.d

echo "=== 安装 镭神 雷达 udev 规则 ==="

# CP2102 芯片，串口号 0001 → sentinel_lidar
cat > ${RULES_DIR}/sentinel_lidar.rules << 'EOF'
KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="sentinel_lidar"
EOF

# CH9102 芯片，已安装驱动，串口号 0001 → sentinel_lidar
cat > ${RULES_DIR}/sentinel_lidar_ch343.rules << 'EOF'
KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="sentinel_lidar"
EOF

# CH9102 芯片，未安装驱动，串口号 0001 → sentinel_lidar
cat > ${RULES_DIR}/sentinel_lidar_acm.rules << 'EOF'
KERNEL=="ttyACM*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="sentinel_lidar"
EOF

# 重新加载规则
udevadm control --reload-rules
sleep 1
udevadm trigger

echo "安装完成。重新插拔雷达 USB 后，设备将固定映射为 /dev/sentinel_lidar"
echo "默认配置路径已对齐 LidarConfig::serialPort = \"/dev/sentinel_lidar\""
