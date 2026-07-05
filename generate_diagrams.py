#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Generate all 8 architecture diagrams for Section 2.3.2 of qianrushi.docx
"""

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Arc, ConnectionPatch
from matplotlib.patches import Rectangle, Polygon, Ellipse, Circle, Wedge, FancyArrow
import matplotlib.font_manager as fm
import numpy as np
import os

# ── Font setup ──────────────────────────────────────────────
# Try multiple Chinese fonts, set via rcParams for matplotlib 2.x compatibility
chinese_fonts = ['SimHei', 'Microsoft YaHei', 'FangSong', 'KaiTi', 'STSong', 'SimSun']
available_fonts = [f.name for f in fm.fontManager.ttflist]
chosen_font = None
for cf in chinese_fonts:
    if cf in available_fonts:
        chosen_font = cf
        break
if not chosen_font:
    chosen_font = available_fonts[0] if available_fonts else 'sans-serif'

plt.rcParams['font.family'] = chosen_font
plt.rcParams['font.sans-serif'] = [chosen_font, 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

prop_title = fm.FontProperties(family=chosen_font, size=11, weight='bold')
prop_label = fm.FontProperties(family=chosen_font, size=9)
prop_small = fm.FontProperties(family=chosen_font, size=8)
prop_tiny = fm.FontProperties(family=chosen_font, size=7)
prop_big = fm.FontProperties(family=chosen_font, size=13, weight='bold')

OUTPUT_DIR = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\diagrams'

# ── Color palette ───────────────────────────────────────────
C_LAYER = ['#1a237e', '#283593', '#1565c0', '#1976d2', '#2e7d32',
           '#ef6c00', '#c62828', '#6a1b9a', '#00838f', '#4e342e', '#37474f']
C_BOX   = '#E3F2FD'
C_BOX_BORDER = '#1565C0'
C_ARROW = '#37474F'
C_HIGHLIGHT = '#FF6F00'
C_ACCENT = '#2E7D32'
C_SOFT = '#ECEFF1'
C_DATA = '#FFF3E0'
C_DATA_BORDER = '#E65100'
C_KERNEL = '#FCE4EC'
C_KERNEL_BORDER = '#C62828'
C_RGA = '#E8F5E9'
C_RGA_BORDER = '#388E3C'


def save_fig(fig, name):
    path = os.path.join(OUTPUT_DIR, name)
    fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white', edgecolor='none')
    plt.close(fig)
    print(f'Saved: {path}')


def draw_box(ax, x, y, w, h, text='', color=C_BOX, border=C_BOX_BORDER,
             font=prop_label, text_color='#212121', fontsize=8, zorder=2, alpha=1.0):
    """Draw a rounded box with text."""
    box = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=2",
                          facecolor=color, edgecolor=border, linewidth=1.2,
                          zorder=zorder, alpha=alpha, mutation_scale=8)
    ax.add_patch(box)
    if text:
        ax.text(x + w/2, y + h/2, text, ha='center', va='center',
                fontproperties=font, fontsize=fontsize, color=text_color,
                zorder=zorder+1, wrap=True)
    return box


def draw_rect_box(ax, x, y, w, h, text='', color=C_BOX, border=C_BOX_BORDER,
                  font=prop_label, text_color='#212121', fontsize=8, zorder=2,
                  linewidth=1.2):
    """Draw a sharp-rectangle box with text."""
    rect = Rectangle((x, y), w, h, facecolor=color, edgecolor=border,
                     linewidth=linewidth, zorder=zorder)
    ax.add_patch(rect)
    if text:
        ax.text(x + w/2, y + h/2, text, ha='center', va='center',
                fontproperties=font, fontsize=fontsize, color=text_color,
                zorder=zorder+1)
    return rect


def draw_arrow(ax, x1, y1, x2, y2, color=C_ARROW, lw=1.0, style='->',
               zorder=1, connectionstyle='arc3,rad=0', ls='-'):
    """Draw an arrow."""
    ax.annotate('', xy=(x2, y2), xytext=(x1, y1),
                arrowprops=dict(arrowstyle=style, color=color, lw=lw,
                               connectionstyle=connectionstyle, linestyle=ls),
                zorder=zorder)


def draw_layer_bg(ax, x, y, w, h, label='', color='#ECEFF1', alpha=0.5, zorder=0):
    """Draw a layer background with label."""
    rect = FancyBboxPatch((x, y), w, h, boxstyle="round,pad=3",
                          facecolor=color, edgecolor='#90A4AE', linewidth=1.0,
                          zorder=zorder, alpha=alpha, linestyle='--',
                          mutation_scale=8)
    ax.add_patch(rect)
    if label:
        ax.text(x + 0.15, y + h - 0.28, label, ha='left', va='top',
                fontproperties=prop_title, fontsize=9, color='#546E7A', zorder=zorder+1)


# ═══════════════════════════════════════════════════════════════
# Diagram 1: 软件系统总体架构图 (Overall 7-Layer Architecture)
# ═══════════════════════════════════════════════════════════════
def diagram_overall_architecture():
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 10)
    ax.set_aspect('equal')
    ax.axis('off')

    # Title
    ax.text(7, 9.7, 'RK3588 Omni-Sentinel 软件系统总体架构', ha='center', va='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # Layer parameters: (y, height, color, label)
    layers = [
        (8.5, 1.0, '#BBDEFB', '交互层 (Interaction)'),
        (7.2, 1.1, '#B3E5FC', '流媒体层 (Streaming)'),
        (5.8, 1.2, '#C8E6C9', '融合层 (Fusion)'),
        (3.8, 1.8, '#FFF9C4', '感知层 (Perception)'),
        (2.2, 1.4, '#FFE0B2', '基础设施层 (Infrastructure)'),
        (1.0, 1.0, '#F8BBD0', '驱动层 (Kernel Drivers)'),
        (0.0, 0.8, '#D7CCC8', '硬件层 (Hardware)'),
    ]

    for y, h, color, label in layers:
        draw_layer_bg(ax, 0.1, y, 13.8, h, label, color=color, alpha=0.35)

    # ── Hardware Layer ──
    hw_x = [0.3, 3.0, 5.5, 8.0, 10.5, 12.3]
    hw_w = [2.4, 2.2, 2.2, 2.2, 1.5, 1.5]
    hw_labels = ['OV13855/ISP\nMIPI-CSI', 'USB UVC\n后视摄像头',
                 'N10Plus\nLiDAR', 'ICM45686\nIMU(SPI)',
                 'NVMe SSD\nPCIe3.0x4', 'DSI触屏\n1024x600']
    for i in range(6):
        draw_rect_box(ax, hw_x[i], 0.1, hw_w[i], 0.6, hw_labels[i],
                      color='#EFEBE9', border='#8D6E63', font=prop_tiny, fontsize=6)

    # ── Kernel Driver Layer ──
    draw_box(ax, 1.0, 1.15, 2.8, 0.7, 'dm-ringbox.ko\n环形块设备驱动',
             color=C_KERNEL, border=C_KERNEL_BORDER, font=prop_small, fontsize=7)
    draw_box(ax, 5.0, 1.15, 2.8, 0.7, 'icm45686_spi.ko\ninv_imu_driver.ko',
             color=C_KERNEL, border=C_KERNEL_BORDER, font=prop_small, fontsize=7)
    draw_box(ax, 9.0, 1.15, 3.5, 0.7, 'OV5647/OV13855 DTS\nV4L2/DRM/MIPI-DSI 内核子系统',
             color=C_KERNEL, border=C_KERNEL_BORDER, font=prop_small, fontsize=6)

    # ── Infrastructure Layer ──
    draw_box(ax, 0.5, 2.35, 3.2, 1.1, 'DmaBufferPool\nDMA-BUF内存池\n(Free-List O(1)分配)',
             color='#FFF8E1', border='#F9A825', font=prop_small, fontsize=7)
    draw_box(ax, 4.2, 2.35, 3.5, 1.1, '3rdparty 库集合\nRGA | MPP | FFmpeg\nRKNN | librga | libdrm',
             color='#FFF8E1', border='#F9A825', font=prop_small, fontsize=7)
    draw_box(ax, 8.2, 2.35, 3.0, 1.1, 'NVMeDataManager\nO_DIRECT裸块写入\n(20B Header+512B对齐)',
             color='#FFF8E1', border='#F9A825', font=prop_small, fontsize=7)

    # ── Perception Layer (4 modules, 2x2 grid) ──
    draw_box(ax, 0.5, 4.0, 3.0, 1.5, 'SentinelVisioner\n视觉采集层\nV4L2捕获+RGA一分三扇出\nepoll异步事件驱动',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 3.8, 4.0, 3.0, 1.5, 'SentinelLslidarer\n雷达驱动层\n3层架构(串口→环形缓冲\n→协议解码+LUT转换)',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 7.1, 4.0, 3.0, 1.5, 'SentinelYoloInfer\nAI推理层\nRKNN DMA零拷贝导入\nINT8量化+双队列扇出',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 10.4, 4.0, 3.0, 1.5, 'icm45686-eis-app\nIMU防抖层\nMadgwick AHRS+梯形积分\nEMA平滑+RGA偏移校正',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)

    # ── Fusion Layer ──
    draw_box(ax, 3.0, 5.95, 6.5, 1.0, 'LidarCameraFusion 融合跟踪层\n两遍融合算法(外参变换→内参投影→分类→整理) + Alpha-Beta多目标跟踪器(6阶段管线,50航迹,16参数)',
             color='#E8F5E9', border='#2E7D32', font=prop_small, fontsize=7)

    # ── Streaming Layer ──
    draw_box(ax, 2.0, 7.35, 8.5, 0.9, 'SentinelStreamer 流媒体层\n双编码器独立架构(720p CBR推流+1080p/720p可切换MP4录像) | OSD叠加(YOLO框+LiDAR点云) | RecordBufferPool环形缓冲',
             color='#E1F5FE', border='#0277BD', font=prop_small, fontsize=7)

    # ── Interaction Layer ──
    draw_box(ax, 1.5, 8.6, 4.5, 0.75, 'SentinelQT 嵌入式触控HMI\nQStackedWidget 4页面 | 多线程Worker\n内嵌HTTP/WebSocket服务器',
             color='#EDE7F6', border='#4527A0', font=prop_small, fontsize=7)
    draw_box(ax, 7.0, 8.6, 4.5, 0.75, 'WebControl Web远程控制\n基于cpp-httplib(27+ REST API)\nWebSocket实时推送 | SPA前端',
             color='#EDE7F6', border='#4527A0', font=prop_small, fontsize=7)

    # ── Data flow arrows ──
    # Visioner → YoloInfer
    draw_arrow(ax, 2.0, 5.5, 5.9, 5.5, '#1565C0', 1.2)
    ax.text(3.95, 5.65, '640x640 RGB888\nDMA-BUF fd', ha='center', fontproperties=prop_tiny, fontsize=5, color='#1565C0')
    # Visioner → Streamer
    draw_arrow(ax, 2.0, 5.45, 5.0, 7.3, '#1565C0', 1.0)
    ax.text(2.5, 6.35, 'NV12\nDMA-BUF', ha='center', fontproperties=prop_tiny, fontsize=5, color='#1565C0')
    # YoloInfer → Fusion
    draw_arrow(ax, 8.6, 5.5, 8.6, 5.95, '#1565C0', 1.0)
    # YoloInfer → Streamer (OSD)
    draw_arrow(ax, 10.1, 5.5, 8.0, 7.3, '#1565C0', 1.0)
    ax.text(9.5, 6.35, 'OSD\n检测框', ha='center', fontproperties=prop_tiny, fontsize=5, color='#1565C0')
    # Lslidarer → Fusion
    draw_arrow(ax, 5.3, 5.5, 5.3, 5.95, '#1565C0', 1.0)
    # EIS → Visioner
    draw_arrow(ax, 11.9, 5.5, 5.0, 5.5, '#EF6C00', 1.0)
    ax.text(8.45, 5.65, 'EIS偏移回调(X,Y)', ha='center', fontproperties=prop_tiny, fontsize=5, color='#EF6C00')
    # EIS → Streamer
    draw_arrow(ax, 11.9, 4.8, 8.5, 7.3, '#EF6C00', 0.8)
    # Fusion → Streamer
    draw_arrow(ax, 6.5, 6.95, 7.5, 7.3, '#1565C0', 0.8)
    ax.text(7.0, 7.15, 'LiDAR OSD', ha='center', fontproperties=prop_tiny, fontsize=5, color='#1565C0')
    # Visioner → Qt (preview)
    draw_arrow(ax, 3.5, 5.5, 3.0, 8.6, '#1565C0', 0.8)
    # Fusion → Qt
    draw_arrow(ax, 7.5, 6.95, 4.5, 8.6, '#1565C0', 0.8)
    # Streamer → recorder (NVMeDataManager)
    draw_arrow(ax, 3.5, 7.3, 9.0, 3.45, '#1565C0', 1.0)
    ax.text(5.0, 5.0, 'RecordBufferPool\n150帧环形缓冲', ha='center', fontproperties=prop_tiny, fontsize=5, color='#1565C0')
    # DmaBufferPool → all perception modules
    draw_arrow(ax, 2.1, 3.45, 2.0, 4.0, '#F9A825', 0.7)
    # NVMeDataManager → dm-ringbox
    draw_arrow(ax, 9.7, 2.35, 2.4, 1.85, '#C62828', 1.0)
    ax.text(6.0, 1.9, 'bio device mapper', ha='center', fontproperties=prop_tiny, fontsize=5, color='#C62828')
    # Qt <-> WebControl
    draw_arrow(ax, 4.0, 9.35, 7.0, 9.35, '#4527A0', 1.0)
    draw_arrow(ax, 7.0, 9.15, 4.0, 9.15, '#4527A0', 1.0)
    ax.text(5.5, 9.5, 'REST API / WebSocket\n双向互控', ha='center', fontproperties=prop_tiny, fontsize=5, color='#4527A0')

    # ── Legend ──
    legend_y = 0.15
    ax.text(0.5, legend_y, '数据流 →', fontproperties=prop_tiny, fontsize=6, color=C_ARROW)
    ax.text(2.0, legend_y, '控制流 ⇢', fontproperties=prop_tiny, fontsize=6, color='#4527A0')
    ax.text(3.5, legend_y, 'DMA-BUF零拷贝: V4L2→RGA→NPU→MPP', fontproperties=prop_tiny, fontsize=6, color='#1565C0')
    ax.text(7.0, legend_y, '硬件加速器: RGA 2D | NPU 6TOPS | MPP H.264', fontproperties=prop_tiny, fontsize=6, color='#2E7D32')

    save_fig(fig, '01_overall_architecture.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 2: SentinelVisioner 数据流图 (One-to-Three Fan-Out)
# ═══════════════════════════════════════════════════════════════
def diagram_visioner_fanout():
    fig, ax = plt.subplots(1, 1, figsize=(14, 7))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 7)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(7, 6.8, 'SentinelVisioner 数据流图 — 一分三扇出架构', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # ── V4L2 Capture (left) ──
    draw_box(ax, 0.3, 4.5, 3.0, 1.8, 'V4L2 相机采集\n━━━━━━━━━━━\n/dev/video11(MIPI-CSI)\n/dev/video21(USB UVC)\nepoll异步事件驱动\nDMA-BUF直接导出',
             color='#E8EAF6', border='#283593', font=prop_small, fontsize=7)

    # ── Format negotiation ──
    draw_box(ax, 3.8, 4.7, 2.2, 1.4, '格式协商\n━━━━━━━━\n• MJPG→FFmpeg\n  软件解码NV12\n• YUYV→RGA\n  YVYU→NV12\n• NV12→直通\n  RGA imcopy',
             color='#FFF8E1', border='#F9A825', font=prop_tiny, fontsize=6)

    # ── Central NV12 buffer ──
    draw_box(ax, 6.5, 4.8, 2.0, 1.2, 'NV12\n中间缓冲\n(统一格式)\nDMA-BUF',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)

    # ── EIS injection ──
    draw_box(ax, 6.5, 2.8, 2.0, 1.0, 'EIS偏移注入\nEMA平滑\noffset=α·cur\n+(1-α)·prev\nα=0.4',
             color='#FFF3E0', border='#E65100', font=prop_tiny, fontsize=6)

    # ── RGA Fan-Out into 3 paths ──
    # Path 1: NPU
    draw_box(ax, 9.2, 5.6, 4.3, 1.2, '路径① NPU推理池 (rga_process_to_rgb)\nNV12→640×640 RGB888 Letterbox+EIS偏移\n→ npuTaskQueue (ThreadSafeQueue)',
             color='#E8F5E9', border='#2E7D32', font=prop_tiny, fontsize=6)

    # Path 2: Preview
    draw_box(ax, 9.2, 4.2, 4.3, 1.2, '路径② 预览池 (rga_convert_to_rgb_full)\nNV12→全分辨率 BGR888\n→ previewTaskQueue (ThreadSafeQueue)',
             color='#E8F5E9', border='#388E3C', font=prop_tiny, fontsize=6)

    # Path 3: Stream
    draw_box(ax, 9.2, 2.8, 4.3, 1.0, '路径③ 推流池 (rga_copy_buffer/imcopy)\nNV12→NV12 硬件DMA拷贝+EIS元数据\n→ processTaskQueue (ThreadSafeQueue)',
             color='#E8F5E9', border='#43A047', font=prop_tiny, fontsize=6)

    # ── Consumers (right side) ──
    draw_box(ax, 9.2, 1.2, 4.3, 0.8, '消费者: SentinelYoloInfer | SentinelQT | SentinelStreamer',
             color='#F3E5F5', border='#7B1FA2', font=prop_small, fontsize=7)

    # ── Arrows ──
    draw_arrow(ax, 3.3, 5.4, 3.8, 5.4, C_ARROW, 1.2)
    draw_arrow(ax, 6.0, 5.4, 6.5, 5.4, C_ARROW, 1.2)
    draw_arrow(ax, 8.5, 5.4, 9.2, 6.0, C_ARROW, 0.8)
    draw_arrow(ax, 8.5, 5.2, 9.2, 4.8, C_ARROW, 0.8)
    draw_arrow(ax, 8.5, 5.0, 9.2, 3.4, C_ARROW, 0.8)
    draw_arrow(ax, 7.5, 3.8, 7.5, 4.8, C_ARROW, 0.8)
    draw_arrow(ax, 11.35, 4.2, 11.35, 2.8, C_ARROW, 0.6)
    draw_arrow(ax, 11.35, 2.8, 11.35, 2.0, C_ARROW, 0.6)
    draw_arrow(ax, 11.35, 5.6, 11.35, 2.0, C_ARROW, 0.6)

    # RGA operation labels
    ax.text(6.5, 3.65, 'EIS偏移\n(x,y)', ha='center', fontproperties=prop_tiny, fontsize=5, color='#E65100')
    ax.text(13.5, 6.2, '640×640\nRGB888', ha='center', fontproperties=prop_tiny, fontsize=5, color='#2E7D32')
    ax.text(13.5, 4.8, '全分辨率\nBGR888', ha='center', fontproperties=prop_tiny, fontsize=5, color='#388E3C')
    ax.text(13.5, 3.3, 'NV12\nDMA-BUF', ha='center', fontproperties=prop_tiny, fontsize=5, color='#43A047')

    # RGA hardware annotation
    draw_box(ax, 0.3, 2.2, 5.0, 0.8, 'RGA 2D硬件加速引擎 (improcess IM_SYNC 同步模式)\n3次独立RGA操作/帧，均在GPU/DRM上下文执行，CPU零像素搬运',
             color='#E8F5E9', border='#388E3C', font=prop_tiny, fontsize=6)

    # Camera type table
    draw_box(ax, 0.3, 0.5, 8.0, 1.5, '相机兼容矩阵:\nISP(MIPI) | NV12直出 | MPLANE模式\nUSB NV12   | RGA imcopy安全拷贝\nUSB YUYV   | RGA YVYU_422→NV12转换\nUSB MJPG   | mmap读取→FFmpeg软件解码→NV12',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=6, text_color='#37474F')

    save_fig(fig, '02_visioner_fanout.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 3: RKNN NPU DMA零拷贝推理流程图
# ═══════════════════════════════════════════════════════════════
def diagram_npu_inference():
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(7, 7.8, 'RKNN NPU DMA零拷贝推理流程 — SentinelYoloInfer', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # Left column: Data flow
    # DMA-BUF Input
    draw_box(ax, 0.5, 6.3, 3.0, 1.2, 'NpuBufferGuard (RAII)\n从Visioner npuTaskQueue\ntry_get_npu()阻塞获取\n640×640 RGB888 DMA-BUF fd',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)

    # rknn_create_mem_from_fd
    draw_box(ax, 0.5, 4.8, 3.0, 1.2, 'DMA零拷贝导入\nrknn_create_mem_from_fd()\nDMA-BUF fd → NPU物理地址\n→ rknn_tensor_mem',
             color='#FFF3E0', border='#E65100', font=prop_small, fontsize=7)

    # rknn_set_io_mem
    draw_box(ax, 0.5, 3.2, 3.0, 1.2, 'IO内存绑定\nrknn_set_io_mem(ctx,\ninput_mem, input_attrs)\nNPU直接访问DMA物理内存',
             color='#FFF3E0', border='#E65100', font=prop_small, fontsize=7)

    # rknn_run
    draw_box(ax, 0.5, 1.6, 3.0, 1.2, 'NPU INT8量化推理\nrknn_run(ctx)\nYOLOv8n (640×640×3输入)\n9个输出分支 | 6 TOPS算力',
             color='#E8F5E9', border='#2E7D32', font=prop_small, fontsize=7)

    # Right column: Post-processing & Output
    # postProcess
    draw_box(ax, 4.5, 5.5, 3.5, 2.0, '后处理 (postProcess)\n━━━━━━━━━━━━━━\n• 9输出分支解码\n• 边界框解码\n  (anchor→绝对坐标)\n• 置信度过滤 (>0.25)\n• NMS去重 (IoU>0.45)',
             color='#E1F5FE', border='#0277BD', font=prop_small, fontsize=7)

    # Output fan-out
    draw_box(ax, 4.5, 3.2, 3.5, 1.8, '推理结果双队列扇出\n━━━━━━━━━━━━━━\n• fusionQueue → LidarCameraFusion\n  (阻塞消费,融合跟踪用)\n• osdQueue → SentinelStreamer\n  (非阻塞轮询,OSD叠加用)',
             color='#F3E5F5', border='#7B1FA2', font=prop_small, fontsize=7)

    # Per-camera context
    draw_box(ax, 9.0, 5.5, 4.5, 2.5, '每路相机独立推理上下文\n━━━━━━━━━━━━━━━━\nInferThreadContext (per camera):\n• 独立RKNN上下文 (独立IO内存)\n• 独立推理线程 (独立轮询周期)\n• Yolov8RknnEngine共享模型文件\n  (多context复用同一.rknn权重,\n   NPU硬件时隙复用)\n• CPU推理时的FPS: 理论30FPS→实际\n  双路并行仍有充足NPU余量(48.2%)',
             color='#E8EAF6', border='#283593', font=prop_tiny, fontsize=6)

    # Arrows
    draw_arrow(ax, 3.5, 6.9, 4.5, 6.5, C_ARROW, 1.2)
    draw_arrow(ax, 3.5, 5.4, 4.5, 5.5, C_ARROW, 0.8)
    draw_arrow(ax, 3.5, 5.4, 4.5, 3.0, C_ARROW, 0.8)
    draw_arrow(ax, 3.5, 3.8, 4.5, 4.0, C_ARROW, 1.0)
    draw_arrow(ax, 3.5, 2.2, 4.5, 3.5, C_ARROW, 1.0)
    draw_arrow(ax, 8.0, 6.5, 9.0, 6.5, C_ARROW, 0.8)

    # Downward main flow arrows
    draw_arrow(ax, 2.0, 6.3, 2.0, 6.0, C_ARROW, 1.5)
    draw_arrow(ax, 2.0, 4.8, 2.0, 4.4, C_ARROW, 1.5)
    draw_arrow(ax, 2.0, 3.2, 2.0, 2.8, C_ARROW, 1.5)

    # Annotations
    ax.text(3.8, 6.9, 'BBox\n列表', ha='center', fontproperties=prop_tiny, fontsize=5, color=C_ARROW)
    ax.text(3.8, 6.0, '原始\n输出', ha='center', fontproperties=prop_tiny, fontsize=5, color=C_ARROW)
    ax.text(3.8, 4.3, 'NMS\n后结果', ha='center', fontproperties=prop_tiny, fontsize=5, color=C_ARROW)

    # Performance metrics box
    draw_box(ax, 9.0, 2.5, 4.5, 1.5, '性能指标 (单路, YOLOv8n INT8)\n━━━━━━━━━━━━━━━━━━━\n• 单帧推理: 27.3ms (vs CPU 187.5ms, 6.9×加速)\n• 吞吐量: 36.6 FPS (vs CPU 5.3 FPS)\n• mAP@0.5: 0.453 (精度损失<3%)',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=6)

    # Zero-copy annotation
    draw_box(ax, 0.5, 0.3, 13.0, 1.0, 'DMA-BUF零拷贝全链路: V4L2(RKISP) → DMA-BUF fd → RGA(letterbox) → DMA-BUF fd → rknn_create_mem_from_fd(NPU直接访问) → 推理结果 → 双队列扇出\n关键特性: 全程无CPU memcpy, 无dma_buf_map/mmap, fd传递共享同一dma-buf物理内存页 → 最大化RK3588异构算力利用率',
             color='#E8F5E9', border='#388E3C', font=prop_tiny, fontsize=6, text_color='#1B5E20')

    save_fig(fig, '03_npu_inference.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 4: LiDAR驱动三层架构流程图
# ═══════════════════════════════════════════════════════════════
def diagram_lidar_architecture():
    fig, ax = plt.subplots(1, 1, figsize=(14, 7.5))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 7.5)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(7, 7.3, 'SentinelLslidarer LiDAR驱动三层架构', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # ── Layer 1: SerialPort ──
    draw_layer_bg(ax, 0.2, 5.3, 13.6, 1.7, '第一层: SerialPort — POSIX Raw-Mode 串口通信', color='#BBDEFB', alpha=0.5)
    draw_box(ax, 0.5, 5.5, 3.5, 1.3, '设备层\n/dev/sentinel_lidar\nUART 460800bps\n8N1, 3.3V TTL',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 4.5, 5.5, 3.5, 1.3, '同步头扫描\n扫描0xA5 0x5A\n固定包长108字节\nCRC校验验证',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 8.5, 5.5, 3.5, 1.3, '原始包输出\nuint8_t[108]\n含32点×2回波\n时间戳CLOCK_MONOTONIC',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_arrow(ax, 4.0, 6.15, 4.5, 6.15, C_ARROW, 1.0)
    draw_arrow(ax, 8.0, 6.15, 8.5, 6.15, C_ARROW, 1.0)

    # ── Layer 2: RingBuffer (SWCR) ──
    draw_layer_bg(ax, 0.2, 3.2, 13.6, 1.8, '第二层: RingBuffer — SWCR无锁环形缓冲 (Single Writer / Consumer Reader)', color='#FFF9C4', alpha=0.5)
    draw_box(ax, 0.5, 3.4, 3.5, 1.4, '缓冲区结构\n• 10 Slot × 540 LidarPoint\n  = ~65KB预分配内存\n• 每Slot含时间戳+点数\n• begin_write()/commit_write()',
             color='#FFF8E1', border='#F9A825', font=prop_tiny, fontsize=6)
    draw_box(ax, 4.5, 3.4, 4.0, 1.4, '无锁并发机制 (SWCR)\n• std::atomic<uint32_t> 序列号\n• memory_order_release (写端)\n• memory_order_acquire (读端)\n• write_index()/copy_slot()非阻塞读\n• 写端单线程, 读端单线程',
             color='#FFF8E1', border='#F9A825', font=prop_tiny, fontsize=6)
    draw_box(ax, 9.0, 3.4, 3.5, 1.4, '整帧获取\n• get_closest_frame(ts, out)\n• 按相机时间戳线性检索\n• 取最小|Δt|的完整帧\n• 非阻塞,零拷贝指针传递',
             color='#FFF8E1', border='#F9A825', font=prop_tiny, fontsize=6)
    draw_arrow(ax, 4.0, 4.1, 4.5, 4.1, C_ARROW, 1.0)
    draw_arrow(ax, 8.5, 4.1, 9.0, 4.1, C_ARROW, 1.0)

    # ── Layer 3: Main Driver (reader_loop_) ──
    draw_layer_bg(ax, 0.2, 1.0, 13.6, 1.9, '第三层: 主驱动 — 协议解码 + 扫描边界检测 + LUT极坐标转换', color='#C8E6C9', alpha=0.5)
    draw_box(ax, 0.5, 1.2, 2.8, 1.5, '协议解码\n• 108字节N10Plus协议\n• 16角度×2回波→\n  32 DecodedPoint/包\n• 解析方位角/距离/\n  强度字段',
             color='#E8F5E9', border='#2E7D32', font=prop_tiny, fontsize=6)
    draw_box(ax, 3.8, 1.2, 3.5, 1.5, '扫描边界检测\n• 方位角360°穿越识别\n• 连续方位角(Δθ>350°)\n→ 判定为新扫描圈开始\n• 首圈丢弃(部分扫描圈)\n• 整圈540点组装',
             color='#E8F5E9', border='#388E3C', font=prop_tiny, fontsize=6)
    draw_box(ax, 7.8, 1.2, 3.5, 1.5, 'LUT加速极坐标转换\n• sin/cos预计算表\n  36000条目(0.01°分辨率)\n• (r,θ)→(x,y)查表O(1)\n• x = r·cos(θ), y = r·sin(θ)',
             color='#E8F5E9', border='#43A047', font=prop_tiny, fontsize=6)
    draw_box(ax, 11.8, 1.2, 2.0, 1.5, '输出\nLidarFrame\n540点/帧\n10Hz',
             color='#C8E6C9', border='#2E7D32', font=prop_small, fontsize=7)
    draw_arrow(ax, 3.3, 1.95, 3.8, 1.95, C_ARROW, 1.0)
    draw_arrow(ax, 7.3, 1.95, 7.8, 1.95, C_ARROW, 1.0)
    draw_arrow(ax, 11.3, 1.95, 11.8, 1.95, C_ARROW, 1.0)

    # ── Cross-layer arrows ──
    draw_arrow(ax, 7.0, 5.3, 7.0, 5.2, C_ARROW, 1.5)
    draw_arrow(ax, 7.0, 3.2, 7.0, 3.1, C_ARROW, 1.5)

    # Thread & consumer annotation
    draw_box(ax, 0.5, 0.1, 6.5, 0.7, 'readerThread_: start() → reader_loop_ 独立线程 | stop()关闭串口→解除::read()阻塞→join线程',
             color='#F3E5F5', border='#7B1FA2', font=prop_tiny, fontsize=6)
    draw_box(ax, 7.5, 0.1, 6.0, 0.7, 'Consumer: LidarCameraFusion.get_closest_frame(camera_ts)→按时间戳检索最近LiDAR帧',
             color='#F3E5F5', border='#7B1FA2', font=prop_tiny, fontsize=6)

    save_fig(fig, '04_lidar_architecture.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 5: LiDAR-Camera融合跟踪算法流程图
# ═══════════════════════════════════════════════════════════════
def diagram_fusion_tracking():
    fig, ax = plt.subplots(1, 1, figsize=(16, 9))
    ax.set_xlim(0, 16)
    ax.set_ylim(0, 9)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(8, 8.8, 'LiDAR-Camera融合跟踪算法流程 — 两遍融合 + Alpha-Beta六阶段跟踪管线', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # ── Input Section ──
    draw_box(ax, 0.3, 7.3, 2.5, 1.2, '输入\n━━━━\nYOLO检测框\n(DetectionProvider\n回调,过滤person\n≥0.60置信度)',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=6)
    draw_box(ax, 3.3, 7.3, 2.5, 1.2, '输入\n━━━━\nLiDAR帧\n(get_closest_frame\n按时间戳\n最近邻检索)',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=6)
    draw_box(ax, 6.3, 7.3, 2.5, 1.2, '参数\n━━━━\n相机内外参\n(3×3内参K +\n4×4外参T,\n针对2D雷达\nz=0优化)',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=6)

    # ── Two-Pass Fusion (right column) ──
    ax.text(10.0, 8.35, '两遍融合算法', fontproperties=prop_title, fontsize=10, color='#2E7D32')
    draw_box(ax, 9.5, 7.2, 6.0, 1.0, '第一遍: 外参变换(6乘3加,2D优化)→内参投影(u=fx·cx/cz+cx)→边界过滤→二值分类(首次命中) → 累加pointCount',
             color='#E8F5E9', border='#2E7D32', font=prop_tiny, fontsize=6)
    draw_box(ax, 9.5, 5.8, 6.0, 1.0, '第二遍: 排他前缀扫描(计算写入偏移)→分散写入(pointIndex→candidatePointBuf)→紧凑数组整理→FusionResult',
             color='#C8E6C9', border='#388E3C', font=prop_tiny, fontsize=6)
    draw_arrow(ax, 12.5, 7.2, 12.5, 6.8, C_ARROW, 1.2)

    # ── Pre-allocated buffers ──
    draw_box(ax, 9.5, 4.5, 6.0, 1.0, '预分配缓冲区(~12KB,零运行时分配): pointIndices[pMax*bMax] | pointUVs[pMax] | bboxClassified[bMax]',
             color='#FFF9C4', border='#F9A825', font=prop_tiny, fontsize=6)

    # ── Fusion Result ──
    draw_box(ax, 9.5, 3.3, 6.0, 1.0, 'FusionResult输出: bboxPointIndices[] | bboxPointCounts[] | bboxPointU/V[] | 分类点投影UV | 时间戳对齐',
             color='#E1F5FE', border='#0277BD', font=prop_tiny, fontsize=6)

    # ── 6-Stage Tracking Pipeline ──
    ax.text(3.0, 5.5, 'Alpha-Beta多目标跟踪器 — 六阶段管线 (50航迹, 200检测, 16参数可调)',
            fontproperties=prop_title, fontsize=9, color='#6A1B9A')

    stages = [
        (0.3, 4.5, 2.5, 0.8, '阶段① 聚类\nLiDAR点→候选目标\n空间聚类(半径门限)'),
        (3.0, 4.5, 2.5, 0.8, '阶段② Alpha-Beta预测\nX_pred = X + V·Δt\nV_pred = V (匀速)'),
        (5.7, 4.5, 2.5, 0.8, '阶段③ 贪心最近邻关联\n距离门限+检测框IoU\n最优匹配(单次分配)'),
        (8.4, 4.5, 2.5, 0.8, '阶段④ Alpha-Beta校正\nX = X_pred + α·(Z-X_pred)\nV = V_pred + β·(Z-X_pred)/Δt'),
        (11.1, 4.5, 2.5, 0.8, '阶段⑤ 生命周期管理\nTentative→Confirmed\n→Coasting→Deleted'),
        (13.8, 4.5, 2.0, 0.8, '阶段⑥ 告警检查\n距离迟滞\n<3m进入\n>3.5m退出'),
    ]
    for x, y, w, h, text in stages:
        draw_box(ax, x, y, w, h, text,
                 color='#F3E5F5', border='#7B1FA2', font=prop_tiny, fontsize=5.5)

    # Stage arrows
    for i in range(5):
        x_from = stages[i][0] + stages[i][2]
        x_to = stages[i+1][0]
        y_mid = stages[i][1] + stages[i][3]/2
        draw_arrow(ax, x_from, y_mid, x_to, y_mid, '#6A1B9A', 0.8)

    # ── Output ──
    draw_box(ax, 0.3, 2.8, 6.5, 1.4, '跟踪输出\n• TrackedTarget[] (id, state, pos, vel, distance, age)\n• TrackingCallback告警事件 (距离<3m→告警, >3.5m→恢复, 2s冷却)\n• LidarOsdSnapshot (点云+投影UV+检测框 → Streamer OSD)',
             color='#FCE4EC', border='#C62828', font=prop_tiny, fontsize=6)

    # ── Double-Buffer ──
    draw_box(ax, 7.5, 2.8, 5.0, 1.4, '双缓冲线程安全\n• workingTracks_[kMaxTracks] 工作数组\n• snapshotTracks_ 快照副本\n  (mutex保护,非阻塞copy)\n• FusionWorker 100ms轮询copy_tracked_targets()',
             color='#E8EAF6', border='#283593', font=prop_tiny, fontsize=6)

    # ── Arrows between sections ──
    draw_arrow(ax, 5.0, 7.3, 9.5, 6.5, C_ARROW, 0.8)
    draw_arrow(ax, 6.0, 6.8, 9.5, 5.5, C_ARROW, 0.8)
    draw_arrow(ax, 7.0, 5.0, 9.5, 4.8, C_ARROW, 0.8)
    draw_arrow(ax, 12.5, 3.3, 5.0, 4.2, C_ARROW, 0.8)
    draw_arrow(ax, 3.0, 4.2, 3.0, 4.2, C_ARROW, 0.8)

    # ── Cycle annotation ──
    ax.annotate('每帧循环(10Hz LiDAR/30Hz Camera异步)', xy=(14.5, 5.3), xytext=(14.5, 6.5),
                fontproperties=prop_tiny, fontsize=6, color='#6A1B9A',
                arrowprops=dict(arrowstyle='->', color='#6A1B9A', lw=0.8, connectionstyle='arc3,rad=0.5'))

    # Performance
    draw_box(ax, 0.3, 1.2, 4.5, 1.2, '性能指标\n• MOTA: 82.7% | MOTP: 86.3%\n• ID-Switch: 4次/10min\n• 告警响应: 82.1ms (从入侵到告警)',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=6)
    draw_box(ax, 5.5, 1.2, 10.0, 1.2, '关键设计决策:\n• 两遍融合: 首次命中策略(每点仅归属第一个匹配框)→避免重复匹配 | 预分配缓冲区全栈上分配→零malloc | 6乘3加外参变换(针对z=0优化)→亚毫秒延迟\n• Alpha-Beta滤波器(替代Kalman): 无需矩阵求逆→O(1)计算 | 16参数在线可调(Qt界面+config.ini) | 距离迟滞告警(防抖动,2s冷却)',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=5.5)

    save_fig(fig, '05_fusion_tracking.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 6: EIS电子防抖算法流程图
# ═══════════════════════════════════════════════════════════════
def diagram_eis():
    fig, ax = plt.subplots(1, 1, figsize=(14, 7.5))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 7.5)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(7, 7.3, 'EIS电子防抖算法流程 — icm45686-eis-app', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # ── IMU Sensor ──
    draw_box(ax, 0.5, 5.8, 2.5, 1.2, 'ICM45686 IMU\n━━━━━━━━\n/dev/icm45686\nioctl() 100Hz轮询\n6轴数据(3轴加速度\n+3轴陀螺仪)',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)

    # ── Icm45686Reader ──
    draw_box(ax, 3.5, 5.8, 2.5, 1.2, 'Icm45686Reader\n━━━━━━━━\n独立读取线程\nreadLoop() 100Hz\nCLOCK_MONOTONIC\n时间戳标记',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)

    # ── EMA Filter ──
    draw_box(ax, 6.5, 5.8, 2.5, 1.2, 'EMA低通滤波\n━━━━━━━━\nsmoothed=α·current\n+(1-α)·previous\nα=0.4 (可配置)',
             color='#FFF3E0', border='#E65100', font=prop_small, fontsize=7)

    # ── ImuRingBuffer ──
    draw_box(ax, 3.5, 3.8, 5.5, 1.2, 'ImuRingBuffer 时间窗口缓冲区\n━━━━━━━━━━━━━━━━━━\nstd::deque<ImuSample> (512容量) + mutex | getSamplesBetween(startNs, endNs) → 时间窗口切片查询\n存储: timestampNs | accelX/Y/Z (m/s²) | gyroX/Y/Z (rad/s) | temperature (℃)',
             color='#FFF8E1', border='#F9A825', font=prop_tiny, fontsize=6)

    # ── Madgwick AHRS + Integration ──
    draw_box(ax, 0.3, 2.0, 4.0, 1.5, 'Madgwick AHRS 姿态估计\n━━━━━━━━━━━━━━\n• 加速度计+陀螺仪9轴融合\n• 四元数梯度下降优化\n• 输出: 俯仰(Pitch)/翻滚(Roll)\n  角(欧拉角)',
             color='#E8F5E9', border='#2E7D32', font=prop_tiny, fontsize=6.5)
    draw_box(ax, 4.8, 2.0, 4.0, 1.5, '梯形积分 角位移计算\n━━━━━━━━━━━━━━\n• 角速度→角度:\n  θ = ∫ ω(t) dt\n• 梯形积分:\n  Δθ = (ω_t+ω_{t-1})/2·Δt\n• 时间窗口: halfWindowMs\n  (默认5ms)',
             color='#E8F5E9', border='#388E3C', font=prop_tiny, fontsize=6.5)
    draw_box(ax, 9.3, 2.0, 4.2, 1.5, '像素偏移转换\n━━━━━━━━\n• 小角度近似:\n  offset_x = focalX·θ_y\n  offset_y = focalY·θ_x\n   (focal单位: px/rad)\n• 轴映射 signX/signY\n• maxOffsetPixel钳制',
             color='#E8F5E9', border='#43A047', font=prop_tiny, fontsize=6.5)

    # ── Callback Injection ──
    draw_box(ax, 2.5, 0.3, 5.5, 1.2, 'std::function回调注入 (EIS偏移传播)\n━━━━━━━━━━━━━━━━━━━━\ncalculate_eis_offset(focalX, focalY, targetTs, halfWindowMs, &offsetX, &offsetY)\n→ Visioner.set_eis_offset_callback() 注入采集线程 → RGA letterbox裁剪偏移参数',
             color='#FCE4EC', border='#C62828', font=prop_tiny, fontsize=6.5)
    draw_box(ax, 8.5, 0.3, 5.0, 1.2, 'RGA硬件校正 (单次操作)\n━━━━━━━━━━━━━━\nNV12→640x640 RGB888 Letterbox\n + EIS偏移 (horizontalOffset/\nverticalOffset) → 裁剪+缩放\n+偏移补偿, 单次improcess完成\n延迟 < 5ms',
             color='#FCE4EC', border='#C62828', font=prop_tiny, fontsize=6.5)

    # ── Arrows ──
    draw_arrow(ax, 3.0, 6.4, 3.5, 6.4, C_ARROW, 1.2)
    draw_arrow(ax, 6.0, 6.4, 6.5, 6.4, C_ARROW, 1.2)
    draw_arrow(ax, 9.0, 6.4, 5.0, 5.0, C_ARROW, 1.0)
    draw_arrow(ax, 6.25, 5.0, 2.3, 3.5, C_ARROW, 0.8)
    draw_arrow(ax, 6.25, 5.0, 6.8, 3.5, C_ARROW, 0.8)
    draw_arrow(ax, 2.3, 2.0, 2.3, 1.6, C_ARROW, 0.8)
    draw_arrow(ax, 6.8, 2.0, 6.8, 1.6, C_ARROW, 0.8)
    draw_arrow(ax, 11.4, 2.0, 11.4, 1.6, C_ARROW, 0.8)
    draw_arrow(ax, 5.25, 1.5, 5.25, 1.5, C_ARROW, 0.8)
    draw_arrow(ax, 5.25, 3.8, 2.3, 3.5, C_ARROW, 0.6)
    draw_arrow(ax, 8.5, 3.8, 9.3, 3.5, C_ARROW, 0.6)
    draw_arrow(ax, 4.3, 1.8, 4.8, 2.0, C_ARROW, 0.6)
    draw_arrow(ax, 8.8, 1.8, 9.3, 2.0, C_ARROW, 0.6)

    # Performance
    draw_box(ax, 0.5, 5.0, 2.5, 0.6, '防抖指标\n10-200Hz 平均抑制78%\n延迟 <5ms',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=5.5)

    save_fig(fig, '06_eis_algorithm.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 7: 双编码器流媒体架构图
# ═══════════════════════════════════════════════════════════════
def diagram_streamer():
    fig, ax = plt.subplots(1, 1, figsize=(15, 8.5))
    ax.set_xlim(0, 15)
    ax.set_ylim(0, 8.5)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(7.5, 8.3, 'SentinelStreamer 双编码器流媒体架构 — RTSP推流 + MP4录像 + OSD叠加', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # ── Input ──
    draw_box(ax, 0.3, 7.0, 2.5, 1.0, '输入源\n━━━━\nVisioner推流队列\nNV12 DMA-BUF\n(全分辨率)',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 3.2, 7.0, 2.5, 1.0, 'OSD数据源\n━━━━\nYOLO检测框\n(osdQueue,非阻塞)\nLiDAR点云投影\n(LidarOsdSnapshot)',
             color='#E3F2FD', border='#1565C0', font=prop_small, fontsize=7)
    draw_box(ax, 6.2, 7.0, 2.5, 1.0, 'EIS偏移\n━━━━\nDmaBuffer_t\n.eisOffsetX/Y\n.eisActive标志',
             color='#E3F2FD', border='#E65100', font=prop_small, fontsize=7)

    # ── RecordBufferPool ──
    draw_box(ax, 0.3, 5.5, 8.4, 1.0, 'RecordBufferPool 环形帧缓冲 (RGA imcopy硬件DMA拷贝, 150 Slot ≈ 5秒@30fps) → NvmeWorker消费写入NVMe黑匣子',
             color='#FFF8E1', border='#F9A825', font=prop_small, fontsize=7)
    draw_arrow(ax, 1.55, 7.0, 1.55, 6.5, C_ARROW, 1.0)

    # ── Stream Encoder path (left) ──
    ax.text(2.2, 4.6, '推流链路 — 编码器A (streamEncCtx)', fontproperties=prop_title, fontsize=9, color='#1565C0')

    draw_box(ax, 0.3, 3.2, 3.5, 1.2, 'RGA缩放+EIS\nNV12→1280×720 NV12\nhScale+vScale+EIS偏移\nimprocess IM_SYNC\n(单次硬件操作)',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=6)

    draw_box(ax, 0.3, 1.6, 3.5, 1.3, 'OSD叠加 (CPU)\n━━━━━━━━\n• YOLO框: 2px白边框\n+ 类别标签\n  (3×5点阵字体2x缩放)\n• LiDAR点: 2×2色块\n  红<5m 黄5-15m 青>15m\n+ 距离标签\n→ 写入NV12 Y+UV平面',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=5.5)

    draw_box(ax, 0.3, 0.2, 3.5, 1.1, 'MPP H.264编码\n720p CBR 4Mbps\n→ ffmpeg pipe\n→ RTSP推流 (TCP)\n端到端延迟 ~55ms',
             color='#C8E6C9', border='#2E7D32', font=prop_tiny, fontsize=6)

    # Arrow for stream path
    draw_arrow(ax, 2.05, 4.4, 2.05, 4.35, C_ARROW, 1.5)
    draw_arrow(ax, 2.05, 3.2, 2.05, 2.9, C_ARROW, 1.2)
    draw_arrow(ax, 2.05, 1.6, 2.05, 1.3, C_ARROW, 1.2)

    # ── Record Encoder path (right) ──
    ax.text(8.7, 4.6, '录像链路 — 编码器B (recordEncCtx)', fontproperties=prop_title, fontsize=9, color='#2E7D32')

    draw_box(ax, 5.5, 3.2, 3.5, 1.2, '分辨率决策\n━━━━━━\n1080p: 直录全分辨率原画\n  不经缩放, 8Mbps\n720p: RGA缩放至1280x720\n  (复用推流RGA缩放结果)\n  4Mbps',
             color='#E8F5E9', border='#2E7D32', font=prop_tiny, fontsize=6)
    draw_box(ax, 5.5, 1.6, 3.5, 1.3, 'MPP H.264编码\n1080p@8Mbps CBR 或\n720p@4Mbps CBR\n独立PTS基线\n独立生命周期\n→ FFmpeg MP4 muxing\n→ 本地MP4文件',
             color='#C8E6C9', border='#388E3C', font=prop_tiny, fontsize=6)

    draw_arrow(ax, 7.25, 4.4, 7.25, 4.35, C_ARROW, 1.5)
    draw_arrow(ax, 7.25, 3.2, 7.25, 2.9, C_ARROW, 1.2)

    # ── Shared worker thread ──
    draw_box(ax, 9.5, 5.5, 5.0, 1.8, 'workerThread (每路相机共享)\n━━━━━━━━━━━━━━━━\nstream_thread_func_(ctx):\n1. visioner->wait_get_orig_copy_buffer()\n2. recordPool->write_frame() → 环形缓冲\n3. 跳帧检查 (timestamp < baseline)\n4. rga_scale_nv12_to_720p() → 缩放\n5. OSD draw (调用回调获取最新检测框)\n6. encode_and_mux() → 流编码器 → RTSP\n7. encode_and_mux() → 录编码器 → MP4\n8. release buffers → 循环',
             color='#E8EAF6', border='#283593', font=prop_tiny, fontsize=5.5)

    # ── Control flow ──
    draw_box(ax, 9.5, 3.2, 5.0, 1.5, '双编码器独立控制\n━━━━━━━━━━\n• 独立创建/销毁 (每启停周期重建\n  避免DTS残留)\n• 独立PTS基线 (streamPts / recordPts)\n• 独立生命周期 (推流开/关, 录像开/关,\n  可同时运行互不干扰)\n• 分辨率在线切换 (REST API / Qt界面\n  切换, 无需重启编码器)',
             color='#F3E5F5', border='#7B1FA2', font=prop_tiny, fontsize=5.5)

    draw_box(ax, 9.5, 1.8, 5.0, 1.0, 'OSD动态控制\n━━━━━━\n• OSD开关独立 (per camera)\n  YOLO OSD / LiDAR OSD\n  EIS OSD状态显示\n• 回调机制: StreamOsdProvider\n  + StreamLidarOsdProvider\n  (5ms超时非阻塞轮询)',
             color='#F3E5F5', border='#7B1FA2', font=prop_tiny, fontsize=5.5)

    # Arrows to shared thread
    draw_arrow(ax, 4.7, 7.5, 9.5, 7.0, C_ARROW, 0.8)
    draw_arrow(ax, 12.0, 5.5, 5.0, 4.4, C_ARROW, 0.8)
    draw_arrow(ax, 12.0, 5.5, 8.0, 4.4, C_ARROW, 0.8)

    # Key features box
    draw_box(ax, 9.5, 0.3, 5.0, 1.2, '关键特性\n• DMA-BUF零拷贝: RGA→MPP编码器 via MPP fd import\n• ffmpeg pipe重连: ferror()检测→回调通知→自动重连\n• 推流与录像双编码器独立: 互不干扰同时运行',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=5.5)

    save_fig(fig, '07_streamer_architecture.png')


# ═══════════════════════════════════════════════════════════════
# Diagram 8: dm-ringbox 内核环形黑匣子架构图
# ═══════════════════════════════════════════════════════════════
def diagram_dm_ringbox():
    fig, ax = plt.subplots(1, 1, figsize=(14, 8))
    ax.set_xlim(0, 14)
    ax.set_ylim(0, 8)
    ax.set_aspect('equal')
    ax.axis('off')

    ax.text(7, 7.8, 'dm-ringbox 内核环形黑匣子架构 — Device Mapper环形块设备', ha='center',
            fontproperties=prop_big, fontsize=14, color='#1a237e')

    # ── User Space ──
    ax.text(0.5, 7.2, '用户空间', fontproperties=prop_title, fontsize=9, color='#546E7A')
    draw_box(ax, 0.3, 6.3, 4.0, 0.7, 'NVMeDataManager 写入线程\nO_DIRECT write() → /dev/dm-X\nbio扇区对齐, 512B对齐缓冲区',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=6)
    draw_box(ax, 5.0, 6.3, 3.5, 0.7, 'RecordBufferPool\n环形帧缓冲 (用户态)\n150 Slot NV12 DMA-BUF',
             color='#E3F2FD', border='#1565C0', font=prop_tiny, fontsize=6)
    draw_box(ax, 9.0, 6.3, 4.5, 0.7, '数据格式 (20字节Header)\n[magic 0xDEADBEEF|type|timestamp_ns|data_size|pad→512B对齐]',
             color='#FFF3E0', border='#E65100', font=prop_tiny, fontsize=6)

    # ── Kernel Space border ──
    ax.plot([0.2, 13.8, 13.8, 0.2, 0.2], [5.8, 5.8, 1.3, 1.3, 5.8],
            color='#C62828', lw=2, linestyle='--', zorder=0)
    ax.text(0.5, 5.5, '内核空间 (Kernel Space)', fontproperties=prop_title, fontsize=9, color='#C62828')

    # ── Device Mapper Framework ──
    draw_box(ax, 0.3, 4.5, 5.5, 0.8, 'Linux Device Mapper 框架\ndm_register_target(&ringbox_target) → ringbox_map() / ringbox_status() / ringbox_message()',
             color='#FCE4EC', border='#C62828', font=prop_tiny, fontsize=6)

    # ── Core: ringbox_map() ──
    draw_box(ax, 0.3, 2.8, 6.5, 1.4, 'ringbox_map() — bio扇区重映射核心\n━━━━━━━━━━━━━━━━━━━━\n1. 拦截上层bio请求 (in软中断上下文)\n2. 读取逻辑扇区号: L_logical = bio->bi_iter.bi_sector\n3. 环形映射: P_physical = P_start + (L_logical % C_ring)\n   其中 C_ring = 环容量(扇区数), P_start = NVMe起始扇区\n4. 修改: bio->bi_iter.bi_sector = P_physical\n5. 修改: bio->bi_bdev = nvme_bdev (指向NVMe设备)\n6. 返回 DM_MAPIO_REMAPPED (零数据拷贝,仅改指针)',
             color='#FFEBEE', border='#C62828', font=prop_tiny, fontsize=5.5)

    # ── NVMe Physical View ──
    draw_box(ax, 8.0, 4.5, 5.5, 0.8, 'NVMe SSD 物理扇区视图\n━━━━━━━━━━━━━━\nP_start → [0][1][2]...[C_ring-1]\n环形覆盖: 写指针超过C_ring→从0重新开始',
             color='#E8E5CF', border='#8D6E63', font=prop_tiny, fontsize=6)

    # Ring visualization
    ring_y = 2.5
    for i in range(8):
        x = 8.5 + i * 0.65
        c = '#FFCDD2' if i < 2 else '#C8E6C9' if i >= 6 else '#FFF9C4'
        draw_rect_box(ax, x, ring_y, 0.6, 0.5, str(i), color=c, border='#90A4AE',
                      font=prop_tiny, fontsize=5, linewidth=0.5)

    ax.annotate('写指针 →', xy=(9.8, ring_y+0.25), fontproperties=prop_tiny, fontsize=5, color='#C62828')
    ax.plot([9.8, 9.8], [ring_y+0.5, ring_y], color='#C62828', lw=1)

    # ring statistics
    draw_box(ax, 8.0, 1.8, 5.5, 0.9, 'atomic64_t无锁统计\n• sectors_written / sectors_read\n• wrap_count (环回绕次数)\n• bio_count (bio处理计数)\n• 零后台线程, 事件驱动(软中断)',
             color='#FCE4EC', border='#C62828', font=prop_tiny, fontsize=5.5)

    # ── Arrows ──
    draw_arrow(ax, 2.3, 6.3, 2.3, 5.8, '#C62828', 1.5)
    draw_arrow(ax, 2.3, 4.5, 2.3, 4.2, '#C62828', 1.2)

    # ── iSCSI / Export path ──
    draw_box(ax, 0.3, 1.3, 4.5, 1.0, '黑匣子回溯导出\n━━━━━━━━\n• NVMeDataManager读取\n  dm-ringbox逻辑块设备\n• 按时间戳线性扫描\n  (magic 0xDEADBEEF同步)\n• 收集时间窗口内帧数据\n• MPP硬件编码→MP4导出',
             color='#E1F5FE', border='#0277BD', font=prop_tiny, fontsize=6)
    draw_arrow(ax, 3.0, 2.8, 3.0, 2.3, '#0277BD', 1.0)

    # ── Advantages ──
    draw_box(ax, 5.5, 1.3, 8.0, 1.5, 'dm-ringbox 核心优势\n━━━━━━━━━━━━━━\n• 零数据拷贝: 仅修改bio->bi_iter.bi_sector和bio->bi_bdev两个指针 (共24字节)\n• 软中断上下文同步: 无内核线程, 事件驱动, CPU空闲时功耗为零\n• WAF=1.0: 环形裸块写入, 无文件系统元数据更新, 无日志写放大 (vs ext4 WAF=2.1)\n• 尾部延迟0.48ms: 环形映射消除寻道延迟, O_DIRECT直通无页缓存抖动 (vs ext4 12.8ms)\n• 写带宽208MB/s: 仅比裸NVMe低3.2% (215→208), 远优于ext4 178MB/s',
             color='#ECEFF1', border='#607D8B', font=prop_tiny, fontsize=5.5)

    save_fig(fig, '08_dm_ringbox.png')


# ═══════════════════════════════════════════════════════════════
# Main
# ═══════════════════════════════════════════════════════════════
if __name__ == '__main__':
    print("Generating diagrams for Section 2.3.2 ...")
    diagram_overall_architecture()
    diagram_visioner_fanout()
    diagram_npu_inference()
    diagram_lidar_architecture()
    diagram_fusion_tracking()
    diagram_eis()
    diagram_streamer()
    diagram_dm_ringbox()
    print("\nAll 8 diagrams generated successfully!")
    print(f"Output directory: {OUTPUT_DIR}")
