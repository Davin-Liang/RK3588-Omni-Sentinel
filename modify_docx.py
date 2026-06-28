#!/usr/bin/env python3
"""
Modify qianrushi.docx:
1. Abstract: Add 1080p/720p recording + QT/Web mutual control (within 800 chars)
2. Section 1.1: Expand recording and dual terminal features
3. Section 1.3: Add technical feature points
4. Section 2.3.1: Add QT/Web mutual control mechanism details
5. Section 2.3.2: Update SentinelStreamer description
"""

import os
import shutil
import zipfile
from lxml import etree

def get_paragraph_text(p_elem):
    texts = []
    for t in p_elem.iter('{http://schemas.openxmlformats.org/wordprocessingml/2006/main}t'):
        if t.text:
            texts.append(t.text)
    return ''.join(texts)

def set_paragraph_text(p_elem, new_text):
    ns = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
    runs = p_elem.findall(f'{{{ns}}}r')
    if not runs:
        return False
    first_run = runs[0]
    for run in runs[1:]:
        p_elem.remove(run)
    t_elem = first_run.find(f'{{{ns}}}t')
    if t_elem is not None:
        t_elem.text = new_text
        t_elem.set('{http://www.w3.org/XML/1998/namespace}space', 'preserve')
    else:
        t_elem = etree.SubElement(first_run, f'{{{ns}}}t')
        t_elem.text = new_text
        t_elem.set('{http://www.w3.org/XML/1998/namespace}space', 'preserve')
    return True

def clone_paragraph(original_p):
    return etree.fromstring(etree.tostring(original_p))

def main():
    src_docx = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi.docx'
    tmp_dir = r'C:\Users\bcq\AppData\Local\Temp\docx_modify'

    if os.path.exists(tmp_dir):
        shutil.rmtree(tmp_dir)
    os.makedirs(tmp_dir)

    with zipfile.ZipFile(src_docx, 'r') as z:
        z.extractall(tmp_dir)

    doc_path = os.path.join(tmp_dir, 'word', 'document.xml')
    parser = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(doc_path, parser)
    root = tree.getroot()

    ns = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
    body = root.find(f'{{{ns}}}body')
    paragraphs = body.findall(f'{{{ns}}}p')

    print(f"Total paragraphs: {len(paragraphs)}")

    # ============================================================
    # EDIT 1: Abstract - Trim P4-P7, rewrite P8 (target < 800 chars)
    # Original: P4(195)+P5(167)+P6(152)+P7(160)+P8(107)=781
    # ============================================================

    # P4: Remove "（HRC）", "Rockchip ", "All-in-One ", "完整的 " -> save ~26
    p4 = paragraphs[4]
    new_p4 = get_paragraph_text(p4).replace("（HRC）", "").replace("Rockchip ", "").replace("All-in-One ", "").replace("完整的 ", "")
    set_paragraph_text(p4, new_p4)

    # P5: Trim Madgwick description, delay, RGA prefix -> save ~24
    p5 = paragraphs[5]
    new_p5 = get_paragraph_text(p5).replace(
        "通过Madgwick AHRS四元数姿态估计与陀螺仪梯形积分算法",
        "通过Madgwick AHRS姿态估计与梯形积分"
    ).replace("Rockchip ", "").replace("延迟低于5ms", "延迟<5ms").replace(
        "在10-200Hz范围内平均防抖抑制比达78%",
        "10-200Hz平均抑制比78%"
    )
    set_paragraph_text(p5, new_p5)

    # P6: Minor trim -> save ~6
    p6 = paragraphs[6]
    new_p6 = get_paragraph_text(p6).replace(
        "通过两遍融合算法与四态生命周期管理",
        "通过两遍融合与四态管理"
    ).replace("告警响应<100ms", "告警<100ms")
    set_paragraph_text(p6, new_p6)

    # P7: Minor trim -> save ~10
    p7 = paragraphs[7]
    new_p7 = get_paragraph_text(p7).replace(
        "在Linux内核块设备层设计并实现了", "在内核块设备层实现了"
    ).replace("（WAF 1.0 vs ext4的2.1）", "（WAF 1.0 vs ext4 2.1）").replace(
        "持续写入带宽208MB/s", "写带宽208MB/s"
    )
    set_paragraph_text(p7, new_p7)

    # P8: Compact version (~184 chars target, fits within 800 total)
    p8 = paragraphs[8]
    new_p8 = (
        "系统基于MPP硬件编码实现RTSP推流（~55ms）与1080p/720p双分辨率MP4录像"
        "（1080p@8Mbps直录/720p@4Mbps RGA缩放），推流与录像双编码器独立可同时"
        "运行。系统提供Qt5触控HMI与Web远程控制双终端，通过REST API与WebSocket"
        "实现双向互控与状态实时同步。全链路DMA-BUF零拷贝架构充分挖掘RK3588异构算力。"
    )
    set_paragraph_text(p8, new_p8)

    # Verify abstract char count
    total = 0
    for i in [4, 5, 6, 7, 8]:
        t = get_paragraph_text(paragraphs[i])
        total += len(t)
        print(f"P{i}: {len(t)} chars")
    print(f"Abstract total: {total} chars\n")

    # ============================================================
    # EDIT 2: Section 1.1 (P12)
    # ============================================================
    p12 = paragraphs[12]
    old_p12 = get_paragraph_text(p12)
    new_p12 = old_p12.replace(
        "视频推流与录像——MPP硬件H.264编码，RTSP推流延迟55ms，支持OSD叠加（检测框+雷达点云），本地MP4录像",
        "视频推流与录像——MPP硬件H.264编码，RTSP推流延迟55ms，支持OSD叠加（检测框+雷达点云），"
        "本地MP4录像支持1080p/720p双分辨率可切换（1080p@8Mbps直录/720p@4Mbps RGA缩放），"
        "推流与录像双编码器独立可同时运行互不干扰"
    )
    new_p12 = new_p12.replace(
        "双终端管控——嵌入式Qt5触控HMI + Web远程控制（27+ REST API，WebSocket实时推送）",
        "双终端管控——嵌入式Qt5触控HMI + Web远程控制，Qt内嵌HTTP/WebSocket服务器，"
        "通过27+ REST API与WebSocket实时推送实现双向互控与状态实时同步"
    )
    set_paragraph_text(p12, new_p12)
    print("P12 (1.1) updated.")

    # ============================================================
    # EDIT 3: Section 1.3 - Insert new paragraphs after P28
    # ============================================================
    p28 = paragraphs[28]
    new_p_stream = clone_paragraph(p28)
    set_paragraph_text(new_p_stream,
        "双分辨率录像与双编码器架构：系统基于RK3588 MPP硬件H.264编码器，实现RTSP推流与本地"
        "MP4录像的双编码器独立架构。录像支持1080p（1920×1080@8Mbps）与720p（1280×720@4Mbps）"
        "双分辨率动态切换，1080p录像不经缩放直接编码以确保最高画质，720p录像复用推流链路的"
        "RGA缩放结果以节省硬件资源，双编码器独立PTS基线、独立生命周期管理，推流与录像可同时"
        "运行互不干扰。"
    )
    new_p_qt = clone_paragraph(p28)
    set_paragraph_text(new_p_qt,
        "双终端互控架构：系统提供嵌入式Qt5触控HMI（SentinelQT）与Web远程控制SPA（WebControl）"
        "两套界面，共用同一后端11组件软件栈。Qt进程内嵌基于cpp-httplib的HTTP/WebSocket服务器，"
        "Web端控制指令经QMetaObject::invokeMethod（BlockingQueuedConnection）跨线程同步派发至"
        "Qt GUI主线程执行并返回结果；Qt端状态变更通过消息队列以WebSocket实时广播至所有连接"
        "客户端，确保双终端操作状态实时同步、双向互控无缝切换。"
    )
    p29 = paragraphs[29]
    p29_parent = p29.getparent()
    p29_index = list(p29_parent).index(p29)
    p29_parent.insert(p29_index, new_p_stream)
    p29_parent.insert(p29_index + 1, new_p_qt)
    print("Inserted 2 new paragraphs after P28 (before 1.4).")

    # ============================================================
    # EDIT 4: Section 2.3.1 - Add QT/Web mutual control mechanism
    # ============================================================
    paragraphs_updated = body.findall(f'{{{ns}}}p')
    for i, p in enumerate(paragraphs_updated):
        text = get_paragraph_text(p)
        if text.startswith("本系统提供两套用户界面"):
            p123 = p
            old_p123 = text
            break

    new_p123 = old_p123.replace(
        "前端SPA支持桌面和移动端浏览器。系统运行时配置通过config.ini",
        "前端SPA支持桌面和移动端浏览器。两套界面之间的互控通过以下机制实现："
        "Web→Qt方向，HTTP请求到达WebServer工作线程后，通过QMetaObject::invokeMethod"
        "（Qt::BlockingQueuedConnection）跨线程同步派发至Qt GUI主线程执行，确保Widget"
        "操作线程安全并返回执行结果；Qt→Web方向，Qt主线程通过push_status/push_event/"
        "push_tracking三个非阻塞接口将状态JSON推入消息队列（std::queue+mutex），WebServer"
        "内部50ms定时器排空队列并通过WebSocket广播至所有连接客户端。该双向通道确保触控操作"
        "与远程浏览器操作的状态实时同步、无缝切换。系统运行时配置通过config.ini"
    )
    set_paragraph_text(p123, new_p123)
    print("P123 (2.3.1) updated.")

    # ============================================================
    # EDIT 5: Section 2.3.2 (7) SentinelStreamer
    # ============================================================
    for i, p in enumerate(paragraphs_updated):
        text = get_paragraph_text(p)
        if text.startswith("（7）SentinelStreamer（流媒体层）"):
            p156 = p
            old_p156 = text
            break

    new_p156 = old_p156.replace(
        "内部：双编码器独立架构（流720p CBR 2Mbps含OSD，录720p/1080p CBR 4Mbps纯净），"
        "MPP硬件H.264编码（h264_rkmpp），RGA单次裁剪缩放+EIS偏移，FFmpeg管道推流+MP4 muxing，"
        "RecordBufferPool RGA硬件拷贝环形缓冲（150 Slot，约5秒@30fps）。",
        "内部：双编码器独立架构——流编码器固定720p CBR 4Mbps（含OSD检测框与LiDAR点云叠加），"
        "录编码器支持1080p/720p动态切换（1080p@8Mbps直录全分辨率原画/720p@4Mbps经RGA硬件"
        "缩放后编码），两个编码器独立创建、独立PTS基线、可同时运行互不干扰，录制分辨率通过"
        "REST API或Qt界面在线切换无需重启。MPP硬件H.264编码（h264_rkmpp），RGA单次裁剪缩放"
        "+EIS偏移，FFmpeg管道推流+MP4 muxing，RecordBufferPool RGA硬件拷贝环形缓冲"
        "（150 Slot，约5秒@30fps）。"
    )
    set_paragraph_text(p156, new_p156)
    print("P156 (SentinelStreamer) updated.")

    # ============================================================
    # Save and re-pack
    # ============================================================
    tree.write(doc_path, xml_declaration=True, encoding='UTF-8', standalone=True)
    output_path = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi.docx'
    os.remove(output_path)
    with zipfile.ZipFile(output_path, 'w', zipfile.ZIP_DEFLATED) as zout:
        for root_dir, dirs, files in os.walk(tmp_dir):
            for file in files:
                full_path = os.path.join(root_dir, file)
                arcname = os.path.relpath(full_path, tmp_dir)
                zout.write(full_path, arcname)
    print(f"\nNew docx written to {output_path}")
    print("All edits complete!")

if __name__ == '__main__':
    main()
