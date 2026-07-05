#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Insert generated diagrams into qianrushi.docx using lxml for proper namespace handling.
"""

import os
import re
import shutil
import zipfile
import tempfile
from lxml import etree

DOCX_PATH = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi.docx'
BACKUP_PATH = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi_backup.docx'
DIAGRAMS_DIR = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\diagrams'
OUTPUT_PATH = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi_with_diagrams.docx'

NAMESPACES = {
    'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
    'r': 'http://schemas.openxmlformats.org/officeDocument/2006/relationships',
    'wp': 'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing',
    'a': 'http://schemas.openxmlformats.org/drawingml/2006/main',
    'pic': 'http://schemas.openxmlformats.org/drawingml/2006/picture',
    'mc': 'http://schemas.openxmlformats.org/markup-compatibility/2006',
    'ct': 'http://schemas.openxmlformats.org/package/2006/content-types',
    'rel': 'http://schemas.openxmlformats.org/package/2006/relationships',
}

DIAGRAM_MAP = [
    ('01_overall_architecture.png', '软件系统总体架构图', 14.0, 10.0),
    ('02_visioner_fanout.png', 'SentinelVisioner数据流图', 14.0, 7.0),
    ('03_npu_inference.png', 'RKNN NPU DMA零拷贝推理流程', 14.0, 8.0),
    ('04_lidar_architecture.png', 'LiDAR驱动三层架构流程', 14.0, 7.5),
    ('05_fusion_tracking.png', 'LiDAR-Camera融合跟踪算法流程', 16.0, 9.0),
    ('06_eis_algorithm.png', 'EIS电子防抖算法流程', 14.0, 7.5),
    ('07_streamer_architecture.png', '双编码器流媒体架构', 15.0, 8.5),
    ('08_dm_ringbox.png', 'dm-ringbox内核环形黑匣子架构', 14.0, 8.0),
]


def EMU(cm):
    return int(cm * 360000)


def create_drawing_xml(rId, filename, w_cm, h_cm):
    """Build a w:drawing element as an XML string (simpler and reliable)."""
    w_emu = EMU(w_cm)
    h_emu = EMU(h_cm)
    pid = abs(hash(rId + filename)) % (2 ** 31)

    return f'''<w:drawing xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main">
      <wp:inline xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"
                 distT="0" distB="0" distL="0" distR="0">
        <wp:extent cx="{w_emu}" cy="{h_emu}"/>
        <wp:effectExtent l="0" t="0" r="0" b="0"/>
        <wp:docPr id="{pid}" name="Picture_{rId}" descr="{filename}"/>
        <wp:cNvGraphicFramePr>
          <a:graphicFrameLocks xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"
                               noChangeAspect="1"/>
        </wp:cNvGraphicFramePr>
        <a:graphic xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main">
          <a:graphicData uri="http://schemas.openxmlformats.org/drawingml/2006/picture">
            <pic:pic xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture">
              <pic:nvPicPr>
                <pic:cNvPr id="0" name="{filename}"/>
                <pic:cNvPicPr/>
              </pic:nvPicPr>
              <pic:blipFill>
                <a:blip r:embed="{rId}" xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"/>
                <a:stretch>
                  <a:fillRect/>
                </a:stretch>
              </pic:blipFill>
              <pic:spPr>
                <a:xfrm>
                  <a:off x="0" y="0"/>
                  <a:ext cx="{w_emu}" cy="{h_emu}"/>
                </a:xfrm>
                <a:prstGeom prst="rect">
                  <a:avLst/>
                </a:prstGeom>
              </pic:spPr>
            </pic:pic>
          </a:graphicData>
        </a:graphic>
      </wp:inline>
    </w:drawing>'''


def find_placeholder(body, keyword):
    """Find paragraph index containing the placeholder keyword."""
    W = NAMESPACES['w']
    for i, p in enumerate(body.findall(f'{{{W}}}p')):
        text = ''.join(p.itertext()).strip()
        if keyword in text and '此处插入图片' in text:
            return i, p
    return None, None


def process_docx():
    # Create backup
    if not os.path.exists(BACKUP_PATH):
        shutil.copy2(DOCX_PATH, BACKUP_PATH)
        print(f'Backup: {BACKUP_PATH}')

    temp_dir = tempfile.mkdtemp()
    print(f'Temp: {temp_dir}')

    try:
        # Extract docx
        with zipfile.ZipFile(DOCX_PATH, 'r') as z:
            z.extractall(temp_dir)

        W = NAMESPACES['w']

        # Parse document.xml
        doc_path = os.path.join(temp_dir, 'word', 'document.xml')
        doc_tree = etree.parse(doc_path)
        doc_root = doc_tree.getroot()
        body = doc_root.find(f'{{{W}}}body')

        # Parse relationships
        rels_path = os.path.join(temp_dir, 'word', '_rels', 'document.xml.rels')
        rels_tree = etree.parse(rels_path)
        rels_root = rels_tree.getroot()
        REL_NS = NAMESPACES['rel']

        # Find next rId
        existing_ids = []
        for rel in rels_root:
            rid = rel.get('Id', '')
            m = re.match(r'rId(\d+)', rid)
            if m:
                existing_ids.append(int(m.group(1)))
        next_rid = max(existing_ids) + 1 if existing_ids else 1

        # Create media dir
        media_dir = os.path.join(temp_dir, 'word', 'media')
        os.makedirs(media_dir, exist_ok=True)

        # Parse [Content_Types].xml
        ct_path = os.path.join(temp_dir, '[Content_Types].xml')
        ct_tree = etree.parse(ct_path)
        ct_root = ct_tree.getroot()
        CT_NS = NAMESPACES['ct']

        # Ensure PNG default type
        png_exists = any(
            d.get('Extension') == 'png'
            for d in ct_root.findall(f'{{{CT_NS}}}Default')
        )
        if not png_exists:
            d = etree.SubElement(ct_root, f'{{{CT_NS}}}Default')
            d.set('Extension', 'png')
            d.set('ContentType', 'image/png')

        inserted = 0
        # We need to track index shifts because we're modifying body in-place
        # Process from last to first to avoid index issues
        replacements = []

        for img_file, keyword, w_cm, h_cm in DIAGRAM_MAP:
            idx, para = find_placeholder(body, keyword)
            if idx is None:
                print(f'  WARNING: "{keyword}" placeholder not found, skipping')
                continue
            replacements.append((idx, img_file, keyword, w_cm, h_cm))
            print(f'  Found placeholder for "{keyword}" at paragraph index {idx}')

        # Sort by index descending so we can replace without affecting earlier indices
        replacements.sort(key=lambda x: x[0], reverse=True)

        for idx, img_file, keyword, w_cm, h_cm in replacements:
            # Copy image
            img_src = os.path.join(DIAGRAMS_DIR, img_file)
            img_dst_name = f'image_{next_rid}.png'
            img_dst = os.path.join(media_dir, img_dst_name)
            shutil.copy2(img_src, img_dst)

            # Add relationship
            rel = etree.SubElement(rels_root, f'{{{REL_NS}}}Relationship')
            rel.set('Id', f'rId{next_rid}')
            rel.set('Type', 'http://schemas.openxmlformats.org/officeDocument/2006/relationships/image')
            rel.set('Target', f'media/{img_dst_name}')

            # Get the placeholder paragraph
            all_paras = body.findall(f'{{{W}}}p')
            placeholder_elem = all_paras[idx]

            # Build drawing XML and parse it
            drawing_str = create_drawing_xml(f'rId{next_rid}', img_file, w_cm, h_cm)
            drawing_elem = etree.fromstring(drawing_str)

            # Create new paragraph with centered image
            new_para = etree.SubElement(body, f'{{{W}}}p')
            new_para.tail = placeholder_elem.tail  # Preserve text after placeholder

            pPr = etree.SubElement(new_para, f'{{{W}}}pPr')
            jc = etree.SubElement(pPr, f'{{{W}}}jc')
            jc.set(f'{{{W}}}val', 'center')
            spacing = etree.SubElement(pPr, f'{{{W}}}spacing')
            spacing.set(f'{{{W}}}before', '120')
            spacing.set(f'{{{W}}}after', '120')

            run = etree.SubElement(new_para, f'{{{W}}}r')
            run.append(drawing_elem)

            # Create caption paragraph
            caption_para = etree.SubElement(body, f'{{{W}}}p')
            caption_para.tail = new_para.tail
            new_para.tail = None

            cap_pPr = etree.SubElement(caption_para, f'{{{W}}}pPr')
            cap_jc = etree.SubElement(cap_pPr, f'{{{W}}}jc')
            cap_jc.set(f'{{{W}}}val', 'center')
            cap_spacing = etree.SubElement(cap_pPr, f'{{{W}}}spacing')
            cap_spacing.set(f'{{{W}}}after', '200')

            cap_run = etree.SubElement(caption_para, f'{{{W}}}r')
            cap_rPr = etree.SubElement(cap_run, f'{{{W}}}rPr')
            cap_sz = etree.SubElement(cap_rPr, f'{{{W}}}sz')
            cap_sz.set(f'{{{W}}}val', '18')
            cap_color = etree.SubElement(cap_rPr, f'{{{W}}}color')
            cap_color.set(f'{{{W}}}val', '808080')
            cap_t = etree.SubElement(cap_run, f'{{{W}}}t')
            cap_t.set('{http://www.w3.org/XML/1998/namespace}space', 'preserve')
            cap_t.text = f'图{inserted + 1} {keyword}'

            # Remove the placeholder
            body.remove(placeholder_elem)

            print(f'  Inserted: {img_file} -> rId{next_rid}')
            next_rid += 1
            inserted += 1

        # Save modified files
        doc_tree.write(doc_path, xml_declaration=True, encoding='UTF-8', standalone=True)
        rels_tree.write(rels_path, xml_declaration=True, encoding='UTF-8', standalone=True)
        ct_tree.write(ct_path, xml_declaration=True, encoding='UTF-8', standalone=True)

        # Repackage
        with zipfile.ZipFile(OUTPUT_PATH, 'w', zipfile.ZIP_DEFLATED) as zout:
            for root_dir, dirs, files in os.walk(temp_dir):
                for file in files:
                    full_path = os.path.join(root_dir, file)
                    arcname = os.path.relpath(full_path, temp_dir)
                    zout.write(full_path, arcname)

        print(f'\nDone! Inserted {inserted}/{len(DIAGRAM_MAP)} diagrams.')
        print(f'Output: {OUTPUT_PATH}')

    finally:
        shutil.rmtree(temp_dir, ignore_errors=True)


if __name__ == '__main__':
    process_docx()
