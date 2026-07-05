#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Insert generated diagrams into qianrushi.docx at the correct placeholder positions
Uses direct XML manipulation (no python-docx required)
"""

import os
import re
import shutil
import zipfile
import tempfile
import xml.etree.ElementTree as ET
from copy import deepcopy

DOCX_PATH = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi.docx'
BACKUP_PATH = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi_backup.docx'
DIAGRAMS_DIR = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\diagrams'
OUTPUT_PATH = r'C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi_with_diagrams.docx'

# Image-to-placeholder mapping (ordered by appearance in 2.3.2)
DIAGRAM_MAP = [
    # (image_filename, placeholder_keyword, width_cm, height_cm)
    ('01_overall_architecture.png', '软件系统总体架构图', 14.0, 10.0),
    ('02_visioner_fanout.png', 'SentinelVisioner数据流图', 14.0, 7.0),
    ('03_npu_inference.png', 'RKNN NPU DMA零拷贝推理流程', 14.0, 8.0),
    ('04_lidar_architecture.png', 'LiDAR驱动三层架构流程', 14.0, 7.5),
    ('05_fusion_tracking.png', 'LiDAR-Camera融合跟踪算法流程', 16.0, 9.0),
    ('06_eis_algorithm.png', 'EIS电子防抖算法流程', 14.0, 7.5),
    ('07_streamer_architecture.png', '双编码器流媒体架构', 15.0, 8.5),
    ('08_dm_ringbox.png', 'dm-ringbox内核环形黑匣子架构', 14.0, 8.0),
]

NSMAP = {
    'w': 'http://schemas.openxmlformats.org/wordprocessingml/2006/main',
    'r': 'http://schemas.openxmlformats.org/officeDocument/2006/relationships',
    'wp': 'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing',
    'a': 'http://schemas.openxmlformats.org/drawingml/2006/main',
    'pic': 'http://schemas.openxmlformats.org/drawingml/2006/picture',
    'mc': 'http://schemas.openxmlformats.org/markup-compatibility/2006',
    'wps': 'http://schemas.microsoft.com/office/word/2010/wordprocessingShape',
    'wpg': 'http://schemas.microsoft.com/office/word/2010/wordprocessingGroup',
    'wp14': 'http://schemas.microsoft.com/office/word/2010/wordprocessingDrawing',
    'a14': 'http://schemas.microsoft.com/office/drawing/2010/main',
}

for prefix, uri in NSMAP.items():
    ET.register_namespace(prefix, uri)


def EMU(cm):
    """Convert cm to EMU (English Metric Units)"""
    return int(cm * 360000)


def create_image_drawing(rId, filename, w_cm, h_cm):
    """Create a w:drawing element for inline image using ElementTree API."""
    W = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
    WP = 'http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing'
    A = 'http://schemas.openxmlformats.org/drawingml/2006/main'
    PIC = 'http://schemas.openxmlformats.org/drawingml/2006/picture'
    R = 'http://schemas.openxmlformats.org/officeDocument/2006/relationships'

    w_emu = EMU(w_cm)
    h_emu = EMU(h_cm)
    pic_id = abs(hash(rId + filename)) % (2**31)

    # Build from bottom up
    # w:drawing
    drawing = ET.Element(f'{{{W}}}drawing')

    # wp:inline
    inline = ET.SubElement(drawing, f'{{{WP}}}inline')
    inline.set('distT', '0')
    inline.set('distB', '0')
    inline.set('distL', '0')
    inline.set('distR', '0')

    extent = ET.SubElement(inline, f'{{{WP}}}extent')
    extent.set('cx', str(w_emu))
    extent.set('cy', str(h_emu))

    effectExtent = ET.SubElement(inline, f'{{{WP}}}effectExtent')
    effectExtent.set('l', '0')
    effectExtent.set('t', '0')
    effectExtent.set('r', '0')
    effectExtent.set('b', '0')

    docPr = ET.SubElement(inline, f'{{{WP}}}docPr')
    docPr.set('id', str(pic_id))
    docPr.set('name', f'Picture_{rId}')
    docPr.set('descr', filename)

    cNvGraphicFramePr = ET.SubElement(inline, f'{{{WP}}}cNvGraphicFramePr')
    graphicFrameLocks = ET.SubElement(cNvGraphicFramePr, f'{{{A}}}graphicFrameLocks')
    graphicFrameLocks.set('noChangeAspect', '1')

    # a:graphic
    graphic = ET.SubElement(inline, f'{{{A}}}graphic')
    graphicData = ET.SubElement(graphic, f'{{{A}}}graphicData')
    graphicData.set('uri', 'http://schemas.openxmlformats.org/drawingml/2006/picture')

    # pic:pic
    pic = ET.SubElement(graphicData, f'{{{PIC}}}pic')
    nvPicPr = ET.SubElement(pic, f'{{{PIC}}}nvPicPr')
    cNvPr = ET.SubElement(nvPicPr, f'{{{PIC}}}cNvPr')
    cNvPr.set('id', '0')
    cNvPr.set('name', filename)
    cNvPicPr = ET.SubElement(nvPicPr, f'{{{PIC}}}cNvPicPr')

    blipFill = ET.SubElement(pic, f'{{{PIC}}}blipFill')
    blip = ET.SubElement(blipFill, f'{{{A}}}blip')
    blip.set(f'{{{R}}}embed', rId)
    stretch = ET.SubElement(blipFill, f'{{{A}}}stretch')
    fillRect = ET.SubElement(stretch, f'{{{A}}}fillRect')

    spPr = ET.SubElement(pic, f'{{{PIC}}}spPr')
    xfrm = ET.SubElement(spPr, f'{{{A}}}xfrm')
    off = ET.SubElement(xfrm, f'{{{A}}}off')
    off.set('x', '0')
    off.set('y', '0')
    ext = ET.SubElement(xfrm, f'{{{A}}}ext')
    ext.set('cx', str(w_emu))
    ext.set('cy', str(h_emu))
    prstGeom = ET.SubElement(spPr, f'{{{A}}}prstGeom')
    prstGeom.set('prst', 'rect')
    avLst = ET.SubElement(prstGeom, f'{{{A}}}avLst')

    return drawing


def find_placeholder_paragraph(body, keyword):
    """Find a paragraph containing the given keyword in placeholder brackets."""
    for i, p in enumerate(body):
        text = ''.join(p.itertext()).strip()
        if keyword in text and '此处插入图片' in text:
            return i, p
    return None, None


def process_docx():
    """Main processing function."""

    # Create a backup first
    if not os.path.exists(BACKUP_PATH):
        shutil.copy2(DOCX_PATH, BACKUP_PATH)
        print(f'Backup created: {BACKUP_PATH}')

    # Create temp directory
    temp_dir = tempfile.mkdtemp()
    print(f'Working in: {temp_dir}')

    try:
        # Extract docx
        with zipfile.ZipFile(DOCX_PATH, 'r') as z:
            z.extractall(temp_dir)

        # Read document.xml
        doc_xml_path = os.path.join(temp_dir, 'word', 'document.xml')
        ET.register_namespace('', 'http://schemas.openxmlformats.org/wordprocessingml/2006/main')
        tree = ET.parse(doc_xml_path)
        root = tree.getroot()

        w_ns = 'http://schemas.openxmlformats.org/wordprocessingml/2006/main'
        body = root.find(f'{{{w_ns}}}body')

        # Read relationships
        rels_path = os.path.join(temp_dir, 'word', '_rels', 'document.xml.rels')
        rels_tree = ET.parse(rels_path)
        rels_root = rels_tree.getroot()
        rels_ns = 'http://schemas.openxmlformats.org/package/2006/relationships'

        # Determine next rId
        existing_ids = []
        for rel in rels_root:
            rid = rel.get('Id')
            if rid:
                existing_ids.append(int(rid.replace('rId', '')))
        next_rid = max(existing_ids) + 1 if existing_ids else 1

        # Create media directory
        media_dir = os.path.join(temp_dir, 'word', 'media')
        os.makedirs(media_dir, exist_ok=True)

        # Read [Content_Types].xml
        content_types_path = os.path.join(temp_dir, '[Content_Types].xml')
        ct_tree = ET.parse(content_types_path)
        ct_root = ct_tree.getroot()
        ct_ns = 'http://schemas.openxmlformats.org/package/2006/content-types'

        # Ensure PNG content type exists
        png_exists = False
        for default in ct_root.findall(f'{{{ct_ns}}}Default'):
            if default.get('Extension') == 'png':
                png_exists = True
                break
        if not png_exists:
            png_default = ET.SubElement(ct_root, f'{{{ct_ns}}}Default')
            png_default.set('Extension', 'png')
            png_default.set('ContentType', 'image/png')

        inserted = 0
        for img_file, keyword, w_cm, h_cm in DIAGRAM_MAP:
            idx, para = find_placeholder_paragraph(body, keyword)
            if idx is None:
                # Try with partial keyword
                print(f'  WARNING: Placeholder for "{keyword}" not found, trying partial match...')
                idx, para = find_placeholder_paragraph(body, keyword[:6])
            if idx is None:
                print(f'  WARNING: Cannot find placeholder for "{keyword}", skipping')
                continue

            # Copy image to media folder
            img_src = os.path.join(DIAGRAMS_DIR, img_file)
            img_dst_name = f'image_{next_rid}.png'
            img_dst = os.path.join(media_dir, img_dst_name)
            shutil.copy2(img_src, img_dst)

            # Add relationship (must use the same namespace as existing rels)
            rel_ns = rels_root.tag.split('}')[0].strip('{') if '}' in rels_root.tag else rels_ns
            rel = ET.SubElement(rels_root, f'{{{rel_ns}}}Relationship')
            rel.set('Id', f'rId{next_rid}')
            rel.set('Type', 'http://schemas.openxmlformats.org/officeDocument/2006/relationships/image')
            rel.set('Target', f'media/{img_dst_name}')

            # Create a new paragraph with the image centered
            new_para = ET.Element(f'{{{w_ns}}}p')
            pPr = ET.SubElement(new_para, f'{{{w_ns}}}pPr')
            jc = ET.SubElement(pPr, f'{{{w_ns}}}jc')
            jc.set(f'{{{w_ns}}}val', 'center')

            # Add spacing after image
            spacing = ET.SubElement(pPr, f'{{{w_ns}}}spacing')
            spacing.set(f'{{{w_ns}}}before', '120')
            spacing.set(f'{{{w_ns}}}after', '120')

            # Add run with drawing
            run = ET.SubElement(new_para, f'{{{w_ns}}}r')
            drawing_elem = create_image_drawing(f'rId{next_rid}', img_file, w_cm, h_cm)
            run.append(drawing_elem)

            # Add caption paragraph
            caption_para = ET.Element(f'{{{w_ns}}}p')
            cap_pPr = ET.SubElement(caption_para, f'{{{w_ns}}}pPr')
            cap_jc = ET.SubElement(cap_pPr, f'{{{w_ns}}}jc')
            cap_jc.set(f'{{{w_ns}}}val', 'center')
            cap_spacing = ET.SubElement(cap_pPr, f'{{{w_ns}}}spacing')
            cap_spacing.set(f'{{{w_ns}}}after', '200')

            cap_run = ET.SubElement(caption_para, f'{{{w_ns}}}r')
            cap_rPr = ET.SubElement(cap_run, f'{{{w_ns}}}rPr')
            cap_sz = ET.SubElement(cap_rPr, f'{{{w_ns}}}sz')
            cap_sz.set(f'{{{w_ns}}}val', '18')  # 9pt
            cap_color = ET.SubElement(cap_rPr, f'{{{w_ns}}}color')
            cap_color.set(f'{{{w_ns}}}val', '808080')
            cap_t = ET.SubElement(cap_run, f'{{{w_ns}}}t')
            cap_t.set('{http://www.w3.org/XML/1998/namespace}space', 'preserve')
            cap_t.text = f'图{inserted+1} {keyword}'

            # Replace placeholder paragraph with image + caption
            parent = body
            body_list = list(body)
            placeholder_elem = body_list[idx]

            # Remove placeholder
            parent.remove(placeholder_elem)

            # Insert new elements at the same position
            parent.insert(idx, caption_para)
            parent.insert(idx, new_para)

            print(f'  Inserted: {img_file} -> rId{next_rid} (replacing "{keyword}")')
            next_rid += 1
            inserted += 1

        # Save modified XML
        # Write document.xml with proper namespace declarations

        # Need to fix namespace registration for proper output
        doc_xml_str = ET.tostring(root, encoding='unicode')
        # Manually add namespace declarations
        doc_xml_str = doc_xml_str.replace(
            '<ns0:document xmlns:ns0="http://schemas.openxmlformats.org/wordprocessingml/2006/main"',
            '<w:document xmlns:w="http://schemas.openxmlformats.org/wordprocessingml/2006/main"'
            ' xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships"'
            ' xmlns:wp="http://schemas.openxmlformats.org/drawingml/2006/wordprocessingDrawing"'
            ' xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main"'
            ' xmlns:pic="http://schemas.openxmlformats.org/drawingml/2006/picture"'
            ' xmlns:mc="http://schemas.openxmlformats.org/markup-compatibility/2006"'
        )
        doc_xml_str = doc_xml_str.replace('ns0:', 'w:')
        doc_xml_str = doc_xml_str.replace('ns1:', 'r:')
        doc_xml_str = doc_xml_str.replace('ns2:', 'wp:')
        doc_xml_str = doc_xml_str.replace('ns3:', 'a:')
        doc_xml_str = doc_xml_str.replace('ns4:', 'pic:')

        with open(doc_xml_path, 'wb') as f:
            f.write(doc_xml_str.encode('utf-8'))

        # Save relationships
        rels_xml_str = ET.tostring(rels_root, encoding='unicode')
        rels_xml_str = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n' + rels_xml_str
        with open(rels_path, 'wb') as f:
            f.write(rels_xml_str.encode('utf-8'))

        # Save content types
        ct_xml_str = ET.tostring(ct_root, encoding='unicode')
        ct_xml_str = '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>\n' + ct_xml_str
        with open(content_types_path, 'wb') as f:
            f.write(ct_xml_str.encode('utf-8'))

        # Repackage as docx
        with zipfile.ZipFile(OUTPUT_PATH, 'w', zipfile.ZIP_DEFLATED) as zout:
            for root_dir, dirs, files in os.walk(temp_dir):
                for file in files:
                    full_path = os.path.join(root_dir, file)
                    arcname = os.path.relpath(full_path, temp_dir)
                    zout.write(full_path, arcname)

        print(f'\nSuccess! Inserted {inserted}/{len(DIAGRAM_MAP)} diagrams.')
        print(f'Output: {OUTPUT_PATH}')

    finally:
        # Cleanup temp directory
        shutil.rmtree(temp_dir, ignore_errors=True)


if __name__ == '__main__':
    process_docx()
