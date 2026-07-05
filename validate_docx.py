import zipfile
from lxml import etree

path = r"C:\Users\bcq\Desktop\RK3588-Omni-Sentinel\qianrushi_with_diagrams.docx"

with zipfile.ZipFile(path) as z:
    rels = etree.fromstring(z.read("word/_rels/document.xml.rels"))
    print("=== Relationships ===")
    for rel in rels:
        rid = rel.get("Id", "")
        rtype = rel.get("Type", "")
        target = rel.get("Target", "")
        print("  %s: %s -> %s" % (rid, rtype.split("/")[-1], target))

    doc = etree.fromstring(z.read("word/document.xml"))
    w_ns = "http://schemas.openxmlformats.org/wordprocessingml/2006/main"
    drawings = doc.findall(".//{%s}drawing" % w_ns)
    print("\n=== Document: %d drawing elements found ===" % len(drawings))

    media_files = [n for n in z.namelist() if "media/" in n]
    print("\n=== Media files: %d ===" % len(media_files))
    for m in media_files:
        info = z.getinfo(m)
        print("  %s (%d bytes)" % (m, info.file_size))

    print("\n=== Content Types ===")
    ct = etree.fromstring(z.read("[Content_Types].xml"))
    ct_ns = "http://schemas.openxmlformats.org/package/2006/content-types"
    for d in ct.findall("{%s}Default" % ct_ns):
        print("  Default: .%s = %s" % (d.get("Extension"), d.get("ContentType")))

    # Check that wp namespace is present in document
    doc_xml = z.read("word/document.xml").decode("utf-8")
    if "xmlns:wp=" in doc_xml: print("\nwp namespace: OK")
    if "xmlns:a=" in doc_xml: print("a namespace: OK")
    if "xmlns:pic=" in doc_xml: print("pic namespace: OK")

    # Check for placeholders still present
    if "此处插入图片" in doc_xml:
        remaining = doc_xml.count("此处插入图片")
        print("\nWARNING: %d placeholder(s) still remaining!" % remaining)
    else:
        print("\nAll placeholders replaced!")

print("\nValidation complete!")
