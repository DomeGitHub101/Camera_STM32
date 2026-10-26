"""
Make the Thai UI text for the ILI9341 screen as 1-bit bitmaps in C.

The STM32 cannot shape Thai (upper vowels, tone marks and lower vowels have to
be stacked and moved by the font's GPOS rules), so every Thai phrase is shaped
and rendered here once and stored as a bitmap. Numbers are not baked in: each
text style also gets an ASCII font, and the STM32 puts e.g. "เจอโต๊ะ" + "4" +
"— ไม่ใช่" side by side. All bitmaps of one style share the same height and
baseline, so the pieces line up.

Shaping: Pillow's raqm layout when it is available (needs fribidi on Windows),
otherwise HarfBuzz through uharfbuzz + FreeType (raqm uses HarfBuzz inside, so
the result is the same).

Install:  pip install pillow uharfbuzz freetype-py
Run:      python tools/gen_thai_text.py            (from the project folder)
          python tools/gen_thai_text.py --font path/to/Sarabun-Medium.ttf
Output:   Core/Src/thai_text.c, Core/Inc/thai_text.h,
          tools/thai_text_preview.png  <- check the vowels and tone marks here
"""
import argparse
import math
import os
import sys

from PIL import Image, ImageChops, ImageDraw, ImageFont, features

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

FONT_CANDIDATES = [
    os.path.join(ROOT, "tools", "fonts", "Sarabun-Medium.ttf"),
    os.path.join(ROOT, "tools", "fonts", "NotoSansThai-Medium.ttf"),
    os.path.join(ROOT, "tools", "fonts", "NotoSansThai-Regular.ttf"),
    r"C:\Windows\Fonts\LeelawUI.ttf",          # Leelawadee UI (Windows 10/11)
    r"C:\Windows\Fonts\leelawad.ttf",
    "/usr/share/fonts/truetype/noto/NotoSansThai-Regular.ttf",
]

# style name -> pixel size
STYLES = {
    "TITLE":  22,
    "BTN":    22,
    "STATUS": 20,
    "SMALL":  14,
}

# C name -> (style, text). Numbers come from the ASCII font of the same style.
TEXTS = {
    "TXT_TITLE":          ("TITLE",  "เลือกโต๊ะที่จะไปส่ง"),
    "TXT_TABLE":          ("BTN",    "โต๊ะ"),
    "TXT_SEARCHING":      ("STATUS", "กำลังหาโต๊ะ"),
    "TXT_FOUND":          ("STATUS", "เจอโต๊ะ"),
    "TXT_NOT_THIS":       ("STATUS", "— ไม่ใช่"),
    "TXT_FOUND_DONE":     ("STATUS", "แล้ว!"),
    "TXT_ARRIVED":        ("STATUS", "ถึงโต๊ะ"),
    "TXT_ARRIVED_DONE":   ("STATUS", "แล้ว · รับอาหารได้เลย"),
    "TXT_NOT_FOUND":      ("STATUS", "ไม่พบโต๊ะ"),
    "TXT_GOING_HOME":     ("STATUS", "กำลังกลับจุดเริ่มต้น"),
    "TXT_HOME_ARRIVED":   ("STATUS", "ถึงจุดเริ่มต้นแล้ว"),
    "TXT_RECEIVED":       ("STATUS", "รับอาหารแล้ว"),
    "TXT_PLEASE_MOVE":    ("STATUS", "กรุณาหลบหน่อยครับ"),
    "TXT_LINK_LOST":      ("STATUS", "บอร์ดล่างไม่ตอบ"),
    "TXT_CANCEL":         ("SMALL",  "ยกเลิก"),
    "TXT_TAG_FAMILY":     ("SMALL",  "· tag36h11"),
}

ASCII_FIRST, ASCII_LAST = 32, 126


class Renderer:
    """Renders one line of text to an 'L' image with the baseline at a fixed row."""

    def __init__(self, font_path):
        self.path = font_path
        self.use_raqm = features.check("raqm")
        if not self.use_raqm:
            import freetype
            import uharfbuzz as hb
            self.ft = freetype
            self.hb = hb
            self.ft_face = freetype.Face(font_path)
            self.hb_face = hb.Face(hb.Blob.from_file_path(font_path))

    def render(self, text, size):
        """-> (image, baseline_row, pen_start_x, advance_px)"""
        pad = size
        h = size * 3
        base = size * 2
        if self.use_raqm:
            font = ImageFont.truetype(self.path, size, layout_engine=ImageFont.Layout.RAQM)
            adv = font.getlength(text)
            img = Image.new("L", (int(math.ceil(adv)) + 2 * pad, h), 0)
            ImageDraw.Draw(img).text((pad, base), text, font=font, fill=255, anchor="ls")
            return img, base, pad, adv

        hb, ft = self.hb, self.ft
        font = hb.Font(self.hb_face)
        font.scale = (size * 64, size * 64)
        buf = hb.Buffer()
        buf.add_str(text)
        buf.guess_segment_properties()
        hb.shape(font, buf, {})
        for info in buf.glyph_infos:
            if info.codepoint == 0:
                sys.exit(f"font {self.path} has no glyph for a character in {text!r}")
        adv = sum(p.x_advance for p in buf.glyph_positions) / 64
        img = Image.new("L", (int(math.ceil(adv)) + 2 * pad, h), 0)
        self.ft_face.set_char_size(size * 64)
        pen = pad * 64
        for info, pos in zip(buf.glyph_infos, buf.glyph_positions):
            self.ft_face.load_glyph(info.codepoint, ft.FT_LOAD_DEFAULT)
            self.ft_face.glyph.render(ft.FT_RENDER_MODE_NORMAL)
            g = self.ft_face.glyph
            bm = g.bitmap
            if bm.width and bm.rows:
                glyph = Image.frombytes("L", (bm.width, bm.rows), bytes(bm.buffer), "raw", "L", bm.pitch)
                x = round((pen + pos.x_offset) / 64) + g.bitmap_left
                y = base - round(pos.y_offset / 64) - g.bitmap_top
                layer = Image.new("L", img.size, 0)
                layer.paste(glyph, (x, y))
                img = ImageChops.lighter(img, layer)
            pen += pos.x_advance
        return img, base, pad, adv


def to_bits(img):
    """'L' image -> 1-bit rows, MSB first, (w+7)//8 bytes per row"""
    w, h = img.size
    px = img.load()
    out = bytearray()
    for y in range(h):
        for bx in range(0, w, 8):
            b = 0
            for k in range(8):
                if bx + k < w and px[bx + k, y] >= 128:
                    b |= 0x80 >> k
            out.append(b)
    return bytes(out)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--font", help="Thai TTF/OTF (default: Sarabun / Noto Sans Thai in tools/fonts, else Leelawadee UI)")
    args = ap.parse_args()
    font_path = args.font or next((p for p in FONT_CANDIDATES if os.path.exists(p)), None)
    if not font_path:
        sys.exit("no Thai font found: put Sarabun-Medium.ttf in tools/fonts or pass --font")
    r = Renderer(font_path)
    print(f"font: {font_path}")
    print(f"shaping: {'Pillow raqm' if r.use_raqm else 'HarfBuzz (uharfbuzz) + FreeType'}")

    # render everything, then crop each style to one common height
    rendered = {}   # key -> (style, img, base, x0, adv)
    for name, (style, text) in TEXTS.items():
        rendered[name] = (style,) + r.render(text, STYLES[style])
    for style in ("BTN", "STATUS", "SMALL"):
        for c in range(ASCII_FIRST, ASCII_LAST + 1):
            rendered[f"{style}:{c}"] = (style,) + r.render(chr(c), STYLES[style])

    rows = {}       # style -> (top, bottom) rows relative to the baseline
    for style, img, base, x0, adv in rendered.values():
        bbox = img.getbbox()
        if bbox:
            t, b = bbox[1] - base, bbox[3] - base
            top, bot = rows.get(style, (t, b))
            rows[style] = (min(top, t), max(bot, b))

    bitmaps = {}    # key -> (w, h, bits, image)
    for key, (style, img, base, x0, adv) in rendered.items():
        top, bot = rows[style]
        bbox = img.getbbox()
        left = min(x0, bbox[0]) if bbox else x0
        right = max(x0 + int(math.ceil(adv)), bbox[2]) if bbox else x0 + int(math.ceil(adv))
        crop = img.crop((left, base + top, right, base + bot))
        bitmaps[key] = (crop.size[0], crop.size[1], to_bits(crop), crop)

    for name, (style, text) in TEXTS.items():
        w = bitmaps[name][0]
        if w > 300:
            print(f"WARNING: {name} is {w} px wide, the screen is 320")

    write_c(bitmaps, font_path)
    write_preview(bitmaps, rows)


def c_array(name, bits):
    lines = []
    for i in range(0, len(bits), 16):
        lines.append("  " + ", ".join(f"0x{b:02X}" for b in bits[i:i + 16]) + ",")
    return f"static const uint8_t {name}[{len(bits)}] = {{\n" + "\n".join(lines) + "\n};\n"


def write_c(bitmaps, font_path):
    font_name = os.path.basename(font_path)
    head = os.path.join(ROOT, "Core", "Inc", "thai_text.h")
    src = os.path.join(ROOT, "Core", "Src", "thai_text.c")
    with open(head, "w", encoding="utf-8", newline="\n") as f:
        f.write("/* Generated by tools/gen_thai_text.py - do not edit, run the script again */\n")
        f.write("#ifndef THAI_TEXT_H\n#define THAI_TEXT_H\n\n#include <stdint.h>\n\n")
        f.write("/* 1-bit bitmap: rows top to bottom, (w+7)/8 bytes per row, MSB = leftmost pixel */\n")
        f.write("typedef struct { uint16_t w, h; const uint8_t *bits; } mono_bmp_t;\n")
        f.write("/* ASCII font: glyph[c - first] for c in first..first+count-1, all h rows high */\n")
        f.write("typedef struct { uint16_t h; uint8_t first, count; const mono_bmp_t *glyph; } mono_font_t;\n\n")
        for name, (style, text) in TEXTS.items():
            w, h = bitmaps[name][:2]
            f.write(f"extern const mono_bmp_t {name};{' ' * (18 - len(name))}/* {text}  ({w}x{h}) */\n")
        f.write("\n")
        for style in ("BTN", "STATUS", "SMALL"):
            f.write(f"extern const mono_font_t FONT_{style};\n")
        f.write("\n#endif /* THAI_TEXT_H */\n")
    with open(src, "w", encoding="utf-8", newline="\n") as f:
        f.write(f"/* Generated by tools/gen_thai_text.py from {font_name} - do not edit */\n")
        f.write('#include "thai_text.h"\n\n')
        for name, (style, text) in TEXTS.items():
            w, h, bits, _ = bitmaps[name]
            f.write(f"/* {text} */\n")
            f.write(c_array(f"{name}_bits", bits))
            f.write(f"const mono_bmp_t {name} = {{{w}, {h}, {name}_bits}};\n\n")
        for style in ("BTN", "STATUS", "SMALL"):
            entries = []
            for c in range(ASCII_FIRST, ASCII_LAST + 1):
                w, h, bits, _ = bitmaps[f"{style}:{c}"]
                arr = f"{style.lower()}_{c}"
                f.write(c_array(arr, bits))
                entries.append(f"  {{{w}, {h}, {arr}}},")
            f.write(f"static const mono_bmp_t {style.lower()}_glyphs[] = {{\n" + "\n".join(entries) + "\n};\n")
            h = bitmaps[f"{style}:{ASCII_FIRST}"][1]
            f.write(f"const mono_font_t FONT_{style} = {{{h}, {ASCII_FIRST}, {ASCII_LAST - ASCII_FIRST + 1}, "
                    f"{style.lower()}_glyphs}};\n\n")
    print(f"wrote {os.path.relpath(head, ROOT)} and {os.path.relpath(src, ROOT)}")


def write_preview(bitmaps, rows):
    """All texts as the STM32 will draw them (x3), to check vowels and tone marks"""
    bg, fg = (0x0F, 0x1A, 0x24), (0xE9, 0xEE, 0xF3)
    samples = [
        ["TXT_TITLE"],
        ["TXT_TABLE", "BTN:32", "BTN:49"],
        ["TXT_SEARCHING", "STATUS:32", "STATUS:51"],
        ["TXT_FOUND", "STATUS:32", "STATUS:52", "STATUS:32", "TXT_NOT_THIS"],
        ["TXT_FOUND", "STATUS:32", "STATUS:51", "STATUS:32", "TXT_FOUND_DONE"],
        ["TXT_ARRIVED", "STATUS:32", "STATUS:51", "STATUS:32", "TXT_ARRIVED_DONE"],
        ["TXT_NOT_FOUND", "STATUS:32", "STATUS:51"],
        ["TXT_RECEIVED"],
        ["TXT_GOING_HOME"],
        ["TXT_HOME_ARRIVED"],
        ["TXT_PLEASE_MOVE"],
        ["TXT_LINK_LOST"],
        ["TXT_CANCEL"],
        ["SMALL:73", "SMALL:68", "SMALL:32", "SMALL:51", "SMALL:32", "TXT_TAG_FAMILY"],
    ]
    lines = []
    for parts in samples:
        w = sum(bitmaps[p][0] for p in parts)
        h = max(bitmaps[p][1] for p in parts)
        line = Image.new("RGB", (w, h), bg)
        x = 0
        for p in parts:
            pw, ph, _, im = bitmaps[p]
            mask = im.point(lambda v: 255 if v >= 128 else 0)
            line.paste(Image.new("RGB", (pw, ph), fg), (x, 0), mask)
            x += pw
        lines.append(line)
    W = max(320, max(l.size[0] for l in lines)) + 8
    H = sum(l.size[1] + 6 for l in lines) + 6
    sheet = Image.new("RGB", (W, H), bg)
    y = 6
    for l in lines:
        sheet.paste(l, (4, y))
        y += l.size[1] + 6
    sheet = sheet.resize((W * 3, H * 3), Image.NEAREST)
    path = os.path.join(ROOT, "tools", "thai_text_preview.png")
    sheet.save(path)
    print(f"wrote {os.path.relpath(path, ROOT)}  <- open it and check the vowels / tone marks")


if __name__ == "__main__":
    main()
