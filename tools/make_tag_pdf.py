"""
Printable AprilTag sheets (tag36h11) for the delivery tables, one tag per A4 page.

The black square is printed exactly TAG_MM wide (default 142 mm, the size the
STM32 is calibrated for: TAG_SIZE_MM in Core/Src/delivery.c), with a white
quiet zone around it, the label "tag36h11  ID n" below, and a 100 mm check
line to confirm the printer did not scale the page.

Print at 100% / "Actual size" (not "Fit to page").

Install:  pip install pillow opencv-contrib-python   (or opencv-python >= 4.7)
Run:      python tools/make_tag_pdf.py                 -> tools/tags/tags_print_A4.pdf (ID 0 = start point, 1..6 = tables)
          python tools/make_tag_pdf.py --ids 1 3 --mm 142
"""
import argparse
import os

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DPI = 600
A4_MM = (210, 297)


def mm(v):
    return v * DPI / 25.4


def tag_cells(tag_id):
    """8x8 cells of the black square (True = black), from OpenCV's tag36h11 dictionary"""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    img = cv2.aruco.generateImageMarker(d, tag_id, 8 * 50, borderBits=1)
    return img[25::50, 25::50] < 128


def font(size_px, bold=False):
    names = ["arialbd.ttf", "DejaVuSans-Bold.ttf"] if bold else ["arial.ttf", "DejaVuSans.ttf"]
    for n in names:
        for p in (n, os.path.join(r"C:\Windows\Fonts", n)):
            try:
                return ImageFont.truetype(p, size_px)
            except OSError:
                pass
    return ImageFont.load_default()


def page(tag_id, tag_mm):
    W, H = round(mm(A4_MM[0])), round(mm(A4_MM[1]))
    im = Image.new("L", (W, H), 255)
    dr = ImageDraw.Draw(im)
    cells = tag_cells(tag_id)

    side = mm(tag_mm)
    x0 = (W - side) / 2
    y0 = mm(40)
    edge = [round(i * side / 8) for i in range(9)]          # exact total width, no rounding drift
    for r in range(8):
        for c in range(8):
            if cells[r, c]:
                dr.rectangle([round(x0) + edge[c], round(y0) + edge[r],
                              round(x0) + edge[c + 1] - 1, round(y0) + edge[r + 1] - 1], fill=0)

    # corner cut marks one cell outside the black square (cut here: keeps the white quiet zone)
    q = side / 8
    L = mm(6)
    for cx, sx in ((x0 - q, 1), (x0 + side + q, -1)):
        for cy, sy in ((y0 - q, 1), (y0 + side + q, -1)):
            dr.line([round(cx), round(cy), round(cx + sx * L), round(cy)], fill=0, width=3)
            dr.line([round(cx), round(cy), round(cx), round(cy + sy * L)], fill=0, width=3)

    # label under the tag
    y = y0 + side + q + mm(10)
    label = f"tag36h11  ID {tag_id}"
    f = font(round(mm(9)), bold=True)
    tw = dr.textlength(label, font=f)
    dr.text(((W - tw) / 2, y), label, font=f, fill=0)
    y += mm(14)
    info = f"black square {tag_mm:g} mm  -  print at 100% (Actual size)"
    f2 = font(round(mm(4)))
    tw = dr.textlength(info, font=f2)
    dr.text(((W - tw) / 2, y), info, font=f2, fill=0)

    # 100 mm check line
    y += mm(12)
    lx = (W - mm(100)) / 2
    dr.line([round(lx), round(y), round(lx + mm(100)), round(y)], fill=0, width=6)
    for t in (lx, lx + mm(100)):
        dr.line([round(t), round(y - mm(3)), round(t), round(y + mm(3))], fill=0, width=6)
    msg = "this line must measure 100 mm"
    tw = dr.textlength(msg, font=f2)
    dr.text(((W - tw) / 2, y + mm(4)), msg, font=f2, fill=0)
    return im.convert("1", dither=Image.Dither.NONE)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2, 3, 4, 5, 6])
    ap.add_argument("--mm", type=float, default=142.0, help="black square size in mm (TAG_SIZE_MM)")
    ap.add_argument("--out", default=os.path.join(ROOT, "tools", "tags", "tags_print_A4.pdf"))
    args = ap.parse_args()
    if args.mm > A4_MM[0] - 2 * (args.mm / 8) - 10:
        raise SystemExit("tag too big for A4 width")
    pages = [page(i, args.mm) for i in args.ids]
    pages[0].save(args.out, save_all=True, append_images=pages[1:], resolution=DPI)
    print(f"wrote {os.path.relpath(args.out, ROOT)}: IDs {args.ids}, black square {args.mm:g} mm, {DPI} dpi")


if __name__ == "__main__":
    main()
