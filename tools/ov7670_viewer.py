"""
OV7670 live viewer for the NUCLEO-F767ZI firmware.

The STM32 does all the AprilTag detection and command logic. This script only
shows what the camera sees and draws the tags the STM32 reports; it does not
detect or decode anything itself.

Protocol (Core/Src/main.c):
  PC -> STM32 : 'v' = start sending video, 't' = stop (text only, for Tera Term)
  STM32 -> PC : text lines (the same result lines Tera Term shows), plus in video mode
                @TAG det_w det_h id x0 y0 x1 y1 x2 y2 x3 y3   one per tag the STM32 found
                @CMD <command>                                  the STM32's current command
                AA 55 'F' 'G' | width u16 LE | height u16 LE | grey pixels | byte sum u32 LE

Tera Term and this viewer use the same COM port: close one before opening the other.

Install:  pip install pyserial numpy opencv-python
Run:      python ov7670_viewer.py            (finds the ST-LINK port and the speed by itself)
          python ov7670_viewer.py COM5 --baud 2000000

APP_DELIVERY firmware (LCD cat face) runs at 115200 with no video: it answers
'v' with "@MODE TEXT" and the window then shows its output lines instead
(TABLE / FOUND / INVALID / ARRIVED / NOT FOUND / CANCEL / -> $D commands).
Keys in the image window:  s = save PNG,  q / Esc = quit
"""
import argparse
import sys
import time
from collections import deque

import cv2
import numpy as np
import serial
from serial.tools import list_ports

MAGIC = b"\xAA\x55FG"
HEADER_LEN = 8          # magic + width + height
SUM_LEN = 4
MAX_W, MAX_H = 640, 480

# Box colour per tag ID (BGR); other IDs are drawn grey
BOX_COLORS = {
    0: (0, 0, 255),      # red
    1: (0, 255, 0),      # green
    2: (255, 128, 0),    # blue
    3: (0, 165, 255),    # orange
    4: (255, 0, 255),    # magenta
}


class StreamParser:
    """Splits the serial byte stream into text lines and grey frames."""

    def __init__(self):
        self.buf = bytearray()
        self.text = ""

    def _text(self, raw):
        self.text += raw.decode("ascii", "replace")
        *lines, self.text = self.text.split("\n")
        for line in lines:
            yield ("line", line.rstrip("\r"))

    def feed(self, data):
        """Add bytes; yields ("line", str), ("frame", w, h, pixels) or ("bad", reason)."""
        self.buf += data
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                keep = len(MAGIC) - 1          # a magic may be split across reads
                if len(self.buf) > keep:
                    yield from self._text(self.buf[:-keep])
                    del self.buf[:-keep]
                return
            if i > 0:
                yield from self._text(self.buf[:i])
                del self.buf[:i]
            if len(self.buf) < HEADER_LEN:
                return
            w = int.from_bytes(self.buf[4:6], "little")
            h = int.from_bytes(self.buf[6:8], "little")
            if not (0 < w <= MAX_W and 0 < h <= MAX_H):
                del self.buf[:1]                # false magic, skip it
                continue
            total = HEADER_LEN + w * h + SUM_LEN
            if len(self.buf) < total:
                return
            pixels = bytes(self.buf[HEADER_LEN:total - SUM_LEN])
            got_sum = int.from_bytes(self.buf[total - SUM_LEN:total], "little")
            del self.buf[:total]
            if sum(pixels) & 0xFFFFFFFF != got_sum:
                yield ("bad", "checksum mismatch (bytes lost on the UART?)")
                continue
            yield ("frame", w, h, pixels)


def draw(view, tags, command, scale_x, scale_y):
    """Draw the tags reported by the STM32 and its current command."""
    for tag_id, pts in tags:
        p = np.int32([[x * scale_x, y * scale_y] for x, y in pts])
        color = BOX_COLORS.get(tag_id, (160, 160, 160))
        x, y, w, h = cv2.boundingRect(p)
        cv2.rectangle(view, (x - 3, y - 3), (x + w + 3, y + h + 3), color, 2)
        cv2.circle(view, tuple(p[0]), 4, color, -1)          # the tag's top-left corner
        label = f"ID {tag_id}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        ty = max(y - 8, th + 4)
        cv2.rectangle(view, (x - 3, ty - th - 4), (x - 3 + tw + 6, ty + 3), color, -1)
        cv2.putText(view, label, (x, ty), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1, cv2.LINE_AA)
    if command:
        text = f"STM32: {command}"
        cv2.rectangle(view, (0, 0), (view.shape[1], 30), (0, 0, 0), -1)
        size = 0.65
        while size > 0.35 and cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, size, 1)[0][0] > view.shape[1] - 16:
            size -= 0.05                                    # long DRIVE hints: shrink to fit
        cv2.putText(view, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, size, (0, 255, 255), 1, cv2.LINE_AA)


# Text mode (APP_DELIVERY firmware, 115200 baud, no video): colour per line type (BGR)
LINE_COLORS = [
    ("TABLE", (15, 101, 232)),       # orange
    ("FOUND", (127, 208, 53)),       # green
    ("ARRIVED", (127, 208, 53)),
    ("INVALID", (79, 91, 255)),      # red
    ("NOT FOUND", (60, 154, 255)),   # light orange
    ("CANCEL", (243, 238, 233)),
    ("->", (230, 200, 90)),          # commands for the lower board
    ("TOUCH", (150, 150, 150)),
    ("DRIVE", (0, 255, 255)),        # yellow: what the robot should do
]


def draw_log(lines, port, baud):
    """Window with the latest text lines from the STM32 (no video in this mode)"""
    w, line_h = 760, 24
    view = np.full((40 + line_h * len(lines) + 10, w, 3), (36, 26, 15), np.uint8)
    cv2.putText(view, f"STM32 output  {port} @ {baud}  (text mode, no video)", (10, 26),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (243, 238, 233), 1, cv2.LINE_AA)
    for i, line in enumerate(lines):
        color = next((c for key, c in LINE_COLORS if line.startswith(key)), (200, 200, 200))
        cv2.putText(view, line[:90], (10, 60 + i * line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1,
                    cv2.LINE_AA)
    return view


def draw_events(view, lines):
    """Latest STM32 result lines (TABLE / FOUND / INVALID / -> $D ...) at the bottom of the video"""
    if not lines:
        return
    line_h = 22
    top = view.shape[0] - line_h * len(lines) - 6
    overlay = view.copy()
    cv2.rectangle(overlay, (0, top), (view.shape[1], view.shape[0]), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.6, view, 0.4, 0, view)
    for i, line in enumerate(lines):
        color = next((c for key, c in LINE_COLORS if line.startswith(key)), (200, 200, 200))
        cv2.putText(view, line[:70], (8, top + 17 + i * line_h), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
                    cv2.LINE_AA)


def looks_like_text(line):
    printable = sum(32 <= ord(ch) < 127 for ch in line)
    return len(line) >= 4 and printable >= 0.9 * len(line)


def rotate_upright(image, tags, rotation):
    """The camera is mounted on its side: turn the picture (and the tag corners,
    which are fractions of the width / height) clockwise by 'rotation' degrees."""
    if rotation == 90:
        image = cv2.rotate(image, cv2.ROTATE_90_CLOCKWISE)
        tags = [(i, [(1 - y, x) for x, y in pts]) for i, pts in tags]
    elif rotation == 180:
        image = cv2.rotate(image, cv2.ROTATE_180)
        tags = [(i, [(1 - x, 1 - y) for x, y in pts]) for i, pts in tags]
    elif rotation == 270:
        image = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)
        tags = [(i, [(y, 1 - x) for x, y in pts]) for i, pts in tags]
    return np.ascontiguousarray(image), tags


def find_port():
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".lower()
        if "stlink" in text or "st-link" in text or "stmicro" in text:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser(description="Show the OV7670 camera and the tags the STM32 detects")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM5 (default: auto-detect ST-LINK)")
    ap.add_argument("--baud", type=int, default=0,
                    help="must match UART_BAUD in main.c (default: try 2000000 and 115200)")
    ap.add_argument("--scale", type=int, default=0,
                    help="window zoom factor (default: auto, about 640 px wide)")
    ap.add_argument("--no-rotate", action="store_true",
                    help="show the picture as the sensor sends it (landscape 160x120), ignoring @ROT")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        ports = ", ".join(p.device for p in list_ports.comports()) or "none"
        sys.exit(f"ST-LINK port not found, pass it explicitly (available: {ports})")

    bauds = [args.baud] if args.baud else [2000000, 115200]   # video firmware, delivery (text) firmware
    baud_i = 0
    locked = bool(args.baud)
    try:
        ser = serial.Serial(port, bauds[0], timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"cannot open {port}: {e}\n(is Tera Term still connected to it?)")
    print(f"Listening on {port} @ {bauds[0]} baud. Asking the STM32 for video...")

    parser = StreamParser()
    garbage_warned = False
    pending_tags, shown_tags, command = [], [], ""
    rotation = 0            # @ROT from the STM32: clockwise turn that makes the picture upright
    image = None
    frames = 0
    text_mode = False
    log = deque(maxlen=16)
    t_last = time.time()
    t_ask = 0.0
    t_baud = time.time()
    cv2.namedWindow("OV7670", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("OV7670", draw_log(["waiting for the STM32 ..."], port, ser.baudrate))

    try:
        while True:
            # not sure of the speed yet: switch between the two firmware speeds
            if not locked and time.time() - t_baud > 2.5:
                baud_i = (baud_i + 1) % len(bauds)
                ser.baudrate = bauds[baud_i]
                parser = StreamParser()
                t_baud = time.time()
                t_ask = 0.0
                print(f"[viewer] trying {ser.baudrate} baud")

            # ask for video until frames arrive (also after a board reset);
            # delivery firmware answers "@MODE TEXT" instead
            if not text_mode and time.time() - t_last > 2 and time.time() - t_ask > 2:
                ser.write(b"v")
                t_ask = time.time()

            data = ser.read(ser.in_waiting or 1)
            for item in parser.feed(data):
                if item[0] == "line":
                    line = item[1]
                    if not locked and looks_like_text(line):
                        locked = True
                        print(f"[viewer] STM32 found at {ser.baudrate} baud")
                    if line.startswith("@MODE TEXT"):
                        if not text_mode:
                            print("[viewer] delivery firmware: text only (no video), showing its output")
                        text_mode = True
                        cv2.imshow("OV7670", draw_log(list(log), port, ser.baudrate))
                        continue
                    if looks_like_text(line) and not line.startswith("@"):
                        log.append(line)
                    if text_mode and looks_like_text(line):
                        cv2.imshow("OV7670", draw_log(list(log), port, ser.baudrate))
                    if line.startswith("@TAG"):
                        f = line.split()
                        if len(f) == 12:
                            det_w, det_h, tag_id = int(f[1]), int(f[2]), int(f[3])
                            pts = [(int(f[4 + 2 * k]) / det_w, int(f[5 + 2 * k]) / det_h) for k in range(4)]
                            pending_tags.append((tag_id, pts))
                        continue
                    if line.startswith("@ROT"):
                        try:
                            rotation = int(line.split()[1]) % 360
                        except (IndexError, ValueError):
                            pass
                        continue
                    if line.startswith("@CMD"):
                        command = line[5:]
                        continue
                    if line.count("�") > len(line) // 2:
                        if locked and not garbage_warned:
                            print(f"[viewer] receiving garbage -> baud mismatch. Does UART_BAUD in main.c "
                                  f"match {ser.baudrate}?")
                            garbage_warned = True
                        continue
                    print(line)
                elif item[0] == "bad":
                    print(f"[viewer] frame dropped: {item[1]}")
                    pending_tags = []
                else:
                    _, w, h, pixels = item
                    image = np.frombuffer(pixels, dtype=np.uint8).reshape(h, w)
                    shown_tags, pending_tags = pending_tags, []
                    if not args.no_rotate:
                        image, shown_tags = rotate_upright(image, shown_tags, rotation)
                    h, w = image.shape
                    frames += 1
                    t_last = time.time()
                    locked = True

                    scale = args.scale or max(1, min(640 // w, 640 // h))   # upright picture may be tall
                    view = cv2.resize(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), (w * scale, h * scale),
                                      interpolation=cv2.INTER_NEAREST)
                    draw(view, shown_tags, command, w * scale, h * scale)
                    draw_events(view, list(log)[-4:])
                    cv2.imshow("OV7670", view)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and image is not None:
                name = time.strftime("ov7670_%Y%m%d_%H%M%S.png")
                cv2.imwrite(name, image)
                print(f"[viewer] saved {name}")
    finally:
        ser.write(b"t")          # back to text only, so Tera Term can be used again
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
