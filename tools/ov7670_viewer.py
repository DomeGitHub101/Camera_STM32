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
Run:      python ov7670_viewer.py            (finds the ST-LINK port by itself)
          python ov7670_viewer.py COM5 --baud 2000000
Keys in the image window:  s = save PNG,  q / Esc = quit
"""
import argparse
import sys
import time

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
        text = f"STM32 CMD: {command}"
        cv2.rectangle(view, (0, 0), (view.shape[1], 30), (0, 0, 0), -1)
        cv2.putText(view, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 1, cv2.LINE_AA)


def find_port():
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".lower()
        if "stlink" in text or "st-link" in text or "stmicro" in text:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser(description="Show the OV7670 camera and the tags the STM32 detects")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM5 (default: auto-detect ST-LINK)")
    ap.add_argument("--baud", type=int, default=2000000, help="must match UART_BAUD in main.c")
    ap.add_argument("--scale", type=int, default=0,
                    help="window zoom factor (default: auto, about 640 px wide)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        ports = ", ".join(p.device for p in list_ports.comports()) or "none"
        sys.exit(f"ST-LINK port not found, pass it explicitly (available: {ports})")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except serial.SerialException as e:
        sys.exit(f"cannot open {port}: {e}\n(is Tera Term still connected to it?)")
    print(f"Listening on {port} @ {args.baud} baud. Asking the STM32 for video...")

    parser = StreamParser()
    garbage_warned = False
    pending_tags, shown_tags, command = [], [], ""
    image = None
    frames = 0
    t_last = time.time()
    t_ask = 0.0
    cv2.namedWindow("OV7670", cv2.WINDOW_AUTOSIZE)

    try:
        while True:
            # ask for video until frames arrive (also after a board reset)
            if time.time() - t_last > 2 and time.time() - t_ask > 2:
                ser.write(b"v")
                t_ask = time.time()

            data = ser.read(ser.in_waiting or 1)
            for item in parser.feed(data):
                if item[0] == "line":
                    line = item[1]
                    if line.startswith("@TAG"):
                        f = line.split()
                        if len(f) == 12:
                            det_w, det_h, tag_id = int(f[1]), int(f[2]), int(f[3])
                            pts = [(int(f[4 + 2 * k]) / det_w, int(f[5 + 2 * k]) / det_h) for k in range(4)]
                            pending_tags.append((tag_id, pts))
                        continue
                    if line.startswith("@CMD"):
                        command = line[5:]
                        continue
                    if line.count("�") > len(line) // 2:
                        if not garbage_warned:
                            print(f"[viewer] receiving garbage -> baud mismatch. Does UART_BAUD in main.c "
                                  f"match --baud {args.baud}?")
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
                    frames += 1
                    t_last = time.time()

                    scale = args.scale or max(1, 640 // w)
                    view = cv2.resize(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR), (w * scale, h * scale),
                                      interpolation=cv2.INTER_NEAREST)
                    draw(view, shown_tags, command, w * scale, h * scale)
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
