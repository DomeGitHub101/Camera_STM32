"""
OV7670 viewer + AprilTag detector for the NUCLEO-F767ZI firmware (Core/Src/main.c, send_frame()).

Frame format on the serial port:
    AA 55 'F' type | width u16 LE | height u16 LE | pixels | byte sum u32 LE
    type 'G' = 8-bit grayscale (1 byte/pixel), 'R' = RGB565 high byte first (2 bytes/pixel)
Everything outside a frame is the firmware's text log and is printed to the console.

Install:  pip install pyserial numpy opencv-python
Run:      python ov7670_viewer.py                      (finds the ST-LINK port by itself)
          python ov7670_viewer.py COM5 --baud 2000000 --family 36h11
Keys in the image window:  s = save PNG,  b = swap RGB565 byte order,  q / Esc = quit
"""
import argparse
import sys
import time

import cv2
import numpy as np
import serial
from serial.tools import list_ports

MAGIC = b"\xAA\x55F"
HEADER_LEN = 8          # magic(3) + type(1) + width(2) + height(2)
SUM_LEN = 4
MAX_W, MAX_H = 640, 480
BYTES_PER_PIXEL = {ord("G"): 1, ord("R"): 2}

TAG_FAMILIES = {
    "16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "36h11": cv2.aruco.DICT_APRILTAG_36h11,
}

# Tag ID -> robot command. Edit to match your printed tags.
TAG_COMMANDS = {
    0: "STOP",
    1: "FORWARD",
    2: "TURN_LEFT",
    3: "TURN_RIGHT",
    4: "BACKWARD",
}


# Tags that get a coloured box in the window (BGR colour per ID)
BOX_COLORS = {
    1: (0, 255, 0),      # green
    2: (255, 128, 0),    # blue
    3: (0, 165, 255),    # orange
    4: (255, 0, 255),    # magenta
}


class FrameParser:
    """Splits the raw serial byte stream into text and frames."""

    def __init__(self):
        self.buf = bytearray()

    def feed(self, data):
        """Add bytes; yields ("text", str), ("frame", kind, w, h, pixels) or ("bad", reason)."""
        self.buf += data
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # keep a possible partial magic at the end, print the rest as text
                keep = len(MAGIC) - 1
                if len(self.buf) > keep:
                    yield ("text", self.buf[:-keep].decode("ascii", "replace"))
                    del self.buf[:-keep]
                return
            if i > 0:
                yield ("text", self.buf[:i].decode("ascii", "replace"))
                del self.buf[:i]
            if len(self.buf) < HEADER_LEN:
                return
            kind = self.buf[3]
            w = int.from_bytes(self.buf[4:6], "little")
            h = int.from_bytes(self.buf[6:8], "little")
            if kind not in BYTES_PER_PIXEL or not (0 < w <= MAX_W and 0 < h <= MAX_H):
                del self.buf[:1]            # false magic, skip it
                continue
            total = HEADER_LEN + w * h * BYTES_PER_PIXEL[kind] + SUM_LEN
            if len(self.buf) < total:
                return
            pixels = bytes(self.buf[HEADER_LEN:total - SUM_LEN])
            got_sum = int.from_bytes(self.buf[total - SUM_LEN:total], "little")
            del self.buf[:total]
            if sum(pixels) & 0xFFFFFFFF != got_sum:
                yield ("bad", "checksum mismatch (bytes lost on the UART?)")
                continue
            yield ("frame", chr(kind), w, h, pixels)


def rgb565_to_bgr(pixels, w, h, swap):
    px = np.frombuffer(pixels, dtype="<u2" if swap else ">u2").reshape(h, w)
    r = ((px >> 11) & 0x1F).astype(np.uint8)
    g = ((px >> 5) & 0x3F).astype(np.uint8)
    b = (px & 0x1F).astype(np.uint8)
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)
    return np.dstack([b, g, r])


def make_detector(family):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.minMarkerPerimeterRate = 0.02    # allow small (far away) tags in a low-res image
    dictionary = cv2.aruco.getPredefinedDictionary(TAG_FAMILIES[family])
    return cv2.aruco.ArucoDetector(dictionary, params)


def detect_tags(detector, gray):
    """Returns a list of (tag_id, centre_xy, side_px, corners)."""
    corners, ids, _ = detector.detectMarkers(gray)
    tags = []
    if ids is not None:
        for c, tag_id in zip(corners, ids.flatten()):
            pts = c.reshape(4, 2)
            side = float(np.mean(np.linalg.norm(pts - np.roll(pts, 1, axis=0), axis=1)))
            tags.append((int(tag_id), pts.mean(axis=0), side, c))
    return tags


def draw_tags(view, tags, scale):
    """Draw a rectangle + label around tags 1-4; other IDs get a thin grey outline."""
    for tag_id, _, side, c in tags:
        pts = np.int32(c.reshape(4, 2) * scale)
        color = BOX_COLORS.get(tag_id)
        if color is None:
            cv2.polylines(view, [pts], True, (128, 128, 128), 1)
            continue
        x, y, w, h = cv2.boundingRect(pts)
        pad = 4
        cv2.rectangle(view, (x - pad, y - pad), (x + w + pad, y + h + pad), color, 2)
        label = f"ID {tag_id} {TAG_COMMANDS.get(tag_id, '')} ({side:.0f}px)"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        ty = max(y - pad - 4, th + 4)                       # keep the label inside the window
        cv2.rectangle(view, (x - pad, ty - th - 4), (x - pad + tw + 6, ty + 2), color, -1)
        cv2.putText(view, label, (x - pad + 3, ty - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)


def on_tags(tags, width):
    """Hook for the robot: decide what to do from the detected tags."""
    if not tags:
        return
    # use the largest (closest) tag
    tag_id, centre, side, _ = max(tags, key=lambda t: t[2])
    command = TAG_COMMANDS.get(tag_id, "UNKNOWN")
    offset = (centre[0] - width / 2) / (width / 2)      # -1 = far left, +1 = far right
    print(f"[tag] id={tag_id} -> {command}  size={side:.0f}px  offset={offset:+.2f}")
    # TODO: send `command` to the robot car here (e.g. over another serial port / Wi-Fi)


def find_port():
    for p in list_ports.comports():
        text = f"{p.description} {p.manufacturer or ''}".lower()
        if "stlink" in text or "st-link" in text or "stmicro" in text:
            return p.device
    return None


def main():
    ap = argparse.ArgumentParser(description="Show OV7670 frames sent by the STM32 and detect AprilTags")
    ap.add_argument("port", nargs="?", help="serial port, e.g. COM5 (default: auto-detect ST-LINK)")
    ap.add_argument("--baud", type=int, default=2000000, help="must match UART_BAUD in main.c")
    ap.add_argument("--family", choices=TAG_FAMILIES, default="36h11", help="AprilTag family")
    ap.add_argument("--scale", type=int, default=0,
                    help="window zoom factor (default: auto, about 640 px wide)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        ports = ", ".join(p.device for p in list_ports.comports()) or "none"
        sys.exit(f"ST-LINK port not found, pass it explicitly (available: {ports})")

    ser = serial.Serial(port, args.baud, timeout=0.05)
    print(f"Listening on {port} @ {args.baud} baud, AprilTag family tag{args.family}. "
          f"Press reset on the board if nothing shows up.")

    parser = FrameParser()
    detector = make_detector(args.family)
    garbage_warned = False
    swap = False
    image = None
    frames = 0
    t_last = time.time()
    cv2.namedWindow("OV7670", cv2.WINDOW_AUTOSIZE)

    while True:
        data = ser.read(ser.in_waiting or 1)
        for item in parser.feed(data):
            if item[0] == "text":
                text = item[1]
                if text.count("�") > len(text) // 2:
                    # mostly undecodable bytes: the board is not using this baud rate
                    if not garbage_warned:
                        print(f"[viewer] receiving garbage -> baud mismatch. Does UART_BAUD in main.c "
                              f"match --baud {args.baud}, and is the new firmware flashed?")
                        garbage_warned = True
                    continue
                sys.stdout.write(text)
                sys.stdout.flush()
            elif item[0] == "bad":
                print(f"[viewer] frame dropped: {item[1]}")
            else:
                _, kind, w, h, pixels = item
                if kind == "G":
                    gray = np.frombuffer(pixels, dtype=np.uint8).reshape(h, w)
                    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
                else:
                    image = rgb565_to_bgr(pixels, w, h, swap)
                    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                tags = detect_tags(detector, gray)
                on_tags(tags, w)

                frames += 1
                now = time.time()
                print(f"[viewer] frame {frames} ({w}x{h} {'gray' if kind == 'G' else 'rgb565'}), "
                      f"{now - t_last:.2f} s since last, {len(tags)} tag(s)")
                t_last = now

                scale = args.scale or max(1, 640 // w)
                view = cv2.resize(image, (w * scale, h * scale), interpolation=cv2.INTER_NEAREST)
                draw_tags(view, tags, scale)
                cv2.imshow("OV7670", view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("b"):
            swap = not swap
            print(f"[viewer] RGB565 byte swap {'on' if swap else 'off'} (applies from the next frame)")
        if key == ord("s") and image is not None:
            name = time.strftime("ov7670_%Y%m%d_%H%M%S.png")
            cv2.imwrite(name, image)
            print(f"[viewer] saved {name}")

    ser.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
