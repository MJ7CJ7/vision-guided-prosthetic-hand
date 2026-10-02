#!/usr/bin/env python3
"""
Vision-Guided Prosthetic Hand - Python / OpenCV controller

Pipeline:
  Camera -> blur -> HSV threshold -> morphology -> largest contour
         -> features (size, aspect, circularity) -> grip selection
         -> proximity/stability trigger -> serial command -> Arduino -> MG995 servos

Usage:
  python vision_hand.py --port COM3                 # Windows
  python vision_hand.py --port /dev/ttyACM0         # Linux / macOS
  python vision_hand.py --calibrate                 # tune HSV range with sliders
  python vision_hand.py --no-serial                 # vision only (no hardware)

Keys (in the video window):
  q = quit   o = open hand   c = close hand   r = reset state machine
  1/2/3 = force power / pinch / tripod grip
"""

import argparse
import time
from dataclasses import dataclass
from enum import Enum
from typing import Optional

import cv2
import numpy as np

try:
    import serial
except ImportError:  # allow --no-serial without pyserial installed
    serial = None


# --------------------------------------------------------------------------- #
# Configuration
# --------------------------------------------------------------------------- #
# Default HSV range for the target object (a green object here).
# Use --calibrate to find values for your own object.
DEFAULT_HSV_LOW = (35, 80, 60)
DEFAULT_HSV_HIGH = (85, 255, 255)

MIN_CONTOUR_AREA = 800            # px^2, ignore noise below this
GRASP_AREA_RATIO = 0.12           # object bbox area / frame area to trigger grasp
GRASP_CENTER_TOL = 0.25           # object center must be within this fraction of frame center
STABLE_FRAMES_TO_GRASP = 8        # consecutive frames meeting the criteria
LOST_FRAMES_TO_RESET = 20         # frames without object before returning to IDLE
HOLD_SECONDS = 5.0                # auto-release after holding this long (0 = never)
SEND_MIN_INTERVAL = 0.10          # s, rate-limit serial writes


class Grip(Enum):
    POWER = "G1"   # large / cylindrical objects
    PINCH = "G2"   # small objects
    TRIPOD = "G3"  # medium / round objects


class State(Enum):
    IDLE = "IDLE"
    APPROACH = "APPROACH"
    HOLD = "HOLD"


# --------------------------------------------------------------------------- #
# Serial link to Arduino
# --------------------------------------------------------------------------- #
class HandController:
    def __init__(self, port: Optional[str], baud: int = 115200):
        self.ser = None
        self.last_cmd = None
        self.last_sent = 0.0
        if port:
            if serial is None:
                raise RuntimeError("pyserial not installed: pip install pyserial")
            self.ser = serial.Serial(port, baud, timeout=0.05)
            time.sleep(2.0)  # Arduino resets when the port opens
            self.ser.reset_input_buffer()
            print(f"[serial] connected on {port} @ {baud}")
        else:
            print("[serial] running without hardware (dry run)")

    def send(self, cmd: str, force: bool = False) -> None:
        now = time.time()
        if not force and (cmd == self.last_cmd or now - self.last_sent < SEND_MIN_INTERVAL):
            return
        self.last_cmd, self.last_sent = cmd, now
        print(f"[cmd] {cmd}")
        if self.ser:
            self.ser.write((cmd + "\n").encode("ascii"))

    def read_reply(self) -> Optional[str]:
        if self.ser and self.ser.in_waiting:
            return self.ser.readline().decode(errors="ignore").strip()
        return None

    def close(self) -> None:
        if self.ser:
            self.send("O", force=True)
            time.sleep(0.2)
            self.ser.close()


# --------------------------------------------------------------------------- #
# Object detection
# --------------------------------------------------------------------------- #
@dataclass
class Detection:
    contour: np.ndarray
    bbox: tuple          # x, y, w, h
    center: tuple        # cx, cy
    area_ratio: float    # bbox area / frame area
    aspect: float        # long side / short side (>= 1)
    circularity: float   # 4*pi*A / P^2  (1.0 = perfect circle)


class ObjectDetector:
    def __init__(self, hsv_low, hsv_high):
        self.low = np.array(hsv_low, dtype=np.uint8)
        self.high = np.array(hsv_high, dtype=np.uint8)
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    def mask(self, frame: np.ndarray) -> np.ndarray:
        blur = cv2.GaussianBlur(frame, (7, 7), 0)
        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)
        m = cv2.inRange(hsv, self.low, self.high)
        m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self.kernel, iterations=2)
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, self.kernel, iterations=2)
        return m

    def detect(self, frame: np.ndarray):
        m = self.mask(frame)
        contours, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [c for c in contours if cv2.contourArea(c) >= MIN_CONTOUR_AREA]
        if not contours:
            return None, m

        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)
        fh, fw = frame.shape[:2]
        area = cv2.contourArea(c)
        perim = cv2.arcLength(c, True)
        circ = 4 * np.pi * area / (perim * perim) if perim > 0 else 0.0
        aspect = max(w, h) / max(1, min(w, h))
        det = Detection(
            contour=c,
            bbox=(x, y, w, h),
            center=(x + w // 2, y + h // 2),
            area_ratio=(w * h) / float(fw * fh),
            aspect=aspect,
            circularity=circ,
        )
        return det, m


def select_grip(det: Detection) -> Grip:
    """Pick a grip type from simple shape/size features."""
    if det.area_ratio < 0.05:
        return Grip.PINCH            # small object -> precision pinch
    if det.aspect >= 1.6:
        return Grip.POWER            # elongated (bottle, handle) -> wrap
    if det.circularity >= 0.75:
        return Grip.TRIPOD           # ball-like -> three-finger
    return Grip.POWER


# --------------------------------------------------------------------------- #
# Grasp state machine
# --------------------------------------------------------------------------- #
class GraspLogic:
    def __init__(self, hand: HandController):
        self.hand = hand
        self.state = State.IDLE
        self.stable = 0
        self.lost = 0
        self.grip = Grip.POWER
        self.hold_start = 0.0

    def reset(self):
        self.state, self.stable, self.lost = State.IDLE, 0, 0
        self.hand.send("O", force=True)

    def update(self, det: Optional[Detection], frame_shape):
        fh, fw = frame_shape[:2]

        if self.state == State.HOLD:
            if HOLD_SECONDS > 0 and time.time() - self.hold_start > HOLD_SECONDS:
                print("[logic] hold timeout -> releasing")
                self.reset()
            return

        if det is None:
            self.lost += 1
            self.stable = 0
            if self.lost >= LOST_FRAMES_TO_RESET and self.state != State.IDLE:
                self.state = State.IDLE
                self.hand.send("O")
            return

        self.lost = 0
        dx = abs(det.center[0] - fw / 2) / fw
        dy = abs(det.center[1] - fh / 2) / fh
        centered = dx < GRASP_CENTER_TOL and dy < GRASP_CENTER_TOL
        close_enough = det.area_ratio >= GRASP_AREA_RATIO

        if self.state == State.IDLE:
            self.state = State.APPROACH

        if centered and close_enough:
            self.stable += 1
            self.grip = select_grip(det)
            if self.stable >= STABLE_FRAMES_TO_GRASP:
                self.hand.send(self.grip.value, force=True)
                self.state = State.HOLD
                self.hold_start = time.time()
                print(f"[logic] GRASP with {self.grip.name}")
        else:
            self.stable = max(0, self.stable - 1)


# --------------------------------------------------------------------------- #
# HSV calibration tool
# --------------------------------------------------------------------------- #
def run_calibration(cam_index: int):
    cap = cv2.VideoCapture(cam_index)
    win = "HSV calibration (press s to print values, q to quit)"
    cv2.namedWindow(win)
    for name, init, mx in [("H low", DEFAULT_HSV_LOW[0], 179), ("H high", DEFAULT_HSV_HIGH[0], 179),
                           ("S low", DEFAULT_HSV_LOW[1], 255), ("S high", DEFAULT_HSV_HIGH[1], 255),
                           ("V low", DEFAULT_HSV_LOW[2], 255), ("V high", DEFAULT_HSV_HIGH[2], 255)]:
        cv2.createTrackbar(name, win, init, mx, lambda _: None)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        g = lambda n: cv2.getTrackbarPos(n, win)
        low = (g("H low"), g("S low"), g("V low"))
        high = (g("H high"), g("S high"), g("V high"))
        mask = ObjectDetector(low, high).mask(frame)
        result = cv2.bitwise_and(frame, frame, mask=mask)
        cv2.imshow(win, np.hstack([frame, result]))
        k = cv2.waitKey(1) & 0xFF
        if k == ord("s"):
            print(f"--hsv-low {low[0]} {low[1]} {low[2]} --hsv-high {high[0]} {high[1]} {high[2]}")
        elif k == ord("q"):
            break
    cap.release()
    cv2.destroyAllWindows()


# --------------------------------------------------------------------------- #
# Main loop
# --------------------------------------------------------------------------- #
def draw_overlay(frame, det, logic: GraspLogic, fps: float):
    fh, fw = frame.shape[:2]
    # Grasp zone
    zx, zy = int(fw * GRASP_CENTER_TOL), int(fh * GRASP_CENTER_TOL)
    cv2.rectangle(frame, (fw // 2 - zx, fh // 2 - zy), (fw // 2 + zx, fh // 2 + zy), (255, 200, 0), 1)

    if det is not None:
        x, y, w, h = det.bbox
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
        cv2.drawContours(frame, [det.contour], -1, (0, 200, 255), 2)
        cv2.circle(frame, det.center, 5, (0, 0, 255), -1)
        info = f"area {det.area_ratio:.2f}  asp {det.aspect:.1f}  circ {det.circularity:.2f}"
        cv2.putText(frame, info, (x, max(15, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    status = f"{logic.state.value} | grip {logic.grip.name} | stable {logic.stable}/{STABLE_FRAMES_TO_GRASP} | {fps:.0f} FPS"
    cv2.putText(frame, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


def main():
    p = argparse.ArgumentParser(description="Vision-guided prosthetic hand")
    p.add_argument("--port", help="Arduino serial port, e.g. COM3 or /dev/ttyACM0")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--cam", type=int, default=0, help="camera index")
    p.add_argument("--hsv-low", type=int, nargs=3, default=DEFAULT_HSV_LOW)
    p.add_argument("--hsv-high", type=int, nargs=3, default=DEFAULT_HSV_HIGH)
    p.add_argument("--calibrate", action="store_true", help="HSV tuning mode")
    p.add_argument("--no-serial", action="store_true", help="dry run without Arduino")
    args = p.parse_args()

    if args.calibrate:
        run_calibration(args.cam)
        return

    port = None if args.no_serial else args.port
    if port is None and not args.no_serial:
        print("No --port given; running in dry-run mode (use --port to drive the hand).")

    hand = HandController(port, args.baud)
    detector = ObjectDetector(args.hsv_low, args.hsv_high)
    logic = GraspLogic(hand)

    cap = cv2.VideoCapture(args.cam)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit("Cannot open camera")

    hand.send("O", force=True)
    prev = time.time()
    fps = 0.0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frame = cv2.flip(frame, 1)

            det, mask = detector.detect(frame)
            logic.update(det, frame.shape)

            now = time.time()
            fps = 0.9 * fps + 0.1 * (1.0 / max(1e-6, now - prev))
            prev = now

            reply = hand.read_reply()
            if reply:
                print(f"[arduino] {reply}")

            draw_overlay(frame, det, logic, fps)
            cv2.imshow("Prosthetic hand - camera", frame)
            cv2.imshow("Mask", mask)

            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            elif k == ord("o"):
                logic.reset()
            elif k == ord("c"):
                hand.send("C", force=True)
                logic.state, logic.hold_start = State.HOLD, time.time()
            elif k == ord("r"):
                logic.reset()
            elif k in (ord("1"), ord("2"), ord("3")):
                grip = {ord("1"): Grip.POWER, ord("2"): Grip.PINCH, ord("3"): Grip.TRIPOD}[k]
                hand.send(grip.value, force=True)
                logic.grip, logic.state, logic.hold_start = grip, State.HOLD, time.time()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        hand.close()


if __name__ == "__main__":
    main()
