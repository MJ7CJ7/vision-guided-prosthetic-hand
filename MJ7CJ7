# Vision-Guided Prosthetic Hand

Real-time, vision-guided object grasping for a 5-finger prosthetic hand. An **OpenCV** pipeline detects the target object, selects a grip type from its shape and size, and sends the command over serial to **Arduino** firmware that drives **MG995 servo motors**.

**Tech stack:** Python · OpenCV · NumPy · PySerial · Arduino (C++)

---

## How it works

```
Camera → Blur → HSV threshold → Morphology → Largest contour
      → Features (size, aspect ratio, circularity)
      → Grip selection → Stability check → Serial command
      → Arduino firmware → 5x MG995 servos
```

| Object feature | Grip chosen | Command |
|---|---|---|
| Small (bbox < 5% of frame) | Pinch (thumb + index) | `G2` |
| Elongated (aspect ratio ≥ 1.6) | Power grip (all fingers) | `G1` |
| Round (circularity ≥ 0.75) | Tripod (thumb + index + middle) | `G3` |
| Otherwise | Power grip | `G1` |

A grasp triggers only when the object is **centered** in the grasp zone, **close enough**, and stable for several consecutive frames, which avoids false triggers.

## Repository structure

```
.
├── vision_hand.py        # OpenCV detection + grasp logic + serial control
├── firmware/
│   └── firmware.ino      # Arduino servo firmware (serial protocol)
├── docs/
│   └── wiring.md         # Wiring and power notes
├── requirements.txt
├── LICENSE
└── README.md
```

## Quick start (no hardware needed)

```bash
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>
pip install -r requirements.txt
python vision_hand.py --no-serial
```

On Windows, use `py` instead of `python` if `python` isn't recognized.

Hold a **green** object in front of the webcam. Commands are printed in the terminal instead of being sent to hardware.

### Tune detection for your own object

```bash
python vision_hand.py --calibrate
```

Adjust the sliders until only your object is white in the mask, press `s` to print the values, then run:

```bash
python vision_hand.py --no-serial --hsv-low H S V --hsv-high H S V
```

### Controls

| Key | Action |
|---|---|
| `q` | Quit |
| `o` / `r` | Open hand / reset |
| `c` | Close hand |
| `1` / `2` / `3` | Force power / pinch / tripod grip |

## Running with the hardware

1. Upload `firmware/firmware.ino` to an Arduino Uno/Nano (Arduino IDE).
2. Wire the servos as described in [`docs/wiring.md`](docs/wiring.md). **Use a separate 5–6 V supply for the servos.**
3. Run:
   ```bash
   python vision_hand.py --port COM3
   ```
   (`/dev/ttyACM0` on Linux/macOS.)

### Serial protocol (115200 baud)

| Command | Action |
|---|---|
| `O` | Open hand |
| `C` | Close hand |
| `G1` / `G2` / `G3` | Power / pinch / tripod grip |
| `P a b c d e` | Custom angles (0–180) per finger |
| `?` | Report current angles |

## Grasp test results

Fill this in from your own live tests.

| Object | Grip | Trials | Successful grasps | Success rate |
|---|---|---|---|---|
| e.g. Small cap | Pinch | 10 | – | – |
| e.g. Ball | Tripod | 10 | – | – |
| e.g. Bottle | Power | 10 | – | – |

## Limitations and future work

- Detection is color-based, so the object needs a distinct color against the background.
- No force or current feedback yet; grip strength is set by preset angles.
- Planned: YOLO/MobileNet detector for arbitrary objects, force sensing, EMG override.

## License

MIT, see [LICENSE](LICENSE).
