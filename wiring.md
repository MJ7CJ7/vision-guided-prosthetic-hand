# Wiring and power

| Finger | Arduino pin |
|---|---|
| Thumb | D3 |
| Index | D5 |
| Middle | D6 |
| Ring | D9 |
| Pinky | D10 |

## Power (important)

- MG995 servos can draw about 1 A each under load or stall.
- Power them from an **external 5–6 V supply rated 5 A or more**. Never from the Arduino 5V pin.
- Connect the external supply **GND to Arduino GND** (common ground).
- Put a **1000 µF capacitor** across the servo supply.
- Servo signal wires (orange/yellow) go to the Arduino pins above; red to the external +V; brown to GND.

## Calibration

Edit `OPEN_ANGLE`, `CLOSED_ANGLE` and the `GRIP_*` arrays in `firmware/firmware.ino` to match your finger linkage. Start with small angles and test one servo at a time with no fingers attached (`P 90 0 0 0 0`).
