# ROV Joystick Reader (Arduino)

A simple Arduino sketch that reads **two analog joysticks** and prints their positions to the Serial Monitor.

It is useful for:
- testing joystick wiring
- visualizing stick movement in text
- building toward ROV movement controls

## What this code does

The sketch:
1. reads X/Y from joystick 1 and joystick 2
2. recenters values around `0` (roughly `-128` to `128`)
3. reads joystick 1 button press
4. prints a side-by-side ASCII joystick display in Serial Monitor

## Pin setup

| Function | Arduino Pin |
|---|---|
| Joystick 1 X | `A1` |
| Joystick 1 Y | `A0` |
| Joystick 1 Button | `D2` (`INPUT_PULLUP`) |
| Joystick 2 X | `A2` |
| Joystick 2 Y | `A3` |

## Output format

Every loop, Serial output shows:
- a visual grid for **Joystick 1** and **Joystick 2**
- current numeric X/Y values for each joystick
- button state for joystick 1 (`PRESSED` or `NOT PRESSED`)

## Movement logic status

The file also includes an `interpretation(...)` function that is **work in progress**.

- `forward` is partially defined with limits
- other directions are placeholders
- thruster output logic is commented as next step

So right now, the reliable part is the joystick reading + serial visualization.

## How to run

1. Open `joystickCode.ino` in Arduino IDE.
2. Select your board and COM/serial port.
3. Upload the sketch.
4. Open Serial Monitor at **115200 baud**.
5. Move both joysticks and watch the ASCII display update.

## Quick notes

- Values are mapped from Arduino ADC (`0-1023`) to (`0-255`), then corrected to center around zero.
- Loop delay is `200 ms`, so output updates about 5 times per second.
- Joystick button uses pull-up logic, so pressed reads as LOW.
