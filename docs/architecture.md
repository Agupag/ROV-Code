# Architecture

## High-Level System

This repository models a shore-side ROV controller built around an Arduino Nano, two joysticks, an RF24 radio link, an I2C LCD, and optional SD logging.

The repo has three layers:

1. Shore controller sketches.
2. Wokwi simulation assets.
3. A legacy serial-to-UDP bridge.

## Shore Controller Sketches

### Current Primary Sketch

The current primary sketch is [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino).

It preserves the structure of the original shore controller:

- initialize hardware
- listen for payload packets
- refresh control data
- send next control packet through ACK payload
- display telemetry
- optionally log to SD

The most important functions are:

- `setup()`
- `loop()`
- `initializeRF24()`
- `radioCheckAndReply()`
- `readControlSettings()`
- `processPayloadData()`

### Historical Sketches

- [debuggedNewCode.ino](/Users/connor/Desktop/ROV%20Code/debuggedNewCode.ino)
  More debug-focused RF24 sketch with extra defensive behavior and Serial logging.
- [newCode.ino](/Users/connor/Desktop/ROV%20Code/newCode.ino)
  Older integrated variant with more drift from the original shore-code structure.
- [joystickCode.ino](/Users/connor/Desktop/ROV%20Code/joystickCode.ino)
  Original joystick movement interpretation prototype.
- [engr100 code.ino](/Users/connor/Desktop/ROV%20Code/engr100%20code.ino)
  Earlier integration baseline.

## Data Model

The RF24 exchange is built around two structs in the Arduino code:

- `payload_data`
  Payload-to-shore telemetry.
- `control_data`
  Shore-to-payload control.

### `control_data`

This contains:

- `crc`
- `s1angle`
- `s2angle`
- `m1`
- `m2`
- `m3`
- `m4`

Each `mN` field uses:

- `0` off
- `1` forward
- `2` reverse

### `payload_data`

This carries telemetry such as:

- yaw
- pitch
- roll
- voltage
- current
- pressure
- temperature
- timestamp fields
- water intrusion indicator

## Movement Interpretation

The movement logic is joystick-region based.

Joystick 1 is treated as the movement selector:

- corner regions are checked first
- then cardinal/box regions
- else neutral

The selected movement is then converted into explicit thruster commands in `c_data`.

Joystick 2 is used for servo angles:

- X controls `s1angle`
- Y controls `s2angle`

## Wokwi Simulation Architecture

### Diagram

[diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) wires the current controller to:

- 2 joysticks
- an I2C LCD1602
- a microSD card
- a custom payload chip
- 4 thruster LEDs
- a logic analyzer

### Custom Chips

There are two custom chips:

- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c)

The current diagram uses `chip-payload`, so [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) is the active simulation implementation.

The chip models:

- enough nRF24 register behavior for the RF24 library path used by this repo
- RX payload availability
- ACK payload reception
- IRQ behavior
- payload-side visualization

The Wokwi implementation is intentionally scoped. It is a project-specific simulation aid, not a complete transceiver emulator.

## Python Bridge Architecture

The bridge under `bridge/` serves a different workflow from the RF24 path.

[bridge/arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/arduino_virtual_pin_bridge.py):

- reads serial lines
- parses movement and axis values
- reconstructs virtual pin states
- sends UDP JSON packets

It is designed around printed movement frames like:

```text
forward
y1:-27
x1:18
```

This is closest to the original joystick-only workflow, not the main RF24 controller architecture.

## Recommended Mental Model

When working in this repo:

- think of [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino) as the current controller
- think of [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) plus [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) as the simulation environment
- think of the Python bridge as a separate compatibility tool for older serial-output workflows
