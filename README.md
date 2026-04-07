# ROV Shore Controller, Wokwi Simulation, and Bridge

This repository contains a joystick-driven shore controller for an ROV, a Wokwi simulation setup for the controller and payload link, and a legacy host-side serial bridge used for a separate UDP-based simulation workflow.

The current primary Arduino sketch is [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino). It is a simplified joystick-based version of the original shore controller structure and is intended to stay close to the original shore-code flow while matching the actual Nano-based joystick hardware used in this repo.

## Repository Overview

There are three main areas in this workspace:

- Arduino control sketches for the shore controller.
- Wokwi simulation assets, including `diagram.json` and custom chips that emulate the payload-side RF24 behavior.
- A Python bridge under `bridge/` that parses serial movement output from the older joystick-only workflow and mirrors it to UDP.

The most important files are:

- [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino)
- [debuggedNewCode.ino](/Users/connor/Desktop/ROV%20Code/debuggedNewCode.ino)
- [newCode.ino](/Users/connor/Desktop/ROV%20Code/newCode.ino)
- [joystickCode.ino](/Users/connor/Desktop/ROV%20Code/joystickCode.ino)
- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)
- [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c)
- [rf24.chip.json](/Users/connor/Desktop/ROV%20Code/rf24.chip.json)
- [bridge/arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/arduino_virtual_pin_bridge.py)

For more detailed project notes:

- [docs/architecture.md](/Users/connor/Desktop/ROV%20Code/docs/architecture.md)
- [docs/wokwi-setup.md](/Users/connor/Desktop/ROV%20Code/docs/wokwi-setup.md)
- [docs/file-guide.md](/Users/connor/Desktop/ROV%20Code/docs/file-guide.md)

## Primary Arduino Sketch

The primary sketch is [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino).

Its design goals are:

- Keep the original shore controller structure intact where possible.
- Replace the original switch-based motor input scheme with direct joystick input.
- Preserve RF24 request/reply flow with ACK payloads.
- Work with the Nano joystick wiring used in this repo and in Wokwi.
- Stay simple enough to debug quickly.

### Control Flow

The sketch follows this loop:

1. Wait for incoming payload data from the payload-side radio.
2. Read the joystick state and convert it into `c_data`.
3. Push the control packet into the RF24 ACK payload buffer.
4. Display payload telemetry on the LCD.
5. Optionally log telemetry to the SD card.

The three most important runtime functions are:

- `radioCheckAndReply()`
- `readControlSettings()`
- `processPayloadData()`

### Hardware Pin Map

The current sketch uses this Nano pin map:

- RF24 `CE` -> `D8`
- RF24 `CSN` -> `D9`
- RF24 `MOSI` -> `D11`
- RF24 `MISO` -> `D12`
- RF24 `SCK` -> `D13`
- LCD I2C `SDA` -> `A4`
- LCD I2C `SCL` -> `A5`
- Joystick 1 horizontal -> `A2`
- Joystick 1 vertical -> `A3`
- Joystick 1 button -> `D2`
- Joystick 2 horizontal -> `A6`
- Joystick 2 vertical -> `A7`
- SD card `CS` -> `D10`

This pin map is also reflected in [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json).

### Data Structures

The controller exchanges two fixed-size structs:

- `payload_data`
  This is the telemetry packet coming from the payload side.
- `control_data`
  This is the control packet sent back via RF24 ACK payload.

`control_data` contains:

- `s1angle`
- `s2angle`
- `m1`
- `m2`
- `m3`
- `m4`

Each thruster command is encoded as:

- `0` = off
- `1` = forward
- `2` = reverse

### Joystick Interpretation

`readControlSettings()` reads two joystick axes from joystick 1 and maps them into one movement region. It then writes the final thruster commands directly into `c_data.m1` through `c_data.m4`.

The movement priority is:

- corner right
- corner left
- corner bottom left
- corner bottom right
- forward
- left
- right
- reverse
- neutral

Joystick 2 is used for servo angles:

- joystick 2 X controls `s1angle`
- joystick 2 Y controls `s2angle`

### Diagnostics

Set `diagnostics = true` in [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino) to print movement, joystick axes, servo angles, and packet buffers over Serial at `115200` baud.

This is the easiest way to confirm:

- joystick calibration
- movement region selection
- servo angle mapping
- packet contents sent to the payload side

## Wokwi Simulation

The Wokwi simulation is built around:

- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)

### What The Custom Payload Chip Does

The custom `payload` chip is a focused nRF24/payload simulator for this project. It does not attempt to model the entire nRF24L01 feature set. Instead, it models the subset needed by the current shore controller sketch and the RF24 library calls it uses.

The chip currently handles:

- SPI command framing through Wokwi's documented SPI chip API
- basic nRF24-style register reads and writes
- `STATUS` and `FIFO_STATUS` reporting
- `R_RX_PAYLOAD`
- `R_RX_PL_WID`
- `W_TX_PAYLOAD`
- `W_ACK_PAYLOAD`
- `FLUSH_RX`
- `FLUSH_TX`
- IRQ assertion when RX data is pending
- payload telemetry generation
- thruster LED updates based on the received control packet

### Wokwi Circuit Contents

The current [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) includes:

- Arduino Nano
- 2 analog joysticks
- I2C LCD1602 at address `0x27`
- microSD card
- custom payload chip
- 4 thruster LEDs
- logic analyzer for RF24 debug visibility

The logic analyzer is wired to:

- `CE`
- `CSN`
- `SCK`
- `MOSI`
- `MISO`
- `IRQ`

That lets you inspect the radio exchange when debugging the custom chip model.

### Importing Into Wokwi

To run the project in Wokwi, create a browser project and include:

- [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino)
- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)

You do not need `rf24.chip.*` for the current Wokwi diagram, because the diagram instantiates `chip-payload`, not `chip-rf24`.

More detail is in [docs/wokwi-setup.md](/Users/connor/Desktop/ROV%20Code/docs/wokwi-setup.md).

## RF24 Custom Chip Files

There are two custom-chip code paths:

- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c)

They share the same underlying radio-emulation idea but expose different outputs:

- `payload.chip.c`
  Updates one LED per thruster command for a compact payload visualization.
- `rf24.chip.c`
  Exposes separate forward/reverse outputs for each thruster.

In practice:

- Use `payload.chip.*` with the current `diagram.json`.
- Use `rf24.chip.*` only if you want a more explicit motor-output visualization or a different custom Wokwi part setup.

## Python Bridge

The bridge under `bridge/` is a separate workflow from the current RF24 sketch. It is mainly for the older joystick-only serial-output path.

The main script is [bridge/arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/arduino_virtual_pin_bridge.py).

It:

- reads serial output at `115200`
- parses movement tokens and axis lines
- reconstructs virtual pin states
- emits UDP JSON packets to a localhost consumer

This bridge expects frames like:

```text
forward
y1:-27
x1:18
```

It is tied more closely to [joystickCode.ino](/Users/connor/Desktop/ROV%20Code/joystickCode.ino) and related movement-printing variants than to the current RF24 Wokwi controller flow.

### Install And Run

```bash
python3 -m pip install -r bridge/requirements.txt
python3 bridge/arduino_virtual_pin_bridge.py
```

### Run Tests

```bash
python3 -m unittest discover -s bridge/tests
```

The tests cover:

- serial frame parsing
- movement validation
- axis parsing order
- neutral fail-safe payload behavior
- pin-state mapping

## Recommended Workflow

For current project work, use this order of priority:

1. Edit and test [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino).
2. Use Wokwi with [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) and `payload.chip.*` to validate RF24/control flow.
3. Use `diagnostics = true` for Serial debugging.
4. Only use the Python bridge when you explicitly want the serial-to-UDP path for joystick-only movement output.

## File Selection Guidance

If you are not sure which file to edit:

- Edit [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino) for the current controller behavior.
- Read [debuggedNewCode.ino](/Users/connor/Desktop/ROV%20Code/debuggedNewCode.ino) for a more defensive/debug-focused RF24 variant.
- Read [newCode.ino](/Users/connor/Desktop/ROV%20Code/newCode.ino) and [engr100 code.ino](/Users/connor/Desktop/ROV%20Code/engr100%20code.ino) as historical variants.
- Read [joystickCode.ino](/Users/connor/Desktop/ROV%20Code/joystickCode.ino) for the original movement-region logic.
- Edit [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) for Wokwi wiring.
- Edit [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) when the current Wokwi RF24/payload simulation is wrong.
- Edit [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c) only if you intentionally need the alternate chip interface.

## External References

The custom radio model was aligned to these reference sources:

- Wokwi Chips API: https://docs.wokwi.com/chips-api/getting-started
- Wokwi SPI API: https://docs.wokwi.com/chips-api/spi
- RF24 docs: https://rf24.readthedocs.io/en/latest/
- nRF24L01 datasheet mirror: https://www.digikey.com/en/htmldatasheets/production/1432071/0/0/1/nrf24l01-reel.html

## Current Caveats

- The custom chip model is intentionally scoped to the RF24 features used by this project. It is not a full nRF24L01 emulator.
- The Wokwi simulation is useful for control-flow validation and wiring checks, not for analog-perfect behavior.
- The bridge workflow and the RF24 workflow are related but not identical; do not assume changes in one automatically apply to the other.
