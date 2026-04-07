# Project Guidelines

## Scope

This repository contains three related but distinct workflows:

- A Nano-based RF24 shore controller sketch.
- A Wokwi simulation for that controller using a custom payload chip.
- A legacy Python serial-to-UDP bridge for joystick-only movement output.

The current primary controller sketch is [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino). Prefer treating that file as the default source of truth unless the task explicitly targets a historical variant.

## Primary Files

- [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino)
  Main Nano joystick shore controller.
- [debuggedNewCode.ino](/Users/connor/Desktop/ROV%20Code/debuggedNewCode.ino)
  Debug-oriented RF24/controller variant.
- [newCode.ino](/Users/connor/Desktop/ROV%20Code/newCode.ino)
  Older integrated variant with more divergence.
- [joystickCode.ino](/Users/connor/Desktop/ROV%20Code/joystickCode.ino)
  Original joystick interpretation sketch and movement vocabulary reference.
- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
  Current Wokwi wiring for the Nano + custom payload chip setup.
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
  Current Wokwi custom chip used by `diagram.json`.
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)
  Custom part definition for the payload chip.
- [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c)
  Alternate custom RF24-style chip with explicit forward/reverse outputs.
- [rf24.chip.json](/Users/connor/Desktop/ROV%20Code/rf24.chip.json)
  Custom part definition for the alternate RF24 chip.
- [bridge/arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/arduino_virtual_pin_bridge.py)
  Legacy serial-to-UDP bridge for joystick movement output.
- [bridge/tests/test_arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/tests/test_arduino_virtual_pin_bridge.py)
  Unit tests for the bridge.

## Documentation Sources

Keep user-facing documentation aligned with:

- [README.md](/Users/connor/Desktop/ROV%20Code/README.md)
- [docs/architecture.md](/Users/connor/Desktop/ROV%20Code/docs/architecture.md)
- [docs/wokwi-setup.md](/Users/connor/Desktop/ROV%20Code/docs/wokwi-setup.md)
- [docs/file-guide.md](/Users/connor/Desktop/ROV%20Code/docs/file-guide.md)

When behavior changes, update these docs if the change affects:

- hardware pin mappings
- movement interpretation
- packet structure
- Wokwi setup
- custom chip behavior
- bridge serial contract

## Architecture

### Current Controller Path

[newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino) keeps the original shore-code style loop:

- receive payload telemetry
- refresh control data from joysticks
- push control packet through RF24 ACK payload
- render telemetry to LCD
- optionally log to SD

The sketch uses:

- joystick 1 for movement region selection
- joystick 2 for servo angle control
- direct writes to `c_data.m1..m4`
- `RF24::writeAckPayload()` and `RF24::startListening()`

### Simulation Path

[diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) currently instantiates `chip-payload`, so simulation changes should generally target:

- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)

The custom payload chip is a focused RF24/nRF24 subset built around the Wokwi SPI chip API. It should stay compatible with the RF24 library calls used by the main sketch rather than attempting to emulate every radio feature.

### Bridge Path

The Python bridge is not the same thing as the current RF24 controller workflow. It parses serial movement frames and emits UDP packets for another simulator. It depends on printed movement names and axis lines, not RF24 packets.

## Pin Mapping

The default pin map for the current controller and Wokwi wiring is:

- `CE_PIN = 8`
- `CSN_PIN = 9`
- `ANALOG_X_PIN = A2`
- `ANALOG_Y_PIN = A3`
- `ANALOG_BUTTON_PIN = 2`
- `ANALOG_X2_PIN = A6`
- `ANALOG_Y2_PIN = A7`
- `chipSelect = 10`
- LCD I2C on `A4/A5`

Do not change these casually. Historical sketches may still contain older mappings from the original shore hardware. Treat those as historical unless the task explicitly asks to port them.

## Movement Logic Conventions

The movement vocabulary is:

- `forward`
- `reverse`
- `left`
- `right`
- `cornerLeft`
- `cornerRight`
- `cornerBottomLeft`
- `cornerBottomRight`
- `neutral`

Preserve corner-first priority unless the task explicitly asks to change behavior.

Keep movement-to-thruster mapping deterministic on every loop iteration. Avoid incremental or sticky state unless the task explicitly requires latching behavior.

## RF24 And Custom Chip Conventions

When editing [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) or [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c):

- Prefer Wokwi's documented SPI device API rather than hand-rolled GPIO clock decoding.
- Keep the modeled behavior focused on the RF24 calls the Arduino sketch actually uses.
- Preserve compatibility with:
  - `enableAckPayload()`
  - `openReadingPipe()`
  - `startListening()`
  - `available()`
  - `read()`
  - `writeAckPayload()`
- Keep `STATUS`, `FIFO_STATUS`, IRQ, and RX payload availability behavior coherent.
- Keep the payload packet layout aligned with the Arduino `payload_data` and `control_data` structs.

If the diagram still uses `chip-payload`, changes to `rf24.chip.c` will not affect the default Wokwi project until the diagram is changed too.

## Build And Validation

Prefer validation that matches the task:

- Arduino syntax/build checks when toolchain availability is known.
- JSON validation for `diagram.json`.
- Python unit tests for the bridge.
- Wokwi simulation checks when working on `diagram.json` or custom chips.

Examples:

```bash
python3 -m unittest discover -s bridge/tests
python3 -m json.tool diagram.json >/dev/null
```

If `arduino-cli` is available and the board is known, use compile-only checks before upload.

## Known Pitfalls

- The repo contains multiple historical controller variants. Do not assume the latest behavior lives in the oldest file.
- The bridge workflow and the RF24 workflow are related but separate.
- Joystick pin assignments differ between historical sketches. Verify the target file before copying pins.
- The custom radio model is intentionally partial. Be careful not to document it as a full nRF24 emulator.
- Wokwi part names and pin names must stay consistent with `diagram.json` and the `.chip.json` pin lists.

## Documentation Expectations

When the code changes materially, update docs so they explain:

- which file is primary
- what hardware pin map is expected
- how simulation is wired
- what the custom chips do
- how to test the bridge

Favor concrete descriptions over generic summaries. If a behavior is historical or secondary, label it that way.
