# Project Guidelines

## Scope
- This workspace includes two Arduino sketches and one host-side Python bridge.
- Primary files:
  - `joystickCode.ino` for joystick movement interpretation and movement token output.
  - `engr100 code.ino` for RF24 payload communication and control packet generation.
  - `bridge/arduino_virtual_pin_bridge.py` for serial-to-UDP mirroring.
- Keep user-facing setup/usage details in `README.md`; link there instead of duplicating.

## Build And Validation
- Prefer compile-only checks unless upload is explicitly requested.
- If board/FQBN is not specified, ask before compiling.
- Arduino compile example:
  - `arduino-cli compile --fqbn <board-fqbn> joystickCode.ino`
- Arduino upload example (requires explicit port and approval):
  - `arduino-cli upload -p <serial-port> --fqbn <board-fqbn> joystickCode.ino`
- Bridge install/run/test:
  - `python3 -m pip install -r bridge/requirements.txt`
  - `python3 bridge/arduino_virtual_pin_bridge.py`
  - `python3 -m unittest discover -s bridge/tests`
- Runtime serial verification uses `115200` baud.

## Architecture
- `joystickCode.ino`:
  - `loop()` reads joystick axes, normalizes to roughly `-128..128`, then calls `interpretation(...)`.
  - `interpretation(...)` uses threshold boxes with corner-first priority, then cardinal directions.
- `engr100 code.ino`:
  - Handles RF24 packet exchange and maps control inputs to `c_data.m1..m4` values (`0/1/2`).
  - May use manual pin semantics or virtual control-state semantics depending on task.
- `bridge/arduino_virtual_pin_bridge.py`:
  - Parses serial movement frames and emits UDP JSON packets to localhost.

## Conventions
- Keep pin assignments and joystick correction constants at top-level.
- Preserve movement priority order (corners before cardinals) unless a task explicitly asks to change it.
- Keep output state updates deterministic and explicit each cycle.
- Keep serial output contract stable when bridge parsing depends on it (`movement`, `x1`, `y1`).
- Avoid introducing `String` churn in tight loops unless required for existing compatibility.

## Known Pitfalls
- `joystickCode.ino` may include partial/experimental sections; keep movement parsing outputs consistent.
- `engr100 code.ino` pin definitions and joystick pin usage can overlap; verify intent before changing pin behavior.
- Avoid large refactors to pin mappings, threshold boxes, or motor mapping tables unless explicitly requested.
- Bridge parser is strict about movement token validity and numeric axis lines; malformed frame changes can break UDP output.

## Link-First References
- Project setup and usage: `README.md`
- Joystick movement logic: `joystickCode.ino`
- RF24 control pipeline: `engr100 code.ino`
- Bridge and payload contract: `bridge/arduino_virtual_pin_bridge.py`, `bridge/tests/test_arduino_virtual_pin_bridge.py`