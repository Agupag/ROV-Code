# Project Guidelines

## Scope
- This workspace is a single-sketch Arduino project for joystick input and movement interpretation.
- Primary source file: `joystickCode.ino`.
- Existing user-facing project docs are in `README.md`; link there instead of repeating setup details.

## Build And Validation
- Prefer compile-only validation unless the user explicitly asks to upload.
- If a board is not specified, ask before compiling with a fixed FQBN.
- Typical compile flow uses Arduino CLI, for example:
  - `arduino-cli compile --fqbn <board-fqbn> joystickCode.ino`
- Upload requires an explicit port and user approval, for example:
  - `arduino-cli upload -p <serial-port> --fqbn <board-fqbn> joystickCode.ino`
- Runtime verification is via Serial Monitor at `115200` baud.

## Architecture
- `loop()` reads two analog sticks, normalizes axis values around center, and calls `interpretation(...)`.
- `interpretation(...)` maps joystick regions to movement states (corners first, then cardinal directions) and sets output pins.
- Movement behavior is currently threshold-box based, not angle/vector based.

## Conventions
- Keep pin assignments and joystick correction values as top-level constants.
- Preserve the current movement-priority order unless the task explicitly changes behavior.
- Keep output writes explicit and deterministic (set intended HIGH pin, set all others LOW).
- Keep serial debug output readable and grouped when adjusting movement logic.

## Known Pitfalls
- `README.md` contains useful setup context but may not always reflect latest loop timing/details.
- `interpretation(...)` is actively evolving; some comments describe intended behavior not fully implemented yet.
- Avoid large refactors that alter pin mappings or movement thresholds unless requested.

## Link-First References
- Project setup and usage: `README.md`
- Movement and pin logic implementation: `joystickCode.ino`