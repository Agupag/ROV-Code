# ROV Joystick Bridge

This repo contains the Arduino joystick sketch plus a host-side Python bridge that mirrors the sketch's resolved movement into UDP packets for a local PyBullet ROV simulation.

## Arduino sketch

[`joystickCode.ino`](/Users/connor/Desktop/ROV%20Code/joystickCode.ino) reads two analog joysticks, converts joystick 1 into a movement token, and drives one-hot digital outputs on these pins:

- `10` -> `forward`
- `11` -> `right`
- `12` -> `left`
- `13` -> `down`
- `6` -> `cornerRight`
- `5` -> `cornerLeft`
- `4` -> `cornerBottomLeft`
- `7` -> `cornerBottomRight`

The host bridge does not modify the sketch. It watches the sketch's existing serial output and recreates those pin states locally.

Note: the checked-in [`joystickCode.ino`](/Users/connor/Desktop/ROV%20Code/joystickCode.ino) currently has an unfinished `thruster` block at the end, so the bridge assumes the Arduino is already running a compatible sketch variant that prints the same `movement`, `y1`, and `x1` lines over serial.

## Host bridge

Install the Python dependency:

```bash
python3 -m pip install -r bridge/requirements.txt
```

Run the bridge:

```bash
python3 bridge/arduino_virtual_pin_bridge.py
```

Or specify the port explicitly:

```bash
python3 bridge/arduino_virtual_pin_bridge.py --serial-port /dev/cu.usbmodemXXXX
```

Supported flags:

- `--serial-port`
- `--baud` default `115200`
- `--udp-host` default `127.0.0.1`
- `--udp-port` default `8765`
- `--stale-timeout-ms` default `250`

If `--serial-port` is omitted, the bridge auto-detects a likely Arduino port by preferring `/dev/cu.*` devices whose names contain `usbmodem`, `usbserial`, `wchusbserial`, `ttyACM`, or `ttyUSB`. If no match is found, it prints a clear list of discovered ports and keeps retrying until a matching device appears.

## UDP payload contract

The bridge sends one localhost UDP JSON datagram per valid serial frame to `127.0.0.1:8765`.

```json
{
  "source": "arduino_virtual_pin_bridge",
  "sequence": 42,
  "timestamp_ms": 1775000000000,
  "is_live": true,
  "movement": "forward",
  "axes": { "x1": 18, "y1": -27 },
  "pins": { "4": 0, "5": 0, "6": 0, "7": 0, "10": 1, "11": 0, "12": 0, "13": 0 },
  "active_pin": 10
}
```

Fail-safe behavior:

- If no valid frame arrives for more than `250 ms`, the bridge sends one neutral packet with `is_live: false`.
- If the serial link disconnects, the bridge sends the same neutral packet once and keeps retrying until the Arduino reappears.

Neutral fail-safe packet shape:

```json
{
  "source": "arduino_virtual_pin_bridge",
  "sequence": 43,
  "timestamp_ms": 1775000000250,
  "is_live": false,
  "movement": "neutral",
  "axes": { "x1": null, "y1": null },
  "pins": { "4": 0, "5": 0, "6": 0, "7": 0, "10": 0, "11": 0, "12": 0, "13": 0 },
  "active_pin": null
}
```

## Testing

Run the unit tests:

```bash
python3 -m unittest discover -s bridge/tests
```

The tests cover:

- serial frame parsing with blank lines
- `x1` / `y1` parsing in either order
- partial and malformed frames
- unknown movement tokens
- one-hot virtual pin mapping
- neutral fail-safe payload generation
