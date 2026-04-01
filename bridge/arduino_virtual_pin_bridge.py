#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from dataclasses import dataclass
from glob import glob
from typing import Iterable

SOURCE_NAME = "arduino_virtual_pin_bridge"
PIN_NUMBERS = ("4", "5", "6", "7", "10", "11", "12", "13")
# Match joystickCode.ino: each movement can activate multiple virtual outputs.
# Forward thrusters: 4,5,6
# Reverse thrusters: 10,11,12
MOVEMENT_TO_PINS = {
    "forward": ("4", "5", "6"),
    "reverse": ("12", "10", "11"),
    "left": ("12", "11", "4"),
    "right": ("12", "5", "10"),
    "cornerLeft": ("4", "6"),
    "cornerRight": ("5", "6"),
    "cornerBottomLeft": ("10", "12"),
    "cornerBottomRight": ("11", "12"),
}
ALLOWED_MOVEMENTS = frozenset((*MOVEMENT_TO_PINS.keys(), "neutral"))
PREFERRED_PORT_TOKENS = (
    "usbmodem",
    "usbserial",
    "wchusbserial",
    "ttyacm",
    "ttyusb",
)
READ_TIMEOUT_SECONDS = 0.05
RECONNECT_DELAY_SECONDS = 1.0


@dataclass(frozen=True)
class ParsedFrame:
    movement: str
    x1: int
    y1: int


class SerialOutputParser:
    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self._movement: str | None = None
        self._axes: dict[str, int] = {}

    def feed_line(self, raw_line: str) -> ParsedFrame | None:
        line = raw_line.strip()
        if not line:
            return None

        if line in ALLOWED_MOVEMENTS:
            self._movement = line
            self._axes = {}
            return None

        if self._movement is None:
            return None

        if line.startswith("x1:") or line.startswith("y1:"):
            axis_name, raw_value = line.split(":", 1)
            try:
                axis_value = int(raw_value.strip())
            except ValueError:
                self.reset()
                return None

            self._axes[axis_name] = axis_value
            if "x1" in self._axes and "y1" in self._axes:
                frame = ParsedFrame(
                    movement=self._movement,
                    x1=self._axes["x1"],
                    y1=self._axes["y1"],
                )
                self.reset()
                return frame
            return None

        self.reset()
        return None


def parse_serial_lines(lines: Iterable[str]) -> list[ParsedFrame]:
    parser = SerialOutputParser()
    frames: list[ParsedFrame] = []
    for line in lines:
        frame = parser.feed_line(line)
        if frame is not None:
            frames.append(frame)
    return frames


def discover_serial_ports() -> list[str]:
    ports = set(glob("/dev/cu.*"))
    try:
        from serial.tools import list_ports

        ports.update(port.device for port in list_ports.comports() if port.device)
    except Exception:
        pass
    return sorted(ports)


def auto_detect_serial_port() -> str:
    discovered_ports = discover_serial_ports()
    preferred_ports = [
        port
        for port in discovered_ports
        if port.startswith("/dev/cu.")
        and any(token in port.lower() for token in PREFERRED_PORT_TOKENS)
    ]
    if not preferred_ports:
        preferred_ports = [
            port
            for port in discovered_ports
            if any(token in port.lower() for token in PREFERRED_PORT_TOKENS)
        ]

    if preferred_ports:
        return preferred_ports[0]

    if discovered_ports:
        port_lines = "\n".join(f"  - {port}" for port in discovered_ports)
    else:
        port_lines = "  - none"

    raise RuntimeError(
        "Could not auto-detect an Arduino serial port.\n"
        "Use --serial-port to select one explicitly.\n"
        f"Discovered ports:\n{port_lines}"
    )


def build_pin_state(movement: str) -> tuple[dict[str, int], int | None]:
    pins = {pin: 0 for pin in PIN_NUMBERS}
    active_pins = MOVEMENT_TO_PINS.get(movement, ())
    for pin in active_pins:
        pins[pin] = 1
    active_pin = int(active_pins[0]) if active_pins else None
    return pins, active_pin


def build_payload(
    *,
    sequence: int,
    timestamp_ms: int,
    movement: str,
    x1: int | None,
    y1: int | None,
    is_live: bool,
) -> dict[str, object]:
    pins, active_pin = build_pin_state(movement)
    return {
        "source": SOURCE_NAME,
        "sequence": sequence,
        "timestamp_ms": timestamp_ms,
        "is_live": is_live,
        "movement": movement,
        "axes": {"x1": x1, "y1": y1},
        "pins": pins,
        "active_pin": active_pin,
    }


def build_live_payload(frame: ParsedFrame, sequence: int, timestamp_ms: int | None = None) -> dict[str, object]:
    return build_payload(
        sequence=sequence,
        timestamp_ms=current_timestamp_ms() if timestamp_ms is None else timestamp_ms,
        movement=frame.movement,
        x1=frame.x1,
        y1=frame.y1,
        is_live=True,
    )


def build_stale_payload(sequence: int, timestamp_ms: int | None = None) -> dict[str, object]:
    return build_payload(
        sequence=sequence,
        timestamp_ms=current_timestamp_ms() if timestamp_ms is None else timestamp_ms,
        movement="neutral",
        x1=None,
        y1=None,
        is_live=False,
    )


def current_timestamp_ms() -> int:
    return int(time.time() * 1000)


def send_payload(
    sock: socket.socket,
    udp_host: str,
    udp_port: int,
    payload: dict[str, object],
) -> None:
    message = json.dumps(payload, separators=(",", ":")).encode("utf-8")
    sock.sendto(message, (udp_host, udp_port))


def import_serial_module():
    try:
        import serial
    except ImportError as exc:
        raise SystemExit(
            "pyserial is required. Install it with "
            "`python3 -m pip install -r bridge/requirements.txt`."
        ) from exc
    return serial


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mirror Arduino joystick movement into UDP virtual pin updates."
    )
    parser.add_argument("--serial-port", help="Serial port for the Arduino")
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate")
    parser.add_argument(
        "--udp-host",
        default="127.0.0.1",
        help="Destination host for UDP payloads",
    )
    parser.add_argument(
        "--udp-port",
        type=int,
        default=8765,
        help="Destination port for UDP payloads",
    )
    parser.add_argument(
        "--stale-timeout-ms",
        type=int,
        default=250,
        help="Milliseconds without a valid frame before sending neutral fail-safe",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    serial = import_serial_module()
    parser = SerialOutputParser()
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    stale_sent = False
    status_message: str | None = None
    stale_timeout_seconds = max(args.stale_timeout_ms, 1) / 1000.0

    while True:
        try:
            serial_port = args.serial_port or auto_detect_serial_port()
        except RuntimeError as exc:
            if not stale_sent:
                send_payload(
                    udp_socket,
                    args.udp_host,
                    args.udp_port,
                    build_stale_payload(sequence),
                )
                sequence += 1
                stale_sent = True
            if str(exc) != status_message:
                print(str(exc), file=sys.stderr)
                status_message = str(exc)
            time.sleep(RECONNECT_DELAY_SECONDS)
            continue

        try:
            with serial.Serial(serial_port, args.baud, timeout=READ_TIMEOUT_SECONDS) as handle:
                connected_message = f"Connected to {serial_port} at {args.baud} baud."
                if connected_message != status_message:
                    print(connected_message, file=sys.stderr)
                    status_message = connected_message

                parser.reset()
                connected_at = time.monotonic()
                last_frame_at: float | None = None

                while True:
                    raw_line = handle.readline()
                    now = time.monotonic()

                    if raw_line:
                        frame = parser.feed_line(raw_line.decode("utf-8", errors="replace"))
                        if frame is not None:
                            send_payload(
                                udp_socket,
                                args.udp_host,
                                args.udp_port,
                                build_live_payload(frame, sequence),
                            )
                            sequence += 1
                            last_frame_at = now
                            stale_sent = False

                    reference_time = connected_at if last_frame_at is None else last_frame_at
                    if now - reference_time > stale_timeout_seconds and not stale_sent:
                        send_payload(
                            udp_socket,
                            args.udp_host,
                            args.udp_port,
                            build_stale_payload(sequence),
                        )
                        sequence += 1
                        stale_sent = True
        except (serial.SerialException, OSError) as exc:
            parser.reset()
            disconnect_message = f"Serial connection lost: {exc}"
            if disconnect_message != status_message:
                print(disconnect_message, file=sys.stderr)
                status_message = disconnect_message
            if not stale_sent:
                send_payload(
                    udp_socket,
                    args.udp_host,
                    args.udp_port,
                    build_stale_payload(sequence),
                )
                sequence += 1
                stale_sent = True
            time.sleep(RECONNECT_DELAY_SECONDS)


if __name__ == "__main__":
    raise SystemExit(main())
