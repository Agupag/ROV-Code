import unittest

from bridge.arduino_virtual_pin_bridge import (
    MOVEMENT_TO_PINS,
    ParsedFrame,
    PIN_NUMBERS,
    build_live_payload,
    build_pin_state,
    build_stale_payload,
    parse_serial_lines,
)


class SerialOutputParserTests(unittest.TestCase):
    def test_parses_frame_with_blank_lines(self) -> None:
        frames = parse_serial_lines(
            [
                "forward\n",
                " \n",
                "y1:-27\n",
                "x1:18\n",
                " \n",
            ]
        )

        self.assertEqual(frames, [ParsedFrame(movement="forward", x1=18, y1=-27)])

    def test_parses_axes_in_either_order(self) -> None:
        frames = parse_serial_lines(
            [
                "left",
                "x1:-11",
                "y1:42",
                "right",
                "y1:-9",
                "x1:12",
            ]
        )

        self.assertEqual(
            frames,
            [
                ParsedFrame(movement="left", x1=-11, y1=42),
                ParsedFrame(movement="right", x1=12, y1=-9),
            ],
        )

    def test_drops_partial_frame_when_new_movement_arrives(self) -> None:
        frames = parse_serial_lines(
            [
                "forward",
                "y1:-27",
                "cornerRight",
                "x1:20",
                "y1:-22",
            ]
        )

        self.assertEqual(frames, [ParsedFrame(movement="cornerRight", x1=20, y1=-22)])

    def test_resets_on_malformed_axis_value(self) -> None:
        frames = parse_serial_lines(
            [
                "forward",
                "x1:not-a-number",
                "y1:-27",
                "forward",
                "x1:18",
                "y1:-27",
            ]
        )

        self.assertEqual(frames, [ParsedFrame(movement="forward", x1=18, y1=-27)])

    def test_ignores_unknown_movements(self) -> None:
        frames = parse_serial_lines(
            [
                "spin",
                "x1:50",
                "y1:10",
                "reverse",
                "y1:24",
                "x1:-30",
            ]
        )

        self.assertEqual(frames, [ParsedFrame(movement="reverse", x1=-30, y1=24)])


class PayloadTests(unittest.TestCase):
    def test_mapping_for_each_supported_movement(self) -> None:
        for movement, pins in MOVEMENT_TO_PINS.items():
            with self.subTest(movement=movement):
                payload = build_live_payload(
                    ParsedFrame(movement=movement, x1=3, y1=-4),
                    sequence=7,
                    timestamp_ms=123456789,
                )
                self.assertEqual(payload["active_pin"], int(pins[0]))
                for pin in pins:
                    self.assertEqual(payload["pins"][pin], 1)
                self.assertEqual(sum(payload["pins"].values()), len(pins))

    def test_reverse_movement_is_supported(self) -> None:
        payload = build_live_payload(
            ParsedFrame(movement="reverse", x1=0, y1=0),
            sequence=1,
            timestamp_ms=123,
        )

        self.assertEqual(payload["pins"]["10"], 1)
        self.assertEqual(payload["pins"]["11"], 1)
        self.assertEqual(payload["pins"]["12"], 1)

    def test_neutral_mapping_keeps_all_pins_low(self) -> None:
        pins, active_pin = build_pin_state("neutral")

        self.assertIsNone(active_pin)
        self.assertEqual(tuple(pins.keys()), PIN_NUMBERS)
        self.assertEqual(sum(pins.values()), 0)

    def test_stale_payload_matches_fail_safe_contract(self) -> None:
        payload = build_stale_payload(sequence=9, timestamp_ms=123456789)

        self.assertEqual(
            payload,
            {
                "source": "arduino_virtual_pin_bridge",
                "sequence": 9,
                "timestamp_ms": 123456789,
                "is_live": False,
                "movement": "neutral",
                "axes": {"x1": None, "y1": None},
                "pins": {pin: 0 for pin in PIN_NUMBERS},
                "active_pin": None,
            },
        )


if __name__ == "__main__":
    unittest.main()
