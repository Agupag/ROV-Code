# File Guide

## Main Arduino Files

- [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino)
  Current primary controller sketch. Use this first.
- [debuggedNewCode.ino](/Users/connor/Desktop/ROV%20Code/debuggedNewCode.ino)
  Debug-heavy RF24 variant with extra logging behavior.
- [newCode.ino](/Users/connor/Desktop/ROV%20Code/newCode.ino)
  Older integrated variant that diverged more from the original shore-code structure.
- [joystickCode.ino](/Users/connor/Desktop/ROV%20Code/joystickCode.ino)
  Original joystick movement prototype and movement vocabulary reference.
- [engr100 code.ino](/Users/connor/Desktop/ROV%20Code/engr100%20code.ino)
  Earlier integration sketch used as a stepping stone.
- [engr100_joystick_integration_example.ino](/Users/connor/Desktop/ROV%20Code/engr100_joystick_integration_example.ino)
  Example of integrating joystick movement into the ENGR100 control-state model.
- [engr100_uno_led_test.ino](/Users/connor/Desktop/ROV%20Code/engr100_uno_led_test.ino)
  Uno-oriented LED test harness for thruster output visualization.
- [shoreCode-V1.1.ino](/Users/connor/Desktop/ROV%20Code/shoreCode-V1.1.ino)
  Original shore-side baseline.

## Wokwi Simulation Files

- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
  Current Wokwi wiring and part layout.
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
  Active custom payload/radio simulation used by the diagram.
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)
  Part definition for the payload chip.
- [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c)
  Alternate custom RF24 simulation with separate forward/reverse outputs.
- [rf24.chip.json](/Users/connor/Desktop/ROV%20Code/rf24.chip.json)
  Part definition for the alternate RF24 chip.

## Bridge Files

- [bridge/arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/arduino_virtual_pin_bridge.py)
  Serial parser and UDP payload emitter.
- [bridge/tests/test_arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/tests/test_arduino_virtual_pin_bridge.py)
  Unit tests for the bridge parser and payload mapping.
- [bridge/requirements.txt](/Users/connor/Desktop/ROV%20Code/bridge/requirements.txt)
  Python dependencies for the bridge.

## Which File To Edit

Edit [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino) when:

- changing current shore-controller behavior
- adjusting pin mappings for the current Nano design
- changing joystick-to-thruster mapping
- changing the main RF24 control flow

Edit [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) when:

- changing the Wokwi wiring
- adding or removing simulated hardware parts
- changing which custom chip is instantiated

Edit [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) when:

- Wokwi RF24 behavior is wrong in the current diagram
- packet handling is wrong in simulation
- the payload LED outputs do not match the expected control packet

Edit [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c) when:

- you intentionally need the alternate RF24 custom chip
- you want explicit forward/reverse outputs rather than compact LED outputs

Edit [bridge/arduino_virtual_pin_bridge.py](/Users/connor/Desktop/ROV%20Code/bridge/arduino_virtual_pin_bridge.py) when:

- changing the serial frame contract for the bridge workflow
- changing UDP payload format
- changing movement-to-virtual-pin mapping

## Historical Versus Current

Treat these as historical unless the task says otherwise:

- [shoreCode-V1.1.ino](/Users/connor/Desktop/ROV%20Code/shoreCode-V1.1.ino)
- [engr100 code.ino](/Users/connor/Desktop/ROV%20Code/engr100%20code.ino)
- [newCode.ino](/Users/connor/Desktop/ROV%20Code/newCode.ino)

Treat these as current:

- [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino)
- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)

## Supporting References

For broader context, also read:

- [README.md](/Users/connor/Desktop/ROV%20Code/README.md)
- [docs/architecture.md](/Users/connor/Desktop/ROV%20Code/docs/architecture.md)
- [docs/wokwi-setup.md](/Users/connor/Desktop/ROV%20Code/docs/wokwi-setup.md)
