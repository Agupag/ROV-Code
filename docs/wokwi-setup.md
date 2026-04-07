# Wokwi Setup

## Purpose

This document explains how to run the current ROV controller in Wokwi and what each simulation file does.

## Files Required For The Current Simulation

The default Wokwi project should use:

- [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino)
- [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json)
- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)

You do not need [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c) for the current diagram unless you intentionally switch the custom part used in `diagram.json`.

## Current Simulated Wiring

The current [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json) is wired for:

- Arduino Nano
- joystick 1 on `A2`, `A3`, and `D2`
- joystick 2 on `A6`, `A7`
- RF24 payload chip on `D8`, `D9`, `D11`, `D12`, `D13`
- LCD I2C on `A4`, `A5`
- microSD card on `D10`, `D11`, `D12`, `D13`
- logic analyzer on the SPI and IRQ lines

## How To Import

1. Create a new Arduino Nano project in Wokwi.
2. Replace the generated Arduino sketch with [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino).
3. Replace the default `diagram.json` with [diagram.json](/Users/connor/Desktop/ROV%20Code/diagram.json).
4. Add [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) and [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json) to the same Wokwi project.
5. Start the simulation.

## What To Expect

When the simulation is working:

- moving joystick 1 changes the outgoing thruster commands
- the 4 payload LEDs turn on when the corresponding thruster command is nonzero
- the LCD displays synthetic payload telemetry
- the SD card initialization path runs through the normal Arduino code path
- the logic analyzer shows SPI activity on radio transactions

## Best Debugging Method

### Serial Output

Set `diagnostics = true` in [newcodeMINIMAL CHANGE.ino](/Users/connor/Desktop/ROV%20Code/newcodeMINIMAL%20CHANGE.ino) if you want to inspect:

- movement labels
- joystick values
- servo angles
- control packet contents
- received payload data

### Logic Analyzer

The logic analyzer is attached to:

- `CE`
- `CSN`
- `SCK`
- `MOSI`
- `MISO`
- `IRQ`

Use it when:

- SPI transfers appear broken
- the payload chip is not responding
- `radio.available()` never seems to succeed
- ACK payload timing or RX state looks wrong

## Custom Chip Notes

[payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c) is not a full nRF24 model. It only implements the subset needed by the current controller and the RF24 library usage in this project.

It currently focuses on:

- register reads and writes
- RX payload reporting
- ACK payload input handling
- IRQ assertion
- telemetry packet generation
- thruster visualization

If the controller behavior changes substantially, the custom chip may need to be updated as well.

## Common Problems

### The Custom Chip Does Not Load

Check that both:

- [payload.chip.c](/Users/connor/Desktop/ROV%20Code/payload.chip.c)
- [payload.chip.json](/Users/connor/Desktop/ROV%20Code/payload.chip.json)

are present in the Wokwi project.

### The LCD Stays Blank

Check that:

- the LCD part is I2C-based
- the address is `0x27`
- `SDA` is wired to `A4`
- `SCL` is wired to `A5`

### The Radio Never Seems To Respond

Check:

- `CE` on `D8`
- `CSN` on `D9`
- `MOSI` on `D11`
- `MISO` on `D12`
- `SCK` on `D13`
- logic analyzer traces
- whether `diagnostics = true` produces packet logging

### SD Initialization Fails

Check the SD wiring:

- `CS` on `D10`
- `DI` on `D11`
- `DO` on `D12`
- `SCK` on `D13`

## When To Use `rf24.chip.c`

Use [rf24.chip.c](/Users/connor/Desktop/ROV%20Code/rf24.chip.c) only if you want a different Wokwi part that exposes separate forward and reverse output pins for each motor instead of the current compact payload LED representation.
