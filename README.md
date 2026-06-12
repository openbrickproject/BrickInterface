# BrickInterface

BrickInterface is a modern USB-C board that bridges vintage LEGO electronics where no official solution exists. As of now, it supports the **LEGO Interface A** and **LEGO Power Functions**.

The host talks to the board with a framed binary protocol over USB-CDC, and the board appears as a standard serial port.

## Repository layout

```
firmware/BrickInterface/   CH552T firmware (ch55xduino / SDCC)
  *.ino, *.c, *.h          dispatcher, Interface A, IR engine, packet parser
  tests/                   host-side unit tests (g++)
kicad/                     KiCad hardware design (schematic, PCB, gerbers, BOM)
upload.sh                  flash helper (wchisp)
```

## Hardware

- **MCU:** CH552T (TSSOP-20), internal 24 MHz RC oscillator (no crystal)
- **USB:** USB-C, native full-speed device. Enumerates as USB CDC ACM
- **Interface A:** 20-pin IDC to the 9750
- **IR:** Two IR LEDs in parallel with different beam angles: a narrow TSAL6100 (±10°) for range and a wider TSAL6200 (±17°) for off-axis coverage

## Firmware

Built with [ch55xduino](https://github.com/DeqingSun/ch55xduino) (SDCC toolchain, C). Open `firmware/BrickInterface/BrickInterface.ino` in the Arduino IDE with the CH552 board package installed, select **USB Settings > Default CDC**, and compile.

### Flashing

The board exposes the WCH ROM bootloader (USB VID:PID `4348:55E0`) for flashing.

To enter the bootloader, send the `ENTER_BOOTLOADER` command over the serial port, or power-cycle while holding the BOOT button (SW1) to pull D+ high.

(Note: On board revisions v1.0.4 or lower, SW1 was incorrectly wired to the 5V rail instead of 3.3V. Resistor R9 needs to be removed to pull D+ down, however this breaks SW1. To enter BOOT, bridge R9's location (ie. with tweezers)).

### Tests

Pure-logic and protocol-encoder code is unit-tested on the host (no CH552 required):

```sh
cd firmware/BrickInterface/tests
make run
```

## Protocol

The host drives the board with a framed binary protocol on the USB-CDC serial port.

```
0xAA | LEN | SEQ | CMD | PAYLOAD… | CHK
```

Two commands, shown as hex on the wire:

```
AA 05 10 10 05 FF FF 00      # Interface A: outputs 0 & 2 full on
AA 06 20 20 00 01 17 00 10   # Power Functions: channel 1, output B, forward full
```

The board enumerates with the ch55xduino CDC identity (VID `0x1209`, PID `0xC550`). Use the `GET_CAPABILITIES` handshake to verify which hardware is supported (BrickInterface v1.0.5 and below reports `0x0037`).

See **[PROTOCOL.md](PROTOCOL.md)** for the full wire protocol: frame format, commands, replies, error codes, and worked examples.

## Status

- **Interface A** (6 PWM outputs, 2 counting inputs) — Working
- **Power Functions IR** — Working
- **Legacy IR / RCX IR** — Early implementation, however it's proven unreliable. Not recommended for use yet.

## License

This project is dual-licensed by component:

- **Software** (the firmware in `firmware/`) is licensed under the **Apache License, Version 2.0**. See [`LICENSE`](LICENSE).
- **Hardware** (the KiCad design, gerbers, and BOM in `kicad/`) is licensed under the **CERN Open Hardware Licence Version 2 – Permissive (CERN-OHL-P v2)**. See [`kicad/LICENSE`](kicad/LICENSE).

Both are permissive licenses with a patent grant. Copyright 2024–2026 Nathan Kellenicki.
