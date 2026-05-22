# BrickInterface

A small USB bridge board that lets a computer drive vintage LEGO control
systems over a USB-C virtual serial port. Built around the WCH **CH552T**
8051-core MCU with native USB.

One board speaks to four different LEGO control systems:

- **LEGO Interface A** (set 9750) — 6 PWM outputs and 2 sensor inputs over the
  20-pin ribbon connector
- **LEGO Power Functions IR** (38 kHz)
- **LEGO "Legacy" IR** (76 kHz — Code Pilot / 9V RC train era)
- **LEGO RCX IR** (38 kHz — Mindstorms RCX)

The host talks to the board through a simple framed binary protocol over USB
CDC (it enumerates as a `/dev/cu.usbmodem*` / `COM*` serial port).

## Repository layout

```
firmware/BrickInterface/   CH552T firmware (ch55xduino / SDCC)
  *.ino, *.c, *.h          dispatcher, Interface A, IR engine, packet parser
  tests/                   host-side unit tests (g++, no hardware needed)
kicad/                     KiCad hardware design — schematic, PCB, gerbers, BOM
host/                      host-side tooling (Python)
  dance.py                 demo: cycles IFA outputs + PF IR over the wire
upload.sh                  flash helper (wchisp)
```

## Hardware

- **MCU:** CH552T (TSSOP-20), internal 24 MHz RC oscillator (no crystal)
- **USB:** USB-C, native full-speed device — enumerates as USB CDC ACM
- **Interface A:** 20-pin IDC to the 9750 (6 outputs P1.4–P1.7/P3.0–P3.1,
  2 inputs P1.0/P1.1)
- **IR:** single IR LED on P3.4, driven by a two-transistor constant-current
  sink; carrier is bit-banged from a Timer 2 ISR (the CH552 hardware PWM can't
  hit 38/76 kHz cleanly)

### v1 board erratum — remove R9

On the first hardware revision, the BOOT-button pull path (R9 → BOOT_PULL → SW1)
holds USB D+ high enough at reset that the board enters the ROM bootloader on
every plug-in **and** the bootloader→firmware USB hand-off never completes, so
the CDC port never appears. **Lifting R9 fixes both** — the board boots straight
to firmware and enumerates as a serial port normally.

With R9 removed you don't lose the ability to re-flash: the firmware accepts a
`ENTER_BOOTLOADER` command over USB and jumps to the ROM bootloader in software.
A future board revision should drop R9/SW1 (or add a pull-down on BOOT_PULL).

## Firmware

Built with [ch55xduino](https://github.com/DeqingSun/ch55xduino) (SDCC toolchain,
C only). Open `firmware/BrickInterface/BrickInterface.ino` in the Arduino IDE
with the CH552 board package installed, select **USB Settings → Default CDC**,
and compile.

### Flashing

The board exposes the WCH ROM bootloader (USB VID:PID `4348:55E0`) for flashing.
`upload.sh` wraps [`wchisp`](https://github.com/ch32-rs/wchisp):

```sh
./upload.sh                 # flashes the most recent built .hex
```

To enter the bootloader on an R9-removed board, send the `ENTER_BOOTLOADER`
command over the serial port (the host tool drops the device into ROM), or
power-cycle while momentarily pulling D+ high.

### Tests

Pure-logic and protocol-encoder code is unit-tested on the host (no CH552
required):

```sh
cd firmware/BrickInterface/tests
make run
```

## Host tooling

`host/dance.py` is a reference client and visual self-test: it cycles the
Interface A outputs and fires Power Functions IR commands in sequence.

```sh
cd host
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
./dance               # auto-finds the serial port; Ctrl-C to stop
```

It auto-detects the port by scanning for `/dev/cu.usbmodem*`. Use `--port` to
override, `--once` for a single pass, `--dwell N` to change the per-step time.

## Protocol

USB-CDC wire protocol. All frames, both directions, share one format:

```
+------+------+------+------+----------- ... -----------+------+
| 0xAA | LEN  | SEQ  | CMD  |          PAYLOAD          | CHK  |
+------+------+------+------+----------- ... -----------+------+
 1 byte 1 byte 1 byte 1 byte    0..32 bytes (LEN-2)      1 byte
```

- **SOF** = `0xAA`
- **LEN** = bytes from SEQ through end of payload (`2 + payload_len`)
- **SEQ** = correlation byte; replies echo it, async events use `0x00`
- **CHK** = XOR of LEN, SEQ, CMD, and all payload bytes

Multi-byte fields are little-endian. The parser resynchronises on the next
`0xAA` after any malformed frame.

### Commands (host → device)

| Code | Name | Payload |
|------|------|---------|
| `0x01` | `PING` | — |
| `0x02` | `GET_VERSION` | — |
| `0x03` | `GET_CAPABILITIES` | — |
| `0x04` | `RESET_STATE` | — |
| `0x05` | `ENTER_BOOTLOADER` | — |
| `0x10` | `IFACE_SET_OUTPUTS` | 6 duty bytes (0–255), or 6 + 1 mask byte |
| `0x11` | `IFACE_GET_INPUTS` | — |
| `0x12` | `IFACE_GET_COUNTS` | — |
| `0x13` | `IFACE_RESET_COUNT` | input number (6 or 7) |
| `0x20` | `PF_SEND` | channel (0–3), mode, data, flags |
| `0x30` | `LEGACY_SEND` | channel code (4–7), orange nibble, yellow nibble |
| `0x40` | `IR_ABORT_ALL` | — |
| `0x50` | `RCX_SEND` | carrier mode (ignored), 1–16 data bytes |
| `0x51` | `RCX_SEND_RAW` | carrier mode (ignored), raw bytes |

### Replies / events (device → host)

| Code | Name |
|------|------|
| `0x81` | `PONG` |
| `0x82` | `VERSION` (proto major/minor, fw major/minor) |
| `0x83` | `CAPABILITIES` (uint16 LE bitmap) |
| `0x84` | `OK` |
| `0x90` | `IFACE_INPUTS` (bit0 = input 6, bit1 = input 7) |
| `0x91` | `IFACE_COUNTS` (two uint32 LE edge counts) |
| `0xA0` | `IR_ACCEPTED` (token, engine id) |
| `0xA1` | `IR_DONE` (token, engine id; async, SEQ = `0x00`) |
| `0xE0` | `ERROR` (error code, detail) |

The Interface A outputs are 8-bit software PWM (~100 Hz). PF/Legacy/RCX IR jobs
are queued asynchronously: the device replies `IR_ACCEPTED` immediately and
emits `IR_DONE` when the 5-repeat burst finishes. For held-button / continuous
behaviour the host re-sends the IR command at the protocol's repeat cadence.

### Device discovery

The board enumerates with the generic ch55xduino CDC identity (VID `0x1209`,
PID `0xC550`). A client should filter serial ports by that VID/PID and confirm
identity with a `GET_CAPABILITIES` handshake (BrickInterface reports capability
bitmap `0x0037`).

## Status

- **Interface A** (6 PWM outputs, 2 counting inputs) — working, verified on hardware
- **Power Functions IR** — working, verified against a PF receiver
- **Legacy IR / RCX IR** — implemented, not yet verified against period hardware
- **USB CDC** — working on v1 boards with R9 removed (see erratum above)

## License

This project is dual-licensed by component:

- **Software** — the firmware (`firmware/`) and host tooling (`host/`) — is
  licensed under the **Apache License, Version 2.0**. See [`LICENSE`](LICENSE).
- **Hardware** — the KiCad design, gerbers, and BOM (`kicad/`) — is licensed
  under the **CERN Open Hardware Licence Version 2 – Permissive
  (CERN-OHL-P v2)**. See [`kicad/LICENSE`](kicad/LICENSE).

Both are permissive licenses with a patent grant. Copyright 2024–2026
Nathan Kellenicki.
