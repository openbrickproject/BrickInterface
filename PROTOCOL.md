# BrickInterface Protocol

Framed binary protocol over USB-CDC for driving **LEGO Interface A** (9750 control box) and **LEGO Power Functions** (38 kHz IR) from a host computer.

- **Protocol version:** 1.1 · **Firmware version:** 0.6
- **Transport:** USB-CDC ACM virtual serial port (`/dev/cu.usbmodem*`, `/dev/ttyACM0`, `COM*`). Baud is irrelevant; open at any rate.
- **USB identity:** VID `0x1209`, PID `0xC550`.
- **Endianness:** all multi-byte fields little-endian.

> On Windows, assert DTR on open. The CH55x stack gates TX on DTR.

## Contents

- [Frame format](#frame-format)
- [Command summary](#command-summary)
- [Reply summary](#reply-summary)
- [Error codes](#error-codes)
- [Core / session](#core--session)
- [Interface A](#interface-a)
- [Power Functions](#power-functions)
- [Examples](#examples)
- [References](#references)

---

## Frame format

```
+------+------+------+------+----------- ... -----------+------+
| SOF  | LEN  | SEQ  | CMD  |          PAYLOAD          | CHK  |
+------+------+------+------+----------- ... -----------+------+
 1 byte 1 byte 1 byte 1 byte    0..40 bytes (LEN-2)      1 byte
```

| Field | Bytes | Description |
| --- | --- | --- |
| **SOF** | 1 | Start of frame, always `0xAA`. |
| **LEN** | 1 | Bytes from SEQ through end of payload (`2 + payload_len`). Range 2–42. |
| **SEQ** | 1 | Correlation byte set by sender; replies echo it. Async events use `0x00`. |
| **CMD** | 1 | Command code (host→device) or reply code (device→host). |
| **PAYLOAD** | LEN−2 | Command-specific, 0–40 bytes. |
| **CHK** | 1 | XOR of LEN, SEQ, CMD, and every payload byte. SOF is not covered. |

The receiver discards bytes until `0xAA`, reads LEN (rejecting <2 or >42), reads that many bytes, then verifies CHK. A failing frame is dropped silently and the parser resyncs on the next `0xAA`. There is no inter-byte timeout. (Protocol 1.0 devices cap LEN at 34 and payload at 32 bytes. No command in this document needs more.)

---

## Command summary

| Code | Name | Payload | Reply |
| --- | --- | --- | --- |
| `0x01` | `PING` | — | `PONG` |
| `0x02` | `GET_VERSION` | — | `VERSION` |
| `0x03` | `GET_CAPABILITIES` | — | `CAPABILITIES` |
| `0x04` | `RESET_STATE` | — | `OK` |
| `0x05` | `ENTER_BOOTLOADER` | — | `OK`, then re-enumerates |
| `0x10` | `IFACE_SET_OUTPUTS` | `mask`, duty[…] | `OK` |
| `0x11` | `IFACE_GET_INPUTS` | — | `IFACE_INPUTS` |
| `0x12` | `IFACE_GET_COUNTS` | — | `IFACE_COUNTS` |
| `0x13` | `IFACE_RESET_COUNT` | `input` | `OK` |
| `0x20` | `PF_SEND` | `channel`, `mode`, `data`, `flags` | `IR_ACCEPTED`, then `IR_DONE` |
| `0x40` | `IR_ABORT_ALL` | — | `OK` (plus `IR_DONE` for the aborted send) |

---

## Reply summary

| Code | Name | Payload |
| --- | --- | --- |
| `0x81` | `PONG` | — |
| `0x82` | `VERSION` | `proto_major`, `proto_minor`, `fw_major`, `fw_minor` |
| `0x83` | `CAPABILITIES` | `cap_lo`, `cap_hi` (u16 LE bitmap) |
| `0x84` | `OK` | — |
| `0x90` | `IFACE_INPUTS` | `state` (bit 0 = input 6, bit 1 = input 7) |
| `0x91` | `IFACE_COUNTS` | `count6` (u32 LE), `count7` (u32 LE) |
| `0xA0` | `IR_ACCEPTED` | `token`, `engine_id` |
| `0xA1` | `IR_DONE` | `token`, `engine_id` (async, SEQ `0x00`) |
| `0xE0` | `ERROR` | `error_code`, `detail` |

Every reply echoes the request's SEQ. The exception is `IR_DONE`, which is async (SEQ `0x00`) and is matched to its `IR_ACCEPTED` by `token`.

---

## Error codes

`ERROR` (`0xE0`) payload is `error_code`, `detail` (`detail` often `0`).

| Code | Name | Meaning |
| --- | --- | --- |
| `0x02` | `ERR_BAD_LENGTH` | Payload length doesn't match the command. |
| `0x03` | `ERR_UNKNOWN_CMD` | Command code not recognised. |
| `0x04` | `ERR_BAD_ARGUMENT` | Argument out of range. |
| `0x06` | `ERR_BUSY` | IR engine mid-transmission. Wait for `IR_DONE`, then retry. |

Codes `0x01` (`ERR_BAD_CHECKSUM`), `0x05` (`ERR_QUEUE_FULL`), `0x07` (`ERR_UNSUPPORTED`), and `0x08` (`ERR_INVALID_STATE`) are reserved and never sent. Bad-checksum frames are dropped, not reported.

---

## Core / session

`RESET_STATE` returns the board to a clean slate. It drops all outputs to zero, clears both edge counters, aborts any in-flight IR (emitting the aborted send's `IR_DONE`, see `IR_ABORT_ALL`), and resets the PF toggle bits.

`ENTER_BOOTLOADER` replies `OK`, flushes USB, and jumps to the CH55x ROM bootloader. The port disappears and the board re-enumerates as the WCH flashing device until it is reflashed or power-cycled.

`CAPABILITIES` reports a u16 LE bitmap of the engines present:

| Bit | Mask | Capability |
| --- | --- | --- |
| 0 | `0x0001` | `CAP_INTERFACE_A` |
| 1 | `0x0002` | `CAP_PF_IR` |

A BrickInterface board reports `0x0037`. Reject a device that lacks the bit for the hardware you intend to drive.

---

## Interface A

6 output ports (`0`–`5`) and 2 input ports (`6`, `7`) over the 20-pin ribbon. Every output is 8-bit software PWM (~100 Hz). The duty sets the speed of a motor or the brightness of a light: `0` = off through `255` = full on.

### `IFACE_SET_OUTPUTS` (`0x10`)

Sets any subset of the six outputs in one atomic write. Payload `[mask, d₀, d₁, …]`.

| Field | Description |
| --- | --- |
| `mask` | Bit *i* selects output *i* (bits 0–5). Bits 6–7 ignored. |
| duty bytes | One byte (0–255) per set bit, ascending bit order. Unselected outputs keep their duty. |

Length must equal `1 + popcount(mask)`, else `ERR_BAD_LENGTH`. There is no read-back. The host owns output state.

### `IFACE_GET_INPUTS` (`0x11`)

Reports the state of the two inputs. Reply `IFACE_INPUTS` `state`: bit 0 = input 6, bit 1 = input 7. The inputs are pulled up, so the raw bit is `1` when the input is open and `0` when it is closed (pressed, or pulled to ground by a sensor). This inverts the TC Logo and Control Lab boolean, where a pressed touch sensor reports *true*.

### `IFACE_GET_COUNTS` (`0x12`) / `IFACE_RESET_COUNT` (`0x13`)

Each input has a free-running u32 edge counter. It counts the number of times the input's boolean state changes from false to true: one count per touch press, or per dark slice of an optosensor disk. Electrically that is a HIGH→LOW edge. The count wraps at 2³². `IFACE_GET_COUNTS` reports both counts as `count6` then `count7` (8 bytes). `IFACE_RESET_COUNT` clears one input's count. Its payload is the input number (`6` or `7`).

---

## Power Functions

38 kHz IR. Each `PF_SEND` is a one-shot burst implementing the PF RC v1.20 transmitter schedule in full: a channel-staggered initial delay of (4 − `channel`)·tm before the first message, then the message transmitted five times at start-to-start 5·tm (messages 1→2, 2→3) and (6 + 2·`channel`)·tm (messages 3→4, 4→5), with tm = 16 ms. The channel-dependent delay and spacing are the spec's multi-transmitter etiquette: uncoordinated transmitters on different channels cannot collide on all five copies. Accept-to-`IR_DONE` runs roughly 0.43 s (channel 0) to 0.57 s (channel 3). A `PF_SEND` issued mid-burst gets `ERR_BUSY`.

The board never auto-repeats beyond the burst. Plan for the receiver-side behaviour: `SINGLE_PWM` and `SINGLE_CST` latch until changed, but `COMBO_DIRECT` and `COMBO_PWM` have a lost-IR timeout and stop on their own. Re-send periodically to hold a state in those modes.

### `PF_SEND` (`0x20`)

Transmits one Power Functions message as a 38 kHz IR burst. Payload `channel`, `mode`, `data`, `flags`.

| Field | Description |
| --- | --- |
| `channel` | `0`–`3` (LEGO channels 1–4). `>3` → `ERR_BAD_ARGUMENT`. |
| `mode` | Interpretation of `data`; see below. |
| `data` | Mode-specific byte. |
| `flags` | bit 0 = override toggle (use bit 1 as value); bit 1 = toggle value; bits 2–7 reserved, send 0. If bit 0 clear, board auto-toggles per channel. |

Each channel addresses two outputs, **A** (red) and **B** (blue). In `COMBO_PWM` the protocol carries an address bit where the toggle bit normally sits. The board always sends address 0 (the only address stock receivers listen to), and the `flags` toggle bits are ignored in that mode.

### Modes

| `mode` | Name | `data` byte |
| --- | --- | --- |
| `0x00` | `COMBO_DIRECT` | bits 1–0 = output A, bits 3–2 = output B. Per output: `0` float, `1` forward, `2` reverse, `3` brake. |
| `0x01` | `SINGLE_PWM` | bit 4 = output (0 = A, 1 = B); low nibble = speed step. |
| `0x02` | `SINGLE_CST` | bit 4 = output; low nibble = clear/set/toggle command (PF v1.20 §3). |
| `0x03` | `COMBO_PWM` | bits 7–4 = output B step; bits 3–0 = output A step. |

### Speed steps (`SINGLE_PWM`, `COMBO_PWM`)

| Step | Meaning |
| --- | --- |
| `0` | float (coast) |
| `1`–`7` | forward, 1/7 → full |
| `8` | brake |
| `9`–`15` | reverse, full → 1/7 |

A forward/reverse direction with 0–7 power maps to `step = power` (forward) or `step = 16 − power` (reverse).

### `IR_ABORT_ALL` (`0x40`)

Stops any in-flight or queued IR transmission immediately and forces the IR LED off. Replies `OK`. The aborted request's `IR_DONE` event is still emitted, so every `IR_ACCEPTED` token resolves with exactly one `IR_DONE`, whether the burst completed or was cut short. `RESET_STATE` aborts the same way.

---

## Examples

All bytes hex, as they appear on the wire.

| Direction | Frame | Meaning |
| --- | --- | --- |
| H→D | `AA 02 03 03 02` | `GET_CAPABILITIES` |
| D→H | `AA 04 03 83 37 00 B3` | `CAPABILITIES` = `0x0037` |
| H→D | `AA 05 10 10 05 FF FF 00` | Set outputs 0 & 2 full (`mask=0x05`, duty `FF FF`) |
| D→H | `AA 02 10 84 96` | `OK` |
| H→D | `AA 04 10 10 08 80 8C` | Dim output 3 to 128 (`mask=0x08`) |
| H→D | `AA 02 11 12 01` | `IFACE_GET_COUNTS` |
| D→H | `AA 0A 11 91 05 00 00 00 00 00 00 00 8F` | `IFACE_COUNTS`: count6=5, count7=0 |
| H→D | `AA 06 20 20 00 01 17 00 10` | `PF_SEND` ch1 output B forward full (`SINGLE_PWM`, data `0x17`) |
| D→H | `AA 04 20 A0 7B 01 …` | `IR_ACCEPTED`, token `0x7B`, engine `0x01` (PF) |
| D→H | `AA 04 00 A1 7B 01 …` | `IR_DONE`, token `0x7B` (async, SEQ `0x00`) |

To stop a PF motor, resend with `data = 0x10` (B, float) or `0x18` (B, brake).

---

## References

- [LEGO TC Logo Reference Guide](https://archive.org/details/lego-tc-logo-reference-guide). Reference for the Interface A (9750). It defines the output and input port model, the true/false sense of the touch and optosensors, and the edge-count behaviour reproduced here.
- [LEGO Power Functions RC Protocol v1.20](http://images.groundzero.com.pt/LEGO_Power_Functions_RC_v120.pdf). The 38 kHz encoding and the toggle, escape, channel, mode, and data fields behind `PF_SEND`.
