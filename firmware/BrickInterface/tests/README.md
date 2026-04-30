# BrickInterface firmware tests

Host-side unit tests for the firmware's pure-logic and protocol-encoder code.
Compiles with `g++` on macOS/Linux — no CH552 required.

## Run

```sh
cd firmware/BrickInterface/tests
make run
```

Expected:

```
==> test_packet         10/10 passed
==> test_ir_pf          14/14 passed
==> test_ir_legacy       5/5 passed
==> test_ir_rcx          7/7 passed
=================================
Suites: 4 pass, 0 fail
```

## How it works

Each test file `#include`s the production `.cpp` directly, exposing static
helpers as well as the public API. CH552 hardware (PWM, timers, SFRs) is
stubbed out via `Arduino.h` (in this directory) so the firmware compiles on
the host. No production code changes for testability beyond two minor
adjustments:

- `ir_engine.cpp` includes `protocol.h` for `IR_ENGINE_*` constants
- `pfBuildNibbles` call casts away `volatile` for C++ compatibility

## What's covered

**Layer 1 — pure logic:**
- Packet parser: framing, checksum, length bounds, resynchronisation
- Packet builder: round-trip with parser
- PF nibble builder: all 3 modes, toggle alternation, LRC
- IR math primitives: `pfLRC`, `oddParity`
- RCX framing: header/data/checksum/complement layout

**Layer 2 — phase generators (mock-hardware integration):**
- PF: full 5-repeat transmission, mark/space timing, inter-message gaps
- Legacy: bit timing, carrier inversion for 1s, gap after frame
- RCX: start bit, byte length, transmission completion

## Adding tests

Add a new `TEST(name) { ... }` block to any existing file, or create a new
test file and add it to `TESTS` in the Makefile. Each `TEST` is auto-registered
via `__attribute__((constructor))`.
