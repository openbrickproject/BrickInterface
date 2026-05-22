#!/usr/bin/env bash
# Flash the BrickInterface firmware to a CH552T board.
#
# Prerequisites:
#   1. wchisp installed: `cargo install wchisp`
#   2. Board plugged in and in WCH ROM bootloader mode
#      (blank board: just plug in; flashed board: hold SW1 while plugging in,
#       or send the ENTER_BOOTLOADER serial command beforehand)
#   3. Firmware compiled by Arduino IDE at least once so a .hex exists in the
#      build cache
#
# Usage:
#   ./upload.sh                # flash the latest cached .hex
#   ./upload.sh path/to/X.hex  # flash a specific .hex
#
# After successful flash, the board re-enumerates as VID 0x1209 / PID 0xc550
# and presents a USB CDC serial port.

set -euo pipefail

WCHISP="${WCHISP:-$HOME/.cargo/bin/wchisp}"

if [[ ! -x "$WCHISP" ]]; then
    if ! command -v wchisp >/dev/null 2>&1; then
        echo "error: wchisp not found at $WCHISP and not on PATH"
        echo "       install with: cargo install wchisp"
        exit 1
    fi
    WCHISP="wchisp"
fi

# Resolve the .hex to flash. Caller can pass a path, or we hunt for the
# most recently built cache hex.
if [[ $# -ge 1 ]]; then
    HEX="$1"
else
    HEX=$(find "$HOME/Library/Caches/arduino/sketches" \
                -name "BrickInterface.ino.hex" \
                -print0 2>/dev/null \
            | xargs -0 stat -f '%m %N' 2>/dev/null \
            | sort -rn \
            | awk '{print $2; exit}')
fi

if [[ -z "${HEX:-}" || ! -f "$HEX" ]]; then
    echo "error: no firmware .hex found"
    echo "       compile in Arduino IDE first, then re-run this script,"
    echo "       or pass an explicit path: ./upload.sh path/to/BrickInterface.ino.hex"
    exit 1
fi

echo ">> flashing: $HEX"
echo ">> size: $(wc -c <"$HEX" | tr -d ' ') bytes (Intel HEX, includes record overhead)"

# macOS requires sudo for libusb to claim the WCH bootloader USB device.
# Linux usually doesn't, but sudo works there too.
SUDO=""
if [[ "$(uname)" == "Darwin" ]]; then
    SUDO="sudo"
    echo ">> (using sudo — macOS libusb requires elevated USB access)"
fi

# Probe first — gives a clean error if the board isn't in bootloader mode.
echo ">> probing bootloader..."
if ! $SUDO "$WCHISP" probe 2>&1; then
    echo
    echo "error: no CH55x bootloader detected on USB"
    echo "       - blank board: just plug it in"
    echo "       - flashed board: hold SW1 while plugging in USB"
    echo "       - already-running board: send AA 02 99 05 9E over serial to"
    echo "         enter bootloader from firmware"
    exit 1
fi

echo ">> writing flash..."
$SUDO "$WCHISP" flash "$HEX"

echo
echo "done. The board should re-enumerate as VID 0x1209 / PID 0xc550."
echo "send PING (AA 02 5A 01 59) to the new serial port to verify."
