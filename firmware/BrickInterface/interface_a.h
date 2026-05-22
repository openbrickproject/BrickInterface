#ifndef INTERFACE_A_H
#define INTERFACE_A_H

#include <stdint.h>
#include "board_config.h"

void ifaceInit(void);

// PWM state shared with the Timer 1 ISR. The ISR itself lives in the .ino
// translation unit (SDCC vector installation requirement) and accesses these.
extern volatile __xdata uint8_t pwmDuty[6];
extern volatile __data    uint8_t pwmCounter;

// Apply PWM duties to selected outputs. `mask` bit i set = apply the next
// packed duty byte to output i; bit clear = leave that output untouched.
// `packed_duties` contains exactly popcount(mask) bytes, one per set bit,
// in ascending bit order. Pass mask=0x3F with six bytes to update all six.
void ifaceSetOutputs(uint8_t mask, const uint8_t *packed_duties);

// Force all outputs to 0 (used by RESET_STATE and bootloader entry).
void ifaceClearAllOutputs(void);

// Read both input pin levels. Returns bit 0 = input 6, bit 1 = input 7.
// Pulled-up = HIGH = open = Logo "false".
uint8_t ifaceSampleInputs(void);

// Edge counters for inputs 6 and 7.
// Counts only false-to-true transitions in the LEGO TC Logo sense — i.e.
// the wire pin going from HIGH (open / pullup) to LOW (closed / dark / pressed).
//
// `ifaceEdgePoll()` must be called from the main loop on every iteration.
// `input` is the wire-level port number: 6 or 7.
void ifaceEdgePoll(void);
uint32_t ifaceGetCount(uint8_t input);
void ifaceResetCount(uint8_t input);

#endif
