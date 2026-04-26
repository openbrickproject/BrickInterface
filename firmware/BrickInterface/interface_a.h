#ifndef INTERFACE_A_H
#define INTERFACE_A_H

#include <stdint.h>
#include "board_config.h"

void ifaceInit(void);

// Binary output control. Internally sets per-output PWM duty to 0 or 255.
void ifaceSetOutputs(uint8_t bits);
void ifaceSetOutputMask(uint8_t mask, uint8_t value);
uint8_t ifaceGetOutputs(void);

// PWM control — duty = 0..255 (0 = off, 255 = full on).
void ifaceSetPWM(uint8_t output, uint8_t duty);
void ifaceSetPWMAll(const uint8_t *duties);  // duties[6]

uint8_t ifaceSampleInputs(void);
uint8_t ifaceGetState(void);  // bits 0-5 = outputs (1 if duty>=128), bits 6-7 = inputs

#endif
