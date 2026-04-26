#include <Arduino.h>
#include "interface_a.h"

// Per-output PWM duty cycle (0 = always off, 255 = always on).
// Volatile because Timer 1 ISR consumes these.
static volatile __xdata uint8_t pwmDuty[6] = {0, 0, 0, 0, 0, 0};

// 8-bit free-running counter, ISR compares to each duty to decide GPIO state.
// Counter wraps every 256 ticks, giving the PWM period.
static volatile __data uint8_t pwmCounter = 0;

// ============================================================
// Timer 1 ISR — software PWM driver for the 6 Interface A outputs.
//
// Drives P1.4, P1.5, P1.6, P1.7 (OUT0-3) and P3.1, P3.0 (OUT4-5) directly via
// SBIT writes for minimum overhead. Each ISR call:
//   - increments the 8-bit counter
//   - sets each output bit HIGH if (counter < duty), else LOW
//
// At 100 Hz PWM × 256 levels = 25.6 kHz ISR rate (~39 us period).
// At 24 MIPS, ~30 instructions per ISR -> ~3% CPU.
// ============================================================
void Timer1_ISR(void) __interrupt(3) {
    uint8_t c = pwmCounter++;

    P1_4 = (c < pwmDuty[0]) ? 1 : 0;  // OUT0
    P1_5 = (c < pwmDuty[1]) ? 1 : 0;  // OUT1
    P1_6 = (c < pwmDuty[2]) ? 1 : 0;  // OUT2
    P1_7 = (c < pwmDuty[3]) ? 1 : 0;  // OUT3
    P3_1 = (c < pwmDuty[4]) ? 1 : 0;  // OUT4
    P3_0 = (c < pwmDuty[5]) ? 1 : 0;  // OUT5
}

// ============================================================
static void pwmTimerInit(void) {
    // Timer 1 in mode 2 (8-bit auto-reload), clock = Fsys/12 = 2 MHz.
    // Reload value 256 - 78 = 178 -> overflow every 78 ticks at 2 MHz
    // = ~25.6 kHz interrupt rate -> ~100 Hz PWM at 256 levels.
    TMOD = (TMOD & 0x0F) | 0x20;   // Timer 1 = mode 2 (8-bit auto-reload)
    T2MOD &= ~bT1_CLK;             // Timer 1 clock = Fsys/12
    TH1 = 178;                     // reload value
    TL1 = 178;
    ET1 = 1;                       // enable Timer 1 interrupt
    TR1 = 1;                       // start Timer 1
    EA = 1;                        // global interrupt enable
}

// ============================================================
void ifaceInit(void) {
    // Configure all outputs as push-pull. They start LOW (duty=0).
    pinMode(IFACE_OUT0_PIN, OUTPUT);
    pinMode(IFACE_OUT1_PIN, OUTPUT);
    pinMode(IFACE_OUT2_PIN, OUTPUT);
    pinMode(IFACE_OUT3_PIN, OUTPUT);
    pinMode(IFACE_OUT4_PIN, OUTPUT);
    pinMode(IFACE_OUT5_PIN, OUTPUT);

    // 9750 sensor inputs are switch-to-ground; enable internal pullups so
    // open = HIGH (= "true"). Without this they float and read randomly.
    pinMode(IFACE_IN6_PIN, INPUT_PULLUP);
    pinMode(IFACE_IN7_PIN, INPUT_PULLUP);

    for (uint8_t i = 0; i < 6; i++) pwmDuty[i] = 0;

    pwmTimerInit();
}

// ============================================================
void ifaceSetOutputs(uint8_t bits) {
    bits &= 0x3F;
    for (uint8_t i = 0; i < 6; i++) {
        pwmDuty[i] = ((bits >> i) & 1) ? 255 : 0;
    }
}

void ifaceSetOutputMask(uint8_t mask, uint8_t value) {
    mask &= 0x3F;
    for (uint8_t i = 0; i < 6; i++) {
        if (mask & (1 << i)) {
            pwmDuty[i] = ((value >> i) & 1) ? 255 : 0;
        }
    }
}

uint8_t ifaceGetOutputs(void) {
    // Reconstruct the boolean output state from PWM duties.
    // Threshold at 128 — anything ≥ half-duty reads as "on".
    uint8_t bits = 0;
    for (uint8_t i = 0; i < 6; i++) {
        if (pwmDuty[i] >= 128) bits |= (1 << i);
    }
    return bits;
}

void ifaceSetPWM(uint8_t output, uint8_t duty) {
    if (output < 6) pwmDuty[output] = duty;
}

void ifaceSetPWMAll(const uint8_t *duties) {
    for (uint8_t i = 0; i < 6; i++) pwmDuty[i] = duties[i];
}

uint8_t ifaceSampleInputs(void) {
    uint8_t inputs = 0;
    if (digitalRead(IFACE_IN6_PIN)) inputs |= 0x01;
    if (digitalRead(IFACE_IN7_PIN)) inputs |= 0x02;
    return inputs;
}

uint8_t ifaceGetState(void) {
    uint8_t s = ifaceGetOutputs() & 0x3F;
    uint8_t inputs = ifaceSampleInputs();
    if (inputs & 0x01) s |= (1 << 6);
    if (inputs & 0x02) s |= (1 << 7);
    return s;
}
