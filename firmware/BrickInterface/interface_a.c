#include <Arduino.h>
#include "interface_a.h"

// Per-output PWM duty cycle (0 = always off, 255 = always on).
// Volatile because the Timer 2 ISR consumes these.
// Not static — the Timer 2 ISR lives in the .ino translation unit (SDCC vector
// installation requirement) and needs to access these from there.
volatile __xdata uint8_t pwmDuty[6] = {0, 0, 0, 0, 0, 0};

// Free-running counter, ISR compares to each duty to decide GPIO state.
// Wraps every 255 ticks (0..254) so duty 255 means solid on.
volatile __data uint8_t pwmCounter = 0;

// Per-input edge-counter state. Indexed by 0 (input 6) and 1 (input 7).
// Polled from the main loop via `ifaceEdgePoll()`. Declared here so
// `ifaceInit()` can seed `inputPrev[]` from the actual current pin levels.
static __xdata uint32_t inputCount[2] = {0, 0};
// Last-seen wire level. Pulled-up = HIGH = open = Logo "false".
static __data uint8_t inputPrev[2]    = {1, 1};

// The PWM tick runs inside the unified Timer 2 ISR in BrickInterface.ino —
// SDCC requires the ISR definition in the same translation unit as main()
// for the interrupt vector to be installed. See pwmDuty / pwmCounter
// externs in interface_a.h.

// ============================================================
static void pwmTimerInit(void) {
    // Timer 2 in 16-bit auto-reload mode, clock = Fsys/12 = 2 MHz.
    // Base reload 65536 - 26 -> overflow every 26 timer counts = 13 us per
    // ISR -> 76.92 kHz ISR rate (see board_config.h for why; the Legacy IR
    // engine re-arms the fast 6.5 us reload while it transmits).
    //
    // The unified Timer 2 ISR (in BrickInterface.ino) handles three jobs:
    //   - IR carrier toggle (every ISR; the frequency comes from the tick)
    //   - IR envelope phase countdown
    //   - IFA software PWM tick (every 3rd ISR -> 39 us tick
    //     -> ~100 Hz PWM, 255-tick period so duty 255 = solid on)
    //
    // We use Timer 2 because ch55xduino's core declares a weak Timer2Interrupt
    // with the vector pre-installed; user-code overrides get picked up by the
    // linker. There is no equivalent hook for Timer 0 (used by millis/delay)
    // or Timer 1 (no weak declaration in the core).
    T2MOD &= ~bT2_CLK;             // Timer 2 clock = Fsys/12
    RCAP2H = T2_RELOAD_BASE_H;     // 13 us base tick
    RCAP2L = T2_RELOAD_BASE_L;
    TH2 = T2_RELOAD_BASE_H;
    TL2 = T2_RELOAD_BASE_L;
    T2CON = 0x04;                  // auto-reload, internal clock, TR2 = 1 (run)
    ET2 = 1;                       // enable Timer 2 interrupt
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
    // open = HIGH (= Logo "false"). Without this they float and read randomly.
    pinMode(IFACE_IN6_PIN, INPUT_PULLUP);
    pinMode(IFACE_IN7_PIN, INPUT_PULLUP);

    for (uint8_t i = 0; i < 6; i++) pwmDuty[i] = 0;

    // Snapshot the input pin levels so the first edge poll doesn't count a
    // spurious transition based on the static-init `inputPrev` value.
    inputPrev[0] = digitalRead(IFACE_IN6_PIN) ? 1 : 0;
    inputPrev[1] = digitalRead(IFACE_IN7_PIN) ? 1 : 0;
    inputCount[0] = 0;
    inputCount[1] = 0;

    pwmTimerInit();
}

// ============================================================
void ifaceSetOutputs(uint8_t mask, const uint8_t *packed_duties) {
    mask &= 0x3F;
    uint8_t src = 0;
    for (uint8_t i = 0; i < 6; i++) {
        if (mask & (1 << i)) pwmDuty[i] = packed_duties[src++];
    }
}

void ifaceClearAllOutputs(void) {
    for (uint8_t i = 0; i < 6; i++) pwmDuty[i] = 0;
}

uint8_t ifaceSampleInputs(void) {
    uint8_t inputs = 0;
    if (digitalRead(IFACE_IN6_PIN)) inputs |= 0x01;
    if (digitalRead(IFACE_IN7_PIN)) inputs |= 0x02;
    return inputs;
}

// ============================================================
// Edge counters for inputs 6 and 7 (TC Logo `counter` parity).
// ============================================================
// State storage lives near the top of this file so ifaceInit() can seed
// inputPrev[] from the actual current pin levels — see above.

static uint8_t input_idx(uint8_t input) {
    // Map wire-level input number (6 or 7) to array index (0 or 1).
    return (input == 7) ? 1 : 0;
}

// Apply a single edge sample. Public for unit-test access — ISR-free design
// means the test harness can drive transitions deterministically by calling
// this with synthetic levels.
void ifaceCountEdge(uint8_t idx, uint8_t newLevel) {
    if (idx >= 2) return;
    // false->true (Logo) = HIGH->LOW (wire), since the input has an internal
    // pullup and the sensor pulls to ground when active (dark / pressed).
    if (inputPrev[idx] && !newLevel) inputCount[idx]++;
    inputPrev[idx] = newLevel ? 1 : 0;
}

void ifaceEdgePoll(void) {
    ifaceCountEdge(0, digitalRead(IFACE_IN6_PIN));
    ifaceCountEdge(1, digitalRead(IFACE_IN7_PIN));
}

uint32_t ifaceGetCount(uint8_t input) {
    if (input != 6 && input != 7) return 0;
    return inputCount[input_idx(input)];
}

void ifaceResetCount(uint8_t input) {
    if (input != 6 && input != 7) return;
    inputCount[input_idx(input)] = 0;
}
