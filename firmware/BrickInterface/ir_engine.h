#ifndef IR_ENGINE_H
#define IR_ENGINE_H

#include <stdint.h>
#include "board_config.h"

// ============================================================
// Streaming IR engine for CH552T
//
// Instead of pre-computing all phases into a buffer, each protocol
// encoder provides a nextPhase() function that the timer ISR calls
// to get the next (carrier_on, duration_us) pair on demand.
// This uses ~80 bytes of RAM instead of ~1600.
// ============================================================

// --- Protocol timing (in Timer 2 ISR ticks; 1 tick = ~6.5 us @ 153.85 kHz) ---
// Original microsecond values shown in comments for reference.
#define PF_MARK_TICKS         24   // 158 us  (6 cycles of 38 kHz)
#define PF_ZERO_SPACE_TICKS   40   // 263 us
#define PF_ONE_SPACE_TICKS    85   // 553 us
#define PF_START_SPACE_TICKS 158   // 1026 us
#define PF_REPEAT_COUNT        5
#define PF_GAP_CH0_TICKS    2462   // 16 ms
#define PF_GAP_CH1_TICKS    4000   // 26 ms
#define PF_GAP_CH2_TICKS    5538   // 36 ms
#define PF_GAP_CH3_TICKS    7077   // 46 ms

// --- Legacy timing ---
#define LEGACY_BIT_TICKS      32   // 208 us
#define LEGACY_REPEAT_COUNT    5

// --- RCX timing ---
#define RCX_BIT_TICKS         64   // 417 us
#define RCX_MAX_FRAMED_BYTES  37   // 3 header + 16*2 data+complement + 2 checksum

// --- PF modes ---
#define PF_MODE_COMBO_DIRECT    0x00
#define PF_MODE_SINGLE_PWM      0x01
#define PF_MODE_SINGLE_CST      0x02

// --- Protocol state structures ---

typedef struct {
    uint8_t nibbles[4];
    uint8_t channel;
    uint8_t repeat;     // 0 to PF_REPEAT_COUNT-1
    uint8_t pos;        // 0=start, 1-16=bits, 17=stop
    uint8_t phase;      // 0=mark, 1=space
} PFState;

typedef struct {
    uint8_t byte0, byte1;
    uint8_t channelCode;
    uint8_t repeat;
    uint8_t byteIdx;    // 0 or 1
    uint8_t bitIdx;     // 0-10: start, 8 data, parity, stop
    uint8_t inGap;      // 1 = outputting inter-message gap
} LegacyState;

typedef struct {
    uint8_t bytes[RCX_MAX_FRAMED_BYTES];
    uint8_t byteCount;
    uint8_t byteIdx;
    uint8_t bitIdx;     // 0-10
} RCXState;

// --- Active engine ---
#define IR_ACTIVE_NONE      0
#define IR_ACTIVE_PF        1
#define IR_ACTIVE_LEGACY    2
#define IR_ACTIVE_RCX       3

typedef struct {
    uint8_t active;     // IR_ACTIVE_*
    uint8_t token;
    uint8_t engine;     // IR_ENGINE_* (for completion event)
    uint32_t carrier_hz;
    uint8_t duty_pct;
    union {
        PFState pf;
        LegacyState legacy;
        RCXState rcx;
    };
} IRStreamState;

// --- Pending request (1 slot queue) ---
typedef struct {
    uint8_t valid;
    uint8_t engineType; // IR_ACTIVE_*
    uint8_t engine;     // IR_ENGINE_*
    uint8_t token;
    union {
        struct { uint8_t channel, mode, data, flags; } pf;
        struct { uint8_t channelCode, orange, yellow; } legacy;
        struct { uint8_t bytes[RCX_MAX_FRAMED_BYTES]; uint8_t count; uint8_t carrierMode; } rcx;
    };
} IRPending;

// --- Public API ---
void irInit(void);
uint8_t irNextToken(void);

// Start a transmission (returns token, or 0 on failure)
uint8_t irStartPF(uint8_t channel, uint8_t mode, uint8_t data, uint8_t flags);
uint8_t irStartLegacy(uint8_t channelCode, uint8_t orange, uint8_t yellow);
uint8_t irStartRCX(const uint8_t *data, uint8_t dataLen, uint8_t carrierMode);
uint8_t irStartRCXRaw(const uint8_t *rawBytes, uint8_t rawLen, uint8_t carrierMode);

uint8_t irIsBusy(void);
void irAbortAll(void);
void irPoll(void);  // Start pending if idle

// Completion event
uint8_t irGetCompletion(uint8_t *token, uint8_t *engine);

// PF toggle state (per channel)
extern uint8_t pfToggle[4];

// ============================================================
// Shared state with the unified Timer 2 ISR (defined in BrickInterface.ino).
// The ISR runs at 153.85 kHz and handles three responsibilities:
//   - IR carrier toggling (38 kHz / 76 kHz) via carrierMode
//   - Envelope phase progression via envelopeTicks countdown
//   - Interface A software PWM tick (every 6th call)
//
// ir_engine.c owns these vars; the ISR reads/writes them via these externs.
// ============================================================
extern volatile __data uint8_t  carrierMode;       // 0=off, 1=38kHz, 2=76kHz
extern volatile __data uint8_t  carrierPrescale;   // for 38 kHz (toggle every 2 ISRs)
extern volatile __data uint16_t envelopeTicks;     // countdown to next phase load
extern volatile __data uint8_t  completionPending; // set by ISR when transmission ends

// Called from the Timer 2 ISR when envelopeTicks reaches 0. Returns 1 with
// next phase's carrier_mode and tick count, or 0 when transmission complete
// (in which case it also clears state.active to release the engine).
uint8_t irNextPhase(uint8_t *carrier_mode, uint16_t *ticks);

#endif
