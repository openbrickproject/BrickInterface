#ifndef IR_ENGINE_H
#define IR_ENGINE_H

#include <stdint.h>
#include <Arduino.h>   // __data/__xdata qualifiers (ch55xduino on target, stub in tests)
#include "board_config.h"

// ============================================================
// Streaming IR engine for CH552T
//
// Instead of pre-computing all phases into a buffer, each protocol
// encoder provides a nextPhase() function that the timer ISR calls
// to get the next (carrier_on, duration_ticks) pair on demand.
// This uses ~80 bytes of RAM instead of ~1600.
// ============================================================

// --- Protocol timing (in Timer 2 ISR ticks) ---
// Base tick = 13 us (76.92 kHz ISR; the carrier toggles every tick, so the
// base tick also sets the 38.46 kHz carrier used by PF and RCX). The Legacy
// engine re-arms Timer 2 to a 6.5 us tick while it transmits (76.92 kHz
// carrier), so the LEGACY_* constants are in 6.5 us units. See
// board_config.h for the reload values and the ISR-budget rationale.
// Original microsecond values shown in comments for reference.
#define PF_MARK_TICKS         12   // 156 us  (6 cycles of 38.46 kHz)
#define PF_ZERO_SPACE_TICKS   20   // 260 us
#define PF_ONE_SPACE_TICKS    43   // 559 us
#define PF_START_SPACE_TICKS  79   // 1027 us
#define PF_REPEAT_COUNT        5
// tm = maximum message length per PF RC v1.20 (16 ms). Message spacing is
// start-to-start: 5*tm after messages 1 and 2, (6 + 2*Ch)*tm after messages
// 3 and 4, with Ch the 2-bit channel field (0-3).
#define PF_TM_TICKS         1231   // 16 ms

// --- Legacy timing (6.5 us fast ticks — see above) ---
#define LEGACY_BIT_TICKS      32   // 208 us
#define LEGACY_REPEAT_COUNT    5

// --- RCX timing ---
// Default bit length at carrier select 0; selects 1-3 use their own bit
// lengths to hold ~416 us against the changed tick (see rcxCarrier tables
// in ir_engine.c).
#define RCX_BIT_TICKS         32   // 416 us (2400 baud)
// Gap between blind repeats of the same frame: the brick transmits an
// (unheard) reply after each frame and is deaf until it finishes (~45 ms of
// reply airtime + processing). 60 ms at the 13 us base tick; the gap phase
// is emitted in bit-ticks of the selected carrier, so it shrinks/grows a
// few percent with the carrier select — harmless, it only needs to clear
// the reply window.
#define RCX_REPEAT_GAP_TICKS 4615  // ~60 ms
#define RCX_MAX_FRAMED_BYTES  37   // 3 header + 16*2 data+complement + 2 checksum

// PF modes live in protocol.h (shared with the dispatch layer).

// --- Protocol state structures ---

typedef struct {
    uint8_t nibbles[4];
    uint8_t channel;
    uint8_t repeat;     // 0 to PF_REPEAT_COUNT-1
    uint8_t pos;        // 0=start, 1-16=bits, 17=stop
    uint8_t phase;      // 0=mark, 1=space
    uint16_t elapsed;   // ticks emitted in the current message (start-to-start gap math)
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
    uint8_t bitTicks;   // per-carrier bit length (~416 us worth of ticks)
    uint8_t repeat;     // transmissions remaining (incl. the current one)
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
        struct {
            uint8_t bytes[RCX_MAX_FRAMED_BYTES];
            uint8_t count;
            uint8_t reloadL;    // Timer 2 reload low byte (carrier select)
            uint8_t bitTicks;
            uint8_t repeats;    // 1-15 transmissions per command
        } rcx;
    };
} IRPending;

// --- Public API ---
void irInit(void);
uint8_t irNextToken(void);

// Start a transmission (returns token, or 0 on failure).
// RCX `options` byte: low nibble = carrier select (0-3, others -> 0), high
// nibble = repeat count (0 -> 1). Repeats are spaced RCX_REPEAT_GAP_TICKS
// apart; one IR_DONE fires after the last repeat.
uint8_t irStartPF(uint8_t channel, uint8_t mode, uint8_t data, uint8_t flags);
uint8_t irStartLegacy(uint8_t channelCode, uint8_t orange, uint8_t yellow);
uint8_t irStartRCX(const uint8_t *data, uint8_t dataLen, uint8_t options);
uint8_t irStartRCXRaw(const uint8_t *rawBytes, uint8_t rawLen, uint8_t options);

uint8_t irIsBusy(void);
void irAbortAll(void);
void irPoll(void);  // Start pending if idle

// Completion event
uint8_t irGetCompletion(uint8_t *token, uint8_t *engine);

// PF toggle state (per channel)
extern uint8_t pfToggle[4];

// ============================================================
// Shared state with the unified Timer 2 ISR (defined in BrickInterface.ino).
// The ISR runs at the 13 us base tick (6.5 us while Legacy transmits) and
// handles three responsibilities:
//   - IR carrier toggling (every tick when carrierMode != 0)
//   - Envelope phase progression via envelopeTicks countdown
//   - Interface A software PWM tick (every 3rd call)
//
// ir_engine.c owns these vars; the ISR reads/writes them via these externs.
// ============================================================
extern volatile __data uint8_t  carrierMode;       // 0=off, nonzero=toggle (1=38k ctx, 2=76k ctx)
extern volatile __data uint16_t envelopeTicks;     // countdown to next phase load
extern volatile __data uint8_t  completionPending; // set by ISR when transmission ends

// Called from the Timer 2 ISR when envelopeTicks reaches 0. Returns 1 with
// next phase's carrier_mode and tick count, or 0 when transmission complete
// (in which case it also clears state.active to release the engine).
uint8_t irNextPhase(uint8_t *carrier_mode, uint16_t *ticks);

#endif
