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

// --- PF timing constants (microseconds) ---
#define PF_MARK_US          158
#define PF_ZERO_SPACE_US    263
#define PF_ONE_SPACE_US     553
#define PF_START_SPACE_US   1026
#define PF_REPEAT_COUNT     5
#define PF_GAP_CH0_US       16000
#define PF_GAP_CH1_US       26000
#define PF_GAP_CH2_US       36000
#define PF_GAP_CH3_US       46000

// --- Legacy timing ---
#define LEGACY_BIT_US       208
#define LEGACY_REPEAT_COUNT 5

// --- RCX timing ---
#define RCX_BIT_US          417
#define RCX_MAX_FRAMED_BYTES 37  // 3 header + 16*2 data+complement + 2 checksum

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

#endif
