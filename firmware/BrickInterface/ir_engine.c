#include <Arduino.h>
#include "ir_engine.h"
#include "board_config.h"
#include "protocol.h"

// ============================================================
// Streaming IR engine for CH552T — Timer 2 unified ISR architecture.
//
// The Timer 2 ISR (defined in BrickInterface.ino) runs at the 13 us base
// tick (6.5 us while Legacy transmits — see board_config.h) and:
//   - toggles P3.4 every ISR when carrierMode != 0 (38.46 kHz at the base
//     tick for PF/RCX, 76.92 kHz at the Legacy fast tick)
//   - leaves P3.4 low (carrier mode 0 = off, for spaces and idle)
// and counts down `envelopeTicks` every ISR. When it hits 0, the ISR calls
// irNextPhase() to load the next mark/space phase. When the protocol-
// specific nextPhase returns 0, the ISR signals completion via the
// `completionPending` flag and clears state.active.
//
// All envelope durations are expressed in Timer 2 ticks (13 us each at the
// base rate; Legacy's are 6.5 us fast ticks), not microseconds. This avoids
// any conversion math inside the ISR.
//
// We do NOT touch the CH552 PWM module or Timer 0 anymore. The old design
// tried to drive the carrier via PWM2 (couldn't reach 38 kHz cleanly) and
// time the envelope with Timer 0 (collides with ch55xduino's millis/delay).
// Both broken paths are gone.
// ============================================================

// ============================================================
// Shared state with the Timer 2 ISR
// ============================================================
volatile __data uint8_t  carrierMode      = 0;
volatile __data uint16_t envelopeTicks    = 0;
volatile __data uint8_t  completionPending = 0;

// ============================================================
// Engine state
// ============================================================
static volatile IRStreamState state;
static volatile IRPending pending;
static volatile uint8_t completionToken = 0;
static volatile uint8_t completionEngine = 0;
static uint8_t tokenCounter = 1;

uint8_t pfToggle[4] = {0, 0, 0, 0};

// RCX carrier select (payload byte 0 of CMD_RCX_SEND/_RAW): Timer 2 reload
// low byte (high byte is always 0xFF) and the matching ~416 us bit length.
// A diagnostic knob for finding the receivers' real passband against the
// CH552's internal-RC oscillator; 0 is the production default.
//   sel 0: 26 counts -> 13.0 us half-period -> 38.46 kHz, bit 32 (416 us)
//   sel 1: 27 counts -> 13.5 us             -> 37.04 kHz, bit 31 (418 us)
//   sel 2: 25 counts -> 12.5 us             -> 40.00 kHz, bit 33 (412 us)
//   sel 3: 28 counts -> 14.0 us             -> 35.71 kHz, bit 30 (420 us)
#define RCX_CARRIER_SELECTS 4
static const uint8_t rcxCarrierReloadL[RCX_CARRIER_SELECTS]  = { 0xE6, 0xE5, 0xE7, 0xE4 };
static const uint8_t rcxCarrierBitTicks[RCX_CARRIER_SELECTS] = { 32, 31, 33, 30 };

// ============================================================
// Helpers
// ============================================================

static uint8_t oddParity(uint8_t b) {
    uint8_t p = 1;
    for (uint8_t i = 0; i < 8; i++) p ^= (b >> i) & 1;
    return p;
}

// ============================================================
// Protocol nextPhase functions — return duration in TICKS.
// `carrier_on` is 0 (space) or 1 (mark). The caller (irNextPhase) maps
// carrier_on=1 to a carrierMode value based on the active protocol's
// carrier frequency.
// ============================================================

static uint8_t pfNextPhase(uint8_t *carrier_on, uint16_t *ticks) {
    PFState *s = (PFState *)&state.pf;

    if (s->phase == 0) {
        // Mark
        *carrier_on = 1;
        *ticks = PF_MARK_TICKS;
        s->phase = 1;
        s->elapsed += PF_MARK_TICKS;
        return 1;
    }

    // Space
    s->phase = 0;

    if (s->pos == 0) {
        // Start space
        *carrier_on = 0;
        *ticks = PF_START_SPACE_TICKS;
        s->pos = 1;
        s->elapsed += PF_START_SPACE_TICKS;
        return 1;
    }

    if (s->pos >= 1 && s->pos <= 16) {
        // Data bit space
        uint8_t bitIdx = s->pos - 1;
        uint8_t nibIdx = bitIdx / 4;
        uint8_t bitInNib = 3 - (bitIdx % 4);
        uint8_t bit = (s->nibbles[nibIdx] >> bitInNib) & 1;
        *carrier_on = 0;
        *ticks = bit ? PF_ONE_SPACE_TICKS : PF_ZERO_SPACE_TICKS;
        s->pos++;
        s->elapsed += *ticks;
        return 1;
    }

    // pos == 17: after stop mark
    s->repeat++;
    if (s->repeat >= PF_REPEAT_COUNT) {
        return 0;  // Done
    }

    // Inter-message gap, timed start-to-start per PF RC v1.20: 5*tm after
    // messages 1 and 2, (6 + 2*Ch)*tm after messages 3 and 4. The gap phase
    // is the spacing target minus the ticks this message consumed.
    {
        uint16_t target = (s->repeat <= 2)
            ? (uint16_t)(5 * PF_TM_TICKS)
            : (uint16_t)((6 + 2 * s->channel) * PF_TM_TICKS);
        *carrier_on = 0;
        *ticks = (target > s->elapsed) ? (target - s->elapsed)
                                       : PF_START_SPACE_TICKS;
        s->pos = 0;      // Reset for next repeat
        s->elapsed = 0;  // Gap is not part of the next message
        return 1;
    }
}

static uint8_t legacyNextPhase(uint8_t *carrier_on, uint16_t *ticks) {
    LegacyState *s = (LegacyState *)&state.legacy;

    if (s->repeat >= LEGACY_REPEAT_COUNT) return 0;

    *ticks = LEGACY_BIT_TICKS;

    if (s->inGap) {
        s->inGap = 0;
        *carrier_on = 0;
        // Inter-message gap (channel-dependent), minus the ~22-bit message
        // window so total spacing matches the original spec.
        uint16_t gap_ticks;
        switch (s->channelCode) {
            case 4: case 5: gap_ticks = 7846; break;  // 51 ms
            case 6:         gap_ticks = 10462; break; // 68 ms
            case 7:         gap_ticks = 13231; break; // 86 ms
            default:        gap_ticks = 7846; break;
        }
        uint16_t msg_ticks = 22 * LEGACY_BIT_TICKS;
        *ticks = (gap_ticks > msg_ticks) ? (gap_ticks - msg_ticks) : 154;  // 154 ~= 1 ms
        return 1;
    }

    uint8_t byte = (s->byteIdx == 0) ? s->byte0 : s->byte1;

    if (s->bitIdx == 0) {
        *carrier_on = 1;  // Start bit: mark
    } else if (s->bitIdx <= 8) {
        uint8_t bit = (byte >> (s->bitIdx - 1)) & 1;
        *carrier_on = !bit;
    } else if (s->bitIdx == 9) {
        uint8_t par = oddParity(byte);
        *carrier_on = !par;
    } else {
        *carrier_on = 0;  // Stop bit: space
    }

    s->bitIdx++;
    if (s->bitIdx > 10) {
        s->bitIdx = 0;
        s->byteIdx++;
        if (s->byteIdx > 1) {
            s->byteIdx = 0;
            s->repeat++;
            if (s->repeat >= LEGACY_REPEAT_COUNT) {
                return 1;  // emit last phase, return 0 next call
            }
            s->inGap = 1;
        }
    }
    return 1;
}

static uint8_t rcxNextPhase(uint8_t *carrier_on, uint16_t *ticks) {
    RCXState *s = (RCXState *)&state.rcx;

    if (s->byteIdx >= s->byteCount) {
        // Frame finished. More blind repeats? Rewind and emit the
        // inter-repeat gap (the brick's deaf window) as one space phase;
        // the next call starts the frame over.
        if (s->repeat > 1) {
            s->repeat--;
            s->byteIdx = 0;
            s->bitIdx = 0;
            *carrier_on = 0;
            *ticks = RCX_REPEAT_GAP_TICKS;
            return 1;
        }
        return 0;
    }

    *ticks = s->bitTicks;
    uint8_t byte = s->bytes[s->byteIdx];

    if (s->bitIdx == 0) {
        *carrier_on = 1;  // Start bit: mark
    } else if (s->bitIdx <= 8) {
        uint8_t bit = (byte >> (s->bitIdx - 1)) & 1;
        *carrier_on = !bit;
    } else if (s->bitIdx == 9) {
        uint8_t par = oddParity(byte);
        *carrier_on = !par;
    } else {
        *carrier_on = 0;  // Stop bit: space
    }

    s->bitIdx++;
    if (s->bitIdx > 10) {
        s->bitIdx = 0;
        s->byteIdx++;
    }
    return 1;
}

// ============================================================
// irNextPhase — called from the Timer 2 ISR when envelopeTicks hits 0.
//
// Returns 1 with carrier_mode (0/1/2) and ticks filled, or 0 when the
// active transmission is complete (also clears state.active so the engine
// is ready for the next request).
// ============================================================
uint8_t irNextPhase(uint8_t *carrier_mode, uint16_t *ticks) {
    uint8_t carrier_on = 0;
    uint8_t res = 0;

    switch (state.active) {
    case IR_ACTIVE_PF:
        res = pfNextPhase(&carrier_on, ticks);
        break;
    case IR_ACTIVE_LEGACY:
        res = legacyNextPhase(&carrier_on, ticks);
        break;
    case IR_ACTIVE_RCX:
        res = rcxNextPhase(&carrier_on, ticks);
        break;
    default:
        return 0;
    }

    if (!res) {
        // Transmission complete — flag the main loop and release the engine.
        completionToken = state.token;
        completionEngine = state.engine;
        state.active = IR_ACTIVE_NONE;
        return 0;
    }

    // Map carrier_on to carrierMode based on the active protocol's frequency.
    // PF and RCX use 38 kHz (mode 1). Legacy uses 76 kHz (mode 2).
    if (carrier_on) {
        *carrier_mode = (state.active == IR_ACTIVE_LEGACY) ? 2 : 1;
    } else {
        *carrier_mode = 0;
    }
    return 1;
}

// ============================================================
// PF nibble builder
// ============================================================

static uint8_t pfLRC(uint8_t n0, uint8_t n1, uint8_t n2) {
    return 0x0F ^ n0 ^ n1 ^ n2;
}

static uint8_t pfBuildNibbles(uint8_t *nibbles, uint8_t channel, uint8_t mode,
                               uint8_t data, uint8_t flags) {
    if (mode == PF_MODE_COMBO_PWM) {
        // Combo PWM: nibble 1 is "a 1 C C" — the toggle bit position carries
        // the ADDRESS bit instead (PF RC v1.20), and stock receivers only
        // listen on address 0. Always send 0 and leave the channel's toggle
        // state untouched (toggle is not verified in this mode).
        // Nibble 2 = Output B step, nibble 3 = Output A step (both 4-bit).
        // data = (step_b << 4) | step_a
        nibbles[0] = (1 << 2) | (channel & 0x03);
        nibbles[1] = (data >> 4) & 0x0F;
        nibbles[2] = data & 0x0F;
        nibbles[3] = pfLRC(nibbles[0], nibbles[1], nibbles[2]);
        return 1;
    }

    uint8_t toggle;
    if (flags & 0x01) {
        toggle = (flags >> 1) & 1;
    } else {
        toggle = pfToggle[channel];
        pfToggle[channel] ^= 1;
    }

    // The escape bit (nibble 1 bit 2) means "this is a Combo PWM message";
    // it must be 0 for every other mode, so it is not host-settable.
    switch (mode) {
    case PF_MODE_COMBO_DIRECT:
        nibbles[0] = ((toggle & 1) << 3) | (channel & 0x03);
        nibbles[1] = 0x01;
        nibbles[2] = data & 0x0F;
        break;
    case PF_MODE_SINGLE_PWM:
        nibbles[0] = ((toggle & 1) << 3) | (channel & 0x03);
        nibbles[1] = 0x04 | ((data >> 4) & 0x01);
        nibbles[2] = data & 0x0F;
        break;
    case PF_MODE_SINGLE_CST:
        nibbles[0] = ((toggle & 1) << 3) | (channel & 0x03);
        nibbles[1] = 0x06 | ((data >> 4) & 0x01);
        nibbles[2] = data & 0x0F;
        break;
    default:
        return 0;
    }
    nibbles[3] = pfLRC(nibbles[0], nibbles[1], nibbles[2]);
    return 1;
}

// ============================================================
// Public API
// ============================================================

void irInit(void) {
    pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, LOW);

    // Make sure the PWM2 module isn't driving P3.4 (legacy paths in the core
    // may leave it enabled). We drive the pin directly from the Timer 2 ISR.
    PWM_CTRL &= ~bPWM2_OUT_EN;
    PWM_DATA2 = 0;

    state.active = IR_ACTIVE_NONE;
    pending.valid = 0;
    completionPending = 0;
    carrierMode = 0;
    envelopeTicks = 0;
    tokenCounter = 1;
}

uint8_t irNextToken(void) {
    uint8_t t = tokenCounter++;
    if (tokenCounter == 0) tokenCounter = 1;
    return t;
}

uint8_t irIsBusy(void) {
    return state.active != IR_ACTIVE_NONE;
}

void irAbortAll(void) {
    // Stop any in-flight transmission cleanly. We mask Timer 2's interrupt
    // briefly so the ISR can't fire mid-tear-down and observe inconsistent
    // state.
    ET2 = 0;
    envelopeTicks = 0;
    carrierMode = 0;
    P3_4 = 0;
    // Restore the base tick in case a Legacy burst was in flight.
    RCAP2H = T2_RELOAD_BASE_H;
    RCAP2L = T2_RELOAD_BASE_L;
    // Resolve the outstanding token, if any, so every IR_ACCEPTED produces
    // exactly one IR_DONE even when the burst is aborted. At most one of
    // {state, pending} holds a token (single-slot queue); a completion that
    // already fired but hasn't been drained by the main loop is left intact.
    if (state.active != IR_ACTIVE_NONE) {
        completionToken = state.token;
        completionEngine = state.engine;
        completionPending = 1;
    } else if (pending.valid) {
        completionToken = pending.token;
        completionEngine = pending.engine;
        completionPending = 1;
    }
    state.active = IR_ACTIVE_NONE;
    pending.valid = 0;
    ET2 = 1;
}

uint8_t irGetCompletion(uint8_t *token, uint8_t *engine) {
    if (!completionPending) return 0;
    *token = completionToken;
    *engine = completionEngine;
    completionPending = 0;
    return 1;
}

void irPoll(void) {
    // Nothing to do if a transmission is in flight or no request queued.
    if (state.active != IR_ACTIVE_NONE) return;
    if (!pending.valid) return;

    // Move the pending request into active state.
    state.token = pending.token;
    state.engine = pending.engine;

    switch (pending.engineType) {
    case IR_ACTIVE_PF:
        state.active = IR_ACTIVE_PF;
        pfBuildNibbles((uint8_t *)state.pf.nibbles, pending.pf.channel, pending.pf.mode,
                       pending.pf.data, pending.pf.flags);
        state.pf.channel = pending.pf.channel;
        state.pf.repeat = 0;
        state.pf.pos = 0;
        state.pf.phase = 0;
        state.pf.elapsed = 0;
        break;

    case IR_ACTIVE_LEGACY:
        state.active = IR_ACTIVE_LEGACY;
        {
            uint8_t ch = pending.legacy.channelCode;
            uint8_t or_ = pending.legacy.orange;
            uint8_t yl = pending.legacy.yellow;
            uint8_t check = (0x10 - ((ch + or_ + yl) & 0x0F)) & 0x0F;
            state.legacy.byte0 = ((ch & 0x0F) << 4) | (or_ & 0x0F);
            state.legacy.byte1 = ((yl & 0x0F) << 4) | (check & 0x0F);
            state.legacy.channelCode = ch;
            state.legacy.repeat = 0;
            state.legacy.byteIdx = 0;
            state.legacy.bitIdx = 0;
            state.legacy.inGap = 0;
        }
        break;

    case IR_ACTIVE_RCX:
        state.active = IR_ACTIVE_RCX;
        for (uint8_t i = 0; i < pending.rcx.count; i++) {
            state.rcx.bytes[i] = pending.rcx.bytes[i];
        }
        state.rcx.byteCount = pending.rcx.count;
        state.rcx.byteIdx = 0;
        state.rcx.bitIdx = 0;
        state.rcx.bitTicks = pending.rcx.bitTicks;
        state.rcx.repeat = pending.rcx.repeats;
        break;

    default:
        pending.valid = 0;
        return;
    }

    // Arm the tick rate for this engine: Legacy runs the fast (6.5 us)
    // tick for its 76.92 kHz carrier, RCX its per-send carrier select, and
    // PF the 13 us base tick. Takes effect at the next timer overflow.
    if (pending.engineType == IR_ACTIVE_LEGACY) {
        RCAP2H = T2_RELOAD_FAST_H;
        RCAP2L = T2_RELOAD_FAST_L;
    } else if (pending.engineType == IR_ACTIVE_RCX) {
        RCAP2H = 0xFF;
        RCAP2L = pending.rcx.reloadL;
    } else {
        RCAP2H = T2_RELOAD_BASE_H;
        RCAP2L = T2_RELOAD_BASE_L;
    }

    // PF RC v1.20: "The delay before transmitting the first message is
    // (4 - Ch)*tm" — a channel-staggered head start so transmitters keyed
    // in the same instant on different channels miss each other's first
    // message. Implemented by priming the silent countdown below with the
    // delay instead of 1 tick. Other engines start immediately.
    uint16_t primeTicks = 1;
    if (pending.engineType == IR_ACTIVE_PF) {
        primeTicks = (uint16_t)(4 - pending.pf.channel) * PF_TM_TICKS;
    }

    pending.valid = 0;

    // Prime the envelope state machine: the ISR counts envelopeTicks down
    // in silence (carrierMode=0), then loads the first phase. Set this
    // last so the ISR doesn't try to advance before state is fully
    // initialized.
    carrierMode = 0;
    envelopeTicks = primeTicks;
}

// --- Enqueue functions ---

uint8_t irStartPF(uint8_t channel, uint8_t mode, uint8_t data, uint8_t flags) {
    if (channel > 3) return 0;
    if (state.active != IR_ACTIVE_NONE || pending.valid) return 0;

    uint8_t token = irNextToken();
    pending.token = token;
    pending.engine = IR_ENGINE_PF;
    pending.engineType = IR_ACTIVE_PF;
    pending.pf.channel = channel;
    pending.pf.mode = mode;
    pending.pf.data = data;
    pending.pf.flags = flags;
    pending.valid = 1;
    return token;
}

uint8_t irStartLegacy(uint8_t channelCode, uint8_t orange, uint8_t yellow) {
    if (channelCode < 4 || channelCode > 7) return 0;
    if (orange > 0x0F || yellow > 0x0F) return 0;
    if (state.active != IR_ACTIVE_NONE || pending.valid) return 0;

    uint8_t token = irNextToken();
    pending.token = token;
    pending.engine = IR_ENGINE_LEGACY;
    pending.engineType = IR_ACTIVE_LEGACY;
    pending.legacy.channelCode = channelCode;
    pending.legacy.orange = orange;
    pending.legacy.yellow = yellow;
    pending.valid = 1;
    return token;
}

// Unpack the RCX options byte (low nibble carrier, high nibble repeats).
static void rcxParseOptions(uint8_t options) {
    uint8_t carrierSel = options & 0x0F;
    uint8_t repeats = (options >> 4) & 0x0F;
    if (carrierSel >= RCX_CARRIER_SELECTS) carrierSel = 0;
    if (repeats == 0) repeats = 1;
    pending.rcx.reloadL = rcxCarrierReloadL[carrierSel];
    pending.rcx.bitTicks = rcxCarrierBitTicks[carrierSel];
    pending.rcx.repeats = repeats;
}

uint8_t irStartRCX(const uint8_t *data, uint8_t dataLen, uint8_t options) {
    if (dataLen == 0 || dataLen > 16) return 0;
    if (state.active != IR_ACTIVE_NONE || pending.valid) return 0;

    // Build framed packet: 0x55 0xFF 0x00 D1 ~D1 ... Dn ~Dn C ~C
    uint8_t framed[RCX_MAX_FRAMED_BYTES];
    uint8_t pos = 0;
    framed[pos++] = 0x55;
    framed[pos++] = 0xFF;
    framed[pos++] = 0x00;

    uint8_t checksum = 0;
    for (uint8_t i = 0; i < dataLen; i++) {
        framed[pos++] = data[i];
        framed[pos++] = ~data[i];
        checksum += data[i];
    }
    framed[pos++] = checksum;
    framed[pos++] = ~checksum;

    uint8_t token = irNextToken();
    pending.token = token;
    pending.engine = IR_ENGINE_RCX;
    pending.engineType = IR_ACTIVE_RCX;
    for (uint8_t i = 0; i < pos; i++) pending.rcx.bytes[i] = framed[i];
    pending.rcx.count = pos;
    rcxParseOptions(options);
    pending.valid = 1;
    return token;
}

uint8_t irStartRCXRaw(const uint8_t *rawBytes, uint8_t rawLen, uint8_t options) {
    if (rawLen == 0 || rawLen > RCX_MAX_FRAMED_BYTES) return 0;
    if (state.active != IR_ACTIVE_NONE || pending.valid) return 0;

    uint8_t token = irNextToken();
    pending.token = token;
    pending.engine = IR_ENGINE_RCX;
    pending.engineType = IR_ACTIVE_RCX;
    for (uint8_t i = 0; i < rawLen; i++) pending.rcx.bytes[i] = rawBytes[i];
    pending.rcx.count = rawLen;
    rcxParseOptions(options);
    pending.valid = 1;
    return token;
}
