#include <Arduino.h>
#include "ir_engine.h"
#include "board_config.h"

// ============================================================
// CH552T carrier control via PWM2 on P3.4
// ============================================================
// On the CH552, P3.4 is the default output for PWM2 (NOT PWM1).
// PWM1 default = P1.5; PWM1 alternate (bPWM1_PIN_X) = P3.0
// PWM2 default = P3.4; PWM2 alternate (bPWM2_PIN_X) = P3.1
// We use PWM2 in default mode to drive P3.4 (where the IR LED is wired).
//
// PWM_CK_SE: clock prescaler (Fpwm = Fsys / PWM_CK_SE)
// PWM_CYCLE: period in PWM clocks
// PWM_DATA2: duty cycle for PWM2 output

static uint8_t carrierDutyVal = 0;

static void carrierSetFrequency(uint32_t hz, uint8_t dutyPct) {
    // Find prescaler + cycle that fits in 8-bit cycle register
    // Fsys = 24 MHz (ch55xduino default when USB active)
    // PWM freq = 24000000 / prescaler / cycle
    uint8_t prescaler, cycle;

    if (hz == 38000) {
        prescaler = 3;  // 24MHz/3 = 8MHz PWM clock
        cycle = 211;    // 8MHz/211 = 37,915 Hz (err -0.22%)
    } else if (hz == 76000) {
        prescaler = 2;  // 24MHz/2 = 12MHz PWM clock
        cycle = 158;    // 12MHz/158 = 75,949 Hz (err -0.07%)
    } else {
        // Generic calculation
        uint32_t total = 24000000UL / hz;
        prescaler = (total / 255) + 1;
        cycle = total / prescaler;
    }

    carrierDutyVal = ((uint16_t)cycle * dutyPct) / 100;

    // Ensure PWM2 routes to P3.4 (default), not P3.1 (alternate)
    PIN_FUNC &= ~bPWM2_PIN_X;

    PWM_CK_SE = prescaler;
    PWM_CYCLE = cycle;
    PWM_DATA2 = 0;  // Start with carrier off
}

static void carrierEnable(void) {
    PWM_DATA2 = carrierDutyVal;
    PWM_CTRL |= bPWM2_OUT_EN;
}

static void carrierDisable(void) {
    PWM_CTRL &= ~bPWM2_OUT_EN;
    PWM_DATA2 = 0;
    digitalWrite(IR_LED_PIN, LOW);
}

// ============================================================
// Envelope timing via Timer0 (16-bit mode)
// ============================================================
// Fsys/12 = 2 MHz tick (0.5 us per tick)
// Max single period: 65536/2MHz = 32.768 ms
// For longer durations, we count down in chunks.

static volatile uint16_t envelopeRemaining = 0;  // remaining us for long gaps

static void envelopeStart(uint16_t us) {
    TR0 = 0;  // Stop timer

    if (us > 32000) {
        envelopeRemaining = us - 32000;
        us = 32000;
    } else {
        envelopeRemaining = 0;
    }

    uint16_t ticks = us * 2;
    uint16_t load = 65536 - ticks;
    TH0 = load >> 8;
    TL0 = load & 0xFF;
    TF0 = 0;   // Clear overflow flag
    TR0 = 1;   // Start timer
}

static void envelopeStop(void) {
    TR0 = 0;
    ET0 = 0;
}

// ============================================================
// State
// ============================================================

static volatile IRStreamState state;
static volatile IRPending pending;
static volatile uint8_t completionPending = 0;
static volatile uint8_t completionToken = 0;
static volatile uint8_t completionEngine = 0;
static uint8_t tokenCounter = 1;

uint8_t pfToggle[4] = {0, 0, 0, 0};

// ============================================================
// Protocol nextPhase functions
// ============================================================

static uint8_t oddParity(uint8_t b) {
    uint8_t p = 1;
    for (uint8_t i = 0; i < 8; i++) p ^= (b >> i) & 1;
    return p;
}

// --- PF nextPhase ---
// Returns 1 and fills carrier_on/duration_us, or 0 when done.
static uint8_t pfNextPhase(uint8_t *carrier_on, uint16_t *duration_us) {
    PFState *s = (PFState *)&state.pf;

    if (s->phase == 0) {
        // Mark
        *carrier_on = 1;
        *duration_us = PF_MARK_US;
        s->phase = 1;
        return 1;
    }

    // Space
    s->phase = 0;

    if (s->pos == 0) {
        // Start space
        *carrier_on = 0;
        *duration_us = PF_START_SPACE_US;
        s->pos = 1;
        return 1;
    }

    if (s->pos >= 1 && s->pos <= 16) {
        // Data bit space
        uint8_t bitIdx = s->pos - 1;
        uint8_t nibIdx = bitIdx / 4;
        uint8_t bitInNib = 3 - (bitIdx % 4);
        uint8_t bit = (s->nibbles[nibIdx] >> bitInNib) & 1;
        *carrier_on = 0;
        *duration_us = bit ? PF_ONE_SPACE_US : PF_ZERO_SPACE_US;
        s->pos++;
        return 1;
    }

    // pos == 17: after stop mark
    s->repeat++;
    if (s->repeat >= PF_REPEAT_COUNT) {
        return 0;  // Done
    }

    // Inter-message gap
    uint16_t gap;
    switch (s->channel) {
        case 0: gap = PF_GAP_CH0_US; break;
        case 1: gap = PF_GAP_CH1_US; break;
        case 2: gap = PF_GAP_CH2_US; break;
        case 3: gap = PF_GAP_CH3_US; break;
        default: gap = PF_GAP_CH0_US; break;
    }
    *carrier_on = 0;
    *duration_us = gap;
    s->pos = 0;  // Reset for next repeat
    return 1;
}

// --- Legacy nextPhase ---
static uint8_t legacyNextPhase(uint8_t *carrier_on, uint16_t *duration_us) {
    LegacyState *s = (LegacyState *)&state.legacy;

    *duration_us = LEGACY_BIT_US;

    // Handle inter-message gap
    if (s->inGap) {
        s->inGap = 0;
        *carrier_on = 0;
        uint16_t gap_ms;
        switch (s->channelCode) {
            case 4: case 5: gap_ms = 51; break;
            case 6: gap_ms = 68; break;
            case 7: gap_ms = 86; break;
            default: gap_ms = 51; break;
        }
        // Gap minus message duration (~22 bits * 208us = ~4.6ms)
        uint16_t msg_us = 22 * LEGACY_BIT_US;
        uint16_t gap_us = gap_ms * 1000U;
        *duration_us = (gap_us > msg_us) ? (gap_us - msg_us) : 1000;
        return 1;
    }

    uint8_t byte = (s->byteIdx == 0) ? s->byte0 : s->byte1;

    if (s->bitIdx == 0) {
        *carrier_on = 1;  // Start bit: mark
    } else if (s->bitIdx <= 8) {
        uint8_t bit = (byte >> (s->bitIdx - 1)) & 1;
        *carrier_on = !bit;  // 0=carrier on, 1=carrier off
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
            // Finished both bytes
            s->byteIdx = 0;
            s->repeat++;
            if (s->repeat >= LEGACY_REPEAT_COUNT) {
                return 1;  // Return this last phase, will return 0 next call
            }
            s->inGap = 1;
        }
    }
    return 1;
}

// Check if legacy is done (called after nextPhase returns 1)
static uint8_t legacyIsDone(void) {
    LegacyState *s = (LegacyState *)&state.legacy;
    return (s->repeat >= LEGACY_REPEAT_COUNT && s->bitIdx == 0 && !s->inGap);
}

// --- RCX nextPhase ---
static uint8_t rcxNextPhase(uint8_t *carrier_on, uint16_t *duration_us) {
    RCXState *s = (RCXState *)&state.rcx;

    if (s->byteIdx >= s->byteCount) return 0;

    *duration_us = RCX_BIT_US;
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
// Dispatch to active protocol's nextPhase
// ============================================================

static uint8_t irGetNextPhase(uint8_t *carrier_on, uint16_t *duration_us) {
    switch (state.active) {
    case IR_ACTIVE_PF:
        return pfNextPhase(carrier_on, duration_us);
    case IR_ACTIVE_LEGACY: {
        uint8_t r = legacyNextPhase(carrier_on, duration_us);
        if (r && legacyIsDone()) {
            // This was the last phase
            state.active = IR_ACTIVE_NONE;
        }
        return r;
    }
    case IR_ACTIVE_RCX:
        return rcxNextPhase(carrier_on, duration_us);
    default:
        return 0;
    }
}

// ============================================================
// Timer0 ISR — envelope timing
// ============================================================

#ifdef __cplusplus
extern "C" {
#endif

void Timer0_ISR(void) __attribute__((interrupt("1")));
void Timer0_ISR(void) {
    TR0 = 0;  // Stop timer

    // Handle long gaps (counted down in chunks)
    if (envelopeRemaining > 0) {
        uint16_t chunk = envelopeRemaining;
        if (chunk > 32000) chunk = 32000;
        envelopeRemaining -= chunk;
        uint16_t load = 65536 - (chunk * 2);
        TH0 = load >> 8;
        TL0 = load & 0xFF;
        TF0 = 0;
        TR0 = 1;
        return;
    }

    // Get next phase
    uint8_t carrier_on;
    uint16_t duration_us;

    if (!irGetNextPhase(&carrier_on, &duration_us)) {
        // Transmission complete
        carrierDisable();
        envelopeStop();
        state.active = IR_ACTIVE_NONE;
        completionToken = state.token;
        completionEngine = state.engine;
        completionPending = 1;
        return;
    }

    if (carrier_on) {
        carrierEnable();
    } else {
        carrierDisable();
    }
    envelopeStart(duration_us);
}

#ifdef __cplusplus
}
#endif

// ============================================================
// Start transmission from state
// ============================================================

static void beginTransmission(void) {
    carrierSetFrequency(state.carrier_hz, state.duty_pct);

    // Get first phase
    uint8_t carrier_on;
    uint16_t duration_us;
    if (!irGetNextPhase(&carrier_on, &duration_us)) {
        state.active = IR_ACTIVE_NONE;
        return;
    }

    if (carrier_on) {
        carrierEnable();
    } else {
        carrierDisable();
    }

    // Configure Timer0: mode 1 (16-bit), Fsys/12
    TMOD = (TMOD & 0xF0) | 0x01;
    ET0 = 1;  // Enable Timer0 interrupt
    EA = 1;   // Global interrupt enable
    envelopeStart(duration_us);
}

// ============================================================
// PF nibble builder
// ============================================================

static uint8_t pfLRC(uint8_t n0, uint8_t n1, uint8_t n2) {
    return 0x0F ^ n0 ^ n1 ^ n2;
}

static uint8_t pfBuildNibbles(uint8_t *nibbles, uint8_t channel, uint8_t mode,
                               uint8_t data, uint8_t flags) {
    uint8_t toggle;
    if (flags & 0x01) {
        toggle = (flags >> 1) & 1;
    } else {
        toggle = pfToggle[channel];
        pfToggle[channel] ^= 1;
    }
    uint8_t escape = (flags >> 2) & 1;

    switch (mode) {
    case PF_MODE_COMBO_DIRECT:
        nibbles[0] = ((toggle & 1) << 3) | ((escape & 1) << 2) | (channel & 0x03);
        nibbles[1] = 0x01;
        nibbles[2] = data & 0x0F;
        break;
    case PF_MODE_SINGLE_PWM:
        nibbles[0] = ((toggle & 1) << 3) | ((escape & 1) << 2) | (channel & 0x03);
        nibbles[1] = 0x04 | ((data >> 4) & 0x01);
        nibbles[2] = data & 0x0F;
        break;
    case PF_MODE_SINGLE_CST:
        nibbles[0] = ((toggle & 1) << 3) | ((escape & 1) << 2) | (channel & 0x03);
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

    // Initialize PWM module — we use PWM2 (P3.4 default routing)
    PWM_DATA2 = 0;
    PWM_CTRL &= ~bPWM2_OUT_EN;
    PIN_FUNC &= ~bPWM2_PIN_X;  // Ensure PWM2 routes to P3.4, not P3.1

    // Timer0 stopped
    TR0 = 0;
    ET0 = 0;

    state.active = IR_ACTIVE_NONE;
    pending.valid = 0;
    completionPending = 0;
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
    EA = 0;
    TR0 = 0;
    ET0 = 0;
    carrierDisable();
    state.active = IR_ACTIVE_NONE;
    pending.valid = 0;
    EA = 1;
}

uint8_t irGetCompletion(uint8_t *token, uint8_t *engine) {
    if (!completionPending) return 0;
    *token = completionToken;
    *engine = completionEngine;
    completionPending = 0;
    return 1;
}

void irPoll(void) {
    if (state.active != IR_ACTIVE_NONE || !pending.valid) return;

    // Activate pending request
    state.token = pending.token;
    state.engine = pending.engine;

    switch (pending.engineType) {
    case IR_ACTIVE_PF:
        state.active = IR_ACTIVE_PF;
        state.carrier_hz = 38000;
        state.duty_pct = 33;
        pfBuildNibbles(state.pf.nibbles, pending.pf.channel, pending.pf.mode,
                       pending.pf.data, pending.pf.flags);
        state.pf.channel = pending.pf.channel;
        state.pf.repeat = 0;
        state.pf.pos = 0;
        state.pf.phase = 0;
        break;

    case IR_ACTIVE_LEGACY:
        state.active = IR_ACTIVE_LEGACY;
        state.carrier_hz = 76000;
        state.duty_pct = 25;
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
        // RCX is always 38 kHz. The "long range" flag on real RCX hardware
        // changes IR LED drive power, NOT carrier frequency. carrierMode is
        // accepted for API compat but ignored.
        state.carrier_hz = 38000;
        state.duty_pct = 25;
        for (uint8_t i = 0; i < pending.rcx.count; i++) {
            state.rcx.bytes[i] = pending.rcx.bytes[i];
        }
        state.rcx.byteCount = pending.rcx.count;
        state.rcx.byteIdx = 0;
        state.rcx.bitIdx = 0;
        break;

    default:
        pending.valid = 0;
        return;
    }

    pending.valid = 0;
    beginTransmission();
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

uint8_t irStartRCX(const uint8_t *data, uint8_t dataLen, uint8_t carrierMode) {
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
    pending.rcx.carrierMode = carrierMode;
    pending.valid = 1;
    return token;
}

uint8_t irStartRCXRaw(const uint8_t *rawBytes, uint8_t rawLen, uint8_t carrierMode) {
    if (rawLen == 0 || rawLen > RCX_MAX_FRAMED_BYTES) return 0;
    if (state.active != IR_ACTIVE_NONE || pending.valid) return 0;

    uint8_t token = irNextToken();
    pending.token = token;
    pending.engine = IR_ENGINE_RCX;
    pending.engineType = IR_ACTIVE_RCX;
    for (uint8_t i = 0; i < rawLen; i++) pending.rcx.bytes[i] = rawBytes[i];
    pending.rcx.count = rawLen;
    pending.rcx.carrierMode = carrierMode;
    pending.valid = 1;
    return token;
}
