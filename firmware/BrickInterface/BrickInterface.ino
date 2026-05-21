// BrickInterface Firmware
// CH552T (TSSOP-20, ch55xduino)
//
// Supports:
//   - LEGO Interface A (6 outputs, 2 inputs)
//   - LEGO Power Functions IR (38 kHz)
//   - LEGO Legacy IR (76 kHz)
//   - LEGO RCX IR (38 kHz)

// USB CDC mode is selected via Arduino IDE Tools menu (boards.txt build flags),
// not via sketch macros. The descriptor (VID 0x1209, PID 0xc550 — pid.codes/ch55xduino)
// is hardcoded in the ch55xduino core and not user-overridable from the sketch.

#include "board_config.h"
#include "protocol.h"
#include "packet.h"
#include "ir_engine.h"
#include "interface_a.h"
#include "act_led.h"

// --- Capabilities for this board ---
#define DEVICE_CAPS (CAP_INTERFACE_A | CAP_PF_IR | CAP_LEGACY_IR | CAP_IR_DONE_EVENTS | CAP_RCX_IR)

// --- Packet parser ---
static PacketParser parser;

// --- Activity LED state (any-direction USB-CDC byte triggers a pulse) ---
#define ACT_LED_DURATION_MS 30
static unsigned long actLedOffMs = 0;

void actLedPulse(void) {
    digitalWrite(ACT_LED_PIN, HIGH);
    actLedOffMs = millis() + ACT_LED_DURATION_MS;
}

void actLedTick(void) {
    if (actLedOffMs && (long)(millis() - actLedOffMs) >= 0) {
        digitalWrite(ACT_LED_PIN, LOW);
        actLedOffMs = 0;
    }
}

// --- Forward declarations ---
static void handlePacket(const Packet *pkt);
static void doResetState(void);
static void enterBootloader(void);
static uint8_t isValidPFMode(uint8_t mode);
static uint8_t isValidLegacyNibble(uint8_t value);

// ============================================================
// Unified Timer 2 ISR — 153.85 kHz.
//
// Three responsibilities, all run from the same vector:
//   (1) IR carrier toggle on P3.4
//       - carrierMode=2: toggle every ISR → 76.92 kHz output (Legacy)
//       - carrierMode=1: toggle every 2 ISRs → 38.46 kHz output (PF, RCX)
//       - carrierMode=0: pin held low (spaces, idle)
//   (2) IR envelope state machine
//       - decrements envelopeTicks; when it hits 0, calls irNextPhase()
//         to load the next mark/space duration, or signals completion
//   (3) Interface A software PWM tick
//       - every 6th ISR → 25.6 kHz tick rate → 100 Hz PWM at 256 levels
//
// SDCC installs the Timer 2 vector via the weak Timer2Interrupt() in
// ch55xduino's core; our definition here overrides it at link time.
// (Timer 0 and Timer 1 have no equivalent hook from user code.)
// ============================================================
#define IFA_PRESCALE_MAX 6

static volatile __data uint8_t ifaPrescale = 0;

void Timer2Interrupt(void) __interrupt {
    TF2 = 0;  // Timer 2 overflow flag is not auto-cleared

    // (1) Carrier toggle
    if (carrierMode == 2) {
        P3_4 = !P3_4;
    } else if (carrierMode == 1) {
        if (++carrierPrescale >= 2) {
            carrierPrescale = 0;
            P3_4 = !P3_4;
        }
    }

    // (2) Envelope phase progression
    if (envelopeTicks > 0) {
        if (--envelopeTicks == 0) {
            uint8_t cm;
            uint16_t ticks;
            if (irNextPhase(&cm, &ticks)) {
                carrierMode = cm;
                carrierPrescale = 0;
                if (cm == 0) P3_4 = 0;
                envelopeTicks = ticks;
            } else {
                // Transmission complete — irNextPhase already cleared
                // state.active and set the completion token/engine fields.
                carrierMode = 0;
                P3_4 = 0;
                completionPending = 1;
            }
        }
    }

    // (3) Interface A PWM (every 6th ISR ≈ 25.6 kHz tick)
    if (++ifaPrescale >= IFA_PRESCALE_MAX) {
        ifaPrescale = 0;
        uint8_t c = pwmCounter++;
        P1_4 = (c < pwmDuty[0]) ? 1 : 0;
        P1_5 = (c < pwmDuty[1]) ? 1 : 0;
        P1_6 = (c < pwmDuty[2]) ? 1 : 0;
        P1_7 = (c < pwmDuty[3]) ? 1 : 0;
        P3_1 = (c < pwmDuty[4]) ? 1 : 0;
        P3_0 = (c < pwmDuty[5]) ? 1 : 0;
    }
}

// ============================================================
void setup() {
    // USB CDC needs no setup — ch55xduino brings it up automatically when
    // the "Default CDC" USB Settings option is selected. Baud rate is
    // meaningless on USB CDC.

    pinMode(ACT_LED_PIN, OUTPUT);
    digitalWrite(ACT_LED_PIN, LOW);
    pinMode(IR_LED_PIN, OUTPUT);
    digitalWrite(IR_LED_PIN, LOW);

    ifaceInit();   // starts Timer 2 at 153.85 kHz (shared with the IR engine)
    irInit();
    parserInit(&parser);
}

// ============================================================
void loop() {
    // INTEGRATION TEST: IFA + PF IR cycling, 8 states × 2s each.
    //
    // PF data byte for Single Output PWM mode (channel 1, escape=0, addr=0):
    //   bit 4   = output select (0 = A/red, 1 = B/blue)
    //   bits 3-0 = PWM step (0 = float, 7 = fwd7, 9 = rev7)
    //
    //   Blue fwd  = 0x17    Blue rev  = 0x19    Blue float = 0x10
    //   Red  fwd  = 0x07    Red  rev  = 0x09    Red  float = 0x00
    static uint8_t step = 0;
    static uint8_t duties[6] = {0, 0, 0, 0, 0, 0};

    for (uint8_t i = 0; i < 6; i++) duties[i] = 0;
    uint8_t pf_data;
    uint8_t act_led = LOW;

    switch (step) {
        case 0: duties[0] = 255; pf_data = 0x17; act_led = HIGH; break;  // OUT0 + blue fwd
        case 1:                  pf_data = 0x10;                 break;  // off  + blue float
        case 2: duties[1] = 255; pf_data = 0x19; act_led = HIGH; break;  // OUT1 + blue rev
        case 3:                  pf_data = 0x10;                 break;  // off  + blue float
        case 4: duties[2] = 255; pf_data = 0x07; act_led = HIGH; break;  // OUT2 + red  fwd
        case 5:                  pf_data = 0x00;                 break;  // off  + red  float
        case 6: duties[3] = 255; pf_data = 0x09; act_led = HIGH; break;  // OUT3 + red  rev
        default:                 pf_data = 0x00;                 break;  // off  + red  float
    }

    digitalWrite(ACT_LED_PIN, act_led);
    ifaceSetOutputs(duties, 0x3F);

    // Queue the PF command and kick the engine. The Timer 2 ISR drives the
    // transmission to completion in the background (~400 ms), so the delay
    // below doesn't block the IR engine — only this loop.
    irStartPF(0, PF_MODE_SINGLE_PWM, pf_data, 0);
    irPoll();

    delay(2000);

    // Drain any IR completion that fired during the dwell so the engine's
    // completion slot doesn't go stale.
    uint8_t tok, eng;
    irGetCompletion(&tok, &eng);

    step = (step + 1) & 0x7;

    // // Normal dispatcher (restore by replacing the test block above):
    // while (USBSerial_available()) {
    //     uint8_t b = USBSerial_read();
    //     actLedPulse();
    //     parserConsume(&parser, b);
    //     if (parser.ready) {
    //         parser.ready = 0;
    //         handlePacket(&parser.pkt);
    //     }
    // }
    // uint8_t token, engine;
    // if (irGetCompletion(&token, &engine)) {
    //     uint8_t payload[2] = { token, engine };
    //     sendReply(0x00, REPLY_IR_DONE, payload, 2);
    // }
    // actLedTick();
    // irPoll();
    // ifaceEdgePoll();
}

// ============================================================
// Command dispatch
// ============================================================
static void handlePacket(const Packet *pkt) {
    switch (pkt->cmd) {

    // --- Core ---
    case CMD_PING:
        sendReply(pkt->seq, REPLY_PONG, NULL, 0);
        break;

    case CMD_GET_VERSION: {
        uint8_t p[4] = { PROTO_VERSION_MAJOR, PROTO_VERSION_MINOR,
                         FW_VERSION_MAJOR, FW_VERSION_MINOR };
        sendReply(pkt->seq, REPLY_VERSION, p, 4);
        break;
    }

    case CMD_GET_CAPABILITIES: {
        uint16_t caps = DEVICE_CAPS;
        uint8_t p[2] = { (uint8_t)(caps & 0xFF), (uint8_t)(caps >> 8) };
        sendReply(pkt->seq, REPLY_CAPABILITIES, p, 2);
        break;
    }

    case CMD_RESET_STATE:
        doResetState();
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;

    case CMD_ENTER_BOOTLOADER:
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        USBSerial_flush();  // block until USB IN ACK from host
        enterBootloader();
        break;

    // --- Interface A ---
    case CMD_IFACE_SET_OUTPUTS: {
        if (pkt->payload_len != 6 && pkt->payload_len != 7) {
            sendError(pkt->seq, ERR_BAD_LENGTH, 0); break;
        }
        uint8_t mask = (pkt->payload_len == 7) ? pkt->payload[6] : 0x3F;
        ifaceSetOutputs(pkt->payload, mask);
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;
    }

    case CMD_IFACE_GET_INPUTS: {
        uint8_t s = ifaceSampleInputs();
        sendReply(pkt->seq, REPLY_IFACE_INPUTS, &s, 1);
        break;
    }

    case CMD_IFACE_GET_COUNTS: {
        uint32_t c6 = ifaceGetCount(6);
        uint32_t c7 = ifaceGetCount(7);
        uint8_t p[8];
        p[0] = (uint8_t)(c6      ); p[1] = (uint8_t)(c6 >>  8);
        p[2] = (uint8_t)(c6 >> 16); p[3] = (uint8_t)(c6 >> 24);
        p[4] = (uint8_t)(c7      ); p[5] = (uint8_t)(c7 >>  8);
        p[6] = (uint8_t)(c7 >> 16); p[7] = (uint8_t)(c7 >> 24);
        sendReply(pkt->seq, REPLY_IFACE_COUNTS, p, 8);
        break;
    }

    case CMD_IFACE_RESET_COUNT: {
        if (pkt->payload_len != 1) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        if (pkt->payload[0] != 6 && pkt->payload[0] != 7) {
            sendError(pkt->seq, ERR_BAD_ARGUMENT, 0);
            break;
        }
        ifaceResetCount(pkt->payload[0]);
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;
    }

    // --- PF ---
    case CMD_PF_SEND: {
        if (pkt->payload_len != 4) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        if (!isValidPFMode(pkt->payload[1])) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        uint8_t tok = irStartPF(pkt->payload[0], pkt->payload[1],
                                pkt->payload[2], pkt->payload[3]);
        if (!tok) { sendError(pkt->seq, ERR_BUSY, 0); break; }
        uint8_t p[2] = { tok, IR_ENGINE_PF };
        sendReply(pkt->seq, REPLY_IR_ACCEPTED, p, 2);
        break;
    }

    // --- Legacy ---
    case CMD_LEGACY_SEND: {
        if (pkt->payload_len != 3) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        if (!isValidLegacyNibble(pkt->payload[1]) || !isValidLegacyNibble(pkt->payload[2])) {
            sendError(pkt->seq, ERR_BAD_ARGUMENT, 0);
            break;
        }
        uint8_t tok = irStartLegacy(pkt->payload[0], pkt->payload[1], pkt->payload[2]);
        if (!tok) { sendError(pkt->seq, ERR_BUSY, 0); break; }
        uint8_t p[2] = { tok, IR_ENGINE_LEGACY };
        sendReply(pkt->seq, REPLY_IR_ACCEPTED, p, 2);
        break;
    }

    // --- RCX ---
    case CMD_RCX_SEND: {
        if (pkt->payload_len < 2) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        if ((pkt->payload_len - 1) > 16) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        uint8_t tok = irStartRCX(&pkt->payload[1], pkt->payload_len - 1, pkt->payload[0]);
        if (!tok) { sendError(pkt->seq, ERR_BUSY, 0); break; }
        uint8_t p[2] = { tok, IR_ENGINE_RCX };
        sendReply(pkt->seq, REPLY_IR_ACCEPTED, p, 2);
        break;
    }

    case CMD_RCX_SEND_RAW: {
        if (pkt->payload_len < 2) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        uint8_t tok = irStartRCXRaw(&pkt->payload[1], pkt->payload_len - 1, pkt->payload[0]);
        if (!tok) { sendError(pkt->seq, ERR_BUSY, 0); break; }
        uint8_t p[2] = { tok, IR_ENGINE_RCX };
        sendReply(pkt->seq, REPLY_IR_ACCEPTED, p, 2);
        break;
    }

    // --- IR abort ---
    case CMD_IR_ABORT_ALL:
        irAbortAll();
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;

    default:
        sendError(pkt->seq, ERR_UNKNOWN_CMD, 0);
        break;
    }
}

// ============================================================
static uint8_t isValidPFMode(uint8_t mode) {
    switch (mode) {
        case PF_MODE_COMBO_DIRECT:
        case PF_MODE_SINGLE_PWM:
        case PF_MODE_SINGLE_CST:
            return 1;
        default:
            return 0;
    }
}

static uint8_t isValidLegacyNibble(uint8_t value) {
    return value <= 0x0F;
}

// ============================================================
static void doResetState(void) {
    ifaceClearAllOutputs();
    ifaceResetCount(6);
    ifaceResetCount(7);
    irAbortAll();
    pfToggle[0] = pfToggle[1] = pfToggle[2] = pfToggle[3] = 0;
}

// ============================================================
// Jump to CH552 ROM bootloader (entry at 0x3800).
// Tear down USB so the host re-enumerates the WCH bootloader VID/PID,
// disable interrupts, then long-jump into ROM.
static void enterBootloader(void) {
    // Halt active peripherals before tearing down USB and jumping to ROM.
    irAbortAll();
    ifaceClearAllOutputs();
    USB_INT_EN = 0;
    USB_CTRL = 0;
    UDEV_CTRL = 0;       // detach from bus
    delay(100);          // let host see the disconnect
    EA = 0;              // disable all interrupts
#ifdef __SDCC
    __asm__("ljmp #0x3800");
#endif
}
