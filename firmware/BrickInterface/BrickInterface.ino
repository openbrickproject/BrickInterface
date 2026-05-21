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
// Timer 2 ISR — software PWM driver for the 6 Interface A outputs.
//
// We use Timer 2 because ch55xduino's main.c declares the Timer 2 vector
// (pointing to a weak Timer2Interrupt symbol). Defining Timer2Interrupt
// in the sketch overrides the core's empty weak version at link time.
// SDCC only installs interrupt vectors in the translation unit containing
// main(), and main() lives in ch55xduino/cores/.../main.c — not in our .ino —
// so we cannot install a Timer 1 vector from user code.
void Timer2Interrupt(void) __interrupt {
    TF2 = 0;  // clear Timer 2 overflow flag (not auto-cleared)

    uint8_t c = pwmCounter++;
    P1_4 = (c < pwmDuty[0]) ? 1 : 0;  // OUT0
    P1_5 = (c < pwmDuty[1]) ? 1 : 0;  // OUT1
    P1_6 = (c < pwmDuty[2]) ? 1 : 0;  // OUT2
    P1_7 = (c < pwmDuty[3]) ? 1 : 0;  // OUT3
    P3_1 = (c < pwmDuty[4]) ? 1 : 0;  // OUT4
    P3_0 = (c < pwmDuty[5]) ? 1 : 0;  // OUT5
}

// ============================================================
void setup() {
    // USB CDC needs no setup — ch55xduino brings it up automatically when
    // the "Default CDC" USB Settings option is selected. Baud rate is
    // meaningless on USB CDC.

    pinMode(ACT_LED_PIN, OUTPUT);
    digitalWrite(ACT_LED_PIN, LOW);

    ifaceInit();
    // irInit();
    // parserInit(&parser);
}

// ============================================================
void loop() {
    // // Parse incoming packets
    // while (USBSerial_available()) {
    //     uint8_t b = USBSerial_read();
    //     actLedPulse();
    //     parserConsume(&parser, b);
    //     if (parser.ready) {
    //         parser.ready = 0;
    //         handlePacket(&parser.pkt);
    //     }
    // }
    //
    // // IR completion events
    // uint8_t token, engine;
    // if (irGetCompletion(&token, &engine)) {
    //     uint8_t payload[2] = { token, engine };
    //     sendReply(0x00, REPLY_IR_DONE, payload, 2);
    // }
    //
    // actLedTick();
    // irPoll();
    // ifaceEdgePoll();

    // // ISR-BYPASS TEST: directly toggle output 0 without using PWM ISR.
    // // If output 0 toggles 2s/2s: pin is driveable, ISR is the problem.
    // // If output 0 stays on: something else is wrong (pinMode / pin mapping).
    // digitalWrite(IFACE_OUT0_PIN, HIGH);
    // digitalWrite(ACT_LED_PIN, HIGH);
    // delay(2000);
    // digitalWrite(IFACE_OUT0_PIN, LOW);
    // digitalWrite(ACT_LED_PIN, LOW);
    // delay(2000);

    static uint8_t duties[6] = {0, 0, 0, 0, 0, 0};
    for (uint8_t i = 0; i < 6; i++) {
        duties[i] = 255;
        ifaceSetOutputs(duties, 0x3F);
        digitalWrite(ACT_LED_PIN, HIGH);
        delay(2000);
        duties[i] = 0;
        ifaceSetOutputs(duties, 0x3F);
        digitalWrite(ACT_LED_PIN, LOW);
        delay(2000);
    }
    // static uint8_t duties[6] = {0, 0, 0, 0, 0, 0};
    // for (uint8_t i = 0; i < 6; i++) {
    //     duties[i] = 255;
    //     ifaceSetOutputs(duties, 0x3F);
    //     digitalWrite(ACT_LED_PIN, HIGH);
    //     delay(2000);
    //     duties[i] = 0;
    //     ifaceSetOutputs(duties, 0x3F);
    //     digitalWrite(ACT_LED_PIN, LOW);
    //     delay(2000);
    // }
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
