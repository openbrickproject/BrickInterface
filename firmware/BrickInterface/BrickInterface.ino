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

// --- Hold state for PF ---
static struct {
    uint8_t active;
    uint8_t channel, mode, data, flags;
    unsigned long lastSentMs;
} pfHolds[4];

// --- Hold state for Legacy ---
static struct {
    uint8_t active;
    uint8_t channelCode, orange, yellow;
    unsigned long lastSentMs;
} legacyHolds[4];

// PF hold repeat interval (ms)
#define PF_HOLD_INTERVAL_MS  200

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
        actLedOffMs = 0;
    }
}

// --- Forward declarations ---
static void handlePacket(const Packet *pkt);
static void processHolds(void);
static void doResetState(void);
static void enterBootloader(void);
static uint8_t isValidPFMode(uint8_t mode);
static uint8_t isValidLegacyNibble(uint8_t value);

// ============================================================
void setup() {
    Serial.begin(115200);

    pinMode(ACT_LED_PIN, OUTPUT);
    digitalWrite(ACT_LED_PIN, LOW);

    ifaceInit();
    irInit();
    parserInit(&parser);

    for (uint8_t i = 0; i < 4; i++) {
        pfHolds[i].active = 0;
        legacyHolds[i].active = 0;
    }
}

// ============================================================
void loop() {
    // Parse incoming packets
    while (Serial.available()) {
        uint8_t b = Serial.read();
        actLedPulse();
        parserConsume(&parser, b);
        if (parser.ready) {
            parser.ready = 0;
            handlePacket(&parser.pkt);
        }
    }

    // IR completion events
    uint8_t token, engine;
    if (irGetCompletion(&token, &engine)) {
        uint8_t payload[2] = { token, engine };
        sendReply(0x00, REPLY_IR_DONE, payload, 2);
    }

    actLedTick();
    processHolds();
    irPoll();
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
        Serial.flush();  // block until USB IN ACK from host
        enterBootloader();
        break;

    // --- Interface A ---
    case CMD_IFACE_SET_OUTPUTS: {
        if (pkt->payload_len != 1) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        ifaceSetOutputs(pkt->payload[0]);
        uint8_t s = ifaceGetState();
        sendReply(pkt->seq, REPLY_IFACE_STATE, &s, 1);
        break;
    }

    case CMD_IFACE_GET_STATE: {
        uint8_t s = ifaceGetState();
        sendReply(pkt->seq, REPLY_IFACE_STATE, &s, 1);
        break;
    }

    case CMD_IFACE_SET_OUT_MASK: {
        if (pkt->payload_len != 2) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        ifaceSetOutputMask(pkt->payload[0], pkt->payload[1]);
        uint8_t s = ifaceGetState();
        sendReply(pkt->seq, REPLY_IFACE_STATE, &s, 1);
        break;
    }

    case CMD_IFACE_SET_PWM: {
        if (pkt->payload_len != 2) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        if (pkt->payload[0] >= 6) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        ifaceSetPWM(pkt->payload[0], pkt->payload[1]);
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;
    }

    case CMD_IFACE_SET_PWM_ALL: {
        if (pkt->payload_len != 6) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        ifaceSetPWMAll(pkt->payload);
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

    case CMD_PF_START_HOLD: {
        if (pkt->payload_len != 4) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        uint8_t ch = pkt->payload[0];
        if (ch > 3) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        if (!isValidPFMode(pkt->payload[1])) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        pfHolds[ch].active = 1;
        pfHolds[ch].channel = ch;
        pfHolds[ch].mode = pkt->payload[1];
        pfHolds[ch].data = pkt->payload[2];
        pfHolds[ch].flags = pkt->payload[3];
        pfHolds[ch].lastSentMs = 0;
        uint8_t p[2] = { irNextToken(), IR_ENGINE_PF };
        sendReply(pkt->seq, REPLY_IR_ACCEPTED, p, 2);
        break;
    }

    case CMD_PF_STOP_HOLD: {
        if (pkt->payload_len != 1) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        uint8_t ch = pkt->payload[0];
        if (ch > 3) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        pfHolds[ch].active = 0;
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;
    }

    case CMD_PF_STOP_ALL:
        for (uint8_t i = 0; i < 4; i++) pfHolds[i].active = 0;
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;

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

    case CMD_LEGACY_START_HOLD: {
        if (pkt->payload_len != 3) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        uint8_t ch = pkt->payload[0];
        if (ch < 4 || ch > 7) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        if (!isValidLegacyNibble(pkt->payload[1]) || !isValidLegacyNibble(pkt->payload[2])) {
            sendError(pkt->seq, ERR_BAD_ARGUMENT, 0);
            break;
        }
        uint8_t idx = ch - 4;
        legacyHolds[idx].active = 1;
        legacyHolds[idx].channelCode = ch;
        legacyHolds[idx].orange = pkt->payload[1];
        legacyHolds[idx].yellow = pkt->payload[2];
        legacyHolds[idx].lastSentMs = 0;
        uint8_t p[2] = { irNextToken(), IR_ENGINE_LEGACY };
        sendReply(pkt->seq, REPLY_IR_ACCEPTED, p, 2);
        break;
    }

    case CMD_LEGACY_STOP_HOLD: {
        if (pkt->payload_len != 1) { sendError(pkt->seq, ERR_BAD_LENGTH, 0); break; }
        uint8_t ch = pkt->payload[0];
        if (ch < 4 || ch > 7) { sendError(pkt->seq, ERR_BAD_ARGUMENT, 0); break; }
        legacyHolds[ch - 4].active = 0;
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;
    }

    case CMD_LEGACY_STOP_ALL:
        for (uint8_t i = 0; i < 4; i++) legacyHolds[i].active = 0;
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;

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
        for (uint8_t i = 0; i < 4; i++) {
            pfHolds[i].active = 0;
            legacyHolds[i].active = 0;
        }
        sendReply(pkt->seq, REPLY_OK, NULL, 0);
        break;

    default:
        sendError(pkt->seq, ERR_UNKNOWN_CMD, 0);
        break;
    }
}

// ============================================================
static void processHolds(void) {
    unsigned long now = millis();

    for (uint8_t i = 0; i < 4; i++) {
        if (!pfHolds[i].active) continue;
        if (now - pfHolds[i].lastSentMs < PF_HOLD_INTERVAL_MS) continue;
        if (irStartPF(pfHolds[i].channel, pfHolds[i].mode,
                       pfHolds[i].data, pfHolds[i].flags)) {
            pfHolds[i].lastSentMs = now;
            }
    }

    for (uint8_t i = 0; i < 4; i++) {
        if (!legacyHolds[i].active) continue;
        uint16_t interval;
        switch (legacyHolds[i].channelCode) {
            case 4: case 5: interval = 51; break;
            case 6: interval = 68; break;
            case 7: interval = 86; break;
            default: interval = 51; break;
        }
        if (now - legacyHolds[i].lastSentMs < interval) continue;
        if (irStartLegacy(legacyHolds[i].channelCode,
                          legacyHolds[i].orange, legacyHolds[i].yellow)) {
            legacyHolds[i].lastSentMs = now;
            }
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
    ifaceSetOutputs(0);
    irAbortAll();
    for (uint8_t i = 0; i < 4; i++) {
        pfHolds[i].active = 0;
        legacyHolds[i].active = 0;
    }
    pfToggle[0] = pfToggle[1] = pfToggle[2] = pfToggle[3] = 0;
}

// ============================================================
// Jump to CH552 ROM bootloader (entry at 0x3800).
// Tear down USB so the host re-enumerates the WCH bootloader VID/PID,
// disable interrupts, then long-jump into ROM.
static void enterBootloader(void) {
    // Halt active peripherals before tearing down USB and jumping to ROM.
    irAbortAll();
    ifaceSetOutputs(0);
    USB_INT_EN = 0;
    USB_CTRL = 0;
    UDEV_CTRL = 0;       // detach from bus
    delay(100);          // let host see the disconnect
    EA = 0;              // disable all interrupts
#ifdef __SDCC
    __asm__("ljmp #0x3800");
#endif
}
