#include <Arduino.h>
#include "packet.h"
#include "act_led.h"

void parserInit(PacketParser *p) {
    p->state = PARSE_WAIT_SOF;
    p->ready = 0;
}

void parserConsume(PacketParser *p, uint8_t byte) {
    switch (p->state) {
    case PARSE_WAIT_SOF:
        if (byte == PROTO_SOF) p->state = PARSE_READ_LEN;
        break;

    case PARSE_READ_LEN:
        p->len = byte;
        if (p->len < 2 || p->len > (PROTO_MAX_PAYLOAD + 2)) {
            p->state = PARSE_WAIT_SOF;
            break;
        }
        p->index = 0;
        p->xorAcc = p->len;
        p->state = PARSE_READ_BODY;
        break;

    case PARSE_READ_BODY:
        p->buf[p->index] = byte;
        p->xorAcc ^= byte;
        p->index++;
        if (p->index >= p->len) p->state = PARSE_READ_CHK;
        break;

    case PARSE_READ_CHK:
        p->state = PARSE_WAIT_SOF;
        if (p->xorAcc != byte) break;  // Checksum mismatch
        p->pkt.seq = p->buf[0];
        p->pkt.cmd = p->buf[1];
        p->pkt.payload_len = p->len - 2;
        for (uint8_t i = 0; i < p->pkt.payload_len; i++) {
            p->pkt.payload[i] = p->buf[2 + i];
        }
        p->ready = 1;
        break;

    default:
        p->state = PARSE_WAIT_SOF;
        break;
    }
}

uint8_t buildPacket(uint8_t *buf, uint8_t seq, uint8_t cmd,
                    const uint8_t *payload, uint8_t payloadLen) {
    uint8_t len = 2 + payloadLen;
    uint8_t chk = len ^ seq ^ cmd;
    uint8_t pos = 0;

    buf[pos++] = PROTO_SOF;
    buf[pos++] = len;
    buf[pos++] = seq;
    buf[pos++] = cmd;
    for (uint8_t i = 0; i < payloadLen; i++) {
        buf[pos++] = payload[i];
        chk ^= payload[i];
    }
    buf[pos++] = chk;
    return pos;
}

void sendReply(uint8_t seq, uint8_t cmd,
               const uint8_t *payload, uint8_t payloadLen) {
    uint8_t buf[1 + 1 + 1 + 1 + PROTO_MAX_PAYLOAD + 1];
    uint8_t len = buildPacket(buf, seq, cmd, payload, payloadLen);
    Serial.write(buf, len);
    actLedPulse();
}

void sendError(uint8_t seq, uint8_t errorCode, uint8_t detail) {
    uint8_t payload[2] = { errorCode, detail };
    sendReply(seq, REPLY_ERROR, payload, 2);
}
