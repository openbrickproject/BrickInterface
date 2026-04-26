#ifndef PACKET_H
#define PACKET_H

#include "protocol.h"

// Parser states
#define PARSE_WAIT_SOF  0
#define PARSE_READ_LEN  1
#define PARSE_READ_BODY 2
#define PARSE_READ_CHK  3

typedef struct {
    uint8_t state;
    uint8_t len;
    uint8_t index;
    uint8_t buf[PROTO_MAX_PAYLOAD + 2];
    uint8_t xorAcc;
    Packet  pkt;
    uint8_t ready;  // 1 when a complete packet is available
} PacketParser;

void parserInit(PacketParser *p);
void parserConsume(PacketParser *p, uint8_t byte);

// Packet builder — writes framed packet to buf, returns length
uint8_t buildPacket(uint8_t *buf, uint8_t seq, uint8_t cmd,
                    const uint8_t *payload, uint8_t payloadLen);

// Send a reply over USB serial
void sendReply(uint8_t seq, uint8_t cmd,
               const uint8_t *payload, uint8_t payloadLen);

void sendError(uint8_t seq, uint8_t errorCode, uint8_t detail);

#endif
