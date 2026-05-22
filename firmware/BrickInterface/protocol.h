#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>

// --- Framing ---
#define PROTO_SOF           0xAA
#define PROTO_MAX_PAYLOAD   32

// --- Commands (host -> device) ---
#define CMD_PING                0x01
#define CMD_GET_VERSION         0x02
#define CMD_GET_CAPABILITIES    0x03
#define CMD_RESET_STATE         0x04
#define CMD_ENTER_BOOTLOADER    0x05

#define CMD_IFACE_SET_OUTPUTS   0x10
#define CMD_IFACE_GET_INPUTS    0x11
#define CMD_IFACE_GET_COUNTS    0x12
#define CMD_IFACE_RESET_COUNT   0x13

#define CMD_PF_SEND             0x20

#define CMD_LEGACY_SEND         0x30

#define CMD_IR_ABORT_ALL        0x40

#define CMD_RCX_SEND            0x50
#define CMD_RCX_SEND_RAW        0x51

// --- Replies / Events (device -> host) ---
#define REPLY_PONG              0x81
#define REPLY_VERSION           0x82
#define REPLY_CAPABILITIES      0x83
#define REPLY_OK                0x84

#define REPLY_IFACE_INPUTS      0x90
#define REPLY_IFACE_COUNTS      0x91

#define REPLY_IR_ACCEPTED       0xA0
#define REPLY_IR_DONE           0xA1

#define REPLY_ERROR             0xE0

// --- Error codes ---
#define ERR_BAD_CHECKSUM        0x01
#define ERR_BAD_LENGTH          0x02
#define ERR_UNKNOWN_CMD         0x03
#define ERR_BAD_ARGUMENT        0x04
#define ERR_QUEUE_FULL          0x05
#define ERR_BUSY                0x06
#define ERR_UNSUPPORTED         0x07
#define ERR_INVALID_STATE       0x08

// --- IR engine IDs ---
#define IR_ENGINE_PF            0x01
#define IR_ENGINE_LEGACY        0x02
#define IR_ENGINE_RCX           0x03

// --- Capability bits ---
#define CAP_INTERFACE_A         (1 << 0)
#define CAP_PF_IR               (1 << 1)
#define CAP_LEGACY_IR           (1 << 2)
#define CAP_ASYNC_IFACE         (1 << 3)
#define CAP_IR_DONE_EVENTS      (1 << 4)
#define CAP_RCX_IR              (1 << 5)

// --- Version ---
#define PROTO_VERSION_MAJOR     1
#define PROTO_VERSION_MINOR     0
#define FW_VERSION_MAJOR        0
#define FW_VERSION_MINOR        3

// --- PF modes ---
#define PF_MODE_COMBO_DIRECT    0x00
#define PF_MODE_SINGLE_PWM      0x01
#define PF_MODE_SINGLE_CST      0x02
#define PF_MODE_COMBO_PWM       0x03

// --- Parsed packet structure ---
typedef struct {
    uint8_t seq;
    uint8_t cmd;
    uint8_t payload[PROTO_MAX_PAYLOAD];
    uint8_t payload_len;
} Packet;

#endif
