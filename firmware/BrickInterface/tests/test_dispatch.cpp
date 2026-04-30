// Layer 2 — command-dispatch tests.
//
// Covers the handler-level reply framing and the validation paths
// (CMD_PF_SEND with bad mode, CMD_LEGACY_SEND with bad nibbles,
// CMD_RCX_SEND too long, IR busy, etc.). Exercises BrickInterface.ino
// directly by including it after pulling in the other firmware sources.
#include "test_harness.h"
#include "../packet.cpp"
#include "../ir_engine.cpp"
#include "../interface_a.cpp"
#include "../BrickInterface.ino"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void reset_capture(void) {
    Serial.tx_len = 0;
}

static void make_packet(Packet *pkt, uint8_t seq, uint8_t cmd,
                        const uint8_t *payload, uint8_t len) {
    pkt->seq = seq;
    pkt->cmd = cmd;
    pkt->payload_len = len;
    if (payload && len) memcpy(pkt->payload, payload, len);
}

// Decode the first reply frame from Serial capture into seq/cmd/payload[].
// Returns 1 if a valid frame was found, 0 otherwise.
static uint8_t decode_first_reply(uint8_t *seq, uint8_t *cmd,
                                  uint8_t *payload, uint8_t *payload_len) {
    if (Serial.tx_len < 5) return 0;
    if (Serial.tx_buf[0] != 0xAA) return 0;
    uint8_t len = Serial.tx_buf[1];
    if (Serial.tx_len < (uint16_t)(2 + len + 1)) return 0;
    *seq = Serial.tx_buf[2];
    *cmd = Serial.tx_buf[3];
    *payload_len = len - 2;
    if (*payload_len) memcpy(payload, &Serial.tx_buf[4], *payload_len);
    // Verify checksum
    uint8_t chk = len ^ *seq ^ *cmd;
    for (uint8_t i = 0; i < *payload_len; i++) chk ^= payload[i];
    if (chk != Serial.tx_buf[2 + len]) return 0;
    return 1;
}

// Reset firmware state between tests to keep them independent.
static void reset_firmware_state(void) {
    irAbortAll();
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;
    pfToggle[0] = pfToggle[1] = pfToggle[2] = pfToggle[3] = 0;
    test_reset_pin_state();
    reset_capture();
}

// ---------------------------------------------------------------------------
// Core commands
// ---------------------------------------------------------------------------

TEST(dispatch_ping_replies_pong) {
    reset_firmware_state();
    Packet pkt;
    make_packet(&pkt, 0x5A, CMD_PING, NULL, 0);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(seq, 0x5A);
    ASSERT_EQ(cmd, REPLY_PONG);
    ASSERT_EQ(plen, 0);
}

TEST(dispatch_get_version_returns_4_bytes) {
    reset_firmware_state();
    Packet pkt;
    make_packet(&pkt, 0x01, CMD_GET_VERSION, NULL, 0);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_VERSION);
    ASSERT_EQ(plen, 4);
    ASSERT_EQ(payload[0], PROTO_VERSION_MAJOR);
    ASSERT_EQ(payload[1], PROTO_VERSION_MINOR);
}

TEST(dispatch_get_capabilities_includes_interface_a) {
    reset_firmware_state();
    Packet pkt;
    make_packet(&pkt, 0x02, CMD_GET_CAPABILITIES, NULL, 0);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_CAPABILITIES);
    ASSERT_EQ(plen, 2);
    uint16_t caps = payload[0] | (payload[1] << 8);
    ASSERT_TRUE(caps & CAP_INTERFACE_A);
    ASSERT_TRUE(caps & CAP_PF_IR);
    ASSERT_TRUE(caps & CAP_LEGACY_IR);
    ASSERT_TRUE(caps & CAP_RCX_IR);
}

TEST(dispatch_unknown_cmd_returns_error) {
    reset_firmware_state();
    Packet pkt;
    make_packet(&pkt, 0x99, 0xFF /*not a real cmd*/, NULL, 0);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_UNKNOWN_CMD);
}

// ---------------------------------------------------------------------------
// Interface A
// ---------------------------------------------------------------------------

TEST(dispatch_iface_set_outputs_replies_state) {
    reset_firmware_state();
    Packet pkt;
    uint8_t bits = 0x05;
    make_packet(&pkt, 0x10, CMD_IFACE_SET_OUTPUTS, &bits, 1);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_IFACE_STATE);
    ASSERT_EQ(plen, 1);
    // Outputs 0 and 2 set => low 6 bits = 0x05
    ASSERT_EQ(payload[0] & 0x3F, 0x05);
}

TEST(dispatch_iface_set_outputs_bad_length_errors) {
    reset_firmware_state();
    Packet pkt;
    make_packet(&pkt, 0x11, CMD_IFACE_SET_OUTPUTS, NULL, 0);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BAD_LENGTH);
}

TEST(dispatch_iface_set_pwm_replies_ok) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {3, 128};  // output 3, duty 50%
    make_packet(&pkt, 0x12, CMD_IFACE_SET_PWM, p, 2);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_OK);
}

TEST(dispatch_iface_set_pwm_bad_output_errors) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {6, 128};  // output 6 invalid (only 0-5)
    make_packet(&pkt, 0x13, CMD_IFACE_SET_PWM, p, 2);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BAD_ARGUMENT);
}

TEST(dispatch_iface_set_pwm_all_replies_ok) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {10, 20, 30, 40, 50, 60};
    make_packet(&pkt, 0x14, CMD_IFACE_SET_PWM_ALL, p, 6);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_OK);
}

// ---------------------------------------------------------------------------
// PF — invalid mode rejection (the regression we just guarded)
// ---------------------------------------------------------------------------

TEST(dispatch_pf_send_rejects_invalid_mode) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {0 /*ch*/, 0xFF /*invalid mode*/, 0, 0};
    make_packet(&pkt, 0x20, CMD_PF_SEND, p, 4);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BAD_ARGUMENT);
}

TEST(dispatch_pf_send_accepts_valid_mode) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {0, PF_MODE_COMBO_DIRECT, 0x01, 0x01};
    make_packet(&pkt, 0x21, CMD_PF_SEND, p, 4);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_IR_ACCEPTED);
    ASSERT_EQ(plen, 2);
    ASSERT_EQ(payload[1], IR_ENGINE_PF);
}

TEST(dispatch_pf_send_busy_returns_error) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {0, PF_MODE_COMBO_DIRECT, 0x01, 0x01};

    // First send: accepted
    make_packet(&pkt, 0x22, CMD_PF_SEND, p, 4);
    handlePacket(&pkt);

    // Second send while first is still pending: BUSY
    reset_capture();
    make_packet(&pkt, 0x23, CMD_PF_SEND, p, 4);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BUSY);
}

TEST(dispatch_pf_start_hold_rejects_bad_channel) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {4 /*invalid ch*/, PF_MODE_COMBO_DIRECT, 0, 0};
    make_packet(&pkt, 0x24, CMD_PF_START_HOLD, p, 4);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BAD_ARGUMENT);
}

// ---------------------------------------------------------------------------
// Legacy — invalid nibble rejection
// ---------------------------------------------------------------------------

TEST(dispatch_legacy_send_rejects_orange_out_of_range) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {4 /*ch*/, 0x10 /*invalid nibble, > 0x0F*/, 0x00};
    make_packet(&pkt, 0x30, CMD_LEGACY_SEND, p, 3);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BAD_ARGUMENT);
}

TEST(dispatch_legacy_send_rejects_yellow_out_of_range) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {4, 0x07, 0xF0 /*invalid nibble*/};
    make_packet(&pkt, 0x31, CMD_LEGACY_SEND, p, 3);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_ERROR);
    ASSERT_EQ(payload[0], ERR_BAD_ARGUMENT);
}

TEST(dispatch_legacy_send_accepts_valid_nibbles) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {4, 0x07, 0x08};
    make_packet(&pkt, 0x32, CMD_LEGACY_SEND, p, 3);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_IR_ACCEPTED);
    ASSERT_EQ(payload[1], IR_ENGINE_LEGACY);
}

// ---------------------------------------------------------------------------
// RCX — oversized payload rejection
// ---------------------------------------------------------------------------

TEST(dispatch_rcx_send_rejects_too_long) {
    reset_firmware_state();
    Packet pkt;
    // carrier_mode + 17 data bytes — exceeds 16-byte data limit
    uint8_t p[18] = {0};
    make_packet(&pkt, 0x40, CMD_RCX_SEND, p, 18);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    // irStartRCX returns 0 for oversized → handler reports ERR_BUSY (the
    // "couldn't queue" path). Acceptable as the protocol contract — what
    // matters is that we don't crash or silently truncate.
    ASSERT_EQ(cmd, REPLY_ERROR);
}

TEST(dispatch_rcx_send_accepts_valid) {
    reset_firmware_state();
    Packet pkt;
    uint8_t p[] = {0 /*carrier_mode*/, 0x12 /*data*/};
    make_packet(&pkt, 0x41, CMD_RCX_SEND, p, 2);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_IR_ACCEPTED);
    ASSERT_EQ(payload[1], IR_ENGINE_RCX);
}

// ---------------------------------------------------------------------------
// IR control
// ---------------------------------------------------------------------------

TEST(dispatch_ir_abort_all_replies_ok) {
    reset_firmware_state();
    Packet pkt;
    make_packet(&pkt, 0x50, CMD_IR_ABORT_ALL, NULL, 0);
    handlePacket(&pkt);

    uint8_t seq, cmd, payload[32], plen;
    ASSERT_EQ(decode_first_reply(&seq, &cmd, payload, &plen), 1);
    ASSERT_EQ(cmd, REPLY_OK);
}

int main(void) {
    RUN_ALL_TESTS();
}
