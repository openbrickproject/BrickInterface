// Layer 1+2 — RCX IR framing + phase generator tests.
#include "test_harness.h"
#include "../ir_engine.h"
#include "../ir_engine.c"

TEST(rcx_framing_single_byte) {
    // irStartRCX builds the framed packet: 0x55 0xFF 0x00 D ~D C ~C
    // (where C = sum of data bytes mod 256)
    uint8_t data[] = {0x12};
    uint8_t tok = irStartRCX(data, 1, 0);
    ASSERT_TRUE(tok != 0);
    ASSERT_EQ(pending.rcx.count, 7);
    ASSERT_EQ(pending.rcx.bytes[0], 0x55);
    ASSERT_EQ(pending.rcx.bytes[1], 0xFF);
    ASSERT_EQ(pending.rcx.bytes[2], 0x00);
    ASSERT_EQ(pending.rcx.bytes[3], 0x12);
    ASSERT_EQ(pending.rcx.bytes[4], (uint8_t)~0x12);
    ASSERT_EQ(pending.rcx.bytes[5], 0x12);  // checksum = sum of data bytes
    ASSERT_EQ(pending.rcx.bytes[6], (uint8_t)~0x12);

    // Reset for next test
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;
}

TEST(rcx_framing_multi_byte_checksum) {
    uint8_t data[] = {0x10, 0x20, 0x30};
    uint8_t tok = irStartRCX(data, 3, 0);
    ASSERT_TRUE(tok != 0);
    // Framed: header(3) + 3 data pairs(6) + checksum pair(2) = 11
    ASSERT_EQ(pending.rcx.count, 11);
    // Data pairs
    ASSERT_EQ(pending.rcx.bytes[3], 0x10);
    ASSERT_EQ(pending.rcx.bytes[4], (uint8_t)~0x10);
    ASSERT_EQ(pending.rcx.bytes[5], 0x20);
    ASSERT_EQ(pending.rcx.bytes[7], 0x30);
    // Checksum = 0x10 + 0x20 + 0x30 = 0x60
    ASSERT_EQ(pending.rcx.bytes[9], 0x60);
    ASSERT_EQ(pending.rcx.bytes[10], (uint8_t)~0x60);

    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;
}

TEST(rcx_rejects_zero_length) {
    uint8_t dummy = 0;
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;
    ASSERT_EQ(irStartRCX(&dummy, 0, 0), 0);
}

TEST(rcx_rejects_oversized) {
    uint8_t data[20] = {0};
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;
    ASSERT_EQ(irStartRCX(data, 17, 0), 0);   // > 16 = reject
}

TEST(rcx_carrier_mode_ignored_38khz) {
    // carrierMode = 0 should yield 38 kHz
    uint8_t data[] = {0x12};
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;

    ASSERT_TRUE(irStartRCX(data, 1, 0) != 0);
    irPoll();  // promotes pending -> state, applies carrier_hz
    ASSERT_EQ(state.carrier_hz, 38000UL);
    irAbortAll();
}

TEST(rcx_carrier_mode_ignored_long_range_still_38khz) {
    // carrierMode = 1 ("long range") used to switch to 76 kHz — bug.
    // Post-fix: must remain 38 kHz.
    uint8_t data[] = {0x12};
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;

    ASSERT_TRUE(irStartRCX(data, 1, 1) != 0);
    irPoll();
    ASSERT_EQ(state.carrier_hz, 38000UL);
    irAbortAll();
}

// ============================================================================
// Layer 2 — phase generator
// ============================================================================

struct Phase { uint8_t carrier; uint16_t duration; };

TEST(rcx_phase_first_byte_start_bit) {
    state.active = IR_ACTIVE_RCX;
    state.rcx.bytes[0] = 0x55;
    state.rcx.bytes[1] = 0xFF;
    state.rcx.byteCount = 2;
    state.rcx.byteIdx = 0;
    state.rcx.bitIdx = 0;

    uint8_t c; uint16_t d;
    ASSERT_EQ(rcxNextPhase(&c, &d), 1);
    // Start bit: carrier on
    ASSERT_EQ(c, 1);
    ASSERT_EQ(d, RCX_BIT_US);
}

TEST(rcx_phase_done_after_all_bytes) {
    state.active = IR_ACTIVE_RCX;
    state.rcx.bytes[0] = 0xAA;
    state.rcx.byteCount = 1;
    state.rcx.byteIdx = 0;
    state.rcx.bitIdx = 0;

    uint8_t c; uint16_t d;
    int phases = 0;
    while (rcxNextPhase(&c, &d) && phases < 50) phases++;
    // 1 byte = 11 phases (start + 8 data + parity + stop)
    ASSERT_EQ(phases, 11);
}

int main(void) {
    RUN_ALL_TESTS();
}
