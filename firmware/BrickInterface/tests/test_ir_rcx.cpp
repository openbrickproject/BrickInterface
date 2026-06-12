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
    // carrier byte = 0: first phase (start-bit mark) must use carrier mode 1
    // (38 kHz), the only carrier RCX hardware understands.
    uint8_t data[] = {0x12};
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;

    ASSERT_TRUE(irStartRCX(data, 1, 0) != 0);
    irPoll();  // promotes pending -> state
    uint8_t cm; uint16_t d;
    ASSERT_EQ(irNextPhase(&cm, &d), 1);
    ASSERT_EQ(cm, 1);  // 38 kHz carrier mode
    irAbortAll();
}

TEST(rcx_carrier_select_changes_tick_not_mode) {
    // carrier byte = 1 selects the 37.04 kHz tick (reload 27 counts, bit 31
    // ticks). Carrier *mode* stays 1 — frequency comes from the Timer 2
    // rate, not the toggle cadence.
    uint8_t data[] = {0x12};
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;

    ASSERT_TRUE(irStartRCX(data, 1, 1) != 0);
    irPoll();
    ASSERT_EQ(RCAP2H, 0xFF);
    ASSERT_EQ(RCAP2L, 0xE5);  // 65536 - 27
    uint8_t cm; uint16_t d;
    ASSERT_EQ(irNextPhase(&cm, &d), 1);
    ASSERT_EQ(cm, 1);
    ASSERT_EQ(d, 31);         // ~418 us bit at the 13.5 us tick
    irAbortAll();
    ASSERT_EQ(RCAP2L, T2_RELOAD_BASE_L);  // abort restores the base tick
    completionPending = 0;
}

TEST(rcx_carrier_select_out_of_range_clamps_to_default) {
    uint8_t data[] = {0x12};
    pending.valid = 0;
    state.active = IR_ACTIVE_NONE;

    ASSERT_TRUE(irStartRCX(data, 1, 9) != 0);
    irPoll();
    ASSERT_EQ(RCAP2L, T2_RELOAD_BASE_L);
    uint8_t cm; uint16_t d;
    ASSERT_EQ(irNextPhase(&cm, &d), 1);
    ASSERT_EQ(d, RCX_BIT_TICKS);
    irAbortAll();
    completionPending = 0;
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
    state.rcx.bitTicks = RCX_BIT_TICKS;
    state.rcx.repeat = 1;

    uint8_t c; uint16_t d;
    ASSERT_EQ(rcxNextPhase(&c, &d), 1);
    // Start bit: carrier on
    ASSERT_EQ(c, 1);
    ASSERT_EQ(d, RCX_BIT_TICKS);
}

TEST(rcx_phase_done_after_all_bytes) {
    state.active = IR_ACTIVE_RCX;
    state.rcx.bytes[0] = 0xAA;
    state.rcx.byteCount = 1;
    state.rcx.byteIdx = 0;
    state.rcx.bitIdx = 0;
    state.rcx.bitTicks = RCX_BIT_TICKS;
    state.rcx.repeat = 1;

    uint8_t c; uint16_t d;
    int phases = 0;
    while (rcxNextPhase(&c, &d) && phases < 50) phases++;
    // 1 byte = 11 phases (start + 8 data + parity + stop)
    ASSERT_EQ(phases, 11);
}

TEST(rcx_repeats_rewind_with_gap_phase) {
    // options 0x30 = carrier 0, 3 repeats: a 1-byte frame (11 bit phases)
    // must play 3 times with a deaf-window gap phase between repeats, and
    // one completion at the very end.
    irAbortAll();
    completionPending = 0;
    uint8_t data[] = { 0x12 };
    ASSERT_TRUE(irStartRCX(data, 1, 0x30) != 0);
    irPoll();

    int bit_phases = 0, gap_phases = 0;
    uint8_t cm; uint16_t d;
    while (irNextPhase(&cm, &d)) {
        if (d == RCX_REPEAT_GAP_TICKS && cm == 0) {
            gap_phases++;
        } else {
            ASSERT_EQ(d, RCX_BIT_TICKS);
            bit_phases++;
        }
        ASSERT_TRUE(bit_phases <= 300);
    }
    // Framed 0x12 = 7 bytes x 11 bits = 77 phases per repeat.
    ASSERT_EQ(bit_phases, 3 * 77);
    ASSERT_EQ(gap_phases, 2);
    completionPending = 0;
}

TEST(rcx_options_zero_repeats_means_one) {
    irAbortAll();
    completionPending = 0;
    uint8_t data[] = { 0x12 };
    ASSERT_TRUE(irStartRCX(data, 1, 0x02) != 0);  // carrier 2, repeats 0 -> 1
    irPoll();
    ASSERT_EQ(RCAP2L, 0xE7);                      // 40 kHz reload
    int phases = 0;
    uint8_t cm; uint16_t d;
    while (irNextPhase(&cm, &d) && phases < 200) phases++;
    ASSERT_EQ(phases, 77);                        // single transmission
    completionPending = 0;
}

TEST(rcx_start_arms_base_tick) {
    // RCX (like PF) runs the 13 us base tick: carrier toggles every ISR
    // for 38.46 kHz, and RCX_BIT_TICKS = 32 gives the 416 us UART bit.
    irAbortAll();              // clear state left behind by direct-state tests
    completionPending = 0;
    uint8_t data[2] = { 0x21, 0x81 };
    uint8_t tok = irStartRCX(data, 2, 0);
    ASSERT_TRUE(tok != 0);
    irPoll();
    ASSERT_EQ(RCAP2H, T2_RELOAD_BASE_H);
    ASSERT_EQ(RCAP2L, T2_RELOAD_BASE_L);
    irAbortAll();
    completionPending = 0;
}

int main(void) {
    RUN_ALL_TESTS();
}
