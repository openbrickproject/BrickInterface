// Layer 2 — Legacy IR phase generator tests.
#include "test_harness.h"
#include "../ir_engine.h"
#include "../ir_engine.c"

struct Phase { uint8_t carrier; uint16_t duration; };

static int play_legacy(uint8_t channelCode, uint8_t orange, uint8_t yellow,
                       Phase *out, int max) {
    // Build the two bytes per Legacy spec:
    //   byte0 = (channelCode << 4) | orange
    //   byte1 = (yellow << 4) | check
    //   check = (0x10 - (channelCode + orange + yellow)) & 0x0F
    state.active = IR_ACTIVE_LEGACY;
    state.legacy.byte0 = (channelCode << 4) | (orange & 0x0F);
    uint8_t check = (0x10 - (channelCode + orange + yellow)) & 0x0F;
    state.legacy.byte1 = ((yellow & 0x0F) << 4) | check;
    state.legacy.channelCode = channelCode;
    state.legacy.repeat = 0;
    state.legacy.byteIdx = 0;
    state.legacy.bitIdx = 0;
    state.legacy.inGap = 0;

    int n = 0;
    while (n < max) {
        uint8_t c; uint16_t d;
        if (!legacyNextPhase(&c, &d)) break;
        out[n].carrier = c;
        out[n].duration = d;
        n++;
    }
    return n;
}

TEST(legacy_frame_starts_with_mark) {
    Phase phases[300];
    int n = play_legacy(4, 0x07, 0x08, phases, 300);
    ASSERT_TRUE(n > 0);
    // First bit (bit 0 = start) should be a mark (carrier ON)
    ASSERT_EQ(phases[0].carrier, 1);
    ASSERT_EQ(phases[0].duration, LEGACY_BIT_US);
}

TEST(legacy_frame_has_correct_bit_length) {
    Phase phases[300];
    int n = play_legacy(4, 0x07, 0x08, phases, 300);
    // 5 repeats × 22 bits/frame + 4 inter-message gaps = 114 phases
    // (each repeat = 2 bytes × 11 phases = 22 phases)
    ASSERT_EQ(n, 5*22 + 4);
}

TEST(legacy_carrier_inverted_for_ones) {
    // Byte 0 = (4<<4) | 0x07 = 0x47 — bits LSB-first: 1 1 1 0 0 0 1 0
    // bit 1 (data bit 0 = LSB = 1) → carrier_on = !1 = 0 (space)
    Phase phases[300];
    int n = play_legacy(4, 0x07, 0x08, phases, 300);
    (void)n;
    // bit 0 = start (mark, ON)
    ASSERT_EQ(phases[0].carrier, 1);
    // bit 1 = data bit 0 of byte 0 = LSB of 0x47 = 1 → carrier off
    ASSERT_EQ(phases[1].carrier, 0);
    // bit 4 = data bit 3 of byte 0 = bit 3 of 0x47 = 0 → carrier on
    ASSERT_EQ(phases[4].carrier, 1);
}

TEST(legacy_inter_message_gap_after_first_frame) {
    Phase phases[300];
    int n = play_legacy(4, 0x07, 0x08, phases, 300);
    (void)n;
    // After 22 bits (frame done), next phase is gap
    ASSERT_EQ(phases[22].carrier, 0);
    // Gap duration for channel 4: 51 ms minus message duration ~4.6 ms
    ASSERT_TRUE(phases[22].duration > 40000);
}

TEST(legacy_check_digit_makes_sum_modulo_16_zero) {
    // The Legacy spec: (channel + orange + yellow + check) & 0x0F == 0
    uint8_t channel = 5, orange = 0x07, yellow = 0x0F;
    uint8_t check = (0x10 - (channel + orange + yellow)) & 0x0F;
    ASSERT_EQ(((channel + orange + yellow + check) & 0x0F), 0);
}

int main(void) {
    RUN_ALL_TESTS();
}
