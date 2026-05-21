// Layer 1+2 — PF IR encoder + phase generator tests.
// Includes ir_engine.cpp directly to access static helpers and phase generators.
#include "test_harness.h"
#include "../ir_engine.h"
#include "../ir_engine.c"

// ============================================================================
// Layer 1: pure-math primitives
// ============================================================================

TEST(pf_LRC_zero_inputs) { ASSERT_EQ(pfLRC(0, 0, 0), 0x0F); }
TEST(pf_LRC_canonical)   { ASSERT_EQ(pfLRC(0x05, 0x01, 0x07), 0x0F ^ 0x05 ^ 0x01 ^ 0x07); }

TEST(pf_oddParity_zero)   { ASSERT_EQ(oddParity(0x00), 1); }
TEST(pf_oddParity_one)    { ASSERT_EQ(oddParity(0x01), 0); }
TEST(pf_oddParity_FF)     { ASSERT_EQ(oddParity(0xFF), 1); }
TEST(pf_oddParity_55)     { ASSERT_EQ(oddParity(0x55), 1); }

// ============================================================================
// Layer 1: nibble builder
// ============================================================================

TEST(pf_combo_direct_basic) {
    // Channel 0, mode COMBO_DIRECT, data=0x01 (output A forward), flags = override toggle=0
    pfToggle[0] = 0;
    uint8_t nibs[4];
    uint8_t ok = pfBuildNibbles(nibs, 0, PF_MODE_COMBO_DIRECT, 0x01, 0x01 /*override toggle=0*/);
    ASSERT_EQ(ok, 1);
    // n0 = (toggle<<3) | (escape<<2) | channel = 0
    ASSERT_EQ(nibs[0], 0x00);
    // n1 = 0x01 for COMBO_DIRECT
    ASSERT_EQ(nibs[1], 0x01);
    // n2 = data low nibble
    ASSERT_EQ(nibs[2], 0x01);
    // LRC
    ASSERT_EQ(nibs[3], 0x0F ^ 0x00 ^ 0x01 ^ 0x01);
}

TEST(pf_single_pwm_output_b) {
    pfToggle[1] = 0;
    uint8_t nibs[4];
    // Channel 1, SINGLE_PWM, data = 0x17 (bit 4 = output B select, low nibble = 0x07 = forward 7/7)
    pfBuildNibbles(nibs, 1, PF_MODE_SINGLE_PWM, 0x17, 0x01);
    ASSERT_EQ(nibs[0], 0x01);  // channel 1, toggle=0
    ASSERT_EQ(nibs[1], 0x05);  // 0x04 | output B select bit (data>>4 & 1 = 1)
    ASSERT_EQ(nibs[2], 0x07);
}

TEST(pf_single_cst_output_a) {
    pfToggle[2] = 0;
    uint8_t nibs[4];
    pfBuildNibbles(nibs, 2, PF_MODE_SINGLE_CST, 0x03, 0x01);
    ASSERT_EQ(nibs[0], 0x02);  // channel 2
    ASSERT_EQ(nibs[1], 0x06);  // 0x06 | (0>>4)&1 = 0x06
    ASSERT_EQ(nibs[2], 0x03);
}

TEST(pf_toggle_auto_alternates_per_channel) {
    pfToggle[0] = 0;
    pfToggle[1] = 0;
    uint8_t nibs[4];
    pfBuildNibbles(nibs, 0, PF_MODE_COMBO_DIRECT, 0x00, 0x00);  // auto-toggle
    ASSERT_EQ(nibs[0] & 0x08, 0x00);  // first call: toggle=0
    pfBuildNibbles(nibs, 0, PF_MODE_COMBO_DIRECT, 0x00, 0x00);
    ASSERT_EQ(nibs[0] & 0x08, 0x08);  // second call: toggle=1
    // Channel 1 toggle is independent
    pfBuildNibbles(nibs, 1, PF_MODE_COMBO_DIRECT, 0x00, 0x00);
    ASSERT_EQ(nibs[0] & 0x08, 0x00);  // ch1 starts fresh
}

TEST(pf_invalid_mode_returns_zero) {
    uint8_t nibs[4];
    ASSERT_EQ(pfBuildNibbles(nibs, 0, 0xFF, 0, 0), 0);
}

// ============================================================================
// Layer 2: phase generator integration test
// Plays out one full PF transmission and verifies the (carrier, duration) sequence.
// ============================================================================

struct Phase { uint8_t carrier; uint16_t duration; };

static void play_pf(uint8_t channel, uint8_t mode, uint8_t data, uint8_t flags,
                    Phase *out, int *count, int max) {
    // Set up state directly
    state.active = IR_ACTIVE_PF;
    state.pf.repeat = 0;
    state.pf.pos = 0;
    state.pf.phase = 0;
    state.pf.channel = channel;
    pfBuildNibbles((uint8_t *)state.pf.nibbles, channel, mode, data, flags);

    *count = 0;
    while (*count < max) {
        uint8_t c; uint16_t d;
        if (!pfNextPhase(&c, &d)) break;
        out[*count].carrier = c;
        out[*count].duration = d;
        (*count)++;
    }
}

TEST(pf_phase_sequence_5_repeats) {
    Phase phases[200];
    int n = 0;
    pfToggle[0] = 0;
    play_pf(0, PF_MODE_COMBO_DIRECT, 0x01, 0x01, phases, &n, 200);

    // Each repeat: 1 start mark + 1 start space + 16 (mark+space) bits + 1 stop mark = 35 phases
    // Plus 1 inter-message gap between each pair of repeats: 4 gaps for 5 repeats
    // Total: 5*35 + 4 = 179
    ASSERT_EQ(n, 179);

    // Verify first phase is mark
    ASSERT_EQ(phases[0].carrier, 1);
    ASSERT_EQ(phases[0].duration, PF_MARK_US);

    // Verify second phase is start space
    ASSERT_EQ(phases[1].carrier, 0);
    ASSERT_EQ(phases[1].duration, PF_START_SPACE_US);

    // Marks should always be PF_MARK_US (158)
    for (int i = 0; i < n; i++) {
        if (phases[i].carrier == 1) {
            ASSERT_EQ(phases[i].duration, PF_MARK_US);
        }
    }
}

TEST(pf_phase_zero_and_one_spaces) {
    // For COMBO_DIRECT ch=0 data=0x01 (override toggle=0):
    //   nibbles = [0x00, 0x01, 0x01, LRC]
    //   Bit pattern = 0000 0001 0001 LRC...
    Phase phases[200];
    int n = 0;
    pfToggle[0] = 0;
    play_pf(0, PF_MODE_COMBO_DIRECT, 0x01, 0x01, phases, &n, 200);

    // Phases: [0]=mark, [1]=startspace, then 16 (mark, space) pairs for data bits,
    // then [34]=stop mark. The first 4 bits are 0000 -> all zero spaces.
    for (int bit = 0; bit < 4; bit++) {
        int spaceIdx = 2 + bit*2 + 1;  // even-bit pos = mark, odd = space
        ASSERT_EQ(phases[spaceIdx].carrier, 0);
        ASSERT_EQ(phases[spaceIdx].duration, PF_ZERO_SPACE_US);
    }
    // Bit 7 (last bit of nibble 1 = 0x01) should be a one space
    int spaceIdx = 2 + 7*2 + 1;
    ASSERT_EQ(phases[spaceIdx].duration, PF_ONE_SPACE_US);
}

TEST(pf_inter_message_gaps_per_channel) {
    Phase phases[200];
    int n = 0;
    pfToggle[2] = 0;
    play_pf(2, PF_MODE_COMBO_DIRECT, 0x00, 0x01, phases, &n, 200);

    // Each repeat produces 35 phases. Gap is the 36th phase (index 35).
    // Channel 2 → PF_GAP_CH2_US = 36000
    ASSERT_EQ(phases[35].carrier, 0);
    ASSERT_EQ(phases[35].duration, PF_GAP_CH2_US);
}

int main(void) {
    RUN_ALL_TESTS();
}
