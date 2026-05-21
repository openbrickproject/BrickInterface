// Layer 1 — packet parser/builder unit tests.
#include "test_harness.h"
#include "../packet.h"
#include "../packet.c"  // pull in implementation directly

static void feed(PacketParser *p, const uint8_t *bytes, size_t n) {
    for (size_t i = 0; i < n; i++) parserConsume(p, bytes[i]);
}

TEST(parses_minimal_ping_frame) {
    // AA 02 5A 01 59  — len=2, seq=5A, cmd=PING, chk=02^5A^01=59
    PacketParser p;
    parserInit(&p);
    uint8_t f[] = {0xAA, 0x02, 0x5A, 0x01, 0x59};
    feed(&p, f, sizeof(f));
    ASSERT_EQ(p.ready, 1);
    ASSERT_EQ(p.pkt.seq, 0x5A);
    ASSERT_EQ(p.pkt.cmd, 0x01);
    ASSERT_EQ(p.pkt.payload_len, 0);
}

TEST(parses_frame_with_payload) {
    // AA 03 11 10 05 06  — IFACE_SET_OUTPUTS bits=0x05
    PacketParser p;
    parserInit(&p);
    uint8_t f[] = {0xAA, 0x03, 0x11, 0x10, 0x05, 0x07};
    feed(&p, f, sizeof(f));
    ASSERT_EQ(p.ready, 1);
    ASSERT_EQ(p.pkt.seq, 0x11);
    ASSERT_EQ(p.pkt.cmd, 0x10);
    ASSERT_EQ(p.pkt.payload_len, 1);
    ASSERT_EQ(p.pkt.payload[0], 0x05);
}

TEST(rejects_bad_checksum) {
    PacketParser p;
    parserInit(&p);
    uint8_t f[] = {0xAA, 0x02, 0x5A, 0x01, 0x00 /* wrong chk */};
    feed(&p, f, sizeof(f));
    ASSERT_EQ(p.ready, 0);
}

TEST(rejects_zero_length) {
    PacketParser p;
    parserInit(&p);
    uint8_t f[] = {0xAA, 0x00};  // len=0 invalid
    feed(&p, f, sizeof(f));
    ASSERT_EQ(p.ready, 0);
    ASSERT_EQ(p.state, PARSE_WAIT_SOF);
}

TEST(rejects_oversized_length) {
    PacketParser p;
    parserInit(&p);
    uint8_t f[] = {0xAA, 0xFF};  // len > MAX
    feed(&p, f, sizeof(f));
    ASSERT_EQ(p.ready, 0);
    ASSERT_EQ(p.state, PARSE_WAIT_SOF);
}

TEST(resyncs_on_sof_after_garbage) {
    PacketParser p;
    parserInit(&p);
    uint8_t garbage[] = {0xFF, 0x00, 0xBE, 0xEF};
    feed(&p, garbage, sizeof(garbage));
    ASSERT_EQ(p.ready, 0);
    uint8_t valid[] = {0xAA, 0x02, 0x5A, 0x01, 0x59};
    feed(&p, valid, sizeof(valid));
    ASSERT_EQ(p.ready, 1);
}

TEST(parses_max_payload_frame) {
    // 32-byte payload — at the documented max
    PacketParser p;
    parserInit(&p);
    uint8_t buf[1 + 1 + 1 + 1 + 32 + 1];
    buf[0] = 0xAA;
    buf[1] = 2 + 32;
    buf[2] = 0x77;
    buf[3] = 0x51;
    for (int i = 0; i < 32; i++) buf[4 + i] = (uint8_t)i;
    uint8_t chk = buf[1] ^ buf[2] ^ buf[3];
    for (int i = 0; i < 32; i++) chk ^= buf[4 + i];
    buf[36] = chk;
    feed(&p, buf, sizeof(buf));
    ASSERT_EQ(p.ready, 1);
    ASSERT_EQ(p.pkt.payload_len, 32);
    for (int i = 0; i < 32; i++) ASSERT_EQ(p.pkt.payload[i], i);
}

TEST(builds_ping_reply) {
    uint8_t buf[64];
    uint8_t n = buildPacket(buf, 0x5A, 0x81 /*PONG*/, NULL, 0);
    ASSERT_EQ(n, 5);
    uint8_t expected[] = {0xAA, 0x02, 0x5A, 0x81, 0xD9};
    ASSERT_BYTES_EQ(buf, expected, 5);
}

TEST(builds_frame_with_payload) {
    uint8_t buf[64];
    uint8_t payload[] = {0x05};
    uint8_t n = buildPacket(buf, 0x11, 0x90, payload, 1);
    ASSERT_EQ(n, 6);
    // SOF, len=3, seq=0x11, cmd=0x90, payload=0x05, chk=3^0x11^0x90^0x05=0x87
    uint8_t expected[] = {0xAA, 0x03, 0x11, 0x90, 0x05, 0x87};
    ASSERT_BYTES_EQ(buf, expected, 6);
}

TEST(roundtrip_build_then_parse) {
    uint8_t buf[64];
    uint8_t payload[] = {0x01, 0x02, 0x03, 0x04};
    uint8_t n = buildPacket(buf, 0x33, 0x82, payload, 4);

    PacketParser p;
    parserInit(&p);
    feed(&p, buf, n);
    ASSERT_EQ(p.ready, 1);
    ASSERT_EQ(p.pkt.seq, 0x33);
    ASSERT_EQ(p.pkt.cmd, 0x82);
    ASSERT_EQ(p.pkt.payload_len, 4);
    ASSERT_BYTES_EQ(p.pkt.payload, payload, 4);
}

int main(void) {
    RUN_ALL_TESTS();
}
