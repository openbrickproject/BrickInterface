// Implementations of test framework + Arduino/CH552 stubs.
#include "test_harness.h"
#include "Arduino.h"
#include "../act_led.h"

// Stub activity-LED hooks used by sendReply / packet handling. test_dispatch.cpp
// includes BrickInterface.ino which provides real implementations and shadows
// these (multiple definitions allowed because the harness ones are weak fallbacks
// only linked into non-dispatch tests).
__attribute__((weak)) void actLedPulse(void) {}
__attribute__((weak)) void actLedTick(void) {}

// --- Test framework state ---
int test_failed = 0;
int test_total  = 0;
int test_passed = 0;
const char *current_test_name = "";
struct TestEntry g_tests[256];
int g_test_count = 0;

void test_register(const char *name, TestFn fn) {
    if (g_test_count < 256) {
        g_tests[g_test_count].name = name;
        g_tests[g_test_count].fn   = fn;
        g_test_count++;
    }
}

// --- Arduino API stubs ---
uint8_t test_pin_state[256] = {0};
uint8_t test_pin_input[256] = {0};
static uint8_t pin_modes[256] = {0};

void test_reset_pin_state(void) {
    memset(test_pin_state, 0, sizeof(test_pin_state));
    memset(test_pin_input, 0, sizeof(test_pin_input));
    memset(pin_modes, 0, sizeof(pin_modes));
}

void pinMode(uint8_t pin, uint8_t mode) { pin_modes[pin] = mode; }

void digitalWrite(uint8_t pin, uint8_t value) { test_pin_state[pin] = value ? 1 : 0; }

uint8_t digitalRead(uint8_t pin) { return test_pin_input[pin]; }

void delay(uint32_t /*ms*/) {}

// Test-controllable fake clock. Set `test_now_ms` to whatever you want
// `millis()` to return — used by pulse-width tests to advance time
// deterministically across edge events.
unsigned long test_now_ms = 0;
unsigned long millis(void) { return test_now_ms; }

// --- Stub USB CDC ---
// Tests inspect Serial.tx_buf / Serial.tx_len after sending a frame.
TestSerial Serial;

uint8_t USBSerial_print_n(const uint8_t *buf, int len) {
    for (int i = 0; i < len && Serial.tx_len < sizeof(Serial.tx_buf); i++)
        Serial.tx_buf[Serial.tx_len++] = buf[i];
    return (uint8_t)len;
}

uint8_t USBSerial_write(uint8_t c) {
    if (Serial.tx_len < sizeof(Serial.tx_buf)) Serial.tx_buf[Serial.tx_len++] = c;
    return 1;
}

// --- CH552 SFR stubs (writable globals) ---
volatile uint8_t PIN_FUNC = 0;
volatile uint8_t PWM_CK_SE = 0;
volatile uint8_t PWM_CYCLE = 0;
volatile uint8_t PWM_DATA2 = 0;
volatile uint8_t PWM_CTRL = 0;

volatile uint8_t TMOD = 0, T2MOD = 0;
volatile uint8_t TH0 = 0, TL0 = 0, TF0 = 0, TR0 = 0, ET0 = 0;
volatile uint8_t TH1 = 0, TL1 = 0, TR1 = 0, ET1 = 0;
volatile uint8_t T2CON = 0, RCAP2H = 0, RCAP2L = 0, TH2 = 0, TL2 = 0, TF2 = 0, ET2 = 0;
volatile uint8_t EA = 0;

volatile uint8_t USB_INT_EN = 0, USB_CTRL = 0, UDEV_CTRL = 0;

volatile uint8_t P1_4 = 0, P1_5 = 0, P1_6 = 0, P1_7 = 0;
volatile uint8_t P3_0 = 0, P3_1 = 0, P3_2 = 0, P3_3 = 0;
volatile uint8_t P3_4 = 0, P3_6 = 0, P3_7 = 0;
