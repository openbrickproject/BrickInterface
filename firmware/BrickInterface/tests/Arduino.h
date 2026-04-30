// Test-only Arduino.h stub.
// Replaces ch55xduino's Arduino.h when compiling firmware on the host for unit tests.
// Provides minimal API surface and CH552 SFR stubs as ordinary globals.
#ifndef ARDUINO_H_TESTSTUB
#define ARDUINO_H_TESTSTUB

#include <stdint.h>
#include <stddef.h>
#include <string.h>

// SDCC-specific keywords compile to nothing on the host
#define __xdata
#define __data
#define __code
#define __sfr
#define __interrupt(x)
#define __using(x)
// Note: don't redefine __attribute__ here — g++ uses it for real attributes
// (e.g., test_harness.h's constructor attribute) and the firmware's
// __attribute__((interrupt("1"))) just becomes a benign warning on host.

// Pin modes / logic levels
#define INPUT        0
#define OUTPUT       1
#define INPUT_PULLUP 2
#define HIGH         1
#define LOW          0

// Stub Arduino API (defined in test_harness.cpp)
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t value);
uint8_t digitalRead(uint8_t pin);
void delay(uint32_t ms);
unsigned long millis(void);

// Test harness exposes the recorded GPIO state for assertions
extern uint8_t test_pin_state[256];
extern uint8_t test_pin_input[256];   // host-set readback for digitalRead
void test_reset_pin_state(void);

// Stub Serial — captures bytes written by sendReply for inspection
struct TestSerial {
    uint8_t tx_buf[256];
    uint16_t tx_len;
    void begin(uint32_t) {}
    int available(void) { return 0; }
    int read(void) { return -1; }
    void write(const uint8_t *buf, uint8_t len);
    void write(uint8_t b);
    void flush(void) {}
};
extern TestSerial Serial;

// ===========================================================================
// CH552 SFR + bit stubs — match ch5xx.h symbol names so firmware compiles
// ===========================================================================

// PWM
extern volatile uint8_t PIN_FUNC;
extern volatile uint8_t PWM_CK_SE;
extern volatile uint8_t PWM_CYCLE;
extern volatile uint8_t PWM_DATA2;
extern volatile uint8_t PWM_CTRL;
#define bPWM2_PIN_X  0x08
#define bPWM2_OUT_EN 0x08

// Timer 0/1
extern volatile uint8_t TMOD, T2MOD;
extern volatile uint8_t TH0, TL0, TF0, TR0, ET0;
extern volatile uint8_t TH1, TL1, TR1, ET1;
#define bT1_CLK 0x10
extern volatile uint8_t EA;

// USB
extern volatile uint8_t USB_INT_EN, USB_CTRL, UDEV_CTRL;

// Bit-addressable port pins (used by interface_a.cpp PWM ISR)
extern volatile uint8_t P1_4, P1_5, P1_6, P1_7;
extern volatile uint8_t P3_0, P3_1, P3_2, P3_3, P3_4, P3_6, P3_7;

#endif
