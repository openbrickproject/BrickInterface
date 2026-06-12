#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

// ============================================================
// CH552T Pin Assignments
// ============================================================
// ch55xduino uses logical port-major pin numbering: pin = port*10 + bit.
// P1.0=10, P1.1=11, P1.4=14, P1.5=15, P1.6=16, P1.7=17
// P3.0=30, P3.1=31, P3.2=32, P3.3=33, P3.4=34
// These are NOT the physical TSSOP-20 chip pin numbers.

// Interface A GPIO. Chip pin numbers per official WCH CH552 datasheet.
#define IFACE_OUT0_PIN  14  // P1.4 -> chip pin 2  -> Ribbon pin 6
#define IFACE_OUT1_PIN  15  // P1.5 -> chip pin 3  -> Ribbon pin 8
#define IFACE_OUT2_PIN  16  // P1.6 -> chip pin 4  -> Ribbon pin 10
#define IFACE_OUT3_PIN  17  // P1.7 -> chip pin 5  -> Ribbon pin 12
#define IFACE_OUT4_PIN  31  // P3.1 -> chip pin 9  -> Ribbon pin 14
#define IFACE_OUT5_PIN  30  // P3.0 -> chip pin 10 -> Ribbon pin 16
#define IFACE_IN6_PIN   11  // P1.1 -> chip pin 8  -> Ribbon pin 18
#define IFACE_IN7_PIN   10  // P1.0 -> chip pin 7  -> Ribbon pin 20

// IR LED — P3.4 has PWM2 default routing (NOT PWM1)
#define IR_LED_PIN        34  // P3.4 -> chip pin 12 (PWM2 default)

// Activity LED
#define ACT_LED_PIN       32  // P3.2 -> chip pin 1

// USB is on P3.6 (D+, chip pin 14) and P3.7 (D-, chip pin 15) — dedicated

// ============================================================
// Timer 2 tick rates (timer clock = Fsys/12 = 2 MHz, 0.5 us/count)
// ============================================================
// Base tick: 26 counts = 13 us (76.92 kHz ISR). The carrier toggles every
// tick, giving the 38.46 kHz used by PF and RCX. Chosen so the ISR body
// (~8 us worst case) fits inside one period: at the old 6.5 us tick the
// ISR overran itself, overflow flags were lost, and every envelope ran
// ~1.2x slow with a ~31 kHz carrier — close enough for PF receivers
// (±25% bit tolerance, wideband), fatal for the RCX's 2400 baud UART.
// Fast tick: 13 counts = 6.5 us (153.85 kHz), armed only while the Legacy
// engine transmits — toggling every tick then gives its 76.92 kHz carrier.
// (The ISR still overruns at this rate, so Legacy timing stretches ~1.2x;
// acceptable until Legacy is hardware-validated.)
#define T2_RELOAD_BASE_H  0xFF   // 65536 - 26
#define T2_RELOAD_BASE_L  0xE6
#define T2_RELOAD_FAST_H  0xFF   // 65536 - 13
#define T2_RELOAD_FAST_L  0xF3

#endif
