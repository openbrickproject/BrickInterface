#ifndef ACT_LED_H
#define ACT_LED_H

// Activity LED control — pulses on every USB-CDC byte (RX or TX).
// Auto-turns off after ACT_LED_DURATION_MS of inactivity.

// Call when any byte traverses the serial port.
void actLedPulse(void);

// Call periodically from loop() to turn the LED off after the timeout.
void actLedTick(void);

#endif
