/*
 * pcf8563.h — PCF8563 RTC public interface.
 */
#ifndef PCF8563_H_
#define PCF8563_H_

#include <stdint.h>

/**
 * Initialise the PCF8563 (DTS node: pcf8563@51 on xiao_i2c).
 * Returns 0 on success, -ENODEV if the device is not ready.
 */
int pcf8563_init(void);

/**
 * Read the current time from the PCF8563.
 * Returns Unix seconds (UTC), or 0 if the oscillator has not been set
 * or the device is not initialised.
 */
int64_t pcf8563_read_unix(void);

/**
 * Write a Unix timestamp (UTC) to the PCF8563.
 * Returns 0 on success, -ENODEV if not initialised, other negative on error.
 * Safe to call even if pcf8563_init() failed (returns -ENODEV silently).
 */
int pcf8563_write_unix(int64_t ts);

#endif /* PCF8563_H_ */
