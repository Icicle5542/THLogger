/*
 * imu.h — LSM6DSO IMU (vibration detection) public interface.
 */
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

/* Sampling configuration — exposed for sleep-time calculation in main */
#define IMU_NUM_SAMPLES      1000
#define IMU_SAMPLE_PERIOD_MS 10

/**
 * Initialise the LSM6DSO on the shared I²C bus (i2c30).
 * Returns 0 on success, negative errno on failure.
 */
int imu_init(void);

/**
 * Sample the accelerometer for ~10 s and return two vibration metrics.
 *
 *   rms_mg_out  — RMS of AC (gravity-removed) acceleration, all axes (mg).
 *   peak_mg_out — One-sided peak amplitude (mg).
 *
 * Returns 0 on success, -ENODEV if not initialised, other negative on error.
 */
int imu_sample_vibration(int32_t *rms_mg_out, int32_t *peak_mg_out);

#endif /* IMU_H_ */
