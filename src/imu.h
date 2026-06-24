/*
 * imu.h — LSM6DSO IMU (vibration detection) public interface.
 */
#ifndef IMU_H_
#define IMU_H_

#include <stdint.h>

/* Sampling configuration — exposed for sleep-time calculation in main */
#define IMU_ODR_HZ             416
#define IMU_SAMPLE_DURATION_MS 10000
#define IMU_NUM_SAMPLES        ((IMU_ODR_HZ * IMU_SAMPLE_DURATION_MS) / 1000)

/**
 * Initialise the LSM6DSO on the shared I²C bus (i2c30).
 * Returns 0 on success, negative errno on failure.
 */
int imu_init(void);

/**
 * Sample the accelerometer for ~10 s and return two vibration metrics.
 *
 * Both metrics are RMS-based and suited to long-running motor vibration.
 * Brief impulses (knocks, glitches) are averaged out rather than reported
 * as large peaks.
 *
 *   rms_mg_out  — Combined RSS RMS over the full capture window (mg).
 *   peak_mg_out — Highest 1 s sub-window combined RMS in the capture (mg).
 *
 * Returns 0 on success, -ENODEV if not initialised, other negative on error.
 */
int imu_sample_vibration(int32_t *rms_mg_out, int32_t *peak_mg_out);

#endif /* IMU_H_ */
