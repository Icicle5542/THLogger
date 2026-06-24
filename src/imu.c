/*
 * imu.c — LSM6DSO raw I2C driver for motor-vibration detection.
 *
 * Tuned for long-running motor vibration (30+ minutes): each 10 s capture
 * window should see continuous vibration when the motor is on.  Brief impulses
 * (knocks, electrical glitches) are rejected by using RMS-based metrics only.
 *
 * Sampling strategy:
 *   - Accelerometer ODR 416 Hz, +-2 g (61 ug / LSB — highest sensitivity)
 *   - On-chip high-pass filter (gravity removed in hardware)
 *   - One sample per data-ready flag (~10 seconds total)
 *   - Full-window combined RSS RMS  -> sustained average level (VibRMS)
 *   - Max 1 s sub-window combined RMS -> strongest sustained second (VibPeak)
 */

#include "imu.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(imu, CONFIG_LOG_DEFAULT_LEVEL);

#define LSM6DSO_I2C_ADDR       0x6A

#define LSM6DSO_REG_WHO_AM_I   0x0F
#define LSM6DSO_WHO_AM_I_VAL   0x6A

#define LSM6DSO_REG_CTRL1_XL   0x10
#define LSM6DSO_REG_CTRL2_G    0x11
#define LSM6DSO_REG_CTRL3_C    0x12
#define LSM6DSO_REG_CTRL8_XL   0x17
#define LSM6DSO_REG_STATUS     0x1E
#define LSM6DSO_REG_OUTX_L_XL  0x28

#define LSM6DSO_STATUS_XLDA    0x01

#define LSM6DSO_CTRL1_416HZ_2G 0x60
#define LSM6DSO_CTRL8_HP       0x04
#define LSM6DSO_CTRL3_BDU      0x40

#define IMU_SENS_UG_PER_LSB    61
#define XLDA_TIMEOUT_MS        5

/* 1 s sub-windows at 416 Hz — spikes shorter than this are averaged down */
#define IMU_BLOCK_SAMPLES      IMU_ODR_HZ

struct vib_accum {
	int64_t sum_x;
	int64_t sum_y;
	int64_t sum_z;
	int64_t sum_sq_x;
	int64_t sum_sq_y;
	int64_t sum_sq_z;
	int     count;
};

static const struct device *imu_i2c_dev;
static bool imu_ready;

static int lsm6dso_reg_write(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = {reg, val};

	return i2c_write(imu_i2c_dev, tx, sizeof(tx), LSM6DSO_I2C_ADDR);
}

static int lsm6dso_reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte(imu_i2c_dev, LSM6DSO_I2C_ADDR, reg, val);
}

static int lsm6dso_burst_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
	return i2c_burst_read(imu_i2c_dev, LSM6DSO_I2C_ADDR, reg, buf, len);
}

static int wait_xlda(void)
{
	int64_t deadline = k_uptime_get() + XLDA_TIMEOUT_MS;

	while (k_uptime_get() < deadline) {
		uint8_t status = 0;

		if (lsm6dso_reg_read(LSM6DSO_REG_STATUS, &status) == 0 &&
		    (status & LSM6DSO_STATUS_XLDA)) {
			return 0;
		}
	}

	return -ETIMEDOUT;
}

static int lsm6dso_init(void)
{
	uint8_t who_am_i = 0;
	int rc;

	rc = lsm6dso_reg_read(LSM6DSO_REG_WHO_AM_I, &who_am_i);
	if (rc != 0) {
		LOG_ERR("WHO_AM_I read failed: %d", rc);
		return rc;
	}
	if (who_am_i != LSM6DSO_WHO_AM_I_VAL) {
		LOG_ERR("Unexpected WHO_AM_I 0x%02x (want 0x%02x)",
			who_am_i, LSM6DSO_WHO_AM_I_VAL);
		return -ENODEV;
	}

	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL3_C, LSM6DSO_CTRL3_BDU);
	if (rc != 0) {
		LOG_ERR("CTRL3_C write failed: %d", rc);
		return rc;
	}

	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL8_XL, LSM6DSO_CTRL8_HP);
	if (rc != 0) {
		LOG_ERR("CTRL8_XL write failed: %d", rc);
		return rc;
	}

	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL2_G, 0x00);
	if (rc != 0) {
		LOG_ERR("CTRL2_G write failed: %d", rc);
		return rc;
	}

	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, 0x00);
	if (rc != 0) {
		LOG_ERR("CTRL1_XL write failed: %d", rc);
		return rc;
	}

	LOG_INF("LSM6DSO ready (WHO_AM_I=0x%02x, HP+416Hz)", who_am_i);
	return 0;
}

static uint32_t isqrt64(uint64_t n)
{
	if (n == 0) {
		return 0;
	}
	uint64_t x = n;
	uint64_t y = (x + 1) >> 1;

	while (y < x) {
		x = y;
		y = (x + n / x) >> 1;
	}
	return (uint32_t)x;
}

static void vib_accum_reset(struct vib_accum *a)
{
	a->sum_x = 0;
	a->sum_y = 0;
	a->sum_z = 0;
	a->sum_sq_x = 0;
	a->sum_sq_y = 0;
	a->sum_sq_z = 0;
	a->count = 0;
}

static void vib_accum_add(struct vib_accum *a, int16_t x, int16_t y, int16_t z)
{
	a->sum_x    += x;
	a->sum_y    += y;
	a->sum_z    += z;
	a->sum_sq_x += (int64_t)x * x;
	a->sum_sq_y += (int64_t)y * y;
	a->sum_sq_z += (int64_t)z * z;
	a->count++;
}

/*
 * Combined RSS RMS across all axes in milli-g.
 * Uses population variance (mean removed) — appropriate for sustained AC vibration.
 */
static int32_t combined_rms_mg(const struct vib_accum *a)
{
	int64_t n = a->count;
	int64_t nx = n * a->sum_sq_x - a->sum_x * a->sum_x;
	int64_t ny = n * a->sum_sq_y - a->sum_y * a->sum_y;
	int64_t nz = n * a->sum_sq_z - a->sum_z * a->sum_z;

	if (n <= 0) {
		return 0;
	}
	if (nx < 0) {
		nx = 0;
	}
	if (ny < 0) {
		ny = 0;
	}
	if (nz < 0) {
		nz = 0;
	}

	int64_t numer = nx + ny + nz;

	if (numer <= 0) {
		return 0;
	}

	return (int32_t)(((uint64_t)isqrt64((uint64_t)numer) * IMU_SENS_UG_PER_LSB +
			   (uint64_t)n * 500ULL) / ((uint64_t)n * 1000ULL));
}

static void block_finalize(const struct vib_accum *block, int32_t *max_block_rms_mg)
{
	int32_t block_rms = combined_rms_mg(block);

	if (block_rms > *max_block_rms_mg) {
		*max_block_rms_mg = block_rms;
	}
}

int imu_init(void)
{
	imu_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c30));
	if (!device_is_ready(imu_i2c_dev)) {
		LOG_WRN("I2C bus not ready");
		return -ENODEV;
	}

	int rc = lsm6dso_init();

	if (rc == 0) {
		imu_ready = true;
	}
	return rc;
}

int imu_sample_vibration(int32_t *rms_mg_out, int32_t *peak_mg_out)
{
	if (!imu_ready) {
		return -ENODEV;
	}

	struct vib_accum total;
	struct vib_accum block;
	int32_t max_block_rms_mg = 0;
	int rc;

	vib_accum_reset(&total);
	vib_accum_reset(&block);

	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, LSM6DSO_CTRL1_416HZ_2G);
	if (rc != 0) {
		LOG_ERR("Accel enable failed: %d", rc);
		return rc;
	}

	k_sleep(K_MSEC(50));

	for (int i = 0; i < IMU_NUM_SAMPLES; i++) {
		uint8_t raw[6];

		if (wait_xlda() != 0) {
			continue;
		}

		if (lsm6dso_burst_read(LSM6DSO_REG_OUTX_L_XL,
				       raw, sizeof(raw)) != 0) {
			continue;
		}

		int16_t x = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
		int16_t y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
		int16_t z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);

		vib_accum_add(&total, x, y, z);
		vib_accum_add(&block, x, y, z);

		if (block.count >= IMU_BLOCK_SAMPLES) {
			block_finalize(&block, &max_block_rms_mg);
			vib_accum_reset(&block);
		}
	}

	lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, 0x00);

	if (total.count == 0) {
		LOG_WRN("No IMU samples collected");
		return -EIO;
	}

	if (block.count > 0) {
		block_finalize(&block, &max_block_rms_mg);
	}

	*rms_mg_out = combined_rms_mg(&total);
	*peak_mg_out = max_block_rms_mg;

	return 0;
}
