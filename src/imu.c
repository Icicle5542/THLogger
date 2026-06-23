/*
 * imu.c — LSM6DSO raw I2C driver for motor-vibration detection.
 *
 * The LSM6DSO is accessed directly via I2C (address 0x6A), sharing the
 * bus with the DHT20.  No Zephyr sensor driver is used.
 *
 * Sampling strategy:
 *   - Accelerometer ODR 104 Hz, +-2 g (61 ug / LSB sensitivity)
 *   - 1000 samples at 10 ms intervals (~10 seconds total)
 *   - Per-axis variance computed in one pass -> combined RMS in milli-g
 *   - Per-axis min/max tracked -> one-sided peak amplitude in milli-g
 *
 * Overflow analysis at n=1000, |x|<=32767:
 *   n*sum_sq <= 1000^2 * 32767^2 ~= 1.07e15  (fits in int64_t)
 *   sum^2    <= (1000*32767)^2   ~= 1.07e15  (fits in int64_t)
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

#define LSM6DSO_REG_CTRL1_XL   0x10   /* Accelerometer control 1 */
#define LSM6DSO_REG_CTRL2_G    0x11   /* Gyroscope control 2     */
#define LSM6DSO_REG_OUTX_L_XL  0x28   /* Accel X-axis low byte   */

/* CTRL1_XL value: ODR = 104 Hz (0100), FS = +-2 g (00) -> 0x40 */
#define LSM6DSO_CTRL1_104HZ_2G 0x40

/* Sensitivity at +-2 g: 61 ug / LSB */
#define IMU_SENS_UG_PER_LSB    61

static const struct device *imu_i2c_dev;
static bool imu_ready;

/* -- Low-level I2C helpers ----------------------------------------- */

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

/* -- Initialisation ------------------------------------------------ */

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

	/* Gyroscope: power down — not needed for vibration detection */
	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL2_G, 0x00);
	if (rc != 0) {
		LOG_ERR("CTRL2_G write failed: %d", rc);
		return rc;
	}

	/* Accelerometer: power down — activated per sampling window */
	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, 0x00);
	if (rc != 0) {
		LOG_ERR("CTRL1_XL write failed: %d", rc);
		return rc;
	}

	LOG_INF("LSM6DSO ready (WHO_AM_I=0x%02x)", who_am_i);
	return 0;
}

/* -- Integer square root (floor) ----------------------------------- */

/*
 * Newton-Raphson integer sqrt; converges in O(log2 n) iterations.
 * Returns floor(sqrt(n)).
 */
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

/* -- Public API ---------------------------------------------------- */

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

	int64_t sum_x = 0, sum_y = 0, sum_z = 0;
	int64_t sum_sq_x = 0, sum_sq_y = 0, sum_sq_z = 0;
	int32_t min_x = INT16_MAX, max_x = INT16_MIN;
	int32_t min_y = INT16_MAX, max_y = INT16_MIN;
	int32_t min_z = INT16_MAX, max_z = INT16_MIN;
	int rc;

	/* Enable accelerometer at 104 Hz, +-2 g */
	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, LSM6DSO_CTRL1_104HZ_2G);
	if (rc != 0) {
		LOG_ERR("Accel enable failed: %d", rc);
		return rc;
	}

	/* Wait for the first two ODR periods to settle (~20 ms) */
	k_sleep(K_MSEC(20));

	for (int i = 0; i < IMU_NUM_SAMPLES; i++) {
		uint8_t raw[6];

		if (lsm6dso_burst_read(LSM6DSO_REG_OUTX_L_XL,
				       raw, sizeof(raw)) == 0) {
			int16_t x = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
			int16_t y = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
			int16_t z = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);

			sum_x    += x;
			sum_y    += y;
			sum_z    += z;
			sum_sq_x += (int64_t)x * x;
			sum_sq_y += (int64_t)y * y;
			sum_sq_z += (int64_t)z * z;

			if (x < min_x) min_x = (int32_t)x;
			if (x > max_x) max_x = (int32_t)x;
			if (y < min_y) min_y = (int32_t)y;
			if (y > max_y) max_y = (int32_t)y;
			if (z < min_z) min_z = (int32_t)z;
			if (z > max_z) max_z = (int32_t)z;
		}

		k_sleep(K_MSEC(IMU_SAMPLE_PERIOD_MS));
	}

	/* Power down accelerometer between measurement cycles */
	lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, 0x00);

	const int64_t n = IMU_NUM_SAMPLES;

	int64_t var_x = (n * sum_sq_x - sum_x * sum_x) / (n * n);
	int64_t var_y = (n * sum_sq_y - sum_y * sum_y) / (n * n);
	int64_t var_z = (n * sum_sq_z - sum_z * sum_z) / (n * n);

	/* Guard against rounding artefacts */
	if (var_x < 0) var_x = 0;
	if (var_y < 0) var_y = 0;
	if (var_z < 0) var_z = 0;

	/* Combined RMS in LSB, then convert to milli-g (61 ug = 0.061 mg) */
	uint32_t rms_lsb = isqrt64((uint64_t)(var_x + var_y + var_z));

	*rms_mg_out = (int32_t)(((uint64_t)rms_lsb * IMU_SENS_UG_PER_LSB + 500)
				/ 1000);

	/* Peak: half-range per axis, Euclidean combination */
	int32_t px = (max_x - min_x) / 2;
	int32_t py = (max_y - min_y) / 2;
	int32_t pz = (max_z - min_z) / 2;
	int64_t peak_sq = (int64_t)px * px + (int64_t)py * py +
			  (int64_t)pz * pz;
	uint32_t peak_lsb = isqrt64((uint64_t)peak_sq);

	*peak_mg_out = (int32_t)(((uint64_t)peak_lsb * IMU_SENS_UG_PER_LSB + 500)
				 / 1000);

	return 0;
}
