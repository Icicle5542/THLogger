/*
 * THLogger — Temperature & Humidity Logger with persistent NVS storage.
 *
 * Serial shell commands (115200 baud on USB/UART):
 *   thlog show          — print all stored log entries
 *   thlog clear         — erase the log
 *   thtime set <YYYY-MM-DD> <HH:MM:SS>  — set the UTC time reference
 *   thtime get          — print the current derived time
 *
 * Storage layout (storage_partition, 36 KB at 0x15c000):
 *   NVS key  1 : write_idx   (uint32_t) — monotonic entry counter
 *   NVS key  2 : base_time_s (int64_t)  — unix seconds set by user
 *   NVS key 10…(10+MAX-1) : th_log_entry structs in a circular ring
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/devicetree.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/timeutil.h>
#include <time.h>
#include <stdio.h>
#include <string.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* Built-in LED (led0 alias, active-low on XIAO nRF54L15) */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* ------------------------------------------------------------------ */
/* NVS layout                                                          */
/* ------------------------------------------------------------------ */
#define NVS_KEY_WRITE_IDX   1u
#define NVS_KEY_BASE_TIME   2u
#define NVS_KEY_ENTRY_BASE  10u   /* keys 10 … 10+MAX_LOG_ENTRIES-1  */

/*
 * NVS partition: 512 KB (128 × 4 KB RRAM pages), defined in the DTS overlay.
 * Usable space: 127 sectors × 4096 B = 520,192 B.
 * Per entry: 32 B data + 8 B NVS ATE = 40 B  →  max ≈ 13,000 entries.
 * 10,000 entries × 40 B = 400,000 B — well within budget.
 * At one entry per 5 minutes: ~34.7 days of continuous logging.
 */
#define MAX_LOG_ENTRIES     10000u

struct th_log_entry {
	int64_t timestamp_s;
	int32_t temp_val1;
	int32_t temp_val2;
	int32_t hum_val1;
	int32_t hum_val2;
	int32_t vibration_rms_mg;  /* RMS of AC acceleration, milli-g */
	int32_t vibration_peak_mg; /* One-sided peak amplitude, milli-g */
};

static struct nvs_fs nvs;

/* ------------------------------------------------------------------ */
/* Software clock                                                       */
/*   base_time_s   : unix seconds at the moment set by the user       */
/*   base_uptime_ms: k_uptime_get() value when base_time_s was set    */
/* Both survive reboots via NVS key 2.  After a cold boot the stored  */
/* base is reused so new entries carry sensible timestamps straight    */
/* away; the user can refine at any time with "thtime set".           */
/* ------------------------------------------------------------------ */
static int64_t base_time_s;
static int64_t base_uptime_ms;

static int64_t get_timestamp_s(void)
{
	return base_time_s + (k_uptime_get() - base_uptime_ms) / 1000;
}

/* ------------------------------------------------------------------ */
/* Monotonic write counter (persisted in NVS)                          */
/* ------------------------------------------------------------------ */
static uint32_t write_idx;

/* ------------------------------------------------------------------ */
/* LSM6DSO IMU — raw I2C driver for motor-vibration detection          */
/*                                                                      */
/* The sensor is accessed directly via I2C (address 0x6A, same bus as  */
/* the DHT20).  No Zephyr sensor driver is used so no DTS binding is   */
/* required; only the I2C bus device handle is needed.                 */
/*                                                                      */
/* Sampling strategy                                                    */
/*   • Accelerometer ODR 104 Hz, ±2 g (61 µg / LSB sensitivity)       */
/*   • Poll at 10 ms intervals for 10 seconds → 1000 samples           */
/*   • Compute per-axis variance in one pass, convert to milli-g RMS   */
/*   • Track per-axis min/max for one-sided peak amplitude             */
/* ------------------------------------------------------------------ */

#define LSM6DSO_I2C_ADDR       0x6A

#define LSM6DSO_REG_WHO_AM_I   0x0F
#define LSM6DSO_WHO_AM_I_VAL   0x6A

#define LSM6DSO_REG_CTRL1_XL   0x10   /* Accelerometer control 1 */
#define LSM6DSO_REG_CTRL2_G    0x11   /* Gyroscope control 2     */
#define LSM6DSO_REG_OUTX_L_XL  0x28   /* Accel X-axis low byte   */

/* CTRL1_XL value: ODR = 104 Hz (0100), FS = ±2 g (00) → 0x40 */
#define LSM6DSO_CTRL1_104HZ_2G 0x40

/* Sensitivity at ±2 g: 61 µg / LSB */
#define IMU_SENS_UG_PER_LSB    61

/* 1000 samples × 10 ms = 10 seconds of vibration data */
#define IMU_NUM_SAMPLES        1000
#define IMU_SAMPLE_PERIOD_MS   10

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

/* -- IMU initialisation -------------------------------------------- */

static int lsm6dso_init(void)
{
	uint8_t who_am_i = 0;
	int rc;

	rc = lsm6dso_reg_read(LSM6DSO_REG_WHO_AM_I, &who_am_i);
	if (rc != 0) {
		LOG_ERR("IMU: WHO_AM_I read failed: %d", rc);
		return rc;
	}
	if (who_am_i != LSM6DSO_WHO_AM_I_VAL) {
		LOG_ERR("IMU: unexpected WHO_AM_I 0x%02x (want 0x%02x)",
			who_am_i, LSM6DSO_WHO_AM_I_VAL);
		return -ENODEV;
	}

	/* Gyroscope: power down — not needed for vibration detection */
	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL2_G, 0x00);
	if (rc != 0) {
		LOG_ERR("IMU: CTRL2_G write failed: %d", rc);
		return rc;
	}

	/* Accelerometer: power down — activated per sampling window */
	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, 0x00);
	if (rc != 0) {
		LOG_ERR("IMU: CTRL1_XL write failed: %d", rc);
		return rc;
	}

	LOG_INF("IMU: LSM6DSO ready (WHO_AM_I=0x%02x)", who_am_i);
	return 0;
}

/* -- Integer square root (floor) ----------------------------------- */

/*
 * Newton–Raphson integer sqrt; converges in O(log₂ n) iterations.
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

/* -- Vibration sampling -------------------------------------------- */

/*
 * imu_sample_vibration — sample the accelerometer for ~10 seconds and
 * return two vibration metrics (both in milli-g):
 *
 *   rms_mg_out  : RMS of the AC (gravity-removed) acceleration signal,
 *                 combined across all three axes as √(σx²+σy²+σz²).
 *                 Typical values: quiet desk < 5 mg; running motor > 20 mg.
 *
 *   peak_mg_out : One-sided peak amplitude = half the observed range per
 *                 axis, combined as a Euclidean norm.
 *
 * Variance is computed in one pass using the identity:
 *   var = (n·Σx² − (Σx)²) / n²
 *
 * Overflow at n=1000, |x|≤32767:
 *   n·sum_sq ≤ 1000² · 32767² ≈ 1.07×10¹⁵  (fits in int64_t)
 *   sum²     ≤ (1000·32767)²  ≈ 1.07×10¹⁵  (fits in int64_t)
 */
static int imu_sample_vibration(int32_t *rms_mg_out, int32_t *peak_mg_out)
{
	int64_t sum_x = 0, sum_y = 0, sum_z = 0;
	int64_t sum_sq_x = 0, sum_sq_y = 0, sum_sq_z = 0;
	int32_t min_x = INT16_MAX, max_x = INT16_MIN;
	int32_t min_y = INT16_MAX, max_y = INT16_MIN;
	int32_t min_z = INT16_MAX, max_z = INT16_MIN;
	int rc;

	/* Enable accelerometer at 104 Hz, ±2 g */
	rc = lsm6dso_reg_write(LSM6DSO_REG_CTRL1_XL, LSM6DSO_CTRL1_104HZ_2G);
	if (rc != 0) {
		LOG_ERR("IMU: accel enable failed: %d", rc);
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

	/* Combined RMS in LSB, then convert to milli-g (61 µg = 0.061 mg) */
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

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/*
 * Format a unix timestamp as "YYYY-MM-DD HH:MM:SS".
 * Self-contained Gregorian algorithm — no gmtime() dependency.
 */
static void format_timestamp(int64_t ts, char *buf, size_t len)
{
	static const uint8_t mdays[12] = {31, 28, 31, 30, 31, 30,
					  31, 31, 30, 31, 30, 31};

	if (ts <= 0) {
		snprintf(buf, len, "(time not set)");
		return;
	}

	int64_t days = ts / 86400;
	int64_t rem  = ts % 86400;
	int hh = (int)(rem / 3600);
	int mm = (int)((rem % 3600) / 60);
	int ss = (int)(rem % 60);

	int year = 1970;

	while (1) {
		bool leap = ((year % 4 == 0) && (year % 100 != 0)) ||
			    (year % 400 == 0);
		int diy = leap ? 366 : 365;

		if (days < diy) {
			break;
		}
		days -= diy;
		year++;
	}

	year = (year > 9999) ? 9999 : year;

	bool leap = ((year % 4 == 0) && (year % 100 != 0)) ||
		    (year % 400 == 0);
	int month = 1;

	for (int m = 0; m < 12; m++) {
		int md = mdays[m] + (m == 1 && leap ? 1 : 0);

		if (days < md) {
			month = m + 1;
			break;
		}
		days -= md;
	}

	int day = (int)days + 1;

	snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d",
		 year, month, day, hh, mm, ss);
}

/* Parse "YYYY-MM-DD". Returns 0 on success. */
static int parse_date_str(const char *s, int *year, int *month, int *day)
{
	if (strlen(s) != 10 || s[4] != '-' || s[7] != '-') {
		return -EINVAL;
	}
	for (int i = 0; i < 10; i++) {
		if (i == 4 || i == 7) {
			continue;
		}
		if (s[i] < '0' || s[i] > '9') {
			return -EINVAL;
		}
	}
	*year  = (s[0]-'0')*1000 + (s[1]-'0')*100 + (s[2]-'0')*10 + (s[3]-'0');
	*month = (s[5]-'0')*10 + (s[6]-'0');
	*day   = (s[8]-'0')*10 + (s[9]-'0');
	return 0;
}

/* Parse "HH:MM:SS". Returns 0 on success. */
static int parse_time_str(const char *s, int *hh, int *mm, int *ss)
{
	if (strlen(s) != 8 || s[2] != ':' || s[5] != ':') {
		return -EINVAL;
	}
	for (int i = 0; i < 8; i++) {
		if (i == 2 || i == 5) {
			continue;
		}
		if (s[i] < '0' || s[i] > '9') {
			return -EINVAL;
		}
	}
	*hh = (s[0]-'0')*10 + (s[1]-'0');
	*mm = (s[3]-'0')*10 + (s[4]-'0');
	*ss = (s[6]-'0')*10 + (s[7]-'0');
	return 0;
}

/* ------------------------------------------------------------------ */
/* NVS helpers                                                          */
/* ------------------------------------------------------------------ */

static int nvs_init_storage(void)
{
	/*
	 * storage_partition is defined (and its 512 KB size locked) in the
	 * DTS overlay.  FIXED_PARTITION_* macros read directly from the DTS
	 * so no hardcoded addresses are needed here.
	 */
	struct flash_pages_info page_info;
	int rc;

	nvs.flash_device = FIXED_PARTITION_DEVICE(storage_partition);
	if (!device_is_ready(nvs.flash_device)) {
		LOG_ERR("NVS flash device not ready");
		return -ENODEV;
	}

	nvs.offset = FIXED_PARTITION_OFFSET(storage_partition);

	rc = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset,
					 &page_info);
	if (rc < 0) {
		LOG_ERR("Failed to get flash page info: %d", rc);
		return rc;
	}

	nvs.sector_size  = (uint16_t)page_info.size;
	nvs.sector_count = (uint16_t)(FIXED_PARTITION_SIZE(storage_partition)
				  / page_info.size);

	rc = nvs_mount(&nvs);
	if (rc < 0) {
		LOG_ERR("NVS mount failed: %d", rc);
		return rc;
	}

	LOG_INF("NVS mounted: offset=0x%lx sector_size=%u sectors=%u",
		(unsigned long)nvs.offset, nvs.sector_size, nvs.sector_count);
	return 0;
}

static int log_store_entry(const struct th_log_entry *entry)
{
	uint16_t slot = (uint16_t)(write_idx % MAX_LOG_ENTRIES);
	int rc;

	rc = nvs_write(&nvs, NVS_KEY_ENTRY_BASE + slot,
		       entry, sizeof(*entry));
	if (rc < 0) {
		LOG_ERR("NVS write entry failed: %d", rc);
		return rc;
	}

	write_idx++;
	rc = nvs_write(&nvs, NVS_KEY_WRITE_IDX, &write_idx, sizeof(write_idx));
	if (rc < 0) {
		LOG_ERR("NVS write_idx update failed: %d", rc);
	}
	return rc;
}

/* ------------------------------------------------------------------ */
/* Shell: thlog show / thlog clear                                     */
/* ------------------------------------------------------------------ */

static int cmd_thlog_show(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	uint32_t count = (write_idx < MAX_LOG_ENTRIES) ? write_idx
						       : MAX_LOG_ENTRIES;

	if (count == 0) {
		shell_print(sh, "Log is empty.");
		return 0;
	}

	/*
	 * When the ring buffer is full, write_idx % MAX points at the
	 * oldest slot (the next one to be overwritten).
	 */
	uint32_t oldest_slot = (write_idx >= MAX_LOG_ENTRIES)
				? (write_idx % MAX_LOG_ENTRIES) : 0u;

	shell_print(sh, "%-21s  %-10s  %-14s  %-12s  %s",
		    "Timestamp (UTC)", "Temp (C)", "Humidity (%)",
		    "VibRMS (mg)", "VibPeak (mg)");
	shell_print(sh,
		    "------------------------------------------------------------------------");

	for (uint32_t i = 0; i < count; i++) {
		uint16_t slot = (uint16_t)((oldest_slot + i) % MAX_LOG_ENTRIES);
		struct th_log_entry e;
		char timebuf[32];
		int rc;

		rc = nvs_read(&nvs, NVS_KEY_ENTRY_BASE + slot,
			      &e, sizeof(e));
		if (rc < 0) {
			shell_print(sh, "[slot %u] read error: %d", slot, rc);
			continue;
		}

		format_timestamp(e.timestamp_s, timebuf, sizeof(timebuf));
		shell_print(sh, "%-21s  %4d.%02d       %3d.%02d         %6d        %6d",
			    timebuf,
			    e.temp_val1, e.temp_val2 / 10000,
			    e.hum_val1,  e.hum_val2  / 10000,
			    e.vibration_rms_mg, e.vibration_peak_mg);
	}

	shell_print(sh,
		    "------------------------------------------------------------------------");
	shell_print(sh, "%u entr%s (max %u).",
		    count, count == 1u ? "y" : "ies", MAX_LOG_ENTRIES);
	return 0;
}

static int cmd_thlog_clear(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	write_idx = 0;
	int rc = nvs_write(&nvs, NVS_KEY_WRITE_IDX,
			   &write_idx, sizeof(write_idx));
	if (rc < 0) {
		shell_error(sh, "Failed to clear log: %d", rc);
		return rc;
	}
	shell_print(sh, "Log cleared.");
	return 0;
}

/* ------------------------------------------------------------------ */
/* Shell: thtime set / thtime get                                       */
/* ------------------------------------------------------------------ */

static int cmd_thtime_set(const struct shell *sh, size_t argc, char **argv)
{
	/* argv: [0]="set"  [1]="YYYY-MM-DD"  [2]="HH:MM:SS" */
	if (argc < 3) {
		shell_error(sh, "Usage: thtime set <YYYY-MM-DD> <HH:MM:SS>");
		return -EINVAL;
	}

	int year, month, day, hh, mm, ss;

	if (parse_date_str(argv[1], &year, &month, &day) != 0) {
		shell_error(sh, "Bad date — expected YYYY-MM-DD");
		return -EINVAL;
	}
	if (parse_time_str(argv[2], &hh, &mm, &ss) != 0) {
		shell_error(sh, "Bad time — expected HH:MM:SS");
		return -EINVAL;
	}
	if (month < 1 || month > 12 || day < 1 || day > 31 ||
	    hh > 23 || mm > 59 || ss > 59) {
		shell_error(sh, "Value out of range");
		return -EINVAL;
	}

	struct tm t = {
		.tm_year = year - 1900,
		.tm_mon  = month - 1,
		.tm_mday = day,
		.tm_hour = hh,
		.tm_min  = mm,
		.tm_sec  = ss,
	};

	base_time_s    = timeutil_timegm64(&t);
	base_uptime_ms = k_uptime_get();

	/* Persist so timestamps survive a reboot */
	nvs_write(&nvs, NVS_KEY_BASE_TIME, &base_time_s, sizeof(base_time_s));

	char buf[32];

	format_timestamp(base_time_s, buf, sizeof(buf));
	shell_print(sh, "Time set to: %s UTC", buf);
	return 0;
}

static int cmd_thtime_get(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	char buf[32];

	format_timestamp(get_timestamp_s(), buf, sizeof(buf));
	shell_print(sh, "Current time: %s UTC", buf);
	return 0;
}

/* ------------------------------------------------------------------ */
/* Shell registration                                                   */
/* ------------------------------------------------------------------ */

SHELL_STATIC_SUBCMD_SET_CREATE(sub_thlog,
	SHELL_CMD(show,  NULL, "Print all log entries", cmd_thlog_show),
	SHELL_CMD(clear, NULL, "Erase the log",         cmd_thlog_clear),
	SHELL_SUBCMD_SET_END
);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_thtime,
	SHELL_CMD_ARG(set, NULL,
		      "Set UTC time: <YYYY-MM-DD> <HH:MM:SS>",
		      cmd_thtime_set, 3, 0),
	SHELL_CMD(get, NULL, "Print current time", cmd_thtime_get),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(thlog,  &sub_thlog,  "THLogger log commands",  NULL);
SHELL_CMD_REGISTER(thtime, &sub_thtime, "THLogger time commands", NULL);

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void)
{
	LOG_INF("THLogger starting");

	/*
	 * DHT20 requires >=100 ms after VDD power-on before I2C is ready.
	 * The driver SYS_INIT runs before main(), so guard here.
	 */
	k_sleep(K_MSEC(200));

	/* --- NVS ----------------------------------------------------- */
	if (nvs_init_storage() == 0) {
		/* Restore the monotonic write counter */
		if (nvs_read(&nvs, NVS_KEY_WRITE_IDX,
			     &write_idx, sizeof(write_idx)) < 0) {
			write_idx = 0;
		}

		/* Restore the last known real-world time reference */
		int64_t stored_time = 0;

		if (nvs_read(&nvs, NVS_KEY_BASE_TIME,
			     &stored_time, sizeof(stored_time)) > 0 &&
		    stored_time > 0) {
			base_time_s    = stored_time;
			base_uptime_ms = k_uptime_get();
			LOG_INF("Restored time base from NVS");
		}

		uint32_t count = (write_idx < MAX_LOG_ENTRIES)
				 ? write_idx : MAX_LOG_ENTRIES;

		LOG_INF("Log: %u entr%s stored (max %u)",
			count, count == 1u ? "y" : "ies", MAX_LOG_ENTRIES);
	} else {
		LOG_WRN("NVS unavailable — logging disabled");
	}

	/* --- DHT20 --------------------------------------------------- */
	const struct device *dht20 = DEVICE_DT_GET_ANY(aosong_dht20);

	if (!device_is_ready(dht20)) {
		LOG_ERR("DHT20 not ready — check wiring and power");
		/* Stay alive so MCUboot does not mark the image failed */
		while (1) {
			k_sleep(K_SECONDS(5));
		}
	}

	/* --- LED ----------------------------------------------------- */
	if (gpio_is_ready_dt(&led)) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
	} else {
		LOG_WRN("LED GPIO not ready");
	}

	/* --- IMU (LSM6DSO) ------------------------------------------- */
	imu_i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c30));
	if (!device_is_ready(imu_i2c_dev)) {
		LOG_WRN("IMU: I2C bus not ready — vibration logging disabled");
	} else if (lsm6dso_init() != 0) {
		LOG_WRN("IMU: init failed — vibration logging disabled");
	} else {
		imu_ready = true;
	}

	/* 500 ms blink to signal a successful start */
	gpio_pin_set_dt(&led, 0);
	k_sleep(K_MSEC(500));
	gpio_pin_set_dt(&led, 1);

	/* --- Sampling loop ------------------------------------------- */
	while (1) {
		/*
		 * 1. IMU vibration sampling (~10 seconds).
		 *    Runs first so the accelerometer window aligns with the
		 *    start of every measurement cycle, giving a consistent
		 *    time reference for motor-state analysis.
		 */
		int32_t vib_rms_mg = 0, vib_peak_mg = 0;

		if (imu_ready) {
			if (imu_sample_vibration(&vib_rms_mg, &vib_peak_mg) != 0) {
				LOG_WRN("IMU: vibration sampling failed");
			} else {
				LOG_INF("IMU: vib RMS=%d mg  peak=%d mg",
					vib_rms_mg, vib_peak_mg);
			}
		}

		/* 2. DHT20 temperature & humidity */
		struct sensor_value temp, humidity;
		int rc = sensor_sample_fetch(dht20);

		if (rc < 0) {
			LOG_ERR("Fetch failed: %d", rc);
		} else {
			sensor_channel_get(dht20, SENSOR_CHAN_AMBIENT_TEMP,
					   &temp);
			sensor_channel_get(dht20, SENSOR_CHAN_HUMIDITY,
					   &humidity);

			struct th_log_entry entry = {
				.timestamp_s       = get_timestamp_s(),
				.temp_val1         = temp.val1,
				.temp_val2         = temp.val2,
				.hum_val1          = humidity.val1,
				.hum_val2          = humidity.val2,
				.vibration_rms_mg  = vib_rms_mg,
				.vibration_peak_mg = vib_peak_mg,
			};

			log_store_entry(&entry);

			/* 500 ms blink to signal a successful sample */
			gpio_pin_set_dt(&led, 0);
			k_sleep(K_MSEC(500));
			gpio_pin_set_dt(&led, 1);
		}

		/*
		 * 3. Sleep for the remainder of the 5-minute cycle.
		 *    The IMU window already consumed ~10 seconds, so subtract
		 *    that when the IMU is active to keep a consistent period.
		 */
		int sleep_s = (5 * 60) - (imu_ready ? (IMU_NUM_SAMPLES *
							    IMU_SAMPLE_PERIOD_MS /
							    1000) : 0);

		if (sleep_s > 0) {
			k_sleep(K_SECONDS(sleep_s));
		}
	}

	return 0;
}
