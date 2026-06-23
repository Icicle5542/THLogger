/*
 * pcf8563.c — PCF8563 RTC wrapper.
 *
 * Provides Unix-second read/write over Zephyr's RTC API.
 * The PCF8563 sits on the XIAO Expansion Board (I2C address 0x51) with
 * a CR1220 battery backup, keeping accurate time across power cycles.
 */

#include "pcf8563.h"

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/rtc.h>
#include <zephyr/sys/timeutil.h>
#include <zephyr/logging/log.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>

LOG_MODULE_REGISTER(app_rtc, CONFIG_LOG_DEFAULT_LEVEL);

#define PCF8563_NODE DT_NODELABEL(pcf8563)

static const struct device *rtc_dev;

/* ------------------------------------------------------------------ */
/* Internal helpers                                                     */
/* ------------------------------------------------------------------ */

/*
 * Decompose a Unix timestamp into struct rtc_time (UTC).
 * Uses the same Gregorian algorithm as format_timestamp() in main.c.
 */
static void unix_to_rtc_time(int64_t ts, struct rtc_time *rt)
{
	static const uint8_t mdays[12] = {31, 28, 31, 30, 31, 30,
					  31, 31, 30, 31, 30, 31};

	int64_t days = ts / 86400;
	int64_t rem  = ts % 86400;

	rt->tm_hour  = (int)(rem / 3600);
	rt->tm_min   = (int)((rem % 3600) / 60);
	rt->tm_sec   = (int)(rem % 60);
	rt->tm_nsec  = 0;
	/* 1970-01-01 was a Thursday (4); 0 = Sunday in tm_wday */
	rt->tm_wday  = (int)((days + 4) % 7);
	rt->tm_isdst = -1;

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

	bool leap = ((year % 4 == 0) && (year % 100 != 0)) ||
		    (year % 400 == 0);
	int yday  = 0;
	int month = 1;

	for (int m = 0; m < 12; m++) {
		int md = mdays[m] + (m == 1 && leap ? 1 : 0);

		if (days < md) {
			month = m + 1;
			break;
		}
		yday += md;
		days -= md;
	}

	rt->tm_year = year - 1900;
	rt->tm_mon  = month - 1;
	rt->tm_mday = (int)days + 1;
	rt->tm_yday = yday + (int)days;
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

int pcf8563_init(void)
{
	rtc_dev = DEVICE_DT_GET(PCF8563_NODE);
	if (!device_is_ready(rtc_dev)) {
		LOG_WRN("PCF8563 not ready — check I2C wiring");
		rtc_dev = NULL;
		return -ENODEV;
	}
	return 0;
}

int64_t pcf8563_read_unix(void)
{
	if (rtc_dev == NULL) {
		return 0;
	}

	struct rtc_time rt;
	int rc = rtc_get_time(rtc_dev, &rt);

	if (rc != 0) {
		LOG_WRN("rtc_get_time failed: %d", rc);
		return 0;
	}

	struct tm t = {
		.tm_year = rt.tm_year,
		.tm_mon  = rt.tm_mon,
		.tm_mday = rt.tm_mday,
		.tm_hour = rt.tm_hour,
		.tm_min  = rt.tm_min,
		.tm_sec  = rt.tm_sec,
	};
	int64_t ts = timeutil_timegm64(&t);

	return (ts > 0) ? ts : 0;
}

int pcf8563_write_unix(int64_t ts)
{
	if (rtc_dev == NULL) {
		return -ENODEV;
	}

	struct rtc_time rt;

	unix_to_rtc_time(ts, &rt);
	int rc = rtc_set_time(rtc_dev, &rt);

	if (rc != 0) {
		LOG_WRN("rtc_set_time failed: %d", rc);
	}
	return rc;
}
