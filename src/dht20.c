/*
 * dht20.c — DHT20 temperature & humidity sensor wrapper.
 *
 * Thin wrapper around the Zephyr aosong,dht20 sensor driver that caches
 * the device pointer and exposes a simple measure() function.
 */

#include "dht20.h"

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(dht20, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *dht20_dev;

int dht20_init(void)
{
	dht20_dev = DEVICE_DT_GET_ANY(aosong_dht20);
	if (!device_is_ready(dht20_dev)) {
		LOG_ERR("DHT20 not ready — check wiring and power");
		return -ENODEV;
	}
	return 0;
}

int dht20_measure(struct sensor_value *temp, struct sensor_value *humidity)
{
	int rc = sensor_sample_fetch(dht20_dev);

	if (rc < 0) {
		LOG_ERR("Fetch failed: %d", rc);
		return rc;
	}

	sensor_channel_get(dht20_dev, SENSOR_CHAN_AMBIENT_TEMP, temp);
	sensor_channel_get(dht20_dev, SENSOR_CHAN_HUMIDITY, humidity);
	return 0;
}
