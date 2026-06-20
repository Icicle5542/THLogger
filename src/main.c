#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

int main(void)
{
	LOG_INF("THLogger starting");

	/* DHT20 requires at least 100 ms after power-on before responding to I2C.
	 * The driver SYS_INIT may have run before the sensor was ready, so give it
	 * extra time before we start communicating with it. */
	k_sleep(K_MSEC(200));


	const struct device *dht20 = DEVICE_DT_GET_ANY(aosong_dht20);

	if (!device_is_ready(dht20)) {
		LOG_ERR("DHT20 device not ready — check wiring and power");
		// Do not return; keep the app alive so MCUboot does not mark it failed
		while (1) {
			k_sleep(K_SECONDS(5));
		}
	}

	while (1) {
		struct sensor_value temp, humidity;
		int rc;

		rc = sensor_sample_fetch(dht20);
		if (rc < 0) {
			LOG_ERR("Failed to fetch sample: %d", rc);
		} else {
			sensor_channel_get(dht20, SENSOR_CHAN_AMBIENT_TEMP, &temp);
			sensor_channel_get(dht20, SENSOR_CHAN_HUMIDITY, &humidity);

			LOG_INF("Temperature: %d.%02d C  Humidity: %d.%02d %%",
				temp.val1, temp.val2 / 10000,
				humidity.val1, humidity.val2 / 10000);
		}

		k_sleep(K_SECONDS(2));
	}

	return 0;
}
