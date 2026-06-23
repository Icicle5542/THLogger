/*
 * dht20.h — DHT20 temperature & humidity sensor public interface.
 */
#ifndef DHT20_H_
#define DHT20_H_

#include <zephyr/drivers/sensor.h>

/**
 * Initialise the DHT20 (Zephyr driver, I²C address 0x38).
 * Returns 0 on success, -ENODEV if the device is not ready.
 */
int dht20_init(void);

/**
 * Fetch a temperature and humidity sample from the DHT20.
 *
 * @param temp      Output: temperature in Zephyr sensor_value format.
 * @param humidity  Output: relative humidity in Zephyr sensor_value format.
 * @return 0 on success, negative errno on failure.
 */
int dht20_measure(struct sensor_value *temp, struct sensor_value *humidity);

#endif /* DHT20_H_ */
