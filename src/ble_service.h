/*
 * ble_service.h — BLE Nordic UART Service (NUS) for THLogger.
 *
 * Call ble_service_init() once from main() after NVS is mounted.
 * The BLE service accepts text commands over NUS RX and streams
 * log data back via NUS TX notifications.
 *
 * Commands (sent by the Python client via NUS RX):
 *   "show"            — stream all log entries, terminated by "END\n"
 *   "status"          — respond with entry count and device name
 *   "name:<NewName>"  — persist a new BLE device name in NVS
 */

#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

/**
 * @brief Initialise BLE stack, register NUS, load device name from NVS,
 *        and begin advertising.
 *
 * Must be called after NVS is mounted (nvs_ready == true).
 * Blocks until the BT controller is ready.
 *
 * @return 0 on success, negative errno on failure.
 */
int ble_service_init(void);

/**
 * @brief Get the current BLE device name.
 *
 * @return Pointer to a null-terminated name string (static storage).
 */
const char *ble_service_get_name(void);

/**
 * @brief Set a new BLE device name and persist it in NVS.
 *
 * The new name takes effect for future advertising (after disconnect
 * or reboot).  Maximum 29 characters (BLE scan-response limit).
 *
 * @param name  New device name (null-terminated).
 * @return 0 on success, negative errno on failure.
 */
int ble_service_set_name(const char *name);

#endif /* BLE_SERVICE_H */
