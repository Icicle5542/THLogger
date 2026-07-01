/*
 * ble_service.c — BLE NUS command/response service for THLogger.
 *
 * Modeled on the ThermalCam project's ble_stream.c.
 * Accepts text commands over NUS RX and streams log data back over NUS TX.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <bluetooth/services/nus.h>
#include <zephyr/logging/log.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ble_service.h"
#include "thlog_data.h"

LOG_MODULE_REGISTER(ble_service, LOG_LEVEL_INF);

/* ---- BLE device name --------------------------------------------------- */
#define BLE_NAME_MAX_LEN  29   /* BLE scan-response payload limit */
static char s_ble_name[BLE_NAME_MAX_LEN + 1]; /* current device name */

/* ---- LED0: BLE connected indicator ---- */
static const struct gpio_dt_spec s_led0 =
	GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* ---- Module-level state ---- */
static struct bt_conn *s_conn;          /* NULL when not connected       */
static bool           s_notify_enabled; /* true once peer writes CCCD=1  */
static K_SEM_DEFINE(s_bt_ready, 0, 1);

/* Work item for advertising restart (must run outside BT RX thread) */
static void adv_restart_work_handler(struct k_work *work);
static K_WORK_DEFINE(s_adv_restart_work, adv_restart_work_handler);

/*
 * Command thread: processes NUS RX commands on a dedicated stack so the
 * system workqueue is not blocked during long NVS reads or BLE TX loops.
 * The BLE stack's own work items can then drain the TX buffer freely.
 */
#define CMD_BUF_SIZE        64
#define CMD_THREAD_STACK    4096
#define CMD_THREAD_PRIORITY 7

static char     s_cmd_buf[CMD_BUF_SIZE];
static uint16_t s_cmd_len;

static K_THREAD_STACK_DEFINE(s_cmd_stack, CMD_THREAD_STACK);
static struct k_thread s_cmd_thread_data;
static K_SEM_DEFINE(s_cmd_sem, 0, 1);

static void cmd_thread_fn(void *p1, void *p2, void *p3);

/* ---- Binary show-protocol framing bytes -------------------------------- */
static const uint8_t SHOW_SOF[] = {0xAAu, 0x55u, 0xDDu, 0x22u};
static const uint8_t SHOW_EOF[] = {0x22u, 0xDDu, 0x55u, 0xAAu};

/* ---- Advertising / scan-response data ---------------------------------- */

static const struct bt_data s_adv[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* Scan response is built dynamically so it uses the NVS-loaded name */
static struct bt_data s_sd[1];

static void build_scan_response(void)
{
	s_sd[0].type = BT_DATA_NAME_COMPLETE;
	s_sd[0].data = (const uint8_t *)s_ble_name;
	s_sd[0].data_len = (uint8_t)strlen(s_ble_name);
}

/* ---- Send raw bytes over NUS TX in MTU-sized chunks -------------------- */

static int nus_send_raw(const uint8_t *data, size_t len)
{
	if (!s_conn || !s_notify_enabled) {
		return -ENOTCONN;
	}

	uint32_t mtu = bt_nus_get_mtu(s_conn);

	if (mtu == 0) {
		mtu = 20;
	}

	for (size_t offset = 0; offset < len;) {
		size_t chunk = MIN(mtu, (uint32_t)(len - offset));
		int ret = bt_nus_send(s_conn,
				      data + offset,
				      (uint16_t)chunk);
		if (ret == -EAGAIN || ret == -ENOMEM) {
			/* BLE TX buffer full — yield briefly and retry */
			k_sleep(K_MSEC(5));
			continue;
		}
		if (ret) {
			LOG_ERR("bt_nus_send failed at offset %zu (err %d)",
				offset, ret);
			return ret;
		}
		offset += chunk;
	}
	return 0;
}

static int nus_send_str(const char *str, size_t len)
{
	return nus_send_raw((const uint8_t *)str, len);
}

/* ---- Command handlers -------------------------------------------------- */

static void handle_show_cmd(void)
{
	if (!nvs_ready) {
		static const char err[] = "ERR:nvs\nEND\n";
		nus_send_str(err, sizeof(err) - 1);
		return;
	}

	uint32_t count = (write_idx < MAX_LOG_ENTRIES) ? write_idx
						       : MAX_LOG_ENTRIES;
	uint32_t oldest_slot = (write_idx >= MAX_LOG_ENTRIES)
				? (write_idx % MAX_LOG_ENTRIES) : 0u;

	/*
	 * Binary frame: SOF(4) + count(4) + N×entry(32) + EOF(4)
	 * Running on the dedicated command thread so the system workqueue
	 * remains free for the BLE stack to drain its TX buffer.
	 */
	nus_send_raw(SHOW_SOF, sizeof(SHOW_SOF));
	nus_send_raw((const uint8_t *)&count, sizeof(count));

	for (uint32_t i = 0; i < count; i++) {
		if (!s_conn || !s_notify_enabled) {
			LOG_WRN("BLE disconnected during show — aborting");
			return;
		}

		uint16_t slot = (uint16_t)((oldest_slot + i) % MAX_LOG_ENTRIES);
		struct th_log_entry e;

		if (nvs_read(&nvs, NVS_KEY_ENTRY_BASE + slot,
			     &e, sizeof(e)) < 0) {
			memset(&e, 0, sizeof(e));
		}

		nus_send_raw((const uint8_t *)&e, sizeof(e));

		/* Yield every 8 entries so the BLE stack can drain its TX buffer */
		if ((i % 8) == 7) {
			k_sleep(K_MSEC(10));
		}
	}

	nus_send_raw(SHOW_EOF, sizeof(SHOW_EOF));
	LOG_INF("BLE: sent %u entries (binary, %u bytes)",
		count,
		(unsigned)(sizeof(SHOW_SOF) + 4u +
			   count * sizeof(struct th_log_entry) +
			   sizeof(SHOW_EOF)));
}

static void handle_status_cmd(void)
{
	char buf[128];
	uint32_t count = (write_idx < MAX_LOG_ENTRIES) ? write_idx
						       : MAX_LOG_ENTRIES;
	int n = snprintf(buf, sizeof(buf),
			 "Name: %s\nEntries: %u / %u\nUptime: %lld s\nEND\n",
			 s_ble_name, count, MAX_LOG_ENTRIES,
			 k_uptime_get() / 1000);
	if (n > 0) {
		nus_send_str(buf, (size_t)n);
	}
}

static void handle_name_cmd(const char *new_name)
{
	if (strlen(new_name) == 0 || strlen(new_name) > BLE_NAME_MAX_LEN) {
		char err[] = "ERROR: name must be 1-29 characters\nEND\n";
		nus_send_str(err, strlen(err));
		return;
	}

	int rc = ble_service_set_name(new_name);
	char buf[64];
	int n;

	if (rc == 0) {
		n = snprintf(buf, sizeof(buf),
			     "Name set to: %s (re-advertise on reconnect)\nEND\n",
			     s_ble_name);
	} else {
		n = snprintf(buf, sizeof(buf),
			     "ERROR: failed to save name (%d)\nEND\n", rc);
	}
	if (n > 0) {
		nus_send_str(buf, (size_t)n);
	}
}

/* ---- Command thread (dedicated stack, not the system workqueue) --------- */

static void cmd_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		k_sem_take(&s_cmd_sem, K_FOREVER);

		/* Null-terminate and trim trailing whitespace */
		s_cmd_buf[s_cmd_len] = '\0';
		char *cmd = s_cmd_buf;

		/* Trim leading/trailing whitespace */
		while (*cmd == ' ' || *cmd == '\t' || *cmd == '\r' || *cmd == '\n') {
			cmd++;
		}
		size_t len = strlen(cmd);

		while (len > 0 && (cmd[len - 1] == ' ' || cmd[len - 1] == '\t' ||
				   cmd[len - 1] == '\r' || cmd[len - 1] == '\n')) {
			cmd[--len] = '\0';
		}

		LOG_INF("BLE RX command: \"%s\"", cmd);

		if (strcmp(cmd, "show") == 0) {
			handle_show_cmd();
		} else if (strcmp(cmd, "status") == 0) {
			handle_status_cmd();
		} else if (strncmp(cmd, "name:", 5) == 0) {
			handle_name_cmd(cmd + 5);
		} else {
			char buf[64];
			int n = snprintf(buf, sizeof(buf),
					 "ERROR: unknown command \"%s\"\nEND\n", cmd);
			if (n > 0) {
				nus_send_str(buf, (size_t)n);
			}
		}
	}
}

/* ---- BT controller ready callback -------------------------------------- */

static void bt_ready_cb(int err)
{
	if (err) {
		LOG_ERR("bt_enable failed (err %d)", err);
		return;
	}
	k_sem_give(&s_bt_ready);
}

/* ---- Advertising restart (system workqueue) ----------------------------- */

static void adv_restart_work_handler(struct k_work *work)
{
	ARG_UNUSED(work);

	build_scan_response();
	int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
				  s_adv, ARRAY_SIZE(s_adv),
				  s_sd,  ARRAY_SIZE(s_sd));
	if (ret) {
		LOG_ERR("Advertising restart failed (err %d)", ret);
	} else {
		LOG_INF("Re-advertising as \"%s\"", s_ble_name);
	}
}

/* ---- Connection callbacks ---------------------------------------------- */

static void connected_cb(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		LOG_ERR("Connection failed (err 0x%02x)", err);
		return;
	}
	s_conn = bt_conn_ref(conn);
	gpio_pin_set_dt(&s_led0, 1);
	LOG_INF("BLE peer connected");
}

static void disconnected_cb(struct bt_conn *conn, uint8_t reason)
{
	LOG_INF("BLE peer disconnected (reason 0x%02x)", reason);

	gpio_pin_set_dt(&s_led0, 0);
	s_notify_enabled = false;

	if (s_conn) {
		bt_conn_unref(s_conn);
		s_conn = NULL;
	}

	k_work_submit(&s_adv_restart_work);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected    = connected_cb,
	.disconnected = disconnected_cb,
};

/* ---- NUS callbacks ----------------------------------------------------- */

static void nus_send_enabled_cb(enum bt_nus_send_status status)
{
	s_notify_enabled = (status == BT_NUS_SEND_STATUS_ENABLED);
	LOG_INF("NUS notifications %s",
		s_notify_enabled ? "enabled" : "disabled");
}

static void nus_received_cb(struct bt_conn *conn,
			     const uint8_t *data, uint16_t len)
{
	ARG_UNUSED(conn);

	if (len == 0 || len >= CMD_BUF_SIZE) {
		return;
	}

	/* Copy to command buffer and defer processing to the workqueue
	 * so we don't block the BT RX thread during long operations. */
	memcpy(s_cmd_buf, data, len);
	s_cmd_len = len;
	k_sem_give(&s_cmd_sem);  /* wake the command thread */
}

/* ---- Public API -------------------------------------------------------- */

int ble_service_init(void)
{
	/* Load device name from NVS, fall back to compile-time default */
	strncpy(s_ble_name, CONFIG_BT_DEVICE_NAME, BLE_NAME_MAX_LEN);
	s_ble_name[BLE_NAME_MAX_LEN] = '\0';

	if (nvs_ready) {
		char stored[BLE_NAME_MAX_LEN + 1];
		int rc = nvs_read(&nvs, NVS_KEY_BLE_NAME,
				  stored, sizeof(stored) - 1);
		if (rc > 0) {
			stored[rc] = '\0';
			strncpy(s_ble_name, stored, BLE_NAME_MAX_LEN);
			s_ble_name[BLE_NAME_MAX_LEN] = '\0';
			LOG_INF("Loaded BLE name from NVS: \"%s\"", s_ble_name);
		}
	}

	/* Configure LED0 (BLE connected indicator), initially off */
	if (gpio_is_ready_dt(&s_led0)) {
		gpio_pin_configure_dt(&s_led0, GPIO_OUTPUT_INACTIVE);
	}

	/* Enable BT controller */
	int ret = bt_enable(bt_ready_cb);

	if (ret) {
		LOG_ERR("bt_enable failed (err %d)", ret);
		return ret;
	}
	k_sem_take(&s_bt_ready, K_FOREVER);
	LOG_INF("BT stack ready");

	/* Set the runtime device name (overrides CONFIG_BT_DEVICE_NAME) */
	ret = bt_set_name(s_ble_name);

	if (ret) {
		LOG_ERR("bt_set_name failed (err %d)", ret);
		return ret;
	}

	/* Register NUS callbacks (RX for commands, send_enabled for CCCD) */
	static struct bt_nus_cb nus_cb = {
		.received     = nus_received_cb,
		.send_enabled = nus_send_enabled_cb,
	};

	ret = bt_nus_init(&nus_cb);
	if (ret) {
		LOG_ERR("NUS init failed (err %d)", ret);
		return ret;
	}

	/* Launch the dedicated command thread */
	k_thread_create(&s_cmd_thread_data, s_cmd_stack,
			K_THREAD_STACK_SIZEOF(s_cmd_stack),
			cmd_thread_fn, NULL, NULL, NULL,
			CMD_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&s_cmd_thread_data, "ble_cmd");

	/* Start advertising */
	build_scan_response();
	ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1,
			      s_adv, ARRAY_SIZE(s_adv),
			      s_sd,  ARRAY_SIZE(s_sd));
	if (ret) {
		LOG_ERR("Advertising start failed (err %d)", ret);
		return ret;
	}

	LOG_INF("Advertising as \"%s\"", s_ble_name);
	return 0;
}

const char *ble_service_get_name(void)
{
	return s_ble_name;
}

int ble_service_set_name(const char *name)
{
	size_t len = strlen(name);

	if (len == 0 || len > BLE_NAME_MAX_LEN) {
		return -EINVAL;
	}

	strncpy(s_ble_name, name, BLE_NAME_MAX_LEN);
	s_ble_name[BLE_NAME_MAX_LEN] = '\0';

	/* Update the runtime BT name */
	int ret = bt_set_name(s_ble_name);

	if (ret) {
		LOG_ERR("bt_set_name failed (err %d)", ret);
		return ret;
	}

	/* Persist to NVS */
	if (nvs_ready) {
		int rc = nvs_write(&nvs, NVS_KEY_BLE_NAME,
				   s_ble_name, strlen(s_ble_name));
		if (rc < 0) {
			LOG_ERR("NVS write BLE name failed: %d", rc);
			return rc;
		}
	}

	LOG_INF("BLE name set to \"%s\"", s_ble_name);
	return 0;
}
