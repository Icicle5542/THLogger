/*
 * thlog_data.h — Shared data types, NVS key layout, and externs for THLogger.
 *
 * Both main.c and ble_service.c include this header to access the log
 * storage format and global NVS state.
 */

#ifndef THLOG_DATA_H
#define THLOG_DATA_H

#include <zephyr/fs/nvs.h>
#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/* NVS key layout                                                      */
/* ------------------------------------------------------------------ */
#define NVS_KEY_WRITE_IDX   1u
#define NVS_KEY_BASE_TIME   2u
#define NVS_KEY_BLE_NAME    3u    /* BLE device name string (max 29 chars) */
#define NVS_KEY_ENTRY_BASE  10u   /* keys 10 … 10+MAX_LOG_ENTRIES-1       */

#define MAX_LOG_ENTRIES     10000u

/* ------------------------------------------------------------------ */
/* Log entry (32 bytes — stored in NVS circular ring)                  */
/* ------------------------------------------------------------------ */
struct th_log_entry {
	int64_t timestamp_s;
	int32_t temp_val1;
	int32_t temp_val2;
	int32_t hum_val1;
	int32_t hum_val2;
	int32_t vibration_rms_mg;
	int32_t vibration_peak_mg;
};

/* ------------------------------------------------------------------ */
/* Globals owned by main.c, shared with ble_service.c                  */
/* ------------------------------------------------------------------ */
extern struct nvs_fs nvs;
extern bool          nvs_ready;
extern uint32_t      write_idx;

/* ------------------------------------------------------------------ */
/* Utility functions implemented in main.c                             */
/* ------------------------------------------------------------------ */

/** Current UTC timestamp derived from software clock. */
int64_t get_timestamp_s(void);

/** Format a unix timestamp as "YYYY-MM-DD HH:MM:SS" into @p buf. */
void format_timestamp(int64_t ts, char *buf, size_t len);

#endif /* THLOG_DATA_H */
