/*
 * Device identity, runtime state, and paired-device registry
 * for DECT NR+ mesh network
 */

#ifndef IDENTITY_H
#define IDENTITY_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>
#include "protocol.h"

/* ===== Runtime device state (defined in main.c) ===== */

/* Device identity and role */
extern uint16_t device_id;
extern device_type_t my_device_type;
extern uint8_t my_hop_num;

/* Last received RSSI in dBm (updated on every PDC, defined in radio.c) */
extern volatile int16_t last_rssi_dbm;

/* Button semaphores */
extern struct k_sem btn2_sem;  /* small data send */
extern struct k_sem btn3_sem;  /* 50KB large data send */
extern struct k_sem btn4_sem;  /* 75KB large data send */

/* Device entry points */
void gateway_main(void);
void anchor_main(void);
void sensor_main(void);

/* ===== Node identity (NVM) ===== */

/* Node identity record — stored in NVM for anchors and sensors.
 * For mesh anchors: min_hop is the lowest hop among neighbors (my_hop = min_hop + 1).
 * For sensors: parent_id/parent_hop still used for tree-style uplink.
 */
typedef struct {
	uint16_t device_id;
	uint8_t device_type;
	uint16_t parent_id;     /* sensors: parent ID; anchors: best-route neighbor ID */
	uint8_t parent_hop;     /* sensors: parent hop; anchors: min hop among neighbors */
} __attribute__((packed)) node_identity_t;

/*
 * Unified NVS layout (same for gateway, anchor, sensor):
 *   Key 0x000          — Self identity (node_identity_t)
 *   Key 0x001 .. 0x008 — 8 paired anchors (paired_device_info_t)
 *   Key 0x009 .. 0x018 — 16 paired sensors (paired_device_info_t)
 *   Key 0x019+         — Reserved / application data
 *
 * Gateway uses 0x001-0x018 for its children.
 * Anchor uses 0x000 for parent identity + 0x001-0x018 for its children.
 * Sensor uses 0x000 for parent identity (no children).
 */
#define NVS_IDENTITY_KEY    0x000
#define NVS_ANCHOR_BASE     0x001
#define NVS_ANCHOR_MAX      8
#define NVS_SENSOR_BASE     0x009   /* 0x001 + 8 */
#define NVS_SENSOR_MAX      16

/* Identity helpers (defined in identity.c) */
int node_store_identity(const node_identity_t *id);
int node_load_identity(node_identity_t *id);
bool node_has_identity(void);

/* ===== Paired device registry ===== */

/* Paired device record — stored in NVS */
typedef struct {
	uint16_t device_id;
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t version_patch;
} __attribute__((packed)) paired_device_info_t;

typedef struct {
	uint16_t nvs_base;
	int max_entries;
	const char *label;      /* "Anchor" or "Sensor", for logging */
} paired_store_t;

/* Add a device with version info. Updates version if already present. */
int paired_store_add(const paired_store_t *ps, uint16_t dev_id,
		     uint8_t ver_major, uint8_t ver_minor, uint16_t ver_patch);

/* Check if a device ID is in the store */
bool paired_store_contains(const paired_store_t *ps, uint16_t dev_id);

/* Count how many entries are stored */
int paired_store_count(const paired_store_t *ps);

/* Print all stored entries */
void paired_store_print(const paired_store_t *ps);

/* Get device info at NVS slot index. Returns 0 on success, -ENOENT if empty. */
int paired_store_get(const paired_store_t *ps, int index, uint16_t *dev_id);

/* Get full device info at NVS slot index. Returns 0 on success. */
int paired_store_get_info(const paired_store_t *ps, int index,
			  paired_device_info_t *info);

/* Get device info by device ID. Returns 0 on success, -ENOENT if not found. */
int paired_store_find(const paired_store_t *ps, uint16_t dev_id,
		      paired_device_info_t *info);

/* Update the stored version for a device (e.g., after successful OTA) */
int paired_store_update_version(const paired_store_t *ps, uint16_t dev_id,
				uint8_t ver_major, uint8_t ver_minor,
				uint16_t ver_patch);

#endif /* IDENTITY_H */
