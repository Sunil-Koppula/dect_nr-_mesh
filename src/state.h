/*
 * Shared device state for DECT NR+ mesh network
 */

#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include "packet.h"

/* Node identity record — stored in NVM for anchors and sensors */
typedef struct {
	uint16_t device_id;
	uint8_t device_type;
	uint16_t parent_id;
	uint8_t parent_hop;
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

/* Identity helpers (shared by all device types, defined in storage.c) */
int node_store_identity(const node_identity_t *id);
int node_load_identity(node_identity_t *id);
bool node_has_identity(void);

/* Device entry points */
void gateway_main(void);
void anchor_main(void);
void sensor_main(void);

/* Device identity and role (defined in main.c) */
extern uint16_t device_id;
extern device_type_t my_device_type;
extern uint8_t my_hop_num;

/* Last received RSSI in dBm (updated on every PDC, defined in radio.c) */
extern volatile int16_t last_rssi_dbm;

/* Button semaphores (defined in main.c) */
extern struct k_sem btn2_sem;  /* small data send */
extern struct k_sem btn3_sem;  /* 50KB large data send */
extern struct k_sem btn4_sem;  /* 75KB large data send */

#endif /* STATE_H */
