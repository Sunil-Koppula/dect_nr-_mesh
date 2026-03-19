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
