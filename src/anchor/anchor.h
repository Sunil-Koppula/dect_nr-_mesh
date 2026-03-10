/*
 * Anchor device logic for DECT NR+ mesh network
 *
 * NVS layout for anchor:
 *   Key 0x000          — Own identity (anchor_identity_t)
 *   Key 0x001 .. 0x064 — 100 paired anchor IDs  (uint16_t each)
 *   Key 0x065 .. 0x0C8 — 100 paired sensor IDs  (uint16_t each)
 */

#ifndef ANCHOR_H
#define ANCHOR_H

#include <stdint.h>
#include <stdbool.h>

/* Anchor own identity record */
typedef struct {
	uint16_t device_id;
	uint8_t device_type;
	uint16_t parent_id;
	uint8_t parent_hop;
} __attribute__((packed)) anchor_identity_t;

/* NVS key offsets */
#define ANCHOR_IDENTITY_KEY  0x000
#define ANCHOR_ANCHOR_BASE   0x001
#define ANCHOR_ANCHOR_MAX    100
#define ANCHOR_SENSOR_BASE   0x065   /* 0x001 + 100 */
#define ANCHOR_SENSOR_MAX    100

void anchor_main(void);

/* Anchor storage helpers */
int anchor_store_identity(const anchor_identity_t *id);
int anchor_load_identity(anchor_identity_t *id);
bool anchor_has_identity(void);

int anchor_store_anchor(uint16_t anchor_id);
int anchor_store_sensor(uint16_t sensor_id);
bool anchor_is_anchor_paired(uint16_t anchor_id);
bool anchor_is_sensor_paired(uint16_t sensor_id);
int anchor_get_anchor_count(void);
int anchor_get_sensor_count(void);
void anchor_print_paired(void);

#endif /* ANCHOR_H */
