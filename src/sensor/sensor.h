/*
 * Sensor device logic for DECT NR+ mesh network
 *
 * NVS layout for sensor:
 *   Key 0x000 — Own identity (sensor_identity_t)
 */

#ifndef SENSOR_H
#define SENSOR_H

#include <stdint.h>
#include <stdbool.h>

/* Sensor own identity record */
typedef struct {
	uint16_t device_id;
	uint8_t device_type;
	uint16_t parent_id;
	uint8_t parent_hop;
} __attribute__((packed)) sensor_identity_t;

/* NVS key offset */
#define SENSOR_IDENTITY_KEY  0x000

void sensor_main(void);

/* Sensor storage helpers */
int sensor_store_identity(const sensor_identity_t *id);
int sensor_load_identity(sensor_identity_t *id);
bool sensor_has_identity(void);

#endif /* SENSOR_H */
