/*
 * Sensor device logic for DECT NR+ mesh network
 *
 * NVS layout for sensor:
 *   Key 0x000 — Own identity (node_identity_t)
 */

#ifndef SENSOR_H
#define SENSOR_H

#include "../state.h"

/* NVS key offset */
#define SENSOR_IDENTITY_KEY  0x000

void sensor_main(void);

/* Sensor identity helpers */
int sensor_store_identity(const node_identity_t *id);
int sensor_load_identity(node_identity_t *id);
bool sensor_has_identity(void);

#endif /* SENSOR_H */
