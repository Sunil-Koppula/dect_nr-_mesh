/*
 * Gateway device logic for DECT NR+ mesh network
 *
 * NVS layout for gateway:
 *   Key 0x000 .. 0x063  — 100 paired anchor IDs  (uint16_t each)
 *   Key 0x064 .. 0x0C7  — 100 paired sensor IDs  (uint16_t each)
 */

#ifndef GATEWAY_H
#define GATEWAY_H

#include <stdint.h>
#include <stdbool.h>

/* NVS key offsets */
#define GW_ANCHOR_BASE   0x000
#define GW_ANCHOR_MAX    100
#define GW_SENSOR_BASE   0x064   /* 0x000 + 100 */
#define GW_SENSOR_MAX    100

void gateway_main(void);

/* Gateway storage helpers */
int gw_store_anchor(uint16_t anchor_id);
int gw_store_sensor(uint16_t sensor_id);
bool gw_is_anchor_paired(uint16_t anchor_id);
bool gw_is_sensor_paired(uint16_t sensor_id);
int gw_get_anchor_count(void);
int gw_get_sensor_count(void);
void gw_print_paired(void);

#endif /* GATEWAY_H */
