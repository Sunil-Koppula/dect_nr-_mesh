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

/* NVS key offsets */
#define GW_ANCHOR_BASE   0x000
#define GW_ANCHOR_MAX    100
#define GW_SENSOR_BASE   0x064   /* 0x000 + 100 */
#define GW_SENSOR_MAX    100

void gateway_main(void);

#endif /* GATEWAY_H */
