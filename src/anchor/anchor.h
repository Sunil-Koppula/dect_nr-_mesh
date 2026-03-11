/*
 * Anchor device logic for DECT NR+ mesh network
 *
 * NVS layout for anchor:
 *   Key 0x000          — Own identity (node_identity_t)
 *   Key 0x001 .. 0x064 — 100 paired anchor IDs  (uint16_t each)
 *   Key 0x065 .. 0x0C8 — 100 paired sensor IDs  (uint16_t each)
 */

#ifndef ANCHOR_H
#define ANCHOR_H

#include "../state.h"

/* NVS key offsets */
#define ANCHOR_IDENTITY_KEY  0x000
#define ANCHOR_ANCHOR_BASE   0x001
#define ANCHOR_ANCHOR_MAX    100
#define ANCHOR_SENSOR_BASE   0x065   /* 0x001 + 100 */
#define ANCHOR_SENSOR_MAX    100

void anchor_main(void);

/* Anchor identity helpers */
int anchor_store_identity(const node_identity_t *id);
int anchor_load_identity(node_identity_t *id);
bool anchor_has_identity(void);

#endif /* ANCHOR_H */
