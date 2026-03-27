/*
 * Mesh discovery logic for DECT NR+ mesh network
 */

#ifndef MESH_H
#define MESH_H

#include <stdint.h>
#include "protocol.h"

/* Discovery candidate from a pair response */
struct discovery_candidate {
	uint8_t device_type;
	uint16_t device_id;
	uint32_t hash;
	uint8_t hop_num;
	int16_t rssi_2;
};

#define MAX_CANDIDATES 8

void discovery_reset(void);
const struct discovery_candidate *discovery_best(void);
int discovery_count(void);
void discovery_add_response(const pair_response_packet_t *pkt, int16_t rssi_2);

#endif /* MESH_H */
