/*
 * Mesh protocol shared utilities for DECT NR+ mesh network
 */

#ifndef MESH_H
#define MESH_H

#include <stdint.h>
#include <stddef.h>
#include "packet.h"

/* Compute pairing hash */
uint32_t compute_pair_hash(uint16_t dev_id, uint32_t random_num);

/* Random number */
uint32_t next_random(void);

/* TX helpers */
int send_pair_request(uint32_t handle, uint32_t random_num);
int send_pair_response(uint32_t handle, uint16_t dst_id, uint32_t hash);
int send_pair_confirm(uint32_t handle, uint8_t status);

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
