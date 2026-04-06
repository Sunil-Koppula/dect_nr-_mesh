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
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t version_patch;
};

#define MAX_CANDIDATES 8

/* RSSI threshold for anchor-to-anchor mesh links (dBm * 2).
 * Runtime-configurable via AT+SET_RSSI_<dBm>, stored in NVM key 0x019.
 * Default: -75 dBm if not set in NVM. */
#define MESH_RSSI_DEFAULT_DBM  (-75)
#define NVS_RSSI_THRESHOLD_KEY 0x019

extern int16_t mesh_rssi_threshold_2;

/* Load RSSI threshold from NVM (call once at startup).
 * Sets mesh_rssi_threshold_2 from NVM, or default if not stored. */
void mesh_rssi_threshold_load(void);

/* Store RSSI threshold to NVM. rssi_dbm is in dBm (e.g. -50). */
int mesh_rssi_threshold_store(int16_t rssi_dbm);

void discovery_reset(void);
const struct discovery_candidate *discovery_best(void);
int discovery_count(void);
void discovery_add_response(const pair_response_packet_t *pkt, int16_t rssi_2);
const struct discovery_candidate *discovery_get(int index);

/*
 * Sort candidates by mesh priority:
 *   1. Gateway or anchor with minimum hop count
 *   2. Anchors with RSSI > -75 dBm
 * Returns the number of valid candidates after sorting/filtering.
 */
int discovery_sort_mesh(void);

/* === Neighbor table for true mesh topology === */

#define MAX_NEIGHBORS 8

struct mesh_neighbor {
	uint16_t device_id;
	uint8_t device_type;
	uint8_t hop_num;
	int16_t rssi_2;         /* last known RSSI * 2 */
	bool active;
};

/* In-memory neighbor table (populated during pairing) */
extern struct mesh_neighbor neighbor_table[MAX_NEIGHBORS];
extern int neighbor_count;

/* Add a neighbor to the table. Returns 0 on success. */
int neighbor_add(uint16_t device_id, uint8_t device_type,
		 uint8_t hop_num, int16_t rssi_2);

/* Find the neighbor with the lowest hop count (for data relay).
 * Returns NULL if no neighbors. */
const struct mesh_neighbor *neighbor_best_route(void);

/* Reset the neighbor table */
void neighbor_reset(void);

/* === Scan nearby devices === */

/* Broadcast a pair request and collect responses for 1 second.
 * Logs and displays each responding device's type, ID, hop, and RSSI.
 * tx_handle/rx_handle: PHY handles for TX/RX.
 * Returns number of devices found. */
int scan_nearby(uint32_t tx_handle, uint32_t rx_handle);

#endif /* MESH_H */
