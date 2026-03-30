/*
 * Mesh protocol shared utilities for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/app_version.h>
#include "mesh.h"
#include "mesh_tx.h"
#include "crc.h"
#include "radio.h"
#include "identity.h"

LOG_MODULE_DECLARE(app);

/* Discovery state */
static struct discovery_candidate candidates[MAX_CANDIDATES];
static int candidate_count;

void discovery_reset(void)
{
	candidate_count = 0;
}

void discovery_add_response(const pair_response_packet_t *pkt, int16_t rssi_2)
{
	if (candidate_count >= MAX_CANDIDATES) {
		LOG_WRN("Candidate list full");
		return;
	}

	/* Sensors and anchors can only pair with gateways or anchors */
	if ((my_device_type == DEVICE_TYPE_SENSOR ||
	     my_device_type == DEVICE_TYPE_ANCHOR) &&
	    pkt->device_type == DEVICE_TYPE_SENSOR) {
		return;
	}

	struct discovery_candidate *c = &candidates[candidate_count++];
	c->device_type = pkt->device_type;
	c->device_id = pkt->device_id;
	c->hash = pkt->hash;
	c->hop_num = pkt->hop_num;
	c->rssi_2 = rssi_2;
	c->version_major = pkt->version_major;
	c->version_minor = pkt->version_minor;
	c->version_patch = pkt->version_patch;
}

const struct discovery_candidate *discovery_best(void)
{
	if (candidate_count == 0) {
		return NULL;
	}

	const struct discovery_candidate *best = &candidates[0];

	for (int i = 1; i < candidate_count; i++) {
		const struct discovery_candidate *c = &candidates[i];

		/* Gateway always wins */
		if (c->device_type == DEVICE_TYPE_GATEWAY &&
		    best->device_type != DEVICE_TYPE_GATEWAY) {
			best = c;
			continue;
		}
		if (best->device_type == DEVICE_TYPE_GATEWAY &&
		    c->device_type != DEVICE_TYPE_GATEWAY) {
			continue;
		}

		/* Prefer lower hop count */
		if (c->hop_num < best->hop_num) {
			best = c;
			continue;
		}
		if (c->hop_num > best->hop_num) {
			continue;
		}

		/* Same hop: prefer better RSSI */
		if (c->rssi_2 > best->rssi_2) {
			best = c;
		}
	}

	return best;
}

int discovery_count(void)
{
	return candidate_count;
}

const struct discovery_candidate *discovery_get(int index)
{
	if (index < 0 || index >= candidate_count) {
		return NULL;
	}
	return &candidates[index];
}

/*
 * Sort candidates by mesh priority and filter:
 *   1. Gateways and anchors with minimum hop count come first
 *   2. Then anchors with RSSI above threshold (-75 dBm)
 *   3. Discard anchors below RSSI threshold
 * Returns count of valid candidates after filtering.
 */
int discovery_sort_mesh(void)
{
	if (candidate_count == 0) {
		return 0;
	}

	/* Simple insertion sort by priority */
	for (int i = 1; i < candidate_count; i++) {
		struct discovery_candidate tmp = candidates[i];
		int j = i - 1;

		while (j >= 0) {
			struct discovery_candidate *c = &candidates[j];
			bool swap = false;

			/* Gateway always ranks higher than anchor */
			if (tmp.device_type == DEVICE_TYPE_GATEWAY &&
			    c->device_type != DEVICE_TYPE_GATEWAY) {
				swap = true;
			} else if (tmp.device_type == c->device_type) {
				/* Same type: prefer lower hop */
				if (tmp.hop_num < c->hop_num) {
					swap = true;
				} else if (tmp.hop_num == c->hop_num &&
					   tmp.rssi_2 > c->rssi_2) {
					/* Same hop: prefer better RSSI */
					swap = true;
				}
			}

			if (!swap) {
				break;
			}
			candidates[j + 1] = candidates[j];
			j--;
		}
		candidates[j + 1] = tmp;
	}

	/* Filter out anchors below RSSI threshold */
	int valid = 0;

	for (int i = 0; i < candidate_count; i++) {
		if (candidates[i].device_type == DEVICE_TYPE_GATEWAY) {
			/* Gateways always kept */
			valid++;
		} else if (candidates[i].rssi_2 >= MESH_RSSI_THRESHOLD_2) {
			valid++;
		} else {
			break; /* sorted, so remaining are worse */
		}
	}

	candidate_count = valid;
	return valid;
}

/* === Neighbor table === */

struct mesh_neighbor neighbor_table[MAX_NEIGHBORS];
int neighbor_count;

void neighbor_reset(void)
{
	memset(neighbor_table, 0, sizeof(neighbor_table));
	neighbor_count = 0;
}

int neighbor_add(uint16_t device_id, uint8_t device_type,
		 uint8_t hop_num, int16_t rssi_2)
{
	/* Check if already in table — update if so */
	for (int i = 0; i < neighbor_count; i++) {
		if (neighbor_table[i].device_id == device_id) {
			neighbor_table[i].hop_num = hop_num;
			neighbor_table[i].rssi_2 = rssi_2;
			neighbor_table[i].active = true;
			return 0;
		}
	}

	if (neighbor_count >= MAX_NEIGHBORS) {
		LOG_WRN("Neighbor table full");
		return -ENOMEM;
	}

	struct mesh_neighbor *n = &neighbor_table[neighbor_count++];

	n->device_id = device_id;
	n->device_type = device_type;
	n->hop_num = hop_num;
	n->rssi_2 = rssi_2;
	n->active = true;

	return 0;
}

const struct mesh_neighbor *neighbor_best_route(void)
{
	const struct mesh_neighbor *best = NULL;

	for (int i = 0; i < neighbor_count; i++) {
		if (!neighbor_table[i].active) {
			continue;
		}
		if (best == NULL) {
			best = &neighbor_table[i];
			continue;
		}
		/* Gateway always wins */
		if (neighbor_table[i].device_type == DEVICE_TYPE_GATEWAY &&
		    best->device_type != DEVICE_TYPE_GATEWAY) {
			best = &neighbor_table[i];
			continue;
		}
		if (best->device_type == DEVICE_TYPE_GATEWAY &&
		    neighbor_table[i].device_type != DEVICE_TYPE_GATEWAY) {
			continue;
		}
		/* Prefer lower hop count */
		if (neighbor_table[i].hop_num < best->hop_num) {
			best = &neighbor_table[i];
		} else if (neighbor_table[i].hop_num == best->hop_num &&
			   neighbor_table[i].rssi_2 > best->rssi_2) {
			best = &neighbor_table[i];
		}
	}

	return best;
}

/* CRC-16/CCITT (polynomial 0x1021) */
uint16_t compute_crc16_continue(uint16_t crc, const void *data, uint32_t len)
{
	const uint8_t *p = data;

	for (uint32_t i = 0; i < len; i++) {
		crc ^= (uint16_t)p[i] << 8;
		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ 0x1021;
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

uint16_t compute_crc16(const void *data, uint32_t len)
{
	return compute_crc16_continue(0xFFFF, data, len);
}

uint32_t compute_pair_hash(uint16_t dev_id, uint32_t random_num)
{
	uint32_t h = (uint32_t)dev_id ^ random_num;
	h = ((h << 13) | (h >> 19)) ^ (h * 0x5bd1e995);
	return h;
}

uint32_t next_random(void)
{
	return sys_rand32_get();
}

/* TX helpers */

int send_pair_request(uint32_t handle, uint32_t random_num)
{
	pair_request_packet_t pkt = {
		.packet_type = PACKET_TYPE_PAIR_REQUEST,
		.device_type = (uint8_t)my_device_type,
		.device_id = device_id,
		.random_num = random_num,
		.version_major = APP_VERSION_MAJOR,
		.version_minor = APP_VERSION_MINOR,
		.version_patch = APP_PATCHLEVEL,
	};
	return transmit(handle, &pkt, sizeof(pkt));
}

int send_pair_response(uint32_t handle, uint16_t dst_id, uint32_t hash)
{
	pair_response_packet_t pkt = {
		.packet_type = PACKET_TYPE_PAIR_RESPONSE,
		.device_type = (uint8_t)my_device_type,
		.device_id = device_id,
		.dst_device_id = dst_id,
		.hash = hash,
		.hop_num = my_hop_num,
		.version_major = APP_VERSION_MAJOR,
		.version_minor = APP_VERSION_MINOR,
		.version_patch = APP_PATCHLEVEL,
	};
	return transmit(handle, &pkt, sizeof(pkt));
}

int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t status)
{
	pair_confirm_packet_t pkt = {
		.packet_type = PACKET_TYPE_PAIR_CONFIRM,
		.device_type = (uint8_t)my_device_type,
		.device_id = device_id,
		.dst_device_id = dst_id,
		.status = status,
		.version_major = APP_VERSION_MAJOR,
		.version_minor = APP_VERSION_MINOR,
		.version_patch = APP_PATCHLEVEL,
	};
	return transmit(handle, &pkt, sizeof(pkt));
}

int send_data(uint32_t handle, uint16_t dst_id, const void *payload,
	      uint16_t payload_len)
{
	uint8_t buf[DATA_LEN_MAX];
	data_packet_t *pkt = (data_packet_t *)buf;
	uint16_t total = DATA_PACKET_SIZE + payload_len + DATA_CRC_SIZE;

	if (total > DATA_LEN_MAX) {
		return -EINVAL;
	}

	pkt->packet_type = PACKET_TYPE_DATA;
	pkt->src_device_id = device_id;
	pkt->dst_device_id = dst_id;
	pkt->payload_len = payload_len;
	if (payload_len > 0) {
		memcpy(pkt->payload, payload, payload_len);
	}

	/* Append CRC16 over payload */
	uint16_t crc = compute_crc16(pkt->payload, payload_len);
	memcpy(&pkt->payload[payload_len], &crc, sizeof(crc));

	return transmit(handle, buf, total);
}
