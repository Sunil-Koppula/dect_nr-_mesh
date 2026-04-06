/*
 * Anchor device logic for DECT NR+ mesh network
 *
 * Anchor is a mesh relay node:
 *   1. Discovers ALL reachable gateways and anchors
 *   2. Pairs with multiple neighbors to form a true mesh (not tree)
 *   3. Enters RX loop — responds to pair requests from new nodes,
 *      and relays data packets via the best-route neighbor
 *
 * Neighbor selection priority:
 *   1. Gateway or anchor with minimum hop count
 *   2. Anchors with RSSI > -75 dBm
 *
 * Same single-threaded RX/TX approach as gateway to avoid modem conflicts.
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <nrf_modem_dect_phy.h>
#include "../identity.h"
#include "../log_all.h"
#include "../protocol.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../mesh_tx.h"
#include "../crc.h"
#include "../large_data.h"
#include "../psram.h"
#include "../nvs_store.h"
#include "../at_cmd.h"
#include "../common.h"

LOG_MODULE_REGISTER(anchor, CONFIG_ANCHOR_LOG_LEVEL);

#define RX_HANDLE 2

#define PAIR_RETRY_MAX      5
#define REDISCOVERY_INTERVAL_MS  (30 * 60 * 1000)  /* 30 minutes */

/* Paired device stores */
static const paired_store_t device_store = {
	.nvs_base = NVS_ANCHOR_BASE,
	.max_entries = NVS_ANCHOR_MAX,
	.label = "Device",
};

static const paired_store_t sensor_store = {
	.nvs_base = NVS_SENSOR_BASE,
	.max_entries = NVS_SENSOR_MAX,
	.label = "Sensor",
};

/* === Recalculate hop number from neighbor table === */

static void recalculate_hop(void)
{
	const struct mesh_neighbor *best = neighbor_best_route();

	if (!best) {
		return;
	}

	uint8_t new_hop = best->hop_num + 1;

	if (new_hop != my_hop_num) {
		ALL_INF("Hop changed: %d -> %d (best neighbor ID:%d hop:%d)",
			my_hop_num, new_hop, best->device_id, best->hop_num);
		my_hop_num = new_hop;

		/* Update NVM */
		node_identity_t identity = {
			.device_id = device_id,
			.device_type = DEVICE_TYPE_ANCHOR,
			.parent_id = best->device_id,
			.parent_hop = best->hop_num,
		};
		int err = node_store_identity(&identity);

		if (err) {
			ALL_ERR("Failed to update identity, err %d", err);
		}
	}
}

/* === Downstream-only forwarding === */

static bool is_downstream(uint16_t dev_id)
{
	for (int i = 0; i < neighbor_count; i++) {
		if (neighbor_table[i].device_id == dev_id) {
			return neighbor_table[i].hop_num > my_hop_num;
		}
	}
	return false;
}

static bool is_upstream(uint16_t dev_id)
{
	for (int i = 0; i < neighbor_count; i++) {
		if (neighbor_table[i].device_id == dev_id) {
			return neighbor_table[i].hop_num < my_hop_num;
		}
	}
	return false;
}

/* Forward a packet to downstream anchors + all sensors.
 * skip_id: sender to exclude (0 = skip none).
 * dst_offset: byte offset of dst_device_id in the packet. */
static void forward_downstream(void *pkt, size_t pkt_size,
			       size_t dst_offset, uint16_t skip_id,
			       const char *label)
{
	uint8_t *buf = (uint8_t *)pkt;

	/* Downstream anchors only */
	for (int i = 0; i < device_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&device_store, i, &dev_id) != 0) {
			continue;
		}
		if (dev_id == skip_id || !is_downstream(dev_id)) {
			continue;
		}

		memcpy(buf + dst_offset, &dev_id, sizeof(dev_id));
		int err = transmit_and_wait(pkt, pkt_size);

		if (err) {
			ALL_ERR("Failed to fwd %s to ID:%d", label, dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* All paired sensors */
	for (int i = 0; i < sensor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&sensor_store, i, &dev_id) != 0) {
			continue;
		}

		memcpy(buf + dst_offset, &dev_id, sizeof(dev_id));
		int err = transmit_and_wait(pkt, pkt_size);

		if (err) {
			ALL_ERR("Failed to fwd %s to sensor ID:%d",
				label, dev_id);
		}
		k_sleep(K_MSEC(10));
	}
}

/* === Mesh pairing: discover and pair with ALL valid neighbors === */

static int anchor_do_mesh_pairing(void)
{
	int err;

	for (int attempt = 0; attempt < PAIR_RETRY_MAX; attempt++) {
		ALL_INF("Mesh pairing attempt %d/%d", attempt + 1,
			PAIR_RETRY_MAX);

		uint32_t rand_num = next_random();
		uint32_t expected_hash = compute_pair_hash(device_id, rand_num);

		err = send_pair_request(TX_HANDLE, rand_num);
		if (err) {
			ALL_ERR("Failed to send pair request, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		ALL_INF("Pair request sent, listening for responses...");

		discovery_reset();

		err = receive_ms(RX_HANDLE, 1000);
		if (err) {
			ALL_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		drain_discovery_responses(true);

		k_sleep(K_MSEC(10));

		if (discovery_count() == 0) {
			ALL_WRN("No responses received, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		/* Sort and filter candidates by mesh priority */
		int valid = discovery_sort_mesh();

		ALL_INF("Found %d valid mesh neighbors", valid);

		if (valid == 0) {
			ALL_WRN("No candidates above RSSI threshold, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		/* Pair with ALL valid candidates */
		neighbor_reset();
		int paired = 0;

		for (int i = 0; i < valid; i++) {
			const struct discovery_candidate *c = discovery_get(i);

			if (!c) {
				continue;
			}

			if (c->hash != expected_hash) {
				ALL_WRN("Hash mismatch for ID:%d, skipping",
					c->device_id);
				continue;
			}

			err = send_pair_confirm(TX_HANDLE, c->device_id,
						STATUS_SUCCESS);
			if (err) {
				ALL_ERR("Failed to send pair confirm to ID:%d, err %d",
					c->device_id, err);
				continue;
			}
			k_sem_take(&operation_sem, K_FOREVER);

			/* Add to neighbor table */
			err = neighbor_add(c->device_id, c->device_type,
					   c->hop_num, c->rssi_2);
			if (err) {
				ALL_ERR("Failed to add neighbor ID:%d, err %d",
					c->device_id, err);
				continue;
			}

			/* Store in NVS paired device store with version */
			err = paired_store_add(&device_store, c->device_id,
					       c->device_type,
					       c->version_major,
					       c->version_minor,
					       c->version_patch);
			if (err) {
				ALL_ERR("Failed to store neighbor ID:%d in NVS, err %d",
					c->device_id, err);
			}

			paired++;
			ALL_INF("Paired with %s ID:%d (hop:%d, RSSI:%d)",
				device_type_str(c->device_type),
				c->device_id, c->hop_num, c->rssi_2 / 2);
		}

		if (paired == 0) {
			ALL_WRN("No successful pairings, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		/* Determine my hop from the best (lowest hop) neighbor */
		recalculate_hop();

		ALL_INF("Mesh pairing complete: %d neighbors, my hop:%d",
			paired, my_hop_num);

		for (int i = 0; i < neighbor_count; i++) {
			struct mesh_neighbor *n = &neighbor_table[i];

			ALL_INF("  Neighbor[%d]: %s ID:%d hop:%d RSSI:%d",
				i, device_type_str(n->device_type),
				n->device_id, n->hop_num, n->rssi_2 / 2);
		}

		return 0;
	}

	ALL_ERR("Mesh pairing failed after %d attempts", PAIR_RETRY_MAX);
	return -ETIMEDOUT;
}

/* === Packet handlers (RX loop, after paired) === */

static void handle_pair_confirm(const pair_confirm_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("Pair confirm from %s ID:%d status:%s",
		device_type_str(pkt->device_type), pkt->device_id,
		(pkt->status == STATUS_SUCCESS) ? "SUCCESS" : "FAILURE");

	if (pkt->status != STATUS_SUCCESS) {
		return;
	}

	const paired_store_t *store;

	if (pkt->device_type == DEVICE_TYPE_GATEWAY) {
		store = &device_store;
		neighbor_add(pkt->device_id, pkt->device_type, 0, 0);
	} else if (pkt->device_type == DEVICE_TYPE_ANCHOR) {
		store = &device_store;
		/* Also add to neighbor table for mesh routing */
		neighbor_add(pkt->device_id, pkt->device_type,
			     my_hop_num + 1, 0);
	} else if (pkt->device_type == DEVICE_TYPE_SENSOR) {
		store = &sensor_store;
	} else {
		ALL_WRN("Unexpected device type %d in pair confirm",
			pkt->device_type);
		return;
	}

	int err = paired_store_add(store, pkt->device_id,
				   pkt->device_type,
				   pkt->version_major, pkt->version_minor,
				   pkt->version_patch);

	if (err) {
		ALL_ERR("Failed to store %s ID:%d, err %d",
			store->label, pkt->device_id, err);
	} else {
		ALL_INF("%s ID:%d v%d.%d.%d paired and stored in NVM",
			store->label, pkt->device_id,
			pkt->version_major, pkt->version_minor,
			pkt->version_patch);
	}
}

static void handle_data_request(const data_request_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("DATA_REQUEST from ID:%d, forwarding downstream...",
		pkt->src_device_id);

	data_request_packet_t fwd = {
		.packet_type = PACKET_TYPE_DATA_REQUEST,
		.src_device_id = device_id,
		.request_type = pkt->request_type,
	};

	forward_downstream(&fwd, sizeof(fwd),
			   offsetof(data_request_packet_t, dst_device_id),
			   pkt->src_device_id, "DATA_REQUEST");
}

/* Send a PARENT_RESPONSE for a device back upstream */
static void send_parent_response(uint16_t to_id, uint16_t about_id,
				 uint8_t dev_type, uint8_t hop)
{
	node_identity_t id;
	uint16_t parent_id = device_id;  /* default: this anchor is the parent */
	uint8_t parent_type = DEVICE_TYPE_ANCHOR;

	/* If querying about ourselves, report our actual parent */
	if (about_id == device_id && node_load_identity(&id) == 0) {
		parent_id = id.parent_id;
		parent_type = (id.parent_hop == 0)
			? DEVICE_TYPE_GATEWAY : DEVICE_TYPE_ANCHOR;
	}

	parent_response_packet_t resp = {
		.packet_type = PACKET_TYPE_PARENT_RESPONSE,
		.src_device_id = about_id,
		.dst_device_id = to_id,
		.device_type = dev_type,
		.parent_id = parent_id,
		.parent_type = parent_type,
		.hop_num = hop,
	};

	int err = transmit_and_wait(&resp, sizeof(resp));

	if (err) {
		ALL_ERR("Failed to send PARENT_RESPONSE for ID:%d", about_id);
	}
	k_sleep(K_MSEC(10));
}

static void handle_parent_query(const parent_query_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	uint16_t target = pkt->target_id;
	bool query_all = (target == 0);

	ALL_INF("PARENT_QUERY from ID:%d target:%s",
		pkt->src_device_id,
		query_all ? "ALL" : "specific");

	/* Always respond with own parent info */
	if (query_all || target == device_id) {
		node_identity_t id;

		if (node_load_identity(&id) == 0) {
			send_parent_response(pkt->src_device_id, device_id,
					     DEVICE_TYPE_ANCHOR, my_hop_num);
		}
		if (!query_all && target == device_id) {
			return;  /* Found — no need to forward */
		}
	}

	/* Check if the specific target is in our sensor_store */
	if (!query_all && paired_store_contains(&sensor_store, target)) {
		ALL_INF("SENSOR ID:%d found locally, responding", target);
		send_parent_response(pkt->src_device_id, target,
				     DEVICE_TYPE_SENSOR, my_hop_num);
		return;  /* Found — no need to forward */
	}

	/* Check if the specific target is in our device_store (child anchor) */
	if (!query_all && paired_store_contains(&device_store, target) &&
	    is_downstream(target)) {
		ALL_INF("ANCHOR ID:%d found locally, responding", target);
		send_parent_response(pkt->src_device_id, target,
				     DEVICE_TYPE_ANCHOR, my_hop_num + 1);
		return;  /* Found — no need to forward */
	}

	/* Not found locally — forward query.
	 * query_all: downstream only (hop > my_hop).
	 * specific: peers + downstream (hop >= my_hop) so the query
	 * can reach devices behind same-level anchors. Skip sender
	 * and upstream to prevent loops back to gateway. */
	for (int i = 0; i < device_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&device_store, i, &dev_id) != 0) {
			continue;
		}
		if (dev_id == pkt->src_device_id) {
			continue;
		}

		bool downstream_only = query_all;

		if (downstream_only && !is_downstream(dev_id)) {
			continue;
		}
		if (!downstream_only && is_upstream(dev_id)) {
			continue;
		}

		parent_query_packet_t fwd = {
			.packet_type = PACKET_TYPE_PARENT_QUERY,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
			.target_id = target,
		};
		transmit_and_wait(&fwd, sizeof(fwd));
		k_sleep(K_MSEC(10));
	}

	/* Always forward to all paired sensors */
	for (int i = 0; i < sensor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&sensor_store, i, &dev_id) != 0) {
			continue;
		}

		parent_query_packet_t fwd = {
			.packet_type = PACKET_TYPE_PARENT_QUERY,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
			.target_id = target,
		};
		transmit_and_wait(&fwd, sizeof(fwd));
		k_sleep(K_MSEC(10));
	}
}

static void handle_parent_response(const parent_response_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("PARENT_RESPONSE from %s ID:%d (parent:%d hop:%d), relaying...",
		device_type_str(pkt->device_type),
		pkt->src_device_id, pkt->parent_id, pkt->hop_num);

	/* Relay to best-route neighbor (towards gateway) */
	const struct mesh_neighbor *best = neighbor_best_route();

	if (!best) {
		ALL_ERR("No route to relay PARENT_RESPONSE");
		return;
	}

	parent_response_packet_t relay = {
		.packet_type = PACKET_TYPE_PARENT_RESPONSE,
		.src_device_id = pkt->src_device_id,
		.dst_device_id = best->device_id,
		.device_type = pkt->device_type,
		.parent_id = pkt->parent_id,
		.parent_type = pkt->parent_type,
		.hop_num = pkt->hop_num,
	};

	int err = transmit_and_wait(&relay, sizeof(relay));

	if (err) {
		ALL_ERR("Failed to relay PARENT_RESPONSE to ID:%d",
			best->device_id);
	}
}

static void send_repair_and_reset(uint16_t skip_id)
{
	ALL_INF("Broadcasting REPAIR downstream...");

	repair_packet_t fwd = {
		.packet_type = PACKET_TYPE_REPAIR,
		.src_device_id = device_id,
	};

	forward_downstream(&fwd, sizeof(fwd),
			   offsetof(repair_packet_t, dst_device_id),
			   skip_id, "REPAIR");

	factory_reset_reboot();
}

static void handle_repair(const repair_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("REPAIR from ID:%d", pkt->src_device_id);
	send_repair_and_reset(pkt->src_device_id);
}

static void handle_set_rssi(const set_rssi_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("SET_RSSI %d dBm from ID:%d", pkt->rssi_dbm,
		pkt->src_device_id);

	set_rssi_packet_t fwd = {
		.packet_type = PACKET_TYPE_SET_RSSI,
		.src_device_id = device_id,
		.rssi_dbm = pkt->rssi_dbm,
	};

	forward_downstream(&fwd, sizeof(fwd),
			   offsetof(set_rssi_packet_t, dst_device_id),
			   pkt->src_device_id, "SET_RSSI");

	rssi_store_and_reboot(pkt->rssi_dbm);
}

static void handle_data(const data_packet_t *pkt, uint16_t len, int16_t rssi_2)
{
	int status = common_data_verify_and_ack(pkt, len, rssi_2);
	if (status != STATUS_SUCCESS) {
		return;
	}

	/* Relay data upstream only (to neighbors with lower hop) */
	uint16_t payload_len = pkt->payload_len;
	uint16_t relay_len = DATA_PACKET_SIZE + payload_len + DATA_CRC_SIZE;
	uint8_t relay_buf[DATA_LEN_MAX];
	data_packet_t *relay = (data_packet_t *)relay_buf;

	relay->packet_type = PACKET_TYPE_DATA;
	relay->src_device_id = device_id;
	relay->payload_len = payload_len;
	memcpy(relay->payload, pkt->payload, payload_len + DATA_CRC_SIZE);

	int relayed = 0;

	for (int i = 0; i < device_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&device_store, i, &dev_id) != 0) {
			continue;
		}
		if (dev_id == pkt->src_device_id) {
			continue;
		}
		if (!is_upstream(dev_id)) {
			continue;
		}

		relay->dst_device_id = dev_id;

		int err = transmit_and_wait(relay_buf, relay_len);
		if (err) {
			ALL_ERR("Failed to relay data to ID:%d, err %d",
				dev_id, err);
			continue;
		}

		relayed++;
		ALL_INF("Data relayed upstream to ID:%d", dev_id);
	}

	if (relayed == 0) {
		ALL_WRN("No upstream neighbors to relay data to");
	}
}

/* === Process all queued packets (called when RX window ends) === */

static void process_queue(void)
{
	struct rx_queue_item item;

	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		if (item.len < 1) {
			continue;
		}

		uint8_t pkt_type = item.data[0];

		switch (pkt_type) {
		case PACKET_TYPE_PAIR_REQUEST:
			if (item.len >= PAIR_REQUEST_PACKET_SIZE) {
				common_handle_pair_request(
					(const pair_request_packet_t *)item.data,
					item.rssi_2);
			}
			break;

		case PACKET_TYPE_PAIR_CONFIRM:
			if (item.len >= PAIR_CONFIRM_PACKET_SIZE) {
				handle_pair_confirm(
					(const pair_confirm_packet_t *)item.data);
			}
			break;

		case PACKET_TYPE_DATA:
			if (item.len >= DATA_PACKET_SIZE) {
				handle_data(
					(const data_packet_t *)item.data,
					item.len, item.rssi_2);
			}
			break;

		case PACKET_TYPE_DATA_ACK:
			if (item.len >= DATA_ACK_PACKET_SIZE) {
				const data_ack_packet_t *ack =
					(const data_ack_packet_t *)item.data;
				if (ack->dst_device_id == device_id) {
					if (ack->status == STATUS_SUCCESS) {
						ALL_INF("Neighbor ACK from ID:%d: SUCCESS",
							ack->src_device_id);
					} else {
						ALL_WRN("Neighbor ACK from ID:%d: CRC FAIL",
							ack->src_device_id);
					}
				}
			}
			break;

		case PACKET_TYPE_LARGE_DATA_INIT:
		case PACKET_TYPE_LARGE_DATA_TRANSFER:
		case PACKET_TYPE_LARGE_DATA_END:
			/* Handled by PSRAM writer thread via ring buffer */
			break;

		case PACKET_TYPE_LARGE_DATA_ACK:
		case PACKET_TYPE_LARGE_DATA_NACK:
			/* Handled during large_data_send() own RX loop */
			break;

		case PACKET_TYPE_DATA_REQUEST:
			if (item.len >= DATA_REQUEST_PACKET_SIZE) {
				handle_data_request(
					(const data_request_packet_t *)
						item.data);
			}
			break;

		case PACKET_TYPE_PARENT_QUERY:
			if (item.len >= PARENT_QUERY_PACKET_SIZE) {
				handle_parent_query(
					(const parent_query_packet_t *)
						item.data);
			}
			break;

		case PACKET_TYPE_PARENT_RESPONSE:
			if (item.len >= PARENT_RESPONSE_PACKET_SIZE) {
				handle_parent_response(
					(const parent_response_packet_t *)
						item.data);
			}
			break;

		case PACKET_TYPE_REPAIR:
			if (item.len >= REPAIR_PACKET_SIZE) {
				handle_repair(
					(const repair_packet_t *)item.data);
			}
			break;

		case PACKET_TYPE_SET_RSSI:
			if (item.len >= SET_RSSI_PACKET_SIZE) {
				handle_set_rssi(
					(const set_rssi_packet_t *)item.data);
			}
			break;

		default:
			break;
		}
	}
}

/* === Periodic re-discovery: single attempt to find new neighbors === */

static void anchor_rediscovery(void)
{
	int device_count = paired_store_count(&device_store);

	if (device_count >= device_store.max_entries) {
		ALL_INF("Device store full (%d/%d), skipping rediscovery",
			device_count, device_store.max_entries);
		return;
	}

	ALL_INF("Rediscovery: %d/%d device slots used, searching...",
		device_count, device_store.max_entries);

	uint32_t rand_num = next_random();
	uint32_t expected_hash = compute_pair_hash(device_id, rand_num);

	int err = send_pair_request(TX_HANDLE, rand_num);

	if (err) {
		ALL_ERR("Rediscovery: failed to send pair request, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	discovery_reset();

	err = receive_ms(RX_HANDLE, 1000);
	if (err) {
		ALL_ERR("Rediscovery: receive failed, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	drain_discovery_responses(false);

	if (discovery_count() == 0) {
		ALL_INF("Rediscovery: no new neighbors found");
		return;
	}

	int valid = discovery_sort_mesh();
	int new_paired = 0;

	for (int i = 0; i < valid; i++) {
		const struct discovery_candidate *c = discovery_get(i);

		if (!c) {
			continue;
		}

		if (c->hash != expected_hash) {
			continue;
		}

		/* Already paired: just update neighbor hop/RSSI */
		if (paired_store_contains(&device_store, c->device_id)) {
			neighbor_add(c->device_id, c->device_type,
				     c->hop_num, c->rssi_2);
			ALL_INF("Rediscovery: updated neighbor %s ID:%d (hop:%d, RSSI:%d)",
				device_type_str(c->device_type),
				c->device_id, c->hop_num, c->rssi_2 / 2);
			continue;
		}

		/* Check if store is full for new devices */
		if (paired_store_count(&device_store) >= device_store.max_entries) {
			continue;
		}

		err = send_pair_confirm(TX_HANDLE, c->device_id,
					STATUS_SUCCESS);
		if (err) {
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		neighbor_add(c->device_id, c->device_type,
			     c->hop_num, c->rssi_2);

		paired_store_add(&device_store, c->device_id,
				 c->device_type,
				 c->version_major, c->version_minor,
				 c->version_patch);

		new_paired++;
		ALL_INF("Rediscovery: paired with %s ID:%d (hop:%d, RSSI:%d)",
			device_type_str(c->device_type),
			c->device_id, c->hop_num, c->rssi_2 / 2);
	}

	ALL_INF("Rediscovery: %d new neighbors paired (%d/%d slots used)",
		new_paired, paired_store_count(&device_store),
		device_store.max_entries);

	/* Recalculate hop from all neighbors (including newly discovered) */
	recalculate_hop();
}

/* === Anchor entry point === */

/* AT command handler paired store pointers (defined in at_cmd.c) */
extern const void *gw_anchor_store_ptr;
extern const void *gw_sensor_store_ptr;

void anchor_main(void)
{
	/* Set paired store pointers for AT command handler */
	gw_anchor_store_ptr = &device_store;  /* gateways + anchors */
	gw_sensor_store_ptr = &sensor_store;

	ALL_INF("Anchor mode started (ID:%d)", device_id);

	/* Check if already paired */
	node_identity_t identity;

	if (node_has_identity() &&
	    node_load_identity(&identity) == 0) {
		my_hop_num = identity.parent_hop + 1;

		/* Restore best-route neighbor from NVM as initial entry */
		neighbor_reset();
		neighbor_add(identity.parent_id, DEVICE_TYPE_UNKNOWN,
			     identity.parent_hop, 0);

		/* Ensure best-route neighbor is in paired store */
		if (!paired_store_contains(&device_store, identity.parent_id)) {
			paired_store_add(&device_store, identity.parent_id,
					 DEVICE_TYPE_UNKNOWN,
					 0, 0, 0);
		}

		ALL_INF("Restored from NVM: best-route ID:%d (hop:%d, my hop:%d)",
			identity.parent_id, identity.parent_hop, my_hop_num);
	} else {
		ALL_INF("Not paired, starting mesh discovery...");
		int err = anchor_do_mesh_pairing();
		if (err) {
			ALL_ERR("Mesh pairing failed, err %d", err);
			return;
		}
	}

	paired_store_print(&device_store);
	paired_store_print(&sensor_store);
	ALL_INF("  Button 4: scan nearby devices");

	/* RX loop — receive from neighbors, respond, relay, and handle
	 * large data transfers. Periodically re-discover new neighbors.
	 */
	int64_t last_rediscovery = k_uptime_get();

	while (true) {
		if (k_sem_take(&btn4_sem, K_NO_WAIT) == 0) {
			scan_nearby(TX_HANDLE, RX_HANDLE);
		}

		if (k_sem_take(&repair_sem, K_NO_WAIT) == 0) {
			send_repair_and_reset(0);
			/* Does not return — reboots */
		}

		int err = receive_ms(RX_HANDLE, 1000);
		if (err) {
			ALL_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Wait for RX window to end, or break early if large data
		 * END arrives (need to send ACK promptly) */
		bool cancelled = false;

		while (true) {
			if (k_sem_take(&operation_sem, K_MSEC(10)) == 0) {
				break;
			}
			if (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
				nrf_modem_dect_phy_cancel(RX_HANDLE);
				k_sem_take(&operation_sem, K_FOREVER);
				if (k_sem_take(&operation_sem, K_MSEC(100)) != 0) {
					k_sleep(K_MSEC(50));
				}
				k_sem_reset(&operation_sem);
				cancelled = true;
				break;
			}
		}

		process_queue();
		large_data_send_pending_ack(TX_HANDLE);

		/* Relay completed large data to best-route neighbor
		 * (single route, NOT mesh flood — too expensive for large data) */
		if (!large_data_any_active_session()) {
			uint8_t ld_slot;
			uint32_t ld_size;
			uint8_t ld_file_type;
			uint16_t ld_src_id;

			if (large_data_get_completed(&ld_slot, &ld_size,
						     &ld_file_type,
						     &ld_src_id)) {
				const struct mesh_neighbor *best =
					neighbor_best_route();

				if (!best) {
					ALL_ERR("No route for large data relay");
					large_data_free_completed(ld_src_id);
				} else {
					ALL_INF("Relaying %d bytes from ID:%d "
						"to best-route ID:%d",
						ld_size, ld_src_id,
						best->device_id);

					uint8_t *relay_buf = k_malloc(ld_size);

					if (!relay_buf) {
						ALL_ERR("Failed to alloc %d "
							"bytes for relay",
							ld_size);
						large_data_free_completed(
							ld_src_id);
					} else {
						uint32_t addr =
							(uint32_t)ld_slot *
							LARGE_DATA_SLOT_SIZE;
						int rerr = psram_read(
							addr, relay_buf,
							ld_size);

						if (rerr) {
							ALL_ERR("PSRAM read "
								"failed, err %d",
								rerr);
						} else {
							rerr = large_data_send(
								TX_HANDLE,
								RX_HANDLE,
								best->device_id,
								ld_file_type,
								relay_buf,
								ld_size);
							if (rerr) {
								ALL_ERR("Relay "
									"failed, "
									"err %d",
									rerr);
							} else {
								ALL_INF("Large "
									"data "
									"relay OK");
							}
						}
						k_free(relay_buf);
						large_data_free_completed(
							ld_src_id);
					}
				}
			}
		}

		large_data_cleanup_stale_sessions();

		/* Drain extra end_sem gives */
		while (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
		}

		if (cancelled) {
			k_sleep(K_MSEC(10));
			k_sem_reset(&operation_sem);
		}

		/* Periodic rediscovery every 10 minutes */
		if (paired_store_count(&device_store) < device_store.max_entries &&
		    (k_uptime_get() - last_rediscovery) >= REDISCOVERY_INTERVAL_MS) {
			anchor_rediscovery();
			last_rediscovery = k_uptime_get();
		}

		k_sleep(K_MSEC(10));
	}
}
