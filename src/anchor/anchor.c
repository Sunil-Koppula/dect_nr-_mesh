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

LOG_MODULE_REGISTER(anchor, CONFIG_ANCHOR_LOG_LEVEL);

#define TX_HANDLE 1
#define RX_HANDLE 2

#define PAIR_RETRY_MAX      5
#define REDISCOVERY_INTERVAL_MS  (10 * 60 * 1000)  /* 10 minutes */

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

/* === TX helper: transmit and wait for completion === */

static int transmit_and_wait(void *data, size_t len)
{
	int err = transmit(TX_HANDLE, data, len);

	if (err) {
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	return 0;
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

		/* Drain all queued responses */
		struct rx_queue_item item;

		while (rx_queue_get(&item, K_NO_WAIT) == 0) {
			if (item.len < 1) {
				continue;
			}
			if (item.data[0] == PACKET_TYPE_PAIR_RESPONSE &&
			    item.len >= PAIR_RESPONSE_PACKET_SIZE) {
				const pair_response_packet_t *resp =
					(const pair_response_packet_t *)item.data;
				if (resp->dst_device_id != device_id) {
					continue;
				}
				discovery_add_response(resp, item.rssi_2);
				ALL_INF("Got pair response from %s ID:%d hop:%d RSSI:%d",
					device_type_str(resp->device_type),
					resp->device_id, resp->hop_num,
					item.rssi_2 / 2);
			}
		}

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
		const struct mesh_neighbor *best = neighbor_best_route();

		my_hop_num = best->hop_num + 1;

		/* Store identity in NVM (best-route neighbor as reference) */
		node_identity_t identity = {
			.device_id = device_id,
			.device_type = DEVICE_TYPE_ANCHOR,
			.parent_id = best->device_id,
			.parent_hop = best->hop_num,
		};

		err = node_store_identity(&identity);
		if (err) {
			ALL_ERR("Failed to store identity, err %d", err);
			return err;
		}

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

static void handle_pair_request(const pair_request_packet_t *pkt, int16_t rssi_2)
{
	ALL_INF("Pair request from %s ID:%d (RSSI:%d)",
		device_type_str(pkt->device_type), pkt->device_id, rssi_2 / 2);

	uint32_t hash = compute_pair_hash(pkt->device_id, pkt->random_num);

	int err = send_pair_response(TX_HANDLE, pkt->device_id, hash);
	if (err) {
		ALL_ERR("Failed to send pair response, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	ALL_INF("Pair response sent to ID:%d", pkt->device_id);
}

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

static void handle_data(const data_packet_t *pkt, uint16_t len, int16_t rssi_2)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	uint16_t payload_len = pkt->payload_len;

	/* Verify CRC */
	uint16_t rx_crc;
	memcpy(&rx_crc, &pkt->payload[payload_len], sizeof(rx_crc));
	uint16_t calc_crc = compute_crc16(pkt->payload, payload_len);
	uint8_t status = (rx_crc == calc_crc) ? STATUS_SUCCESS : STATUS_CRC_FAIL;

	if (status == STATUS_SUCCESS) {
		ALL_INF("Data from ID:%d (%d bytes, RSSI:%d) CRC OK",
			pkt->src_device_id, payload_len, rssi_2 / 2);
	} else {
		ALL_WRN("Data from ID:%d (%d bytes, RSSI:%d) CRC FAIL",
			pkt->src_device_id, payload_len, rssi_2 / 2);
	}

	/* ACK back to the sender */
	data_ack_packet_t ack = {
		.packet_type = PACKET_TYPE_DATA_ACK,
		.src_device_id = device_id,
		.dst_device_id = pkt->src_device_id,
		.hop_num = my_hop_num,
		.status = status,
	};

	int err = transmit_and_wait(&ack, sizeof(ack));

	if (err) {
		ALL_ERR("Failed to send data ACK, err %d", err);
		return;
	}

	ALL_INF("Data ACK sent to ID:%d (status:%s)", pkt->src_device_id,
		status == STATUS_SUCCESS ? "OK" : "CRC_FAIL");

	/* Only relay upstream if CRC was good */
	if (status != STATUS_SUCCESS) {
		return;
	}

	/* Relay data to ALL paired devices (skip the sender) */
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

		/* Don't relay back to the sender */
		if (dev_id == pkt->src_device_id) {
			continue;
		}

		relay->dst_device_id = dev_id;

		err = transmit_and_wait(relay_buf, relay_len);
		if (err) {
			ALL_ERR("Failed to relay data to ID:%d, err %d",
				dev_id, err);
			continue;
		}

		relayed++;
		ALL_INF("Data relayed from ID:%d to ID:%d",
			pkt->src_device_id, dev_id);
	}

	if (relayed == 0) {
		ALL_WRN("No paired devices to relay data to");
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
				handle_pair_request(
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

	/* Drain responses */
	struct rx_queue_item item;

	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		if (item.len < 1) {
			continue;
		}
		if (item.data[0] == PACKET_TYPE_PAIR_RESPONSE &&
		    item.len >= PAIR_RESPONSE_PACKET_SIZE) {
			const pair_response_packet_t *resp =
				(const pair_response_packet_t *)item.data;
			if (resp->dst_device_id != device_id) {
				continue;
			}
			discovery_add_response(resp, item.rssi_2);
		}
	}

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

		/* Skip already paired devices */
		if (paired_store_contains(&device_store, c->device_id)) {
			continue;
		}

		/* Check if store is full */
		if (paired_store_count(&device_store) >= device_store.max_entries) {
			break;
		}

		if (c->hash != expected_hash) {
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
