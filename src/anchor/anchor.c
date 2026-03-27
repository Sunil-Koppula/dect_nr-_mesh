/*
 * Anchor device logic for DECT NR+ mesh network
 *
 * Anchor is a relay node:
 *   1. First pairs with a parent (gateway or another anchor)
 *   2. Then enters RX loop — responds to pair requests from children,
 *      and relays data packets upstream to its parent
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
#include "../nvs_store.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

#define PAIR_RETRY_MAX      5

/* Parent info (loaded from NVM or set during pairing) */
static uint16_t parent_id;

/* Paired device stores */
static const paired_store_t anchor_store = {
	.nvs_base = NVS_ANCHOR_BASE,
	.max_entries = NVS_ANCHOR_MAX,
	.label = "Anchor",
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

/* === Pairing: anchor pairs with a parent (gateway or another anchor) === */

static int anchor_do_pairing(void)
{
	int err;

	for (int attempt = 0; attempt < PAIR_RETRY_MAX; attempt++) {
		ALL_INF("Pairing attempt %d/%d", attempt + 1, PAIR_RETRY_MAX);

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
				ALL_INF("Got pair response from %s ID:%d hop:%d",
					device_type_str(resp->device_type),
					resp->device_id, resp->hop_num);
			}
		}

		k_sleep(K_MSEC(10));

		if (discovery_count() == 0) {
			ALL_WRN("No responses received, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		const struct discovery_candidate *best = discovery_best();

		ALL_INF("Best candidate: %s ID:%d hop:%d RSSI:%d",
			device_type_str(best->device_type),
			best->device_id, best->hop_num, best->rssi_2 / 2);

		if (best->hash != expected_hash) {
			ALL_WRN("Hash mismatch, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		err = send_pair_confirm(TX_HANDLE, best->device_id,
				       STATUS_SUCCESS);
		if (err) {
			ALL_ERR("Failed to send pair confirm, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		/* Store identity in NVM */
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

		my_hop_num = best->hop_num + 1;
		parent_id = best->device_id;

		ALL_INF("Paired with %s ID:%d (parent hop:%d, my hop:%d)",
			device_type_str(best->device_type),
			best->device_id, best->hop_num, my_hop_num);

		return 0;
	}

	ALL_ERR("Pairing failed after %d attempts", PAIR_RETRY_MAX);
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

	if (pkt->device_type == DEVICE_TYPE_SENSOR) {
		store = &sensor_store;
	} else if (pkt->device_type == DEVICE_TYPE_ANCHOR) {
		store = &anchor_store;
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

	/* Relay data upstream to parent (payload + CRC intact) */
	uint16_t relay_len = DATA_PACKET_SIZE + payload_len + DATA_CRC_SIZE;
	uint8_t relay_buf[DATA_LEN_MAX];
	data_packet_t *relay = (data_packet_t *)relay_buf;

	relay->packet_type = PACKET_TYPE_DATA;
	relay->src_device_id = device_id;
	relay->dst_device_id = parent_id;
	relay->payload_len = payload_len;
	memcpy(relay->payload, pkt->payload, payload_len + DATA_CRC_SIZE);

	err = transmit_and_wait(relay_buf, relay_len);

	if (err) {
		ALL_ERR("Failed to relay data to parent ID:%d, err %d",
			parent_id, err);
		return;
	}

	ALL_INF("Data relayed from ID:%d to parent ID:%d",
		pkt->src_device_id, parent_id);
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
						ALL_INF("Parent ACK from ID:%d: SUCCESS",
							ack->src_device_id);
					} else {
						ALL_WRN("Parent ACK from ID:%d: CRC FAIL",
							ack->src_device_id);
					}
				}
			}
			break;

		default:
			break;
		}
	}
}

/* === Anchor entry point === */

/* AT command handler paired store pointers (defined in at_cmd.c) */
extern const void *gw_anchor_store_ptr;
extern const void *gw_sensor_store_ptr;

void anchor_main(void)
{
	/* Set paired store pointers for AT command handler */
	gw_anchor_store_ptr = &anchor_store;
	gw_sensor_store_ptr = &sensor_store;

	ALL_INF("Anchor mode started (ID:%d)", device_id);

	/* Check if already paired with a parent */
	node_identity_t identity;

	if (node_has_identity() &&
	    node_load_identity(&identity) == 0) {
		my_hop_num = identity.parent_hop + 1;
		parent_id = identity.parent_id;
		ALL_INF("Already paired with parent ID:%d (parent hop:%d, my hop:%d)",
			identity.parent_id, identity.parent_hop, my_hop_num);
	} else {
		ALL_INF("Not paired, starting discovery...");
		int err = anchor_do_pairing();
		if (err) {
			ALL_ERR("Pairing failed, err %d", err);
			return;
		}
	}

	paired_store_print(&anchor_store);
	paired_store_print(&sensor_store);

	/* RX loop — receive from children, respond and relay */
	while (true) {
		int err = receive_ms(RX_HANDLE, 1000);
		if (err) {
			ALL_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		process_queue();

		k_sleep(K_MSEC(10));
	}
}
