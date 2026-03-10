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
#include "anchor.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../state.h"
#include "../storage.h"

LOG_MODULE_DECLARE(app);

#define ANCHOR_TX_HANDLE 1
#define ANCHOR_RX_HANDLE 2

#define PAIR_RETRY_MAX      5

/* Parent info (loaded from NVM or set during pairing) */
static uint16_t parent_id;

/* === Anchor storage helpers === */

int anchor_store_identity(const anchor_identity_t *id)
{
	return storage_write(ANCHOR_IDENTITY_KEY, id, sizeof(*id));
}

int anchor_load_identity(anchor_identity_t *id)
{
	return storage_read(ANCHOR_IDENTITY_KEY, id, sizeof(*id));
}

bool anchor_has_identity(void)
{
	return storage_exists(ANCHOR_IDENTITY_KEY);
}

static int find_slot(uint16_t base, int max, uint16_t dev_id, bool *found)
{
	uint16_t stored;
	int first_empty = -1;

	*found = false;
	for (int i = 0; i < max; i++) {
		if (storage_read(base + i, &stored, sizeof(stored)) == 0) {
			if (stored == dev_id) {
				*found = true;
				return i;
			}
		} else if (first_empty < 0) {
			first_empty = i;
		}
	}
	return first_empty;
}

int anchor_store_anchor(uint16_t anchor_id)
{
	bool found;
	int slot = find_slot(ANCHOR_ANCHOR_BASE, ANCHOR_ANCHOR_MAX, anchor_id, &found);
	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("Anchor storage full");
		return -ENOMEM;
	}
	return storage_write(ANCHOR_ANCHOR_BASE + slot, &anchor_id, sizeof(anchor_id));
}

int anchor_store_sensor(uint16_t sensor_id)
{
	bool found;
	int slot = find_slot(ANCHOR_SENSOR_BASE, ANCHOR_SENSOR_MAX, sensor_id, &found);
	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("Sensor storage full");
		return -ENOMEM;
	}
	return storage_write(ANCHOR_SENSOR_BASE + slot, &sensor_id, sizeof(sensor_id));
}

bool anchor_is_anchor_paired(uint16_t anchor_id)
{
	bool found;
	find_slot(ANCHOR_ANCHOR_BASE, ANCHOR_ANCHOR_MAX, anchor_id, &found);
	return found;
}

bool anchor_is_sensor_paired(uint16_t sensor_id)
{
	bool found;
	find_slot(ANCHOR_SENSOR_BASE, ANCHOR_SENSOR_MAX, sensor_id, &found);
	return found;
}

int anchor_get_anchor_count(void)
{
	int count = 0;
	uint16_t tmp;
	for (int i = 0; i < ANCHOR_ANCHOR_MAX; i++) {
		if (storage_read(ANCHOR_ANCHOR_BASE + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

int anchor_get_sensor_count(void)
{
	int count = 0;
	uint16_t tmp;
	for (int i = 0; i < ANCHOR_SENSOR_MAX; i++) {
		if (storage_read(ANCHOR_SENSOR_BASE + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

void anchor_print_paired(void)
{
	uint16_t id;

	anchor_identity_t self;
	if (anchor_load_identity(&self) == 0) {
		LOG_INF("Anchor: ID:%d parent:%d parent_hop:%d my_hop:%d",
			self.device_id, self.parent_id, self.parent_hop,
			self.parent_hop + 1);
	}

	LOG_INF("=== Child Anchors (%d) ===", anchor_get_anchor_count());
	for (int i = 0; i < ANCHOR_ANCHOR_MAX; i++) {
		if (storage_read(ANCHOR_ANCHOR_BASE + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] Anchor ID:%d", i, id);
		}
	}

	LOG_INF("=== Child Sensors (%d) ===", anchor_get_sensor_count());
	for (int i = 0; i < ANCHOR_SENSOR_MAX; i++) {
		if (storage_read(ANCHOR_SENSOR_BASE + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] Sensor ID:%d", i, id);
		}
	}
}

/* === Pairing: anchor pairs with a parent (gateway or another anchor) === */

static int anchor_do_pairing(void)
{
	int err;

	for (int attempt = 0; attempt < PAIR_RETRY_MAX; attempt++) {
		LOG_INF("Pairing attempt %d/%d", attempt + 1, PAIR_RETRY_MAX);

		uint32_t rand_num = next_random();
		uint32_t expected_hash = compute_pair_hash(device_id, rand_num);

		err = send_pair_request(ANCHOR_TX_HANDLE, rand_num);
		if (err) {
			LOG_ERR("Failed to send pair request, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		LOG_INF("Pair request sent, listening for responses...");

		discovery_reset();

		err = receive(ANCHOR_RX_HANDLE);
		if (err) {
			LOG_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}

		/* Wait for RX window to complete naturally */
		k_sem_take(&operation_sem, K_FOREVER);

		/* RX done — drain all queued responses */
		struct rx_queue_item item;

		while (rx_queue_get(&item, K_NO_WAIT) == 0) {
			if (item.len < 1) {
				continue;
			}

			if (item.data[0] == PACKET_TYPE_PAIR_RESPONSE &&
			    item.len >= PAIR_RESPONSE_PACKET_SIZE) {
				const pair_response_packet_t *resp =
					(const pair_response_packet_t *)item.data;
				/* Only accept responses addressed to us */
				if (resp->dst_device_id != device_id) {
					continue;
				}
				discovery_add_response(resp, item.rssi_2);
				LOG_INF("Got pair response from %s ID:%d hop:%d",
					device_type_str(resp->device_type),
					resp->device_id, resp->hop_num);
			}
		}

		k_sleep(K_MSEC(10));

		if (discovery_count() == 0) {
			LOG_WRN("No responses received, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		const struct discovery_candidate *best = discovery_best();

		LOG_INF("Best candidate: %s ID:%d hop:%d RSSI:%d",
			device_type_str(best->device_type),
			best->device_id, best->hop_num, best->rssi_2 / 2);

		if (best->hash != expected_hash) {
			LOG_WRN("Hash mismatch, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		err = send_pair_confirm(ANCHOR_TX_HANDLE, best->device_id,
				       PAIR_STATUS_SUCCESS);
		if (err) {
			LOG_ERR("Failed to send pair confirm, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		/* Store identity in NVM */
		anchor_identity_t identity = {
			.device_id = device_id,
			.device_type = DEVICE_TYPE_ANCHOR,
			.parent_id = best->device_id,
			.parent_hop = best->hop_num,
		};

		err = anchor_store_identity(&identity);
		if (err) {
			LOG_ERR("Failed to store identity, err %d", err);
			return err;
		}

		my_hop_num = best->hop_num + 1;
		parent_id = best->device_id;

		LOG_INF("Paired with %s ID:%d (parent hop:%d, my hop:%d)",
			device_type_str(best->device_type),
			best->device_id, best->hop_num, my_hop_num);

		return 0;
	}

	LOG_ERR("Pairing failed after %d attempts", PAIR_RETRY_MAX);
	return -ETIMEDOUT;
}

/* === Packet handlers (RX loop, after paired) === */

static void handle_pair_request(const pair_request_packet_t *pkt, int16_t rssi_2)
{
	LOG_INF("Pair request from %s ID:%d (RSSI:%d)",
		device_type_str(pkt->device_type), pkt->device_id, rssi_2 / 2);

	uint32_t hash = compute_pair_hash(pkt->device_id, pkt->random_num);

	int err = send_pair_response(ANCHOR_TX_HANDLE, pkt->device_id, hash);
	if (err) {
		LOG_ERR("Failed to send pair response, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	LOG_INF("Pair response sent to ID:%d", pkt->device_id);
}

static void handle_pair_confirm(const pair_confirm_packet_t *pkt, int16_t rssi_2)
{
	/* Only accept confirms addressed to us */
	if (pkt->dst_device_id != device_id) {
		return;
	}

	LOG_INF("Pair confirm from %s ID:%d status:%s",
		device_type_str(pkt->device_type), pkt->device_id,
		(pkt->status == PAIR_STATUS_SUCCESS) ? "SUCCESS" : "FAILURE");

	if (pkt->status != PAIR_STATUS_SUCCESS) {
		return;
	}

	int err;

	if (pkt->device_type == DEVICE_TYPE_SENSOR) {
		err = anchor_store_sensor(pkt->device_id);
		if (err) {
			LOG_ERR("Failed to store sensor ID:%d, err %d",
				pkt->device_id, err);
		} else {
			LOG_INF("Sensor ID:%d paired and stored in NVM",
				pkt->device_id);
		}
	} else if (pkt->device_type == DEVICE_TYPE_ANCHOR) {
		err = anchor_store_anchor(pkt->device_id);
		if (err) {
			LOG_ERR("Failed to store anchor ID:%d, err %d",
				pkt->device_id, err);
		} else {
			LOG_INF("Anchor ID:%d paired and stored in NVM",
				pkt->device_id);
		}
	} else {
		LOG_WRN("Unexpected device type %d in pair confirm",
			pkt->device_type);
	}
}

static void handle_data(const data_packet_t *pkt, uint16_t len, int16_t rssi_2)
{
	uint16_t payload_len = len - DATA_PACKET_SIZE;

	LOG_INF("Data from ID:%d -> ID:%d (%d bytes, RSSI:%d)",
		pkt->src_device_id, pkt->dst_device_id,
		payload_len, rssi_2 / 2);

	/* If data is addressed to us, ACK it and relay upstream to parent */
	if (pkt->dst_device_id == device_id) {
		/* ACK back to the sender */
		data_ack_packet_t ack = {
			.packet_type = PACKET_TYPE_DATA_ACK,
			.src_device_id = device_id,
			.dst_device_id = pkt->src_device_id,
			.hop_num = my_hop_num,
			.status = 0,
		};

		int err = transmit(ANCHOR_TX_HANDLE, &ack, sizeof(ack));
		if (err) {
			LOG_ERR("Failed to send data ACK, err %d", err);
			return;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		LOG_DBG("Data ACK sent to ID:%d", pkt->src_device_id);

		/* Relay data upstream to parent */
		uint8_t relay_buf[DATA_LEN_MAX];
		data_packet_t *relay = (data_packet_t *)relay_buf;

		relay->packet_type = PACKET_TYPE_DATA;
		relay->src_device_id = pkt->src_device_id;
		relay->dst_device_id = parent_id;
		if (payload_len > 0) {
			memcpy(relay->payload, pkt->payload, payload_len);
		}

		err = transmit(ANCHOR_TX_HANDLE, relay_buf,
			       DATA_PACKET_SIZE + payload_len);
		if (err) {
			LOG_ERR("Failed to relay data to parent ID:%d, err %d",
				parent_id, err);
			return;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		LOG_INF("Data relayed from ID:%d to parent ID:%d",
			pkt->src_device_id, parent_id);
	}
}

/* === Process all queued packets (called when RX window ends) === */

static void anchor_process_queue(void)
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
					(const pair_confirm_packet_t *)item.data,
					item.rssi_2);
			}
			break;

		case PACKET_TYPE_DATA:
			if (item.len >= DATA_PACKET_SIZE) {
				handle_data(
					(const data_packet_t *)item.data,
					item.len, item.rssi_2);
			}
			break;

		default:
			LOG_WRN("Unknown packet type 0x%02x", pkt_type);
			break;
		}
	}
}

/* === Anchor entry point === */

void anchor_main(void)
{
	LOG_INF("Anchor mode started (ID:%d)", device_id);

	/* Check if already paired with a parent */
	anchor_identity_t identity;

	if (anchor_has_identity() &&
	    anchor_load_identity(&identity) == 0) {
		my_hop_num = identity.parent_hop + 1;
		parent_id = identity.parent_id;
		LOG_INF("Already paired with parent ID:%d (parent hop:%d, my hop:%d)",
			identity.parent_id, identity.parent_hop, my_hop_num);
	} else {
		LOG_INF("Not paired, starting discovery...");
		int err = anchor_do_pairing();
		if (err) {
			LOG_ERR("Pairing failed, err %d", err);
			return;
		}
	}

	anchor_print_paired();

	/* RX loop — receive from children, respond and relay */
	while (true) {
		int err = receive(ANCHOR_RX_HANDLE);
		if (err) {
			LOG_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		k_sem_take(&operation_sem, K_FOREVER);

		anchor_process_queue();

		k_sleep(K_MSEC(10));
	}
}
