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
#include "../paired_store.h"
#include "../large_data.h"
#include "../flash_store.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

#define PAIR_RETRY_MAX      5

/* Parent info (loaded from NVM or set during pairing) */
static uint16_t parent_id;

/* Paired device stores */
static const paired_store_t anchor_store = {
	.nvs_base = ANCHOR_ANCHOR_BASE,
	.max_entries = ANCHOR_ANCHOR_MAX,
	.label = "Anchor",
};

static const paired_store_t sensor_store = {
	.nvs_base = ANCHOR_SENSOR_BASE,
	.max_entries = ANCHOR_SENSOR_MAX,
	.label = "Sensor",
};

/* === Anchor identity helpers === */

int anchor_store_identity(const node_identity_t *id)
{
	return storage_write(ANCHOR_IDENTITY_KEY, id, sizeof(*id));
}

int anchor_load_identity(node_identity_t *id)
{
	return storage_read(ANCHOR_IDENTITY_KEY, id, sizeof(*id));
}

bool anchor_has_identity(void)
{
	return storage_exists(ANCHOR_IDENTITY_KEY);
}

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
		LOG_INF("Pairing attempt %d/%d", attempt + 1, PAIR_RETRY_MAX);

		uint32_t rand_num = next_random();
		uint32_t expected_hash = compute_pair_hash(device_id, rand_num);

		err = send_pair_request(TX_HANDLE, rand_num);
		if (err) {
			LOG_ERR("Failed to send pair request, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		LOG_INF("Pair request sent, listening for responses...");

		discovery_reset();

		err = receive(RX_HANDLE);
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

		err = send_pair_confirm(TX_HANDLE, best->device_id,
				       PAIR_STATUS_SUCCESS);
		if (err) {
			LOG_ERR("Failed to send pair confirm, err %d", err);
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

	int err = send_pair_response(TX_HANDLE, pkt->device_id, hash);
	if (err) {
		LOG_ERR("Failed to send pair response, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	LOG_INF("Pair response sent to ID:%d", pkt->device_id);
}

static void handle_pair_confirm(const pair_confirm_packet_t *pkt)
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

	const paired_store_t *store;

	if (pkt->device_type == DEVICE_TYPE_SENSOR) {
		store = &sensor_store;
	} else if (pkt->device_type == DEVICE_TYPE_ANCHOR) {
		store = &anchor_store;
	} else {
		LOG_WRN("Unexpected device type %d in pair confirm",
			pkt->device_type);
		return;
	}

	int err = paired_store_add(store, pkt->device_id);

	if (err) {
		LOG_ERR("Failed to store %s ID:%d, err %d",
			store->label, pkt->device_id, err);
	} else {
		LOG_INF("%s ID:%d paired and stored in NVM",
			store->label, pkt->device_id);
	}
}

static void handle_data(const data_packet_t *pkt, uint16_t len, int16_t rssi_2)
{
	/* Only accept data addressed to us */
	if (pkt->dst_device_id != device_id) {
		return;
	}

	uint16_t payload_len = pkt->payload_len;

	/* Verify CRC */
	uint16_t rx_crc;
	memcpy(&rx_crc, &pkt->payload[payload_len], sizeof(rx_crc));
	uint16_t calc_crc = compute_crc16(pkt->payload, payload_len);
	uint8_t status = (rx_crc == calc_crc) ? DATA_ACK_SUCCESS : DATA_ACK_CRC_FAIL;

	if (status == DATA_ACK_SUCCESS) {
		LOG_INF("Data from ID:%d (%d bytes, RSSI:%d) CRC OK",
			pkt->src_device_id, payload_len, rssi_2 / 2);
	} else {
		LOG_WRN("Data from ID:%d (%d bytes, RSSI:%d) CRC FAIL",
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
		LOG_ERR("Failed to send data ACK, err %d", err);
		return;
	}

	LOG_INF("Data ACK sent to ID:%d (status:%s)", pkt->src_device_id,
		status == DATA_ACK_SUCCESS ? "OK" : "CRC_FAIL");

	/* Only relay upstream if CRC was good */
	if (status != DATA_ACK_SUCCESS) {
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
		LOG_ERR("Failed to relay data to parent ID:%d, err %d",
			parent_id, err);
		return;
	}

	LOG_INF("Data relayed from ID:%d to parent ID:%d",
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
					if (ack->status == DATA_ACK_SUCCESS) {
						LOG_INF("Parent ACK from ID:%d: SUCCESS",
							ack->src_device_id);
					} else {
						LOG_WRN("Parent ACK from ID:%d: CRC FAIL",
							ack->src_device_id);
					}
				}
			}
			break;

		case PACKET_TYPE_LARGE_DATA_INIT:
		case PACKET_TYPE_LARGE_DATA_TRANSFER:
		case PACKET_TYPE_LARGE_DATA_END:
			/* Handled by flash writer thread via ring buffer */
			break;

		case PACKET_TYPE_LARGE_DATA_ACK:
		case PACKET_TYPE_LARGE_DATA_NACK:
			/* Not applicable here */
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
	node_identity_t identity;

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

	node_identity_t self;
	if (anchor_load_identity(&self) == 0) {
		LOG_INF("Anchor: ID:%d parent:%d parent_hop:%d my_hop:%d",
			self.device_id, self.parent_id, self.parent_hop,
			self.parent_hop + 1);
	}
	paired_store_print(&anchor_store);
	paired_store_print(&sensor_store);

	int flash_err = flash_store_init();
	if (flash_err) {
		LOG_ERR("Flash store init failed, err %d", flash_err);
		return;
	}
	large_data_init();

	/* RX loop — receive from children, respond and relay */
	while (true) {
		int err = receive(RX_HANDLE);
		if (err) {
			LOG_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Wait for either RX window to end or large data END to arrive */
		bool cancelled = false;

		while (true) {
			if (k_sem_take(&operation_sem, K_MSEC(10)) == 0) {
				break; /* RX window ended naturally */
			}
			if (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
				/* END arrived — cancel RX to send ACK now */
				nrf_modem_dect_phy_cancel(RX_HANDLE);
				/* Cancel generates on_op_complete + on_cancel.
				 * Wait for both callbacks to fire. */
				k_sem_take(&operation_sem, K_FOREVER);
				k_sleep(K_MSEC(50));
				k_sem_reset(&operation_sem);
				cancelled = true;
				break;
			}
		}

		process_queue();
		large_data_process_pending_init();

		/* Send any pending large data ACKs */
		large_data_send_pending_ack(TX_HANDLE);

		/* Relay completed large data to parent (read from flash) */
		uint8_t ld_slot;
		uint32_t ld_size;
		uint8_t ld_file_type;
		uint16_t ld_src_id;

		while (large_data_get_completed(&ld_slot, &ld_size,
						&ld_file_type, &ld_src_id)) {
			LOG_INF("Relaying %d bytes from ID:%d to parent ID:%d (flash slot:%d)",
				ld_size, ld_src_id, parent_id, ld_slot);

			/* Read data from flash into a temporary buffer and relay.
			 * Use FLASH_STORE_READ_CHUNK-sized reads to keep
			 * stack usage bounded. Allocate full relay buffer
			 * from heap since large_data_send needs contiguous data. */
			uint8_t *relay_buf = k_malloc(ld_size);

			if (!relay_buf) {
				LOG_ERR("Failed to allocate %d bytes for relay",
					ld_size);
				large_data_free_completed(ld_src_id);
				continue;
			}

			uint32_t remaining = ld_size;
			uint32_t offset = 0;
			bool read_ok = true;

			while (remaining > 0) {
				uint16_t chunk = (remaining > FLASH_STORE_READ_CHUNK) ?
						 FLASH_STORE_READ_CHUNK :
						 (uint16_t)remaining;
				int rerr = flash_store_read(ld_slot, offset,
							    &relay_buf[offset],
							    chunk);
				if (rerr) {
					LOG_ERR("Flash read failed at offset %d, err %d",
						offset, rerr);
					read_ok = false;
					break;
				}
				offset += chunk;
				remaining -= chunk;
			}

			if (read_ok) {
				int relay_err = large_data_send(
					TX_HANDLE, RX_HANDLE,
					parent_id, ld_file_type,
					relay_buf, ld_size);
				if (relay_err) {
					LOG_ERR("Failed to relay large data, err %d",
						relay_err);
				} else {
					LOG_INF("Large data relay to parent complete");
				}
			}

			k_free(relay_buf);
			large_data_free_completed(ld_src_id);
		}

		/* Drain any extra large_data_end_sem gives */
		while (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
		}

		/* After cancel, drain any stale sem gives before next RX */
		if (cancelled) {
			k_sleep(K_MSEC(10));
			k_sem_reset(&operation_sem);
		}

		k_sleep(K_MSEC(10));
	}
}
