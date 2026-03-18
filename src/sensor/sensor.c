/*
 * Sensor device logic for DECT NR+ mesh network
 *
 * Pairing flow:
 *   1. Broadcast PAIR_REQUEST with a random number
 *   2. Listen for PAIR_RESPONSE(s), pick the best candidate
 *   3. Verify hash, send PAIR_CONFIRM(SUCCESS) and store identity in NVM
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include "sensor.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../state.h"
#include "../storage.h"
#include "../large_data.h"
#include "../display.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

#define PAIR_RETRY_MAX     5

/* === Sensor identity helpers === */

int sensor_store_identity(const node_identity_t *id)
{
	return storage_write(SENSOR_IDENTITY_KEY, id, sizeof(*id));
}

int sensor_load_identity(node_identity_t *id)
{
	return storage_read(SENSOR_IDENTITY_KEY, id, sizeof(*id));
}

bool sensor_has_identity(void)
{
	return storage_exists(SENSOR_IDENTITY_KEY);
}

/* === Pairing logic === */

static int sensor_do_pairing(void)
{
	int err;

	for (int attempt = 0; attempt < PAIR_RETRY_MAX; attempt++) {
		LOG_INF("Pairing attempt %d/%d", attempt + 1, PAIR_RETRY_MAX);

		/* Generate random and send pair request (broadcast) */
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

		/* Start receiving to collect pair responses */
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

		/* Pick best candidate */
		const struct discovery_candidate *best = discovery_best();

		LOG_INF("Best candidate: %s ID:%d hop:%d RSSI:%d",
			device_type_str(best->device_type),
			best->device_id, best->hop_num, best->rssi_2 / 2);

		/* Verify hash */
		if (best->hash != expected_hash) {
			LOG_WRN("Hash mismatch, retrying...");
			k_sleep(K_SECONDS(2));
			continue;
		}

		/* Send pair confirm SUCCESS */
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
			.device_type = DEVICE_TYPE_SENSOR,
			.parent_id = best->device_id,
			.parent_hop = best->hop_num,
		};

		err = sensor_store_identity(&identity);
		if (err) {
			LOG_ERR("Failed to store identity, err %d", err);
			return err;
		}

		LOG_INF("Paired with %s ID:%d (parent hop:%d)",
			device_type_str(best->device_type),
			best->device_id, best->hop_num);

		return 0;
	}

	LOG_ERR("Pairing failed after %d attempts", PAIR_RETRY_MAX);
	return -ETIMEDOUT;
}

/* === Data transfer: send data to parent, check for ACK === */

static uint16_t parent_id;
static uint32_t tx_seq;
static uint8_t large_data_send_count;

static void sensor_send_data(void)
{
	/* Build a simple payload: sequence number + device_id */
	struct {
		uint32_t seq;
		uint16_t sensor_id;
	} __attribute__((packed)) payload = {
		.seq = tx_seq++,
		.sensor_id = device_id,
	};

	LOG_INF("Sending data seq:%d to parent ID:%d", payload.seq, parent_id);

	int err = send_data(TX_HANDLE, parent_id,
			    &payload, sizeof(payload));
	if (err) {
		LOG_ERR("Failed to send data, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	/* Listen for ACK (short window) */
	err = receive_ms(RX_HANDLE, 5000);
	if (err) {
		LOG_ERR("Receive failed, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	/* Drain queue and look for our ACK */
	struct rx_queue_item item;
	bool acked = false;

	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		if (item.len < DATA_ACK_PACKET_SIZE) {
			continue;
		}
		if (item.data[0] != PACKET_TYPE_DATA_ACK) {
			continue;
		}
		const data_ack_packet_t *ack =
			(const data_ack_packet_t *)item.data;
		if (ack->dst_device_id == device_id) {
			if (ack->status == DATA_ACK_SUCCESS) {
				LOG_INF("Data ACK from ID:%d: SUCCESS",
					ack->src_device_id);
			} else {
				LOG_WRN("Data ACK from ID:%d: CRC FAIL",
					ack->src_device_id);
			}
			acked = true;
		}
	}

	if (!acked) {
		LOG_WRN("No ACK received for seq:%d", payload.seq - 1);
	}
}

/* === Sensor entry point === */

void sensor_main(void)
{
	LOG_INF("Sensor mode started (ID:%d)", device_id);

	/* Check if already paired */
	node_identity_t identity;

	if (sensor_has_identity() &&
	    sensor_load_identity(&identity) == 0) {
		parent_id = identity.parent_id;
		LOG_INF("Already paired with parent ID:%d (parent hop:%d)",
			identity.parent_id, identity.parent_hop);
	} else {
		LOG_INF("Not paired, starting discovery...");
		int err = sensor_do_pairing();
		if (err) {
			LOG_ERR("Pairing failed, err %d", err);
			return;
		}
		/* parent_id set by sensor_do_pairing via identity store */
		if (sensor_load_identity(&identity) == 0) {
			parent_id = identity.parent_id;
		}
	}

	large_data_send_count = 0;

	LOG_INF("Sensor ready:");
	LOG_INF("  Button 2: send small data");
	LOG_INF("  Button 3: send 50KB sequential");
	LOG_INF("  Button 4: send 75KB sequential");

	while (true) {
		/* Check all button semaphores with short timeout */
		if (k_sem_take(&btn2_sem, K_MSEC(50)) == 0) {
			sensor_send_data();
			continue;
		}
		if (k_sem_take(&btn3_sem, K_MSEC(50)) == 0) {
			/* 50KB sequential: starting byte increments each send */
			uint32_t size = 50 * 1024;
			uint8_t *buf = k_malloc(size);
			if (!buf) {
				LOG_ERR("Failed to allocate 50KB buffer");
				continue;
			}
			uint8_t start = large_data_send_count;
			for (uint32_t i = 0; i < size; i++) {
				buf[i] = (uint8_t)(start + i);
			}
			LOG_INF("Sending 50KB sequential (start:0x%02x) to parent ID:%d",
				start, parent_id);
			large_data_send(TX_HANDLE, RX_HANDLE,
					parent_id, LARGE_DATA_FILE_DATA,
					buf, size);
			k_free(buf);
			large_data_send_count++;
			continue;
		}
		if (k_sem_take(&btn4_sem, K_MSEC(50)) == 0) {
			/* 75KB sequential: starting byte increments each send */
			uint32_t size = 75 * 1024;
			uint8_t *buf = k_malloc(size);
			if (!buf) {
				LOG_ERR("Failed to allocate 75KB buffer");
				continue;
			}
			uint8_t start = large_data_send_count;
			for (uint32_t i = 0; i < size; i++) {
				buf[i] = (uint8_t)(start + i);
			}
			LOG_INF("Sending 75KB sequential (start:0x%02x) to parent ID:%d",
				start, parent_id);
			large_data_send(TX_HANDLE, RX_HANDLE,
					parent_id, LARGE_DATA_FILE_DATA,
					buf, size);
			k_free(buf);
			large_data_send_count++;
			continue;
		}
	}
}
