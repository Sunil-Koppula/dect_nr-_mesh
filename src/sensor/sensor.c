/*
 * Sensor device logic for DECT NR+ mesh network
 *
 * Pairing flow:
 *   1. Broadcast PAIR_REQUEST with a random number
 *   2. Listen for PAIR_RESPONSE(s), pick the best candidate
 *   3. Verify hash, send PAIR_CONFIRM(SUCCESS) and store identity in NVM
 */

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

LOG_MODULE_DECLARE(app);

#define SENSOR_TX_HANDLE 1
#define SENSOR_RX_HANDLE 2

#define PAIR_RETRY_MAX     5
#define PAIR_LISTEN_TIMEOUT K_SECONDS(CONFIG_RX_PERIOD_S)

/* === Sensor storage helpers === */

int sensor_store_identity(const sensor_identity_t *id)
{
	return storage_write(SENSOR_IDENTITY_KEY, id, sizeof(*id));
}

int sensor_load_identity(sensor_identity_t *id)
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

		err = send_pair_request(SENSOR_TX_HANDLE, rand_num);
		if (err) {
			LOG_ERR("Failed to send pair request, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		LOG_INF("Pair request sent, listening for responses...");

		/* Start receiving to collect pair responses */
		discovery_reset();

		err = receive(SENSOR_RX_HANDLE);
		if (err) {
			LOG_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}

		/* Collect responses during the RX window */
		struct rx_queue_item item;

		while (rx_queue_get(&item, PAIR_LISTEN_TIMEOUT) == 0) {
			if (item.len < 1) {
				continue;
			}

			if (item.data[0] == PACKET_TYPE_PAIR_RESPONSE &&
			    item.len >= PAIR_RESPONSE_PACKET_SIZE) {
				const pair_response_packet_t *resp =
					(const pair_response_packet_t *)item.data;
				discovery_add_response(resp, item.rssi_2);
				LOG_INF("Got pair response from %s ID:%d hop:%d",
					device_type_str(resp->device_type),
					resp->device_id, resp->hop_num);
			}
		}

		/* Wait for RX operation to complete */
		k_sem_take(&operation_sem, K_FOREVER);

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
		err = send_pair_confirm(SENSOR_TX_HANDLE, PAIR_STATUS_SUCCESS);
		if (err) {
			LOG_ERR("Failed to send pair confirm, err %d", err);
			k_sleep(K_SECONDS(2));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		/* Store identity in NVM */
		sensor_identity_t identity = {
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

/* === Sensor entry point === */

void sensor_main(void)
{
	LOG_INF("Sensor mode started (ID:%d)", device_id);

	/* Check if already paired */
	sensor_identity_t identity;

	if (sensor_has_identity() &&
	    sensor_load_identity(&identity) == 0) {
		LOG_INF("Already paired with parent ID:%d (parent hop:%d)",
			identity.parent_id, identity.parent_hop);
	} else {
		LOG_INF("Not paired, starting discovery...");
		int err = sensor_do_pairing();
		if (err) {
			LOG_ERR("Pairing failed, err %d", err);
			return;
		}
	}

	/* TODO: normal operation — send data to parent */
	LOG_INF("Sensor paired and ready");
}
