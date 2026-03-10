/*
 * Gateway device logic for DECT NR+ mesh network
 *
 * Single-threaded RX/TX approach:
 *   - Listen for packets during an RX window
 *   - When RX completes, process all queued packets (TX responses as needed)
 *   - Restart RX
 *
 * This avoids modem scheduling conflicts since TX and RX never overlap.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include "gateway.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../state.h"
#include "../storage.h"

LOG_MODULE_DECLARE(app);

#define GW_TX_HANDLE 1
#define GW_RX_HANDLE 2

/* === Gateway storage helpers === */

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

int gw_store_anchor(uint16_t anchor_id)
{
	bool found;
	int slot = find_slot(GW_ANCHOR_BASE, GW_ANCHOR_MAX, anchor_id, &found);
	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("Anchor storage full");
		return -ENOMEM;
	}
	return storage_write(GW_ANCHOR_BASE + slot, &anchor_id, sizeof(anchor_id));
}

int gw_store_sensor(uint16_t sensor_id)
{
	bool found;
	int slot = find_slot(GW_SENSOR_BASE, GW_SENSOR_MAX, sensor_id, &found);
	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("Sensor storage full");
		return -ENOMEM;
	}
	return storage_write(GW_SENSOR_BASE + slot, &sensor_id, sizeof(sensor_id));
}

bool gw_is_anchor_paired(uint16_t anchor_id)
{
	bool found;
	find_slot(GW_ANCHOR_BASE, GW_ANCHOR_MAX, anchor_id, &found);
	return found;
}

bool gw_is_sensor_paired(uint16_t sensor_id)
{
	bool found;
	find_slot(GW_SENSOR_BASE, GW_SENSOR_MAX, sensor_id, &found);
	return found;
}

int gw_get_anchor_count(void)
{
	int count = 0;
	uint16_t tmp;
	for (int i = 0; i < GW_ANCHOR_MAX; i++) {
		if (storage_read(GW_ANCHOR_BASE + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

int gw_get_sensor_count(void)
{
	int count = 0;
	uint16_t tmp;
	for (int i = 0; i < GW_SENSOR_MAX; i++) {
		if (storage_read(GW_SENSOR_BASE + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

void gw_print_paired(void)
{
	uint16_t id;

	LOG_INF("=== Paired Anchors (%d) ===", gw_get_anchor_count());
	for (int i = 0; i < GW_ANCHOR_MAX; i++) {
		if (storage_read(GW_ANCHOR_BASE + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] Anchor ID:%d", i, id);
		}
	}

	LOG_INF("=== Paired Sensors (%d) ===", gw_get_sensor_count());
	for (int i = 0; i < GW_SENSOR_MAX; i++) {
		if (storage_read(GW_SENSOR_BASE + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] Sensor ID:%d", i, id);
		}
	}
}

/* === TX helper: transmit and wait for completion === */

static int gw_transmit(void *data, size_t len)
{
	int err = transmit(GW_TX_HANDLE, data, len);
	if (err) {
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	return 0;
}

/* === Packet handlers === */

static void handle_pair_request(const pair_request_packet_t *pkt, int16_t rssi_2)
{
	LOG_INF("Pair request from %s ID:%d (RSSI:%d)",
		device_type_str(pkt->device_type), pkt->device_id, rssi_2 / 2);

	uint32_t hash = compute_pair_hash(pkt->device_id, pkt->random_num);

	int err = send_pair_response(GW_TX_HANDLE, pkt->device_id, hash);
	if (err) {
		LOG_ERR("Failed to send pair response, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	LOG_INF("Pair response sent to ID:%d", pkt->device_id);
}

static void handle_pair_confirm(const pair_confirm_packet_t *pkt, int16_t rssi_2)
{
	LOG_INF("Pair confirm from %s ID:%d status:%s",
		device_type_str(pkt->device_type), pkt->device_id,
		(pkt->status == PAIR_STATUS_SUCCESS) ? "SUCCESS" : "FAILURE");

	if (pkt->status != PAIR_STATUS_SUCCESS) {
		return;
	}

	int err;

	if (pkt->device_type == DEVICE_TYPE_SENSOR) {
		err = gw_store_sensor(pkt->device_id);
		if (err) {
			LOG_ERR("Failed to store sensor ID:%d, err %d",
				pkt->device_id, err);
		} else {
			LOG_INF("Sensor ID:%d paired and stored in NVM",
				pkt->device_id);
		}
	} else if (pkt->device_type == DEVICE_TYPE_ANCHOR) {
		err = gw_store_anchor(pkt->device_id);
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

	data_ack_packet_t ack = {
		.packet_type = PACKET_TYPE_DATA_ACK,
		.src_device_id = device_id,
		.dst_device_id = pkt->src_device_id,
		.hop_num = my_hop_num,
		.status = 0,
	};

	int err = gw_transmit(&ack, sizeof(ack));
	if (err) {
		LOG_ERR("Failed to send data ACK, err %d", err);
		return;
	}

	LOG_DBG("Data ACK sent to ID:%d", pkt->src_device_id);
}

/* === Process all queued packets (called when RX window ends) === */

static void gw_process_queue(void)
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

/* === Gateway entry point === */

void gateway_main(void)
{
	LOG_INF("Gateway mode started (ID:%d, hop:0)", device_id);
	gw_print_paired();

	while (true) {
		/* Start RX window */
		int err = receive(GW_RX_HANDLE);
		if (err) {
			LOG_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Wait for RX window to complete */
		k_sem_take(&operation_sem, K_FOREVER);

		/* RX done — now process all queued packets and TX responses */
		gw_process_queue();

		/* Small gap before next RX window */
		k_sleep(K_MSEC(10));
	}
}
