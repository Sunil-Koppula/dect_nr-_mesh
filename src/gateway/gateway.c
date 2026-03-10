/*
 * Gateway device logic for DECT NR+ mesh network
 *
 * Two threads:
 *   1. RX thread     — keeps the radio in continuous receive mode
 *   2. Process thread — dequeues received packets and responds accordingly
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

#define GW_RX_STACK_SIZE   2048
#define GW_PROC_STACK_SIZE 2048
#define GW_RX_PRIORITY     7
#define GW_PROC_PRIORITY   8

static struct k_thread rx_thread_data;
static struct k_thread proc_thread_data;
static K_THREAD_STACK_DEFINE(rx_stack, GW_RX_STACK_SIZE);
static K_THREAD_STACK_DEFINE(proc_stack, GW_PROC_STACK_SIZE);

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

	int err = transmit(GW_TX_HANDLE, &ack, sizeof(ack));
	if (err) {
		LOG_ERR("Failed to send data ACK, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	LOG_DBG("Data ACK sent to ID:%d", pkt->src_device_id);
}

/* === Process thread: dequeue and handle packets one by one === */

static void gw_process_thread(void *p1, void *p2, void *p3)
{
	struct rx_queue_item item;

	LOG_INF("Gateway process thread started");

	while (true) {
		if (rx_queue_get(&item, K_FOREVER) != 0) {
			continue;
		}

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

/* === RX thread: keep radio in continuous receive === */

static void gw_rx_thread(void *p1, void *p2, void *p3)
{
	LOG_INF("Gateway RX thread started");

	while (true) {
		int err = receive(GW_RX_HANDLE);
		if (err) {
			LOG_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}
		/* Wait for RX operation to complete before restarting */
		k_sem_take(&operation_sem, K_FOREVER);
	}
}

/* === Gateway entry point === */

void gateway_main(void)
{
	LOG_INF("Gateway mode started (ID:%d, hop:0)", device_id);
	gw_print_paired();

	/* Start RX thread — keeps radio listening continuously */
	k_thread_create(&rx_thread_data, rx_stack,
			K_THREAD_STACK_SIZEOF(rx_stack),
			gw_rx_thread, NULL, NULL, NULL,
			GW_RX_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&rx_thread_data, "gw_rx");

	/* Start process thread — dequeues and handles packets */
	k_thread_create(&proc_thread_data, proc_stack,
			K_THREAD_STACK_SIZEOF(proc_stack),
			gw_process_thread, NULL, NULL, NULL,
			GW_PROC_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&proc_thread_data, "gw_proc");

	/* Block here — threads run independently */
	k_thread_join(&rx_thread_data, K_FOREVER);
}
