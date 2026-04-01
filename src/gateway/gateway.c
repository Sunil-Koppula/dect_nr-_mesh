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

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <nrf_modem_dect_phy.h>
#include "../identity.h"
#include "../nvs_store.h"
#include "../protocol.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../mesh_tx.h"
#include "../crc.h"
#include "../large_data.h"
#include "../psram.h"
#include "../at_cmd.h"
#include "../log_all.h"

LOG_MODULE_REGISTER(gateway, CONFIG_GATEWAY_LOG_LEVEL);

#define TX_HANDLE 1
#define RX_HANDLE 2

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

/* === Packet handlers === */

static void handle_pair_request(const pair_request_packet_t *pkt, int16_t rssi_2)
{
	ALL_INF("Pair request from %s ID:%d (RSSI:%d)",
		device_type_str(pkt->device_type), pkt->device_id, rssi_2 / 2);

	uint32_t hash = compute_pair_hash(pkt->device_id, pkt->random_num);

	int err = send_pair_response(TX_HANDLE, pkt->device_id, hash);

	if (err) {
		LOG_ERR("Failed to send pair response, err %d", err);
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
		LOG_ERR("Failed to store %s ID:%d, err %d",
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
}

/* === Send DATA_REQUEST to all paired devices === */

static void send_data_request_to_all(uint8_t request_type)
{
	const char *type_str = (request_type == DATA_REQUEST_LARGE)
			       ? "LARGE" : "SMALL";

	ALL_INF("Sending %s DATA_REQUEST to all paired devices...", type_str);

	/* Send to all paired anchors */
	for (int i = 0; i < anchor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&anchor_store, i, &dev_id) != 0) {
			continue;
		}

		data_request_packet_t req = {
			.packet_type = PACKET_TYPE_DATA_REQUEST,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
			.request_type = request_type,
		};

		int err = transmit_and_wait(&req, sizeof(req));

		if (err) {
			ALL_ERR("Failed to send DATA_REQUEST to ID:%d", dev_id);
		} else {
			ALL_INF("%s DATA_REQUEST sent to ANCHOR ID:%d",
				type_str, dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* Send to directly paired sensors */
	for (int i = 0; i < sensor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&sensor_store, i, &dev_id) != 0) {
			continue;
		}

		data_request_packet_t req = {
			.packet_type = PACKET_TYPE_DATA_REQUEST,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
			.request_type = request_type,
		};

		int err = transmit_and_wait(&req, sizeof(req));

		if (err) {
			ALL_ERR("Failed to send DATA_REQUEST to ID:%d", dev_id);
		} else {
			ALL_INF("%s DATA_REQUEST sent to SENSOR ID:%d",
				type_str, dev_id);
		}
		k_sleep(K_MSEC(10));
	}
}

/* === Send PARENT_QUERY for a specific sensor === */

static void send_parent_query(uint16_t sensor_id)
{
	ALL_INF("Sending PARENT_QUERY for sensor ID:%d", sensor_id);

	/* Send to all paired anchors (they will relay to the sensor) */
	for (int i = 0; i < anchor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&anchor_store, i, &dev_id) != 0) {
			continue;
		}

		parent_query_packet_t query = {
			.packet_type = PACKET_TYPE_PARENT_QUERY,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
		};

		int err = transmit_and_wait(&query, sizeof(query));

		if (err) {
			ALL_ERR("Failed to send PARENT_QUERY to anchor ID:%d",
				dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* Also send directly to the sensor (if it's directly paired) */
	if (paired_store_contains(&sensor_store, sensor_id)) {
		parent_query_packet_t query = {
			.packet_type = PACKET_TYPE_PARENT_QUERY,
			.src_device_id = device_id,
			.dst_device_id = sensor_id,
		};

		int err = transmit_and_wait(&query, sizeof(query));

		if (err) {
			ALL_ERR("Failed to send PARENT_QUERY to sensor ID:%d",
				sensor_id);
		}
	}
}

/* === Send PARENT_QUERY to ALL paired devices === */

static void send_parent_query_all(void)
{
	ALL_INF("Sending PARENT_QUERY to all paired devices...");

	/* Send to all paired anchors (they respond + relay to children) */
	for (int i = 0; i < anchor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&anchor_store, i, &dev_id) != 0) {
			continue;
		}

		parent_query_packet_t query = {
			.packet_type = PACKET_TYPE_PARENT_QUERY,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
		};

		int err = transmit_and_wait(&query, sizeof(query));

		if (err) {
			ALL_ERR("Failed to send PARENT_QUERY to anchor ID:%d",
				dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* Send to directly paired sensors */
	for (int i = 0; i < sensor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&sensor_store, i, &dev_id) != 0) {
			continue;
		}

		parent_query_packet_t query = {
			.packet_type = PACKET_TYPE_PARENT_QUERY,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
		};

		int err = transmit_and_wait(&query, sizeof(query));

		if (err) {
			ALL_ERR("Failed to send PARENT_QUERY to sensor ID:%d",
				dev_id);
		}
		k_sleep(K_MSEC(10));
	}
}

/* === Handle PARENT_RESPONSE from a sensor/anchor === */

static void handle_parent_response(const parent_response_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("%s ID: %d is at %s ID: %d, Hop: %d",
		device_type_str(pkt->device_type),
		pkt->src_device_id,
		device_type_str(pkt->parent_type),
		pkt->parent_id,
		pkt->hop_num);
}

/* === Send REPAIR to all paired devices, wait 2s, factory reset === */

static void send_repair_all(void)
{
	ALL_INF("Broadcasting REPAIR to all paired devices...");

	/* Send to all paired anchors */
	for (int i = 0; i < anchor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&anchor_store, i, &dev_id) != 0) {
			continue;
		}

		repair_packet_t pkt = {
			.packet_type = PACKET_TYPE_REPAIR,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
		};

		int err = transmit_and_wait(&pkt, sizeof(pkt));

		if (err) {
			ALL_ERR("Failed to send REPAIR to anchor ID:%d",
				dev_id);
		} else {
			ALL_INF("REPAIR sent to ANCHOR ID:%d", dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* Send to directly paired sensors */
	for (int i = 0; i < sensor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&sensor_store, i, &dev_id) != 0) {
			continue;
		}

		repair_packet_t pkt = {
			.packet_type = PACKET_TYPE_REPAIR,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
		};

		int err = transmit_and_wait(&pkt, sizeof(pkt));

		if (err) {
			ALL_ERR("Failed to send REPAIR to sensor ID:%d",
				dev_id);
		} else {
			ALL_INF("REPAIR sent to SENSOR ID:%d", dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	ALL_INF("REPAIR broadcast complete, waiting 2s before self-reset...");
	k_sleep(K_SECONDS(2));

	ALL_INF("Clearing NVM and rebooting...");
	storage_clear_all();
	k_sleep(K_MSEC(500));
	sys_reboot(SYS_REBOOT_COLD);
}

/* === Send SET_RSSI to all paired devices, store locally, reboot === */

static void send_set_rssi_all(int16_t rssi_dbm)
{
	ALL_INF("Broadcasting SET_RSSI %d dBm to all paired devices...",
		rssi_dbm);

	/* Send to all paired anchors */
	for (int i = 0; i < anchor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&anchor_store, i, &dev_id) != 0) {
			continue;
		}

		set_rssi_packet_t pkt = {
			.packet_type = PACKET_TYPE_SET_RSSI,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
			.rssi_dbm = rssi_dbm,
		};

		int err = transmit_and_wait(&pkt, sizeof(pkt));

		if (err) {
			ALL_ERR("Failed to send SET_RSSI to anchor ID:%d",
				dev_id);
		} else {
			ALL_INF("SET_RSSI sent to ANCHOR ID:%d", dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* Send to directly paired sensors */
	for (int i = 0; i < sensor_store.max_entries; i++) {
		uint16_t dev_id;

		if (paired_store_get(&sensor_store, i, &dev_id) != 0) {
			continue;
		}

		set_rssi_packet_t pkt = {
			.packet_type = PACKET_TYPE_SET_RSSI,
			.src_device_id = device_id,
			.dst_device_id = dev_id,
			.rssi_dbm = rssi_dbm,
		};

		int err = transmit_and_wait(&pkt, sizeof(pkt));

		if (err) {
			ALL_ERR("Failed to send SET_RSSI to sensor ID:%d",
				dev_id);
		} else {
			ALL_INF("SET_RSSI sent to SENSOR ID:%d", dev_id);
		}
		k_sleep(K_MSEC(10));
	}

	/* Store locally and reboot */
	ALL_INF("Storing RSSI threshold %d dBm and rebooting...", rssi_dbm);
	mesh_rssi_threshold_store(rssi_dbm);
	k_sleep(K_MSEC(500));
	sys_reboot(SYS_REBOOT_COLD);
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

		case PACKET_TYPE_LARGE_DATA_INIT:
		case PACKET_TYPE_LARGE_DATA_TRANSFER:
		case PACKET_TYPE_LARGE_DATA_END:
			/* Handled by PSRAM writer thread via ring buffer */
			break;

		case PACKET_TYPE_LARGE_DATA_ACK:
		case PACKET_TYPE_LARGE_DATA_NACK:
			/* Not applicable for gateway */
			break;

		case PACKET_TYPE_PARENT_RESPONSE:
			if (item.len >= PARENT_RESPONSE_PACKET_SIZE) {
				handle_parent_response(
					(const parent_response_packet_t *)
						item.data);
			}
			break;

		case PACKET_TYPE_DATA_REQUEST:
			/* Gateway originates, doesn't process incoming */
			break;

		default:
			break;
		}
	}
}

/* === Gateway entry point === */

/* AT command handler paired store pointers (defined in at_cmd.c) */
extern const void *gw_anchor_store_ptr;
extern const void *gw_sensor_store_ptr;

void gateway_main(void)
{
	/* Set paired store pointers for AT command handler */
	gw_anchor_store_ptr = &anchor_store;
	gw_sensor_store_ptr = &sensor_store;

	ALL_INF("Gateway mode started (ID:%d, hop:0)", device_id);
	ALL_INF("  Button 4: scan nearby devices");
	paired_store_print(&anchor_store);
	paired_store_print(&sensor_store);

	while (true) {
		if (k_sem_take(&btn4_sem, K_NO_WAIT) == 0) {
			scan_nearby(TX_HANDLE, RX_HANDLE);
		}

		if (k_sem_take(&send_small_data_sem, K_NO_WAIT) == 0) {
			send_data_request_to_all(DATA_REQUEST_SMALL);
		}

		if (k_sem_take(&send_large_data_sem, K_NO_WAIT) == 0) {
			send_data_request_to_all(DATA_REQUEST_LARGE);
		}

		if (k_sem_take(&parent_query_sem, K_NO_WAIT) == 0) {
			send_parent_query(parent_query_sensor_id);
		}

		if (k_sem_take(&parent_query_all_sem, K_NO_WAIT) == 0) {
			send_parent_query_all();
		}

		if (k_sem_take(&repair_sem, K_NO_WAIT) == 0) {
			send_repair_all();
			/* Does not return — reboots */
		}

		if (k_sem_take(&set_rssi_sem, K_NO_WAIT) == 0) {
			send_set_rssi_all(set_rssi_value);
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

		/* Gateway is final destination: log and free completed sessions */
		uint8_t ld_slot;
		uint32_t ld_size;
		uint8_t ld_file_type;
		uint16_t ld_src_id;

		while (large_data_get_completed(&ld_slot, &ld_size,
						&ld_file_type, &ld_src_id)) {
			ALL_INF("Large data complete: %d bytes from ID:%d "
				"type:%d slot:%d",
				ld_size, ld_src_id, ld_file_type, ld_slot);
			large_data_free_completed(ld_src_id);
		}

		large_data_cleanup_stale_sessions();

		/* Drain extra end_sem gives */
		while (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
		}

		if (cancelled) {
			k_sleep(K_MSEC(10));
			k_sem_reset(&operation_sem);
		}

		k_sleep(K_MSEC(10));
	}
}
