/*
 * Shared helpers for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include "common.h"
#include "identity.h"
#include "protocol.h"
#include "radio.h"
#include "queue.h"
#include "mesh.h"
#include "mesh_tx.h"
#include "crc.h"
#include "nvs_store.h"
#include "log_all.h"

LOG_MODULE_REGISTER(common, CONFIG_COMMON_LOG_LEVEL);

/* === Transmit and wait === */

int transmit_and_wait(void *data, size_t len)
{
	int err = transmit(TX_HANDLE, data, len);

	if (err) {
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	return 0;
}

/* === Pair request handler (used by gateway + anchor) === */

void common_handle_pair_request(const pair_request_packet_t *pkt,
				int16_t rssi_2)
{
	ALL_INF("Pair request from %s ID:%d (RSSI:%d)",
		device_type_str(pkt->device_type), pkt->device_id,
		rssi_2 / 2);

	uint32_t hash = compute_pair_hash(pkt->device_id, pkt->random_num);

	int err = send_pair_response(TX_HANDLE, pkt->device_id, hash);

	if (err) {
		LOG_ERR("Failed to send pair response, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	ALL_INF("Pair response sent to ID:%d", pkt->device_id);
}

/* === Data CRC verify + ACK === */

int common_data_verify_and_ack(const data_packet_t *pkt, uint16_t len,
			       int16_t rssi_2)
{
	if (pkt->dst_device_id != device_id) {
		return -1;
	}

	uint16_t payload_len = pkt->payload_len;
	uint16_t rx_crc;

	memcpy(&rx_crc, &pkt->payload[payload_len], sizeof(rx_crc));
	uint16_t calc_crc = compute_crc16(pkt->payload, payload_len);
	uint8_t status = (rx_crc == calc_crc) ? STATUS_SUCCESS
					      : STATUS_CRC_FAIL;

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
		return -2;
	}

	ALL_INF("Data ACK sent to ID:%d (status:%s)", pkt->src_device_id,
		status == STATUS_SUCCESS ? "OK" : "CRC_FAIL");

	return (int)status;
}

/* === Discovery response draining === */

void drain_discovery_responses(bool log_each)
{
	struct rx_queue_item item;

	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		if (item.len < 1) {
			continue;
		}
		if (item.data[0] != PACKET_TYPE_PAIR_RESPONSE ||
		    item.len < PAIR_RESPONSE_PACKET_SIZE) {
			continue;
		}
		const pair_response_packet_t *resp =
			(const pair_response_packet_t *)item.data;
		if (resp->dst_device_id != device_id) {
			continue;
		}
		discovery_add_response(resp, item.rssi_2);
		if (log_each) {
			ALL_INF("Got pair response from %s ID:%d hop:%d "
				"RSSI:%d",
				device_type_str(resp->device_type),
				resp->device_id, resp->hop_num,
				item.rssi_2 / 2);
		}
	}
}

/* === Factory reset === */

void factory_reset_reboot(void)
{
	ALL_INF("Clearing NVM and rebooting...");
	storage_clear_all();
	k_sleep(K_MSEC(500));
	sys_reboot(SYS_REBOOT_COLD);
}

/* === RSSI store and reboot === */

void rssi_store_and_reboot(int16_t rssi_dbm)
{
	ALL_INF("Storing RSSI threshold %d dBm and rebooting...", rssi_dbm);
	mesh_rssi_threshold_store(rssi_dbm);
	k_sleep(K_MSEC(500));
	sys_reboot(SYS_REBOOT_COLD);
}
