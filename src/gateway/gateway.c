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
#include <nrf_modem_dect_phy.h>
#include "gateway.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../state.h"
#include "../paired_store.h"
#include "../large_data.h"
#include "../flash_store.h"
#include "../log_all.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

/* Paired device stores */
static const paired_store_t anchor_store = {
	.nvs_base = GW_ANCHOR_BASE,
	.max_entries = GW_ANCHOR_MAX,
	.label = "Anchor",
};

static const paired_store_t sensor_store = {
	.nvs_base = GW_SENSOR_BASE,
	.max_entries = GW_SENSOR_MAX,
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
		ALL_WRN("Unexpected device type %d in pair confirm",
			pkt->device_type);
		return;
	}

	int err = paired_store_add(store, pkt->device_id);

	if (err) {
		LOG_ERR("Failed to store %s ID:%d, err %d",
			store->label, pkt->device_id, err);
	} else {
		ALL_INF("%s ID:%d paired and stored in NVM",
			store->label, pkt->device_id);
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
	uint8_t status = (rx_crc == calc_crc) ? DATA_ACK_SUCCESS : DATA_ACK_CRC_FAIL;

	if (status == DATA_ACK_SUCCESS) {
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
		status == DATA_ACK_SUCCESS ? "OK" : "CRC_FAIL");
}


/* === Stream mode: send data every 500ms for 60s === */

#define STREAM_DURATION_MS  60000
#define STREAM_INTERVAL_MS  500

static void handle_stream_request(const stream_request_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("Stream request from ID:%d -- streaming for 60s every 500ms",
		pkt->src_device_id);

	int64_t end_time = k_uptime_get() + STREAM_DURATION_MS;
	uint32_t seq = 0;

	while (k_uptime_get() < end_time) {
		struct {
			uint8_t  packet_type;
			uint16_t src_device_id;
			uint16_t dst_device_id;
			uint32_t seq;
			int64_t  timestamp_ms;
		} __attribute__((packed)) out = {
			.packet_type   = PACKET_TYPE_DATA,
			.src_device_id = device_id,
			.dst_device_id = pkt->src_device_id,
			.seq           = seq++,
			.timestamp_ms  = k_uptime_get(),
		};

		int err = transmit_and_wait(&out, sizeof(out));
		if (err) {
			ALL_WRN("Stream TX failed seq:%d err:%d", seq - 1, err);
		} else if ((seq - 1) % 10 == 0) {
			ALL_INF("Stream TX seq:%d to ID:%d", seq - 1,
				pkt->src_device_id);
		}

		k_sleep(K_MSEC(STREAM_INTERVAL_MS));
	}

	ALL_INF("Stream to ID:%d complete (%d packets sent)",
		pkt->src_device_id, seq);
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
			/* Handled by flash writer thread via ring buffer */
			break;

		case PACKET_TYPE_STREAM_REQUEST:
			if (item.len >= STREAM_REQUEST_PACKET_SIZE) {
				handle_stream_request(
					(const stream_request_packet_t *)item.data);
			}
			break;

		case PACKET_TYPE_DATA_ACK:
		case PACKET_TYPE_LARGE_DATA_ACK:
		case PACKET_TYPE_LARGE_DATA_NACK:
			/* Not applicable for gateway */
			break;

		default:
			ALL_WRN("Unknown packet type 0x%02x", pkt_type);
			break;
		}
	}
}

/* === Gateway entry point === */

void gateway_main(void)
{
	ALL_INF("Gateway mode started (ID:%d, hop:0)", device_id);
	paired_store_print(&anchor_store);
	paired_store_print(&sensor_store);

	int flash_err = flash_store_init();
	if (flash_err) {
		ALL_ERR("Flash store init failed, err %d", flash_err);
		return;
	}
	large_data_init();

	while (true) {
		int err = receive(RX_HANDLE);

		if (err) {
			ALL_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}

		/* Wait for RX window to end or large data END */
		bool cancelled = false;

		while (true) {
			if (k_sem_take(&operation_sem, K_MSEC(10)) == 0) {
				break;
			}
			if (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
				nrf_modem_dect_phy_cancel(RX_HANDLE);
				/* Cancel generates two events:
				 * 1) EVT_CANCELED (on_cancel)
				 * 2) EVT_COMPLETED with err 4105 (on_op_complete)
				 * We must consume both before proceeding to TX,
				 * otherwise the stale op_complete can be mistaken
				 * for a TX completion in send_pending_ack.
				 * Since operation_sem has max count 1, if both
				 * fire before we take, one is lost. Take the
				 * first, then wait for the second with timeout. */
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
		large_data_process_pending_init();
		large_data_send_pending_ack(TX_HANDLE);

		/* Free completed sessions (gateway is the final destination) */
		uint8_t ld_slot;
		uint32_t ld_size;
		uint8_t ld_file_type;
		uint16_t ld_src_id;

		while (large_data_get_completed(&ld_slot, &ld_size,
						&ld_file_type, &ld_src_id)) {
			ALL_INF("Large data received: %d bytes from ID:%d type:%d (flash slot:%d)",
				ld_size, ld_src_id, ld_file_type, ld_slot);
			large_data_free_completed(ld_src_id);
		}

		large_data_cleanup_stale_sessions();

		while (k_sem_take(&large_data_end_sem, K_NO_WAIT) == 0) {
		}

		if (cancelled) {
			k_sleep(K_MSEC(10));
			k_sem_reset(&operation_sem);
		}

		k_sleep(K_MSEC(10));
	}
}
