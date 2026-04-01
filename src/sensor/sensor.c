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
#include <zephyr/sys/reboot.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include "../identity.h"
#include "../protocol.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../mesh_tx.h"
#include "../crc.h"
#include "../large_data.h"
#include "../nvs_store.h"
#include "../log_all.h"

LOG_MODULE_REGISTER(sensor, CONFIG_SENSOR_LOG_LEVEL);

#define TX_HANDLE 1
#define RX_HANDLE 2

#define PAIR_RETRY_MAX     5

/* === Pairing logic === */

static int sensor_do_pairing(void)
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

		node_identity_t identity = {
			.device_id = device_id,
			.device_type = DEVICE_TYPE_SENSOR,
			.parent_id = best->device_id,
			.parent_hop = best->hop_num,
		};

		err = node_store_identity(&identity);
		if (err) {
			ALL_ERR("Failed to store identity, err %d", err);
			return err;
		}

		ALL_INF("Paired with %s ID:%d (parent hop:%d)",
			device_type_str(best->device_type),
			best->device_id, best->hop_num);

		return 0;
	}

	ALL_ERR("Pairing failed after %d attempts", PAIR_RETRY_MAX);
	return -ETIMEDOUT;
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

/* === Data transfer: send data to parent, check for ACK === */

static uint16_t parent_id;
static uint32_t tx_seq;
static uint8_t large_data_send_count;

static void sensor_send_large_data(void)
{
	uint32_t size = 50 * 1024;
	uint8_t *buf = k_malloc(size);

	if (!buf) {
		ALL_ERR("Failed to allocate %d bytes", size);
		return;
	}

	uint8_t start = large_data_send_count;

	for (uint32_t i = 0; i < size; i++) {
		buf[i] = (uint8_t)(start + i);
	}

	ALL_INF("Sending %dKB (start:0x%02x) to parent ID:%d",
		size / 1024, start, parent_id);

	int err = large_data_send(TX_HANDLE, RX_HANDLE,
				  parent_id, LARGE_DATA_FILE_DATA,
				  buf, size);

	k_free(buf);

	if (err) {
		ALL_ERR("Large data send failed, err %d", err);
	}

	large_data_send_count++;
}

static void sensor_send_data(void)
{
	struct {
		uint32_t seq;
		uint16_t sensor_id;
	} __attribute__((packed)) payload = {
		.seq = tx_seq++,
		.sensor_id = device_id,
	};

	ALL_INF("Sending data seq:%d to parent ID:%d", payload.seq, parent_id);

	int err = send_data(TX_HANDLE, parent_id,
			    &payload, sizeof(payload));
	if (err) {
		ALL_ERR("Failed to send data, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	/* Listen for ACK */
	err = receive_ms(RX_HANDLE, 1000);
	if (err) {
		ALL_ERR("Receive failed, err %d", err);
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
			if (ack->status == STATUS_SUCCESS) {
				ALL_INF("Data ACK from ID:%d: SUCCESS",
					ack->src_device_id);
			} else {
				ALL_WRN("Data ACK from ID:%d: CRC FAIL",
					ack->src_device_id);
			}
			acked = true;
		}
	}

	if (!acked) {
		ALL_WRN("No ACK received for seq:%d", payload.seq - 1);
	}
}

/* === Sensor entry point === */

void sensor_main(void)
{
	ALL_INF("Sensor mode started (ID:%d)", device_id);

	node_identity_t identity;

	if (node_has_identity() &&
	    node_load_identity(&identity) == 0) {
		parent_id = identity.parent_id;
		ALL_INF("Already paired with parent ID:%d (parent hop:%d)",
			identity.parent_id, identity.parent_hop);
	} else {
		ALL_INF("Not paired, starting discovery...");
		int err = sensor_do_pairing();
		if (err) {
			ALL_ERR("Pairing failed, err %d", err);
			return;
		}
		if (node_load_identity(&identity) == 0) {
			parent_id = identity.parent_id;
		}
	}

	ALL_INF("Sensor ready (always-on RX):");
	ALL_INF("  Button 2: send small data");
	ALL_INF("  Button 3: send 50KB large data");
	ALL_INF("  Button 4: scan nearby devices");

	while (true) {
		/* Check buttons (non-blocking) */
		if (k_sem_take(&btn2_sem, K_NO_WAIT) == 0) {
			sensor_send_data();
			continue;
		}

		if (k_sem_take(&btn4_sem, K_NO_WAIT) == 0) {
			scan_nearby(TX_HANDLE, RX_HANDLE);
			continue;
		}

		if (k_sem_take(&btn3_sem, K_NO_WAIT) == 0) {
			sensor_send_large_data();
			continue;
		}

		/* Always-on RX: listen for incoming packets for 1 second */
		int err = receive_ms(RX_HANDLE, 1000);

		if (err) {
			ALL_ERR("Receive failed, err %d", err);
			k_sleep(K_SECONDS(1));
			continue;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		/* Process received packets */
		struct rx_queue_item item;

		while (rx_queue_get(&item, K_NO_WAIT) == 0) {
			if (item.len < 1) {
				continue;
			}

			if (item.data[0] == PACKET_TYPE_DATA_REQUEST &&
			    item.len >= DATA_REQUEST_PACKET_SIZE) {
				const data_request_packet_t *req =
					(const data_request_packet_t *)
						item.data;
				if (req->dst_device_id == device_id) {
					if (req->request_type ==
					    DATA_REQUEST_LARGE) {
						ALL_INF("LARGE DATA_REQUEST "
							"from ID:%d",
							req->src_device_id);
						sensor_send_large_data();
					} else {
						ALL_INF("SMALL DATA_REQUEST "
							"from ID:%d",
							req->src_device_id);
						sensor_send_data();
					}
				}
			}

			if (item.data[0] == PACKET_TYPE_REPAIR &&
			    item.len >= REPAIR_PACKET_SIZE) {
				const repair_packet_t *rpkt =
					(const repair_packet_t *)
						item.data;
				if (rpkt->dst_device_id == device_id) {
					ALL_INF("REPAIR from ID:%d, "
						"clearing NVM and "
						"rebooting...",
						rpkt->src_device_id);
					storage_clear_all();
					k_sleep(K_MSEC(500));
					sys_reboot(SYS_REBOOT_COLD);
				}
			}

			if (item.data[0] == PACKET_TYPE_SET_RSSI &&
			    item.len >= SET_RSSI_PACKET_SIZE) {
				const set_rssi_packet_t *spkt =
					(const set_rssi_packet_t *)
						item.data;
				if (spkt->dst_device_id == device_id) {
					ALL_INF("SET_RSSI %d dBm from "
						"ID:%d, storing and "
						"rebooting...",
						spkt->rssi_dbm,
						spkt->src_device_id);
					mesh_rssi_threshold_store(
						spkt->rssi_dbm);
					k_sleep(K_MSEC(500));
					sys_reboot(SYS_REBOOT_COLD);
				}
			}

			if (item.data[0] == PACKET_TYPE_PARENT_QUERY &&
			    item.len >= PARENT_QUERY_PACKET_SIZE) {
				const parent_query_packet_t *qry =
					(const parent_query_packet_t *)
						item.data;
				if (qry->dst_device_id == device_id) {
					node_identity_t id;

					ALL_INF("PARENT_QUERY from ID:%d",
						qry->src_device_id);

					if (node_load_identity(&id) == 0) {
						uint8_t ptype = (id.parent_hop == 0)
							? DEVICE_TYPE_GATEWAY
							: DEVICE_TYPE_ANCHOR;
						parent_response_packet_t resp = {
							.packet_type = PACKET_TYPE_PARENT_RESPONSE,
							.src_device_id = device_id,
							.dst_device_id = qry->src_device_id,
							.device_type = DEVICE_TYPE_SENSOR,
							.parent_id = id.parent_id,
							.parent_type = ptype,
							.hop_num = my_hop_num,
						};

						int perr = transmit_and_wait(
							&resp, sizeof(resp));
						if (perr) {
							ALL_ERR("Failed to send PARENT_RESPONSE");
						} else {
							ALL_INF("PARENT_RESPONSE sent to ID:%d",
								qry->src_device_id);
						}
					}
				}
			}
		}

		k_sleep(K_MSEC(10));
	}
}
