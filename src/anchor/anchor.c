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
#include <pm_config.h>
#include "../state.h"
#include "../log_all.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../storage.h"
#include "../paired_store.h"
#include "../large_data.h"
#include "../psram.h"
#include "../ota.h"
#include "../ota_store.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

#define PAIR_RETRY_MAX      5

/* Parent info (loaded from NVM or set during pairing) */
static uint16_t parent_id;

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

/* === Pairing: anchor pairs with a parent (gateway or another anchor) === */

static int anchor_do_pairing(void)
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

		err = receive(RX_HANDLE);
		if (err) {
			ALL_ERR("Receive failed, err %d", err);
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
				       PAIR_STATUS_SUCCESS);
		if (err) {
			ALL_ERR("Failed to send pair confirm, err %d", err);
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

		err = node_store_identity(&identity);
		if (err) {
			ALL_ERR("Failed to store identity, err %d", err);
			return err;
		}

		my_hop_num = best->hop_num + 1;
		parent_id = best->device_id;

		ALL_INF("Paired with %s ID:%d (parent hop:%d, my hop:%d)",
			device_type_str(best->device_type),
			best->device_id, best->hop_num, my_hop_num);

		return 0;
	}

	ALL_ERR("Pairing failed after %d attempts", PAIR_RETRY_MAX);
	return -ETIMEDOUT;
}

/* === Packet handlers (RX loop, after paired) === */

static void handle_pair_request(const pair_request_packet_t *pkt, int16_t rssi_2)
{
	ALL_INF("Pair request from %s ID:%d (RSSI:%d)",
		device_type_str(pkt->device_type), pkt->device_id, rssi_2 / 2);

	uint32_t hash = compute_pair_hash(pkt->device_id, pkt->random_num);

	int err = send_pair_response(TX_HANDLE, pkt->device_id, hash);
	if (err) {
		ALL_ERR("Failed to send pair response, err %d", err);
		return;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	ALL_INF("Pair response sent to ID:%d", pkt->device_id);
}

static void handle_pair_confirm(const pair_confirm_packet_t *pkt)
{
	/* Only accept confirms addressed to us */
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

	int err = paired_store_add(store, pkt->device_id,
				   pkt->version_major, pkt->version_minor,
				   pkt->version_patch);

	if (err) {
		ALL_ERR("Failed to store %s ID:%d, err %d",
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
		ALL_INF("Data from ID:%d (%d bytes, RSSI:%d) CRC OK",
			pkt->src_device_id, payload_len, rssi_2 / 2);
	} else {
		ALL_WRN("Data from ID:%d (%d bytes, RSSI:%d) CRC FAIL",
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
		ALL_ERR("Failed to send data ACK, err %d", err);
		return;
	}

	ALL_INF("Data ACK sent to ID:%d (status:%s)", pkt->src_device_id,
		status == DATA_ACK_SUCCESS ? "OK" : "CRC_FAIL");

	/* If we have a pending OTA image, check the stored version for this
	 * device before sending. Skip if already at the staging version. */
	if (ota_store_has_valid_image()) {
		ota_image_header_t ota_hdr;
		if (ota_store_read_header(&ota_hdr) == 0 &&
		    ota_hdr.magic == OTA_HEADER_MAGIC) {
			paired_device_info_t dev_info;
			bool needs_ota = true;

			if (paired_store_find(&sensor_store,
					      pkt->src_device_id,
					      &dev_info) == 0) {
				if (dev_info.version_major == ota_hdr.version_major &&
				    dev_info.version_minor == ota_hdr.version_minor &&
				    dev_info.version_patch == ota_hdr.version_patch) {
					needs_ota = false;
				}
			}

			if (needs_ota) {
				int ota_err = ota_send_to_device(TX_HANDLE, RX_HANDLE,
								 pkt->src_device_id);
				if (ota_err == 0) {
					ALL_INF("OTA: sensor ID:%d updated",
						pkt->src_device_id);
					paired_store_update_version(
						&sensor_store, pkt->src_device_id,
						ota_hdr.version_major,
						ota_hdr.version_minor,
						ota_hdr.version_patch);
				} else if (ota_err == -EALREADY) {
					paired_store_update_version(
						&sensor_store, pkt->src_device_id,
						ota_hdr.version_major,
						ota_hdr.version_minor,
						ota_hdr.version_patch);
				} else if (ota_err != -ENOENT) {
					ALL_WRN("OTA: sensor ID:%d failed, err %d",
						pkt->src_device_id, ota_err);
				}
			}
		}
	}

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
		ALL_ERR("Failed to relay data to parent ID:%d, err %d",
			parent_id, err);
		return;
	}

	ALL_INF("Data relayed from ID:%d to parent ID:%d",
		pkt->src_device_id, parent_id);
}

/* === Stream relay: forward request upstream, relay data back down === */

#define STREAM_DURATION_MS  60000
#define STREAM_INTERVAL_MS  500

static void handle_stream_request(const stream_request_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	uint16_t requester_id = pkt->src_device_id;

	ALL_INF("Stream request from ID:%d -- forwarding to parent ID:%d",
		requester_id, parent_id);

	/* Forward request upstream, with ourselves as the target so
	 * the gateway/parent streams back to us */
	stream_request_packet_t fwd = {
		.packet_type   = PACKET_TYPE_STREAM_REQUEST,
		.src_device_id = device_id,
		.dst_device_id = parent_id,
	};

	int err = transmit_and_wait(&fwd, sizeof(fwd));
	if (err) {
		ALL_ERR("Failed to forward stream request, err %d", err);
		return;
	}

	/* Listen and relay: receive from parent, retransmit to sensor */
	ALL_INF("Relaying stream data to ID:%d (60s)...", requester_id);

	err = receive_ms(RX_HANDLE, STREAM_DURATION_MS);
	if (err) {
		ALL_ERR("Receive for stream relay failed, err %d", err);
		return;
	}

	struct rx_queue_item item;
	uint32_t relayed = 0;

	while (k_sem_take(&operation_sem, K_MSEC(100)) != 0) {
		while (rx_queue_get(&item, K_NO_WAIT) == 0) {
			if (item.len < 1 || item.data[0] != PACKET_TYPE_DATA) {
				continue;
			}

			/* Cancel RX briefly to transmit relay packet */
			nrf_modem_dect_phy_cancel(RX_HANDLE);
			k_sem_take(&operation_sem, K_FOREVER);
			if (k_sem_take(&operation_sem, K_MSEC(100)) != 0) {
				k_sleep(K_MSEC(50));
			}
			k_sem_reset(&operation_sem);

			/* Rewrite destination to the requesting sensor */
			uint8_t relay_buf[DATA_LEN_MAX];
			uint16_t relay_len = item.len < DATA_LEN_MAX
					   ? item.len : DATA_LEN_MAX;
			memcpy(relay_buf, item.data, relay_len);

			/* Overwrite src to anchor, dst to requesting sensor */
			uint16_t src_id = device_id;
			uint16_t dst_id = requester_id;
			memcpy(&relay_buf[1], &src_id, sizeof(src_id));
			memcpy(&relay_buf[3], &dst_id, sizeof(dst_id));

			err = transmit_and_wait(relay_buf, relay_len);
			if (err) {
				ALL_WRN("Stream relay TX failed, err %d", err);
			} else {
				relayed++;
				if (relayed % 10 == 0) {
					ALL_INF("Stream relayed: %d", relayed);
				}
			}

			/* Resume receiving */
			err = receive_ms(RX_HANDLE, STREAM_DURATION_MS);
			if (err) {
				ALL_ERR("Resume RX failed, err %d", err);
				goto stream_done;
			}
		}
	}

	/* Drain remaining */
	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		relayed++;
	}

stream_done:
	ALL_INF("Stream relay done: %d packets relayed to ID:%d",
		relayed, requester_id);
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
						ALL_INF("Parent ACK from ID:%d: SUCCESS",
							ack->src_device_id);
					} else {
						ALL_WRN("Parent ACK from ID:%d: CRC FAIL",
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

		case PACKET_TYPE_OTA_INIT:
			if (item.len >= OTA_INIT_PACKET_SIZE) {
				ota_handle_init(TX_HANDLE,
					(const ota_init_packet_t *)item.data);
			}
			break;

		case PACKET_TYPE_OTA_ACK:
			/* Not applicable for anchor as receiver */
			break;

		case PACKET_TYPE_STREAM_REQUEST:
			if (item.len >= STREAM_REQUEST_PACKET_SIZE) {
				handle_stream_request(
					(const stream_request_packet_t *)item.data);
			}
			break;

		case PACKET_TYPE_LARGE_DATA_ACK:
		case PACKET_TYPE_LARGE_DATA_NACK:
			/* Not applicable here */
			break;

		default:
			ALL_WRN("Unknown packet type 0x%02x", pkt_type);
			break;
		}
	}
}

/* === Anchor entry point === */

/* AT command handler paired store pointers (defined in at_cmd.c) */
extern const void *gw_anchor_store_ptr;
extern const void *gw_sensor_store_ptr;

void anchor_main(void)
{
	/* Set paired store pointers for AT command handler */
	gw_anchor_store_ptr = &anchor_store;
	gw_sensor_store_ptr = &sensor_store;

	ALL_INF("Anchor mode started (ID:%d)", device_id);

	/* Check if already paired with a parent */
	node_identity_t identity;

	if (node_has_identity() &&
	    node_load_identity(&identity) == 0) {
		my_hop_num = identity.parent_hop + 1;
		parent_id = identity.parent_id;
		ALL_INF("Already paired with parent ID:%d (parent hop:%d, my hop:%d)",
			identity.parent_id, identity.parent_hop, my_hop_num);
	} else {
		ALL_INF("Not paired, starting discovery...");
		int err = anchor_do_pairing();
		if (err) {
			ALL_ERR("Pairing failed, err %d", err);
			return;
		}
	}

	node_identity_t self;
	if (node_load_identity(&self) == 0) {
		ALL_INF("Anchor: ID:%d parent:%d parent_hop:%d my_hop:%d",
			self.device_id, self.parent_id, self.parent_hop,
			self.parent_hop + 1);
	}
	paired_store_print(&anchor_store);
	paired_store_print(&sensor_store);

	/* On boot, if staging has a valid OTA image, distribute to children.
	 * This handles the case where the anchor just rebooted after self-update
	 * and needs to propagate the image to its own sensors/anchors. */
	if (ota_store_has_valid_image()) {
		ALL_INF("OTA: valid staging image found, distributing to children...");
		ota_distribute_to_children(TX_HANDLE, RX_HANDLE,
					   &anchor_store, &sensor_store);
	}

	/* RX loop — receive from children, respond and relay */
	while (true) {
		int err = receive(RX_HANDLE);
		if (err) {
			ALL_ERR("Receive failed, err %d", err);
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

		/* Send any pending large data ACKs */
		large_data_send_pending_ack(TX_HANDLE);

		/* Handle completed large data sessions */
		if (!large_data_any_active_session()) {
			uint8_t ld_slot;
			uint32_t ld_size;
			uint8_t ld_file_type;
			uint16_t ld_src_id;

			if (large_data_get_completed(&ld_slot, &ld_size,
						     &ld_file_type,
						     &ld_src_id)) {
				if (ld_file_type == LARGE_DATA_FILE_OTA) {
					/* OTA image: stage with version info
					 * for redistribution, then self-update
					 * via MCUboot secondary and reboot.
					 * After reboot, anchor_main will detect
					 * valid staging image and distribute
					 * to children before entering RX loop. */
					ALL_INF("OTA: received %d bytes from ID:%d, staging and applying...",
						ld_size, ld_src_id);
					large_data_free_completed(ld_src_id);
					ota_stage_and_apply(ld_slot, ld_size);
					/* Does not return on success */
				}

				/* Non-OTA: relay to parent */
				ALL_INF("Relaying %d bytes from ID:%d to parent ID:%d (PSRAM slot:%d)",
					ld_size, ld_src_id, parent_id,
					ld_slot);

				uint8_t *relay_buf = k_malloc(ld_size);

				if (!relay_buf) {
					ALL_ERR("Failed to allocate %d bytes for relay",
						ld_size);
					large_data_free_completed(ld_src_id);
				} else {
					uint32_t psram_addr =
						(uint32_t)ld_slot * LARGE_DATA_SLOT_SIZE;
					int rerr = psram_read(psram_addr,
							      relay_buf,
							      ld_size);

					if (rerr) {
						ALL_ERR("PSRAM read failed, err %d",
							rerr);
					} else {
						int relay_err = large_data_send(
							TX_HANDLE, RX_HANDLE,
							parent_id,
							ld_file_type,
							relay_buf, ld_size);
						if (relay_err) {
							ALL_ERR("Failed to relay large data, err %d",
								relay_err);
						} else {
							ALL_INF("Large data relay to parent complete");
						}
					}

					k_free(relay_buf);
					large_data_free_completed(ld_src_id);
				}
			}
		}

		/* Clean up sessions that timed out (e.g. sender gave up) */
		large_data_cleanup_stale_sessions();

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
