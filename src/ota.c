/*
 * OTA firmware distribution protocol for DECT NR+ mesh network
 *
 * Sender: sends OTA_INIT with version, waits for OTA_ACK,
 *         then transfers image via large_data_send_from_flash().
 *
 * Receiver: compares version, sends OTA_ACK, receives image
 *           via large_data into PSRAM, writes to MCUboot secondary,
 *           and reboots.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/app_version.h>
#include <pm_config.h>
#include "ota.h"
#include "ota_store.h"
#include "large_data.h"
#include "identity.h"
#include "psram.h"
#include "radio.h"
#include "queue.h"
#include "crc.h"
#include "mesh_tx.h"
#include "log_all.h"

LOG_MODULE_DECLARE(app);

/* Staging slot address (image data starts after OTA header) */
#define STAGING_DATA_OFFSET (PM_MCUBOOT_SECONDARY_END_ADDRESS + OTA_HEADER_SIZE)

/* Pending OTA version — set by ota_handle_init when accepted */
static bool ota_version_pending;
static uint8_t ota_pending_major;
static uint8_t ota_pending_minor;
static uint16_t ota_pending_patch;

/* Returns true if offered version is newer than running version */
static bool version_is_newer(uint8_t major, uint8_t minor, uint16_t patch)
{
	if (major != APP_VERSION_MAJOR) {
		return major > APP_VERSION_MAJOR;
	}
	if (minor != APP_VERSION_MINOR) {
		return minor > APP_VERSION_MINOR;
	}
	return patch > APP_PATCHLEVEL;
}

/* ===== Sender API ===== */

int ota_send_to_device(uint32_t tx_handle, uint32_t rx_handle,
		       uint16_t dst_id)
{
	/* Read OTA header from staging slot */
	ota_image_header_t hdr;
	int err = ota_store_read_header(&hdr);

	if (err || hdr.magic != OTA_HEADER_MAGIC || hdr.image_size == 0) {
		ALL_ERR("OTA: no valid image in staging slot");
		return -ENOENT;
	}

	ALL_INF("OTA: offering v%d.%d.%d (%d bytes) to ID:%d",
		hdr.version_major, hdr.version_minor, hdr.version_patch,
		hdr.image_size, dst_id);

	/* Step 1: Send OTA_INIT */
	ota_init_packet_t init_pkt = {
		.packet_type = PACKET_TYPE_OTA_INIT,
		.src_device_id = device_id,
		.dst_device_id = dst_id,
		.version_major = hdr.version_major,
		.version_minor = hdr.version_minor,
		.version_patch = hdr.version_patch,
		.image_size = hdr.image_size,
	};

	err = transmit(tx_handle, &init_pkt, sizeof(init_pkt));
	if (err) {
		ALL_ERR("OTA: failed to send OTA_INIT, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	/* Step 2: Wait for OTA_ACK */
	err = receive_ms(rx_handle, 5000);
	if (err) {
		ALL_ERR("OTA: receive for OTA_ACK failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	/* Drain queue looking for OTA_ACK */
	struct rx_queue_item item;
	bool got_ack = false;
	uint8_t ack_status = STATUS_REJECTED;

	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		if (item.len < OTA_ACK_PACKET_SIZE) {
			continue;
		}
		if (item.data[0] != PACKET_TYPE_OTA_ACK) {
			continue;
		}
		const ota_ack_packet_t *ack =
			(const ota_ack_packet_t *)item.data;
		if (ack->dst_device_id == device_id) {
			got_ack = true;
			ack_status = ack->status;
			ALL_INF("OTA: ACK from ID:%d status:%s (running v%d.%d.%d)",
				ack->src_device_id,
				ack_status == STATUS_SUCCESS ? "ACCEPT" : "REJECT",
				ack->version_major, ack->version_minor,
				ack->version_patch);
			break;
		}
	}

	/* Drain remaining */
	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
	}

	if (!got_ack) {
		ALL_WRN("OTA: no ACK from ID:%d", dst_id);
		return -ETIMEDOUT;
	}

	if (ack_status == STATUS_REJECTED) {
		ALL_INF("OTA: ID:%d rejected (same or newer version)", dst_id);
		return -EALREADY;
	}

	/* Step 3: Transfer image via large_data from flash staging slot */
	ALL_INF("OTA: starting image transfer to ID:%d (%d bytes)",
		dst_id, hdr.image_size);

	err = large_data_send_from_flash(tx_handle, rx_handle,
					 dst_id, LARGE_DATA_FILE_OTA,
					 STAGING_DATA_OFFSET,
					 hdr.image_size);
	if (err) {
		ALL_ERR("OTA: image transfer to ID:%d failed, err %d",
			dst_id, err);
		return err;
	}

	ALL_INF("OTA: image transfer to ID:%d complete", dst_id);
	return 0;
}

#define OTA_ANCHOR_RETRY_MAX  3
#define OTA_ANCHOR_RETRY_DELAY_MS  5000

int ota_distribute_to_children(uint32_t tx_handle, uint32_t rx_handle,
			       const void *ps_anchor,
			       const void *ps_sensor)
{
	const paired_store_t *anchor_ps = ps_anchor;
	int updated = 0;

	/* Read staging header for version comparison */
	ota_image_header_t staging_hdr;
	int hdr_err = ota_store_read_header(&staging_hdr);

	if (hdr_err || staging_hdr.magic != OTA_HEADER_MAGIC) {
		ALL_ERR("OTA: cannot read staging header");
		return 0;
	}

	/* Distribute to anchors first with retries.
	 * Anchors are always listening, so if they're reachable we can
	 * keep trying until they accept or reject. */
	for (int i = 0; i < anchor_ps->max_entries; i++) {
		paired_device_info_t info;

		if (paired_store_get_info(anchor_ps, i, &info) != 0) {
			continue;
		}

		/* Skip if anchor already has the staging version */
		if (info.version_major == staging_hdr.version_major &&
		    info.version_minor == staging_hdr.version_minor &&
		    info.version_patch == staging_hdr.version_patch) {
			ALL_INF("OTA: anchor ID:%d already at v%d.%d.%d, skip",
				info.device_id, info.version_major,
				info.version_minor, info.version_patch);
			continue;
		}

		bool done = false;

		for (int attempt = 0; attempt < OTA_ANCHOR_RETRY_MAX && !done;
		     attempt++) {
			if (attempt > 0) {
				ALL_INF("OTA: retrying anchor ID:%d (%d/%d)",
					info.device_id, attempt + 1,
					OTA_ANCHOR_RETRY_MAX);
				k_sleep(K_MSEC(OTA_ANCHOR_RETRY_DELAY_MS));
			}

			int err = ota_send_to_device(tx_handle, rx_handle,
						     info.device_id);
			if (err == 0) {
				updated++;
				ALL_INF("OTA: anchor ID:%d updated", info.device_id);
				paired_store_update_version(
					anchor_ps, info.device_id,
					staging_hdr.version_major,
					staging_hdr.version_minor,
					staging_hdr.version_patch);
				done = true;
			} else if (err == -EALREADY) {
				ALL_INF("OTA: anchor ID:%d already up to date",
					info.device_id);
				paired_store_update_version(
					anchor_ps, info.device_id,
					staging_hdr.version_major,
					staging_hdr.version_minor,
					staging_hdr.version_patch);
				done = true;
			} else {
				ALL_WRN("OTA: anchor ID:%d attempt %d failed, err %d",
					info.device_id, attempt + 1, err);
			}
		}

		if (!done) {
			ALL_ERR("OTA: anchor ID:%d failed after %d attempts",
				info.device_id, OTA_ANCHOR_RETRY_MAX);
		}

		k_sleep(K_MSEC(500));
	}

	/* Sensors are NOT retried here — they sleep and only wake to
	 * send data. OTA_INIT is piggybacked on the DATA_ACK response
	 * when the sensor next contacts us (see handle_data). */
	ALL_INF("OTA: anchor distribution complete, %d updated. "
		"Sensors will be updated when they next send data.", updated);
	return updated;
}

/* ===== Receiver API ===== */

bool ota_handle_init(uint32_t tx_handle, const ota_init_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return false;
	}

	ALL_INF("OTA: init from ID:%d offering v%d.%d.%d (%d bytes)",
		pkt->src_device_id, pkt->version_major,
		pkt->version_minor, pkt->version_patch,
		pkt->image_size);

	bool accept = version_is_newer(pkt->version_major,
				       pkt->version_minor,
				       pkt->version_patch);

	ALL_INF("OTA: running v%d.%d.%d, offered v%d.%d.%d -> %s",
		APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_PATCHLEVEL,
		pkt->version_major, pkt->version_minor, pkt->version_patch,
		accept ? "ACCEPT" : "REJECT");

	/* Remember the offered version so we can write it to staging later */
	if (accept) {
		ota_pending_major = pkt->version_major;
		ota_pending_minor = pkt->version_minor;
		ota_pending_patch = pkt->version_patch;
		ota_version_pending = true;
	}

	/* Send OTA_ACK */
	ota_ack_packet_t ack = {
		.packet_type = PACKET_TYPE_OTA_ACK,
		.src_device_id = device_id,
		.dst_device_id = pkt->src_device_id,
		.status = accept ? STATUS_SUCCESS : STATUS_REJECTED,
		.version_major = APP_VERSION_MAJOR,
		.version_minor = APP_VERSION_MINOR,
		.version_patch = APP_PATCHLEVEL,
	};

	int err = transmit(tx_handle, &ack, sizeof(ack));
	if (err) {
		ALL_ERR("OTA: failed to send OTA_ACK, err %d", err);
		return false;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	ALL_INF("OTA: ACK sent (%s)", accept ? "ACCEPT" : "REJECT");
	return accept;
}

void ota_apply_from_psram(uint8_t psram_slot, uint32_t size)
{
	ALL_INF("OTA: writing %d bytes from PSRAM slot %d to MCUboot secondary",
		size, psram_slot);

	int err = ota_store_write_psram_to_secondary(psram_slot, size);
	if (err) {
		ALL_ERR("OTA: write to secondary failed, err %d", err);
		return;
	}

	ALL_INF("OTA: applying update and rebooting...");
	ota_store_apply_and_reboot();
	/* Does not return on success */
}

void ota_stage_and_apply(uint8_t psram_slot, uint32_t size)
{
	uint8_t v_major = 0, v_minor = 0;
	uint16_t v_patch = 0;

	ota_get_pending_version(&v_major, &v_minor, &v_patch);

	ALL_INF("OTA: staging %d bytes (v%d.%d.%d) for redistribution",
		size, v_major, v_minor, v_patch);

	/* Compute CRC over PSRAM data for the staging header */
	uint16_t crc = 0xFFFF;
	uint8_t crc_buf[512];
	uint32_t psram_addr = (uint32_t)psram_slot * LARGE_DATA_SLOT_SIZE;
	uint32_t remaining = size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > sizeof(crc_buf)) ?
				 sizeof(crc_buf) : (uint16_t)remaining;
		psram_read(psram_addr + offset, crc_buf, chunk);
		crc = compute_crc16_continue(crc, crc_buf, chunk);
		offset += chunk;
		remaining -= chunk;
	}

	/* Write staging header with version info */
	ota_store_erase_staging();

	ota_image_header_t hdr = {
		.magic = OTA_HEADER_MAGIC,
		.image_size = size,
		.version_major = v_major,
		.version_minor = v_minor,
		.version_patch = v_patch,
		.crc16 = crc,
	};
	ota_store_write_header(&hdr);

	/* Copy PSRAM data to staging slot */
	remaining = size;
	offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > sizeof(crc_buf)) ?
				 sizeof(crc_buf) : (uint16_t)remaining;
		psram_read(psram_addr + offset, crc_buf, chunk);
		ota_store_write_staging(offset, crc_buf, chunk);
		offset += chunk;
		remaining -= chunk;
	}

	ALL_INF("OTA: staging complete, self-updating...");

	/* Self-update: PSRAM → MCUboot secondary → reboot */
	ota_apply_from_psram(psram_slot, size);
	/* Does not return on success */
}

bool ota_get_pending_version(uint8_t *major, uint8_t *minor, uint16_t *patch)
{
	if (!ota_version_pending) {
		*major = 0;
		*minor = 0;
		*patch = 0;
		return false;
	}
	*major = ota_pending_major;
	*minor = ota_pending_minor;
	*patch = ota_pending_patch;
	return true;
}
