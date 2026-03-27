/*
 * OTA firmware distribution protocol for DECT NR+ mesh network
 *
 * OTA flow (initiator → target):
 *   1. Send OTA_INIT with new firmware version + image size
 *   2. Wait for OTA_ACK: ACCEPT (version newer) or REJECT (same/older)
 *   3. If accepted, send image via large_data_send_from_flash()
 *      (reuses LARGE_DATA_INIT/TRANSFER/END/ACK/NACK)
 *   4. Target writes received image from PSRAM to MCUboot secondary
 *   5. Target reboots to apply update
 *
 * Distribution order:
 *   - Gateway: OTA to each paired anchor, then each paired sensor
 *   - Anchor:  after self-update + reboot, OTA to own children
 *   - Sensor:  self-update only (leaf node)
 *
 * The OTA image is read from the staging slot on external flash.
 * All device types run the same unified firmware.
 */

#ifndef OTA_H
#define OTA_H

#include <stdint.h>
#include "protocol.h"

/* --- Sender API (gateway / anchor) --- */

/* Distribute OTA to a single target device.
 * Sends OTA_INIT, waits for OTA_ACK, then transfers image via large_data.
 * tx_handle/rx_handle: PHY handles for TX/RX.
 * Returns 0 on success, -EALREADY if target has same/newer version,
 * negative on error. */
int ota_send_to_device(uint32_t tx_handle, uint32_t rx_handle,
		       uint16_t dst_id);

/* Distribute OTA to all paired children (anchors first, then sensors).
 * Reads the paired device stores and calls ota_send_to_device() for each.
 * ps_anchor/ps_sensor: paired stores for this device's children.
 * Returns number of devices successfully updated. */
int ota_distribute_to_children(uint32_t tx_handle, uint32_t rx_handle,
			       const void *ps_anchor,
			       const void *ps_sensor);

/* --- Receiver API (anchor / sensor) --- */

/* Handle an incoming OTA_INIT packet.
 * Compares version with running firmware.
 * Sends OTA_ACK (ACCEPT/REJECT) via tx_handle.
 * If accepted, prepares to receive large_data OTA image.
 * Returns true if OTA was accepted (caller should expect large_data next). */
bool ota_handle_init(uint32_t tx_handle,
		     const ota_init_packet_t *pkt);

/* Called after a completed large_data session with file_type == LARGE_DATA_FILE_OTA.
 * Writes PSRAM data to MCUboot secondary slot and reboots.
 * Does not return on success. */
void ota_apply_from_psram(uint8_t psram_slot, uint32_t size);

/* Store OTA image from PSRAM to staging flash (for redistribution after reboot)
 * and then self-update via MCUboot secondary + reboot.
 * Preserves the version info in the staging header so children can be updated.
 * Does not return on success. */
void ota_stage_and_apply(uint8_t psram_slot, uint32_t size);

/* Get the pending OTA version (set by ota_handle_init when accepted).
 * Returns false if no OTA is pending. */
bool ota_get_pending_version(uint8_t *major, uint8_t *minor, uint16_t *patch);

#endif /* OTA_H */
