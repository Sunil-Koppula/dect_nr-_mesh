/*
 * OTA image storage and MCUboot update management
 *
 * Single unified firmware for all device types (gateway/anchor/sensor).
 * One staging slot on external flash holds the OTA image for distribution.
 *
 * External flash layout (managed by Partition Manager):
 *   mcuboot_secondary  — MCUboot swap slot (auto-sized)
 *   ota_staging        — 512KB (single OTA image)
 */

#ifndef OTA_STORE_H
#define OTA_STORE_H

#include <stdint.h>
#include <stdbool.h>
#include <pm_config.h>

#define OTA_STAGING_SLOT_SIZE  0x80000  /* 512KB */

/* OTA image header — stored at the start of the staging slot */
typedef struct {
	uint32_t magic;         /* OTA_HEADER_MAGIC */
	uint32_t image_size;    /* OTA image data size (after header) */
	uint8_t  version_major;
	uint8_t  version_minor;
	uint16_t version_patch;
	uint16_t crc16;         /* CRC16 over image data */
	uint8_t  reserved[2];
} __attribute__((packed)) ota_image_header_t;

#define OTA_HEADER_MAGIC  0x4F544148  /* "OTAH" */
#define OTA_HEADER_SIZE   sizeof(ota_image_header_t)

/* Initialize OTA store (call after flash_store_init) */
int ota_store_init(void);

/* Erase the staging slot */
int ota_store_erase_staging(void);

/* Write the header to the staging slot */
int ota_store_write_header(const ota_image_header_t *hdr);

/* Read the header from the staging slot */
int ota_store_read_header(ota_image_header_t *hdr);

/* Write image data to the staging slot (offset is relative to after header) */
int ota_store_write_staging(uint32_t offset, const void *data, uint16_t len);

/* Read image data from the staging slot (offset is relative to after header) */
int ota_store_read_staging(uint32_t offset, void *buf, uint16_t len);

/* Check if the staging slot contains a valid OTA image (magic + CRC) */
bool ota_store_has_valid_image(void);

/* Write OTA image from PSRAM slot to MCUboot secondary slot.
 * Used when a device receives OTA via large_data and wants to self-update. */
int ota_store_write_psram_to_secondary(uint8_t psram_slot, uint32_t size);

/* Write OTA image from staging slot to MCUboot secondary slot. */
int ota_store_write_staging_to_secondary(void);

/* Mark the secondary image as pending test and reboot */
int ota_store_apply_and_reboot(void);

/* Confirm the currently running image (prevents MCUboot revert) */
int ota_store_confirm_image(void);

#endif /* OTA_STORE_H */
