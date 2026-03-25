/*
 * OTA image storage and MCUboot update management
 *
 * Single staging slot using raw flash API on external SPI flash.
 * MCUboot secondary slot uses flash_area API for proper trailer handling.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/sys/reboot.h>
#include <pm_config.h>
#include "ota_store.h"
#include "psram.h"
#include "mesh.h"

LOG_MODULE_DECLARE(app);

#define EXT_FLASH_NODE   DT_NODELABEL(gd25wb256)
#define COPY_CHUNK_SIZE  512

/* Staging slot placed immediately after mcuboot_secondary on external flash.
 * PM_MCUBOOT_SECONDARY_END_ADDRESS gives us the first free byte. */
#define STAGING_BASE  PM_MCUBOOT_SECONDARY_END_ADDRESS

static const struct device *flash_dev;
static bool ota_ready;

int ota_store_init(void)
{
	flash_dev = DEVICE_DT_GET(EXT_FLASH_NODE);
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("OTA: external flash not ready");
		return -ENODEV;
	}

	ota_ready = true;
	LOG_INF("OTA store initialized");
	return 0;
}

int ota_store_erase_staging(void)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	LOG_INF("OTA: erasing staging slot");
	return flash_erase(flash_dev, STAGING_BASE, OTA_STAGING_SLOT_SIZE);
}

int ota_store_write_header(const ota_image_header_t *hdr)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	return flash_write(flash_dev, STAGING_BASE, hdr, OTA_HEADER_SIZE);
}

int ota_store_read_header(ota_image_header_t *hdr)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	return flash_read(flash_dev, STAGING_BASE, hdr, OTA_HEADER_SIZE);
}

int ota_store_write_staging(uint32_t offset, const void *data, uint16_t len)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	return flash_write(flash_dev, STAGING_BASE + OTA_HEADER_SIZE + offset,
			   data, len);
}

int ota_store_read_staging(uint32_t offset, void *buf, uint16_t len)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	return flash_read(flash_dev, STAGING_BASE + OTA_HEADER_SIZE + offset,
			  buf, len);
}

bool ota_store_has_valid_image(void)
{
	if (!ota_ready) {
		return false;
	}

	ota_image_header_t hdr;
	int err = ota_store_read_header(&hdr);

	if (err || hdr.magic != OTA_HEADER_MAGIC || hdr.image_size == 0) {
		return false;
	}

	/* Verify CRC over the image data */
	uint8_t buf[COPY_CHUNK_SIZE];
	uint16_t crc = 0xFFFF;
	uint32_t remaining = hdr.image_size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > COPY_CHUNK_SIZE) ?
				 COPY_CHUNK_SIZE : (uint16_t)remaining;

		err = ota_store_read_staging(offset, buf, chunk);
		if (err) {
			return false;
		}

		crc = compute_crc16_continue(crc, buf, chunk);
		offset += chunk;
		remaining -= chunk;
	}

	return crc == hdr.crc16;
}

int ota_store_write_psram_to_secondary(uint8_t psram_slot, uint32_t size)
{
	const struct flash_area *fa;
	int err;

	err = flash_area_open(PM_MCUBOOT_SECONDARY_ID, &fa);
	if (err) {
		LOG_ERR("OTA: failed to open secondary slot, err %d", err);
		return err;
	}

	LOG_INF("OTA: erasing secondary slot (%d bytes)", size);
	err = flash_area_erase(fa, 0, fa->fa_size);
	if (err) {
		LOG_ERR("OTA: secondary erase failed, err %d", err);
		flash_area_close(fa);
		return err;
	}

	LOG_INF("OTA: writing %d bytes from PSRAM slot %d to secondary",
		size, psram_slot);

	uint8_t buf[COPY_CHUNK_SIZE];
	uint32_t psram_addr = (uint32_t)psram_slot * (256 * 1024);
	uint32_t remaining = size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > COPY_CHUNK_SIZE) ?
				 COPY_CHUNK_SIZE : (uint16_t)remaining;

		err = psram_read(psram_addr + offset, buf, chunk);
		if (err) {
			LOG_ERR("OTA: PSRAM read failed at 0x%x, err %d",
				psram_addr + offset, err);
			flash_area_close(fa);
			return err;
		}

		err = flash_area_write(fa, offset, buf, chunk);
		if (err) {
			LOG_ERR("OTA: secondary write failed at 0x%x, err %d",
				offset, err);
			flash_area_close(fa);
			return err;
		}

		offset += chunk;
		remaining -= chunk;
	}

	flash_area_close(fa);
	LOG_INF("OTA: secondary slot written successfully");
	return 0;
}

int ota_store_write_staging_to_secondary(void)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	ota_image_header_t hdr;
	int err = ota_store_read_header(&hdr);

	if (err || hdr.magic != OTA_HEADER_MAGIC) {
		LOG_ERR("OTA: invalid header in staging slot");
		return -EINVAL;
	}

	const struct flash_area *fa;

	err = flash_area_open(PM_MCUBOOT_SECONDARY_ID, &fa);
	if (err) {
		LOG_ERR("OTA: failed to open secondary slot, err %d", err);
		return err;
	}

	LOG_INF("OTA: erasing secondary slot");
	err = flash_area_erase(fa, 0, fa->fa_size);
	if (err) {
		LOG_ERR("OTA: secondary erase failed, err %d", err);
		flash_area_close(fa);
		return err;
	}

	LOG_INF("OTA: writing %d bytes from staging to secondary",
		hdr.image_size);

	uint8_t buf[COPY_CHUNK_SIZE];
	uint32_t remaining = hdr.image_size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > COPY_CHUNK_SIZE) ?
				 COPY_CHUNK_SIZE : (uint16_t)remaining;

		err = ota_store_read_staging(offset, buf, chunk);
		if (err) {
			LOG_ERR("OTA: staging read failed at 0x%x, err %d",
				offset, err);
			flash_area_close(fa);
			return err;
		}

		err = flash_area_write(fa, offset, buf, chunk);
		if (err) {
			LOG_ERR("OTA: secondary write failed at 0x%x, err %d",
				offset, err);
			flash_area_close(fa);
			return err;
		}

		offset += chunk;
		remaining -= chunk;
	}

	flash_area_close(fa);
	LOG_INF("OTA: secondary slot written from staging");
	return 0;
}

int ota_store_apply_and_reboot(void)
{
	int err = boot_request_upgrade(BOOT_UPGRADE_TEST);

	if (err) {
		LOG_ERR("OTA: boot_request_upgrade failed, err %d", err);
		return err;
	}

	LOG_WRN("OTA: rebooting to apply update...");
	k_sleep(K_MSEC(500));
	sys_reboot(SYS_REBOOT_COLD);

	/* Should not reach here */
	return 0;
}

int ota_store_confirm_image(void)
{
	if (!boot_is_img_confirmed()) {
		int err = boot_write_img_confirmed();

		if (err) {
			LOG_ERR("OTA: image confirm failed, err %d", err);
			return err;
		}
		LOG_INF("OTA: image confirmed");
	}

	return 0;
}
