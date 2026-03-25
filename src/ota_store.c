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

int ota_store_copy_secondary_to_staging(void)
{
	if (!ota_ready) {
		return -ENODEV;
	}

	LOG_INF("OTA: copying secondary slot to staging...");

	/* Read the signed image from MCUboot secondary slot (external flash).
	 * This works when called BEFORE reboot — after SMP upload writes the
	 * .signed.bin to secondary, we copy it to staging before MCUboot
	 * swaps it. The image includes the MCUboot header + app + TLV. */
	const struct flash_area *fa;
	int err = flash_area_open(PM_MCUBOOT_SECONDARY_ID, &fa);

	if (err) {
		LOG_ERR("OTA: failed to open secondary slot, err %d", err);
		return err;
	}

	/* Read MCUboot image header to get actual image size.
	 * MCUboot header: magic(4) + load_addr(4) + hdr_size(2) +
	 * protect_tlv_size(2) + img_size(4) + flags(4) + ver(8) + pad(4) = 32 bytes */
	uint8_t mcuboot_hdr[32];

	err = flash_area_read(fa, 0, mcuboot_hdr, sizeof(mcuboot_hdr));
	if (err) {
		LOG_ERR("OTA: secondary header read failed, err %d", err);
		flash_area_close(fa);
		return err;
	}

	uint32_t mcuboot_magic;

	memcpy(&mcuboot_magic, &mcuboot_hdr[0], 4);
	if (mcuboot_magic != 0x96f3b83d) {
		LOG_WRN("OTA: no valid image in secondary slot (magic:0x%08x)",
			mcuboot_magic);
		flash_area_close(fa);
		return -ENOENT;
	}

	uint16_t hdr_size;
	uint16_t protect_tlv_size;
	uint32_t img_size;

	memcpy(&hdr_size, &mcuboot_hdr[8], 2);
	memcpy(&protect_tlv_size, &mcuboot_hdr[10], 2);
	memcpy(&img_size, &mcuboot_hdr[12], 4);

	/* .signed.bin layout:
	 *   [header (hdr_size)]
	 *   [image body (img_size)]
	 *   [protected TLV area (protect_tlv_size)] — optional
	 *   [unprotected TLV info header (4B): magic 0x6907 + total_len]
	 *   [unprotected TLV entries (total_len)]
	 *
	 * Read the unprotected TLV info header to get the final size. */
	uint32_t tlv_info_off = hdr_size + img_size + protect_tlv_size;
	uint8_t tlv_info[4];

	err = flash_area_read(fa, tlv_info_off, tlv_info, sizeof(tlv_info));
	if (err) {
		LOG_ERR("OTA: TLV read failed at 0x%x, err %d",
			tlv_info_off, err);
		flash_area_close(fa);
		return err;
	}

	uint16_t tlv_magic, tlv_len;

	memcpy(&tlv_magic, &tlv_info[0], 2);
	memcpy(&tlv_len, &tlv_info[2], 2);

	uint32_t total_size;

	if (tlv_magic == 0x6907) {
		total_size = tlv_info_off + 4 + tlv_len;
	} else {
		LOG_WRN("OTA: unexpected TLV magic 0x%04x at 0x%x",
			tlv_magic, tlv_info_off);
		/* Fallback: just use header + image + protected TLV */
		total_size = tlv_info_off;
	}

	/* Sanity check */
	if (total_size < hdr_size || total_size > fa->fa_size) {
		LOG_ERR("OTA: invalid image size %d", total_size);
		flash_area_close(fa);
		return -EINVAL;
	}

	/* Read version from MCUboot image header (offset 20) */
	uint8_t ver_major = mcuboot_hdr[20];
	uint8_t ver_minor = mcuboot_hdr[21];
	uint16_t ver_patch;

	memcpy(&ver_patch, &mcuboot_hdr[22], 2);

	LOG_INF("OTA: secondary image: v%d.%d.%d, %d bytes",
		ver_major, ver_minor, ver_patch, total_size);

	/* Erase staging slot */
	err = ota_store_erase_staging();
	if (err) {
		LOG_ERR("OTA: staging erase failed, err %d", err);
		flash_area_close(fa);
		return err;
	}

	/* Copy secondary → staging data area, computing CRC as we go */
	uint8_t buf[COPY_CHUNK_SIZE];
	uint16_t crc = 0xFFFF;
	uint32_t remaining = total_size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > COPY_CHUNK_SIZE) ?
				 COPY_CHUNK_SIZE : (uint16_t)remaining;

		err = flash_area_read(fa, offset, buf, chunk);
		if (err) {
			LOG_ERR("OTA: secondary read failed at 0x%x, err %d",
				offset, err);
			flash_area_close(fa);
			return err;
		}

		crc = compute_crc16_continue(crc, buf, chunk);

		err = ota_store_write_staging(offset, buf, chunk);
		if (err) {
			LOG_ERR("OTA: staging write failed at 0x%x, err %d",
				offset, err);
			flash_area_close(fa);
			return err;
		}

		offset += chunk;
		remaining -= chunk;

		if (offset % (64 * 1024) == 0) {
			LOG_INF("OTA: staging progress %d/%d bytes",
				offset, total_size);
		}
	}

	flash_area_close(fa);

	/* Write OTA header with version from the image */
	ota_image_header_t new_hdr = {
		.magic = OTA_HEADER_MAGIC,
		.image_size = total_size,
		.version_major = ver_major,
		.version_minor = ver_minor,
		.version_patch = ver_patch,
		.crc16 = crc,
	};

	err = ota_store_write_header(&new_hdr);
	if (err) {
		LOG_ERR("OTA: staging header write failed, err %d", err);
		return err;
	}

	LOG_INF("OTA: staging populated (v%d.%d.%d, %d bytes, CRC:0x%04x)",
		ver_major, ver_minor, ver_patch, total_size, crc);
	return 0;
}
