/*
 * External flash storage for large data reassembly
 *
 * Uses the raw flash driver API on the external SPI flash (GD25WB256).
 * Base address comes from Partition Manager.
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "flash_store.h"

LOG_MODULE_REGISTER(flash_store, CONFIG_FLASH_STORE_LOG_LEVEL);

#define EXT_FLASH_NODE DT_NODELABEL(gd25wb256)

static const struct device *flash_dev;
static bool flash_ready;

int flash_store_init(void)
{
	flash_dev = DEVICE_DT_GET(EXT_FLASH_NODE);
	if (!device_is_ready(flash_dev)) {
		LOG_ERR("External flash device not ready");
		return -ENODEV;
	}

	flash_ready = true;
	LOG_INF("External flash initialized");
	return 0;
}

int flash_store_erase_slot(uint8_t slot)
{
	if (!flash_ready || slot >= FLASH_STORE_MAX_SLOTS) {
		return -EINVAL;
	}

	uint32_t offset = FLASH_STORE_BASE_OFFSET +
			  (uint32_t)slot * FLASH_STORE_SLOT_SIZE;

	int err = flash_erase(flash_dev, offset, FLASH_STORE_SLOT_SIZE);

	if (err) {
		LOG_ERR("Flash erase slot %d failed, err %d", slot, err);
	}

	return err;
}

int flash_store_write(uint8_t slot, uint32_t offset,
		      const void *data, uint16_t len)
{
	if (!flash_ready || slot >= FLASH_STORE_MAX_SLOTS) {
		return -EINVAL;
	}

	uint32_t abs_offset = FLASH_STORE_BASE_OFFSET +
			     (uint32_t)slot * FLASH_STORE_SLOT_SIZE + offset;

	return flash_write(flash_dev, abs_offset, data, len);
}

int flash_store_read(uint8_t slot, uint32_t offset,
		     void *buf, uint16_t len)
{
	if (!flash_ready || slot >= FLASH_STORE_MAX_SLOTS) {
		return -EINVAL;
	}

	uint32_t abs_offset = FLASH_STORE_BASE_OFFSET +
			     (uint32_t)slot * FLASH_STORE_SLOT_SIZE + offset;

	return flash_read(flash_dev, abs_offset, buf, len);
}

uint16_t flash_store_compute_crc16(uint8_t slot, uint32_t total_size)
{
	uint8_t buf[FLASH_STORE_READ_CHUNK];
	uint16_t crc = 0xFFFF;
	uint32_t remaining = total_size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > FLASH_STORE_READ_CHUNK) ?
				 FLASH_STORE_READ_CHUNK : (uint16_t)remaining;

		int err = flash_store_read(slot, offset, buf, chunk);

		if (err) {
			LOG_ERR("Flash read for CRC failed at offset %d, err %d",
				offset, err);
			return 0;
		}

		/* CRC-16/CCITT (same algorithm as compute_crc16 in mesh.c) */
		for (uint16_t i = 0; i < chunk; i++) {
			crc ^= (uint16_t)buf[i] << 8;
			for (int j = 0; j < 8; j++) {
				if (crc & 0x8000) {
					crc = (crc << 1) ^ 0x1021;
				} else {
					crc <<= 1;
				}
			}
		}

		offset += chunk;
		remaining -= chunk;
	}

	return crc;
}
