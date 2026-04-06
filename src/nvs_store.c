/*
 * NVS storage layer on external flash for DECT NR+ mesh network
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/devicetree.h>
#include <pm_config.h>
#include "nvs_store.h"

LOG_MODULE_REGISTER(nvs_store, CONFIG_NVS_STORE_LOG_LEVEL);

/* External flash device from the 'nordic,pm-ext-flash' chosen node */
#define EXT_FLASH_DEVICE DEVICE_DT_GET(DT_CHOSEN(nordic_pm_ext_flash))

static struct nvs_fs nvs;
static bool nvs_ready;

int storage_init(void)
{
	int err;
	struct flash_pages_info page_info;

	nvs.flash_device = EXT_FLASH_DEVICE;
	if (!device_is_ready(nvs.flash_device)) {
		LOG_ERR("External flash device not ready");
		return -ENODEV;
	}

	nvs.offset = PM_NVS_STORAGE_OFFSET;
	err = flash_get_page_info_by_offs(nvs.flash_device, nvs.offset, &page_info);
	if (err) {
		LOG_ERR("Failed to get flash page info, err %d", err);
		return err;
	}

	nvs.sector_size = page_info.size;
	nvs.sector_count = PM_NVS_STORAGE_SIZE / page_info.size;

	err = nvs_mount(&nvs);
	if (err) {
		LOG_ERR("NVS mount failed, err %d", err);
		return err;
	}

	nvs_ready = true;
	LOG_INF("NVS storage initialized (external flash, offset 0x%lx, %u sectors)",
		nvs.offset, nvs.sector_count);
	return 0;
}

int storage_write(uint16_t key, const void *data, size_t len)
{
	if (!nvs_ready) {
		return -ENODEV;
	}

	int rc = nvs_write(&nvs, key, data, len);
	if (rc < 0) {
		LOG_ERR("NVS write key 0x%03x failed, err %d", key, rc);
		return rc;
	}
	return 0;
}

int storage_read(uint16_t key, void *data, size_t len)
{
	if (!nvs_ready) {
		return -ENODEV;
	}

	int rc = nvs_read(&nvs, key, data, len);
	if (rc != (int)len) {
		return -ENOENT;
	}
	return 0;
}

int storage_delete(uint16_t key)
{
	if (!nvs_ready) {
		return -ENODEV;
	}
	return nvs_delete(&nvs, key);
}

bool storage_exists(uint16_t key)
{
	if (!nvs_ready) {
		return false;
	}

	uint8_t tmp;
	return nvs_read(&nvs, key, &tmp, sizeof(tmp)) > 0;
}

int storage_clear_all(void)
{
	if (!nvs_ready) {
		return -ENODEV;
	}

	int err = nvs_clear(&nvs);
	if (err) {
		return err;
	}

	/* Re-mount after clear */
	return nvs_mount(&nvs);
}
