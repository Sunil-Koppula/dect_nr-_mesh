/*
 * Device identity and paired-device registry for DECT NR+ mesh network
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "identity.h"
#include "nvs_store.h"
#include "log_all.h"

LOG_MODULE_REGISTER(identity, CONFIG_IDENTITY_LOG_LEVEL);

/* ===== Node identity helpers ===== */

int node_store_identity(const node_identity_t *id)
{
	return storage_write(NVS_IDENTITY_KEY, id, sizeof(*id));
}

int node_load_identity(node_identity_t *id)
{
	return storage_read(NVS_IDENTITY_KEY, id, sizeof(*id));
}

bool node_has_identity(void)
{
	return storage_exists(NVS_IDENTITY_KEY);
}

/* ===== Paired device registry ===== */

static int find_slot(const paired_store_t *ps, uint16_t dev_id, bool *found)
{
	paired_device_info_t info;
	int first_empty = -1;

	*found = false;
	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &info, sizeof(info)) == 0) {
			if (info.device_id == dev_id) {
				*found = true;
				return i;
			}
		} else if (first_empty < 0) {
			first_empty = i;
		}
	}
	return first_empty;
}

int paired_store_add(const paired_store_t *ps, uint16_t dev_id,
		     uint8_t ver_major, uint8_t ver_minor, uint16_t ver_patch)
{
	bool found;
	int slot = find_slot(ps, dev_id, &found);

	if (found) {
		/* Update version if already present */
		paired_device_info_t info = {
			.device_id = dev_id,
			.version_major = ver_major,
			.version_minor = ver_minor,
			.version_patch = ver_patch,
		};
		return storage_write(ps->nvs_base + slot, &info, sizeof(info));
	}
	if (slot < 0) {
		LOG_WRN("%s storage full", ps->label);
		return -ENOMEM;
	}

	paired_device_info_t info = {
		.device_id = dev_id,
		.version_major = ver_major,
		.version_minor = ver_minor,
		.version_patch = ver_patch,
	};
	return storage_write(ps->nvs_base + slot, &info, sizeof(info));
}

bool paired_store_contains(const paired_store_t *ps, uint16_t dev_id)
{
	bool found;

	find_slot(ps, dev_id, &found);
	return found;
}

int paired_store_count(const paired_store_t *ps)
{
	int count = 0;
	paired_device_info_t info;

	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &info, sizeof(info)) == 0) {
			count++;
		}
	}
	return count;
}

int paired_store_get(const paired_store_t *ps, int index, uint16_t *dev_id)
{
	if (index < 0 || index >= ps->max_entries) {
		return -EINVAL;
	}
	paired_device_info_t info;
	int err = storage_read(ps->nvs_base + index, &info, sizeof(info));
	if (err) {
		return err;
	}
	*dev_id = info.device_id;
	return 0;
}

int paired_store_get_info(const paired_store_t *ps, int index,
			  paired_device_info_t *info)
{
	if (index < 0 || index >= ps->max_entries) {
		return -EINVAL;
	}
	return storage_read(ps->nvs_base + index, info, sizeof(*info));
}

int paired_store_find(const paired_store_t *ps, uint16_t dev_id,
		      paired_device_info_t *info)
{
	paired_device_info_t tmp;

	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &tmp, sizeof(tmp)) == 0) {
			if (tmp.device_id == dev_id) {
				*info = tmp;
				return 0;
			}
		}
	}
	return -ENOENT;
}

int paired_store_update_version(const paired_store_t *ps, uint16_t dev_id,
				uint8_t ver_major, uint8_t ver_minor,
				uint16_t ver_patch)
{
	bool found;
	int slot = find_slot(ps, dev_id, &found);

	if (!found) {
		return -ENOENT;
	}

	paired_device_info_t info = {
		.device_id = dev_id,
		.version_major = ver_major,
		.version_minor = ver_minor,
		.version_patch = ver_patch,
	};
	return storage_write(ps->nvs_base + slot, &info, sizeof(info));
}

void paired_store_print(const paired_store_t *ps)
{
	paired_device_info_t info;

	ALL_INF("=== Paired %ss (%d) ===", ps->label, paired_store_count(ps));
	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &info, sizeof(info)) == 0) {
			ALL_INF("  [%d] %s ID:%d v%d.%d.%d", i, ps->label,
				info.device_id, info.version_major,
				info.version_minor, info.version_patch);
		}
	}
}
