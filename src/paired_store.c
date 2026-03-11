/*
 * Shared paired-device storage helpers for DECT NR+ mesh network
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "paired_store.h"
#include "storage.h"

LOG_MODULE_DECLARE(app);

static int find_slot(const paired_store_t *ps, uint16_t dev_id, bool *found)
{
	uint16_t stored;
	int first_empty = -1;

	*found = false;
	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &stored, sizeof(stored)) == 0) {
			if (stored == dev_id) {
				*found = true;
				return i;
			}
		} else if (first_empty < 0) {
			first_empty = i;
		}
	}
	return first_empty;
}

int paired_store_add(const paired_store_t *ps, uint16_t dev_id)
{
	bool found;
	int slot = find_slot(ps, dev_id, &found);

	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("%s storage full", ps->label);
		return -ENOMEM;
	}
	return storage_write(ps->nvs_base + slot, &dev_id, sizeof(dev_id));
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
	uint16_t tmp;

	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

void paired_store_print(const paired_store_t *ps)
{
	uint16_t id;

	LOG_INF("=== Paired %ss (%d) ===", ps->label, paired_store_count(ps));
	for (int i = 0; i < ps->max_entries; i++) {
		if (storage_read(ps->nvs_base + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] %s ID:%d", i, ps->label, id);
		}
	}
}
