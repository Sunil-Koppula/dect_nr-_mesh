/*
 * Anchor device logic for DECT NR+ mesh network
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include "anchor.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../state.h"
#include "../storage.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

/* === Anchor storage helpers === */

int anchor_store_identity(const anchor_identity_t *id)
{
	return storage_write(ANCHOR_IDENTITY_KEY, id, sizeof(*id));
}

int anchor_load_identity(anchor_identity_t *id)
{
	return storage_read(ANCHOR_IDENTITY_KEY, id, sizeof(*id));
}

bool anchor_has_identity(void)
{
	return storage_exists(ANCHOR_IDENTITY_KEY);
}

static int find_slot(uint16_t base, int max, uint16_t dev_id, bool *found)
{
	uint16_t stored;
	int first_empty = -1;

	*found = false;
	for (int i = 0; i < max; i++) {
		if (storage_read(base + i, &stored, sizeof(stored)) == 0) {
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

int anchor_store_anchor(uint16_t anchor_id)
{
	bool found;
	int slot = find_slot(ANCHOR_ANCHOR_BASE, ANCHOR_ANCHOR_MAX, anchor_id, &found);
	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("Anchor storage full");
		return -ENOMEM;
	}
	return storage_write(ANCHOR_ANCHOR_BASE + slot, &anchor_id, sizeof(anchor_id));
}

int anchor_store_sensor(uint16_t sensor_id)
{
	bool found;
	int slot = find_slot(ANCHOR_SENSOR_BASE, ANCHOR_SENSOR_MAX, sensor_id, &found);
	if (found) {
		return 0;
	}
	if (slot < 0) {
		LOG_WRN("Sensor storage full");
		return -ENOMEM;
	}
	return storage_write(ANCHOR_SENSOR_BASE + slot, &sensor_id, sizeof(sensor_id));
}

bool anchor_is_anchor_paired(uint16_t anchor_id)
{
	bool found;
	find_slot(ANCHOR_ANCHOR_BASE, ANCHOR_ANCHOR_MAX, anchor_id, &found);
	return found;
}

bool anchor_is_sensor_paired(uint16_t sensor_id)
{
	bool found;
	find_slot(ANCHOR_SENSOR_BASE, ANCHOR_SENSOR_MAX, sensor_id, &found);
	return found;
}

int anchor_get_anchor_count(void)
{
	int count = 0;
	uint16_t tmp;
	for (int i = 0; i < ANCHOR_ANCHOR_MAX; i++) {
		if (storage_read(ANCHOR_ANCHOR_BASE + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

int anchor_get_sensor_count(void)
{
	int count = 0;
	uint16_t tmp;
	for (int i = 0; i < ANCHOR_SENSOR_MAX; i++) {
		if (storage_read(ANCHOR_SENSOR_BASE + i, &tmp, sizeof(tmp)) == 0) {
			count++;
		}
	}
	return count;
}

void anchor_print_paired(void)
{
	uint16_t id;

	anchor_identity_t self;
	if (anchor_load_identity(&self) == 0) {
		LOG_INF("Anchor: ID:%d parent:%d parent_hop:%d",
			self.device_id, self.parent_id, self.parent_hop);
	}

	LOG_INF("=== Child Anchors (%d) ===", anchor_get_anchor_count());
	for (int i = 0; i < ANCHOR_ANCHOR_MAX; i++) {
		if (storage_read(ANCHOR_ANCHOR_BASE + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] Anchor ID:%d", i, id);
		}
	}

	LOG_INF("=== Child Sensors (%d) ===", anchor_get_sensor_count());
	for (int i = 0; i < ANCHOR_SENSOR_MAX; i++) {
		if (storage_read(ANCHOR_SENSOR_BASE + i, &id, sizeof(id)) == 0) {
			LOG_INF("  [%d] Sensor ID:%d", i, id);
		}
	}
}

/* === Anchor main === */

void anchor_main(void)
{
	LOG_INF("Anchor mode started");

	/* TODO: implement anchor logic */
}
