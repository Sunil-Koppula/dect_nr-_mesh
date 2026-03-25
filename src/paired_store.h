/*
 * Shared paired-device storage helpers for DECT NR+ mesh network
 *
 * Both gateway and anchor store lists of paired child devices in NVS.
 * Each entry stores the device ID and firmware version.
 * This module provides a generic interface parameterized by NVS base key
 * and max entry count, eliminating duplicate code.
 */

#ifndef PAIRED_STORE_H
#define PAIRED_STORE_H

#include <stdint.h>
#include <stdbool.h>

/* Paired device record — stored in NVS */
typedef struct {
	uint16_t device_id;
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t version_patch;
} __attribute__((packed)) paired_device_info_t;

typedef struct {
	uint16_t nvs_base;
	int max_entries;
	const char *label;      /* "Anchor" or "Sensor", for logging */
} paired_store_t;

/* Add a device with version info. Updates version if already present. */
int paired_store_add(const paired_store_t *ps, uint16_t dev_id,
		     uint8_t ver_major, uint8_t ver_minor, uint16_t ver_patch);

/* Check if a device ID is in the store */
bool paired_store_contains(const paired_store_t *ps, uint16_t dev_id);

/* Count how many entries are stored */
int paired_store_count(const paired_store_t *ps);

/* Print all stored entries */
void paired_store_print(const paired_store_t *ps);

/* Get device info at NVS slot index. Returns 0 on success, -ENOENT if empty. */
int paired_store_get(const paired_store_t *ps, int index, uint16_t *dev_id);

/* Get full device info at NVS slot index. Returns 0 on success. */
int paired_store_get_info(const paired_store_t *ps, int index,
			  paired_device_info_t *info);

/* Get device info by device ID. Returns 0 on success, -ENOENT if not found. */
int paired_store_find(const paired_store_t *ps, uint16_t dev_id,
		      paired_device_info_t *info);

/* Update the stored version for a device (e.g., after successful OTA) */
int paired_store_update_version(const paired_store_t *ps, uint16_t dev_id,
				uint8_t ver_major, uint8_t ver_minor,
				uint16_t ver_patch);

#endif /* PAIRED_STORE_H */
