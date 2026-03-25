/*
 * Shared paired-device storage helpers for DECT NR+ mesh network
 *
 * Both gateway and anchor store lists of paired child device IDs in NVS.
 * This module provides a generic interface parameterized by NVS base key
 * and max entry count, eliminating duplicate code.
 */

#ifndef PAIRED_STORE_H
#define PAIRED_STORE_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	uint16_t nvs_base;
	int max_entries;
	const char *label;      /* "Anchor" or "Sensor", for logging */
} paired_store_t;

/* Add a device ID to the store. Returns 0 if already present or newly stored. */
int paired_store_add(const paired_store_t *ps, uint16_t dev_id);

/* Check if a device ID is in the store */
bool paired_store_contains(const paired_store_t *ps, uint16_t dev_id);

/* Count how many entries are stored */
int paired_store_count(const paired_store_t *ps);

/* Print all stored entries */
void paired_store_print(const paired_store_t *ps);

/* Get device ID at NVS slot index. Returns 0 on success, -ENOENT if empty. */
int paired_store_get(const paired_store_t *ps, int index, uint16_t *dev_id);

#endif /* PAIRED_STORE_H */
