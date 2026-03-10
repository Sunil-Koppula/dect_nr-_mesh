/*
 * Generic NVS storage layer for DECT NR+ mesh network
 *
 * Each device type defines its own NVS key offsets and calls
 * these generic functions with the appropriate key.
 */

#ifndef STORAGE_H
#define STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Initialize NVS (call once at startup) */
int storage_init(void);

/* Write data to NVS key. Returns 0 on success. */
int storage_write(uint16_t key, const void *data, size_t len);

/* Read data from NVS key. Returns 0 on success, -ENOENT if not found. */
int storage_read(uint16_t key, void *data, size_t len);

/* Delete NVS key. Returns 0 on success. */
int storage_delete(uint16_t key);

/* Check if NVS key exists */
bool storage_exists(uint16_t key);

#endif /* STORAGE_H */
