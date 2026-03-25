/*
 * External flash storage for large data reassembly
 *
 * Provides a simple block-based API over the external SPI flash.
 * Each large data session gets a reserved region in flash.
 * Fragments are written directly to their offset within the region.
 *
 * Base address is set by Partition Manager (PM_LARGE_DATA_ADDRESS).
 */

#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>
#include <stddef.h>
#include <pm_config.h>

/* Base offset of large_data region on external flash.
 * Placed after mcuboot_secondary (0xe8000) + ota_staging (0x80000). */
#define FLASH_STORE_BASE_OFFSET (PM_MCUBOOT_SECONDARY_END_ADDRESS + 0x80000)

/* Number of flash slots (must match LARGE_DATA_MAX_SESSIONS) */
#define FLASH_STORE_MAX_SLOTS 4

/* Size of each session slot in flash (256KB — fits LARGE_DATA_MAX_SIZE) */
#define FLASH_STORE_SLOT_SIZE (256 * 1024)

/* Flash erase block size (4KB for GD25WB256) */
#define FLASH_STORE_ERASE_SIZE 4096

/* Chunk size for CRC computation and relay reads */
#define FLASH_STORE_READ_CHUNK 512

/* Initialize flash store (get flash device reference) */
int flash_store_init(void);

/* Erase a session slot (must be called before writing fragments) */
int flash_store_erase_slot(uint8_t slot);

/* Write a fragment to a session slot at the given offset */
int flash_store_write(uint8_t slot, uint32_t offset,
		      const void *data, uint16_t len);

/* Read data from a session slot at the given offset */
int flash_store_read(uint8_t slot, uint32_t offset,
		     void *buf, uint16_t len);

/* Compute CRC-16 over data stored in flash (reads in chunks) */
uint16_t flash_store_compute_crc16(uint8_t slot, uint32_t total_size);

#endif /* FLASH_STORE_H */
