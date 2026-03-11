/*
 * External flash storage for large data reassembly
 *
 * Provides a simple block-based API over the external SPI flash.
 * Each large data session gets a reserved region in flash.
 * Fragments are written directly to their offset within the region.
 *
 * Flash layout (1MB total, divided into LARGE_DATA_MAX_SESSIONS slots):
 *   Slot 0: 0x000000 - 0x03FFFF (256KB)
 *   Slot 1: 0x040000 - 0x07FFFF (256KB)
 *   Slot 2: 0x080000 - 0x0BFFFF (256KB)
 *   Slot 3: 0x0C0000 - 0x0FFFFF (256KB)
 */

#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdint.h>
#include <stddef.h>

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
