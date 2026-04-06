/*
 * ESP-PSRAM64 SPI PSRAM driver
 *
 * 8MB (64Mbit) SPI pseudo-static RAM on SPI3.
 * Provides simple read/write API with byte-level access (no erase needed).
 *
 * Pinout (nRF9151 DK — SPI3, same bus as onboard flash):
 *   SCK  = P0.13 (D13)
 *   MOSI = P0.11 (D11)
 *   MISO = P0.12 (D12)
 *   CS   = P0.10 (D10)
 */

#ifndef PSRAM_H
#define PSRAM_H

#include <stdint.h>
#include <stddef.h>

/* ESP-PSRAM64 capacity: 8MB */
#define PSRAM_SIZE (8 * 1024 * 1024)

/* SPI commands */
#define PSRAM_CMD_READ  0x03
#define PSRAM_CMD_WRITE 0x02
#define PSRAM_CMD_RDID  0x9F

/* Initialize the PSRAM (verifies SPI communication via Read ID) */
int psram_init(void);

/* Write data to PSRAM at the given address */
int psram_write(uint32_t addr, const void *data, size_t len);

/* Read data from PSRAM at the given address */
int psram_read(uint32_t addr, void *buf, size_t len);

/* Test: write and read back 16 bytes, log result */
void psram_test_read_write(void);

/* Test: write a known pattern repeatedly (blocks forever) */
void psram_test_write_only(void);

#endif /* PSRAM_H */
