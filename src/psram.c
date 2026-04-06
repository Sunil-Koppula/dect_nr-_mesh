/*
 * ESP-PSRAM64 SPI PSRAM driver
 *
 * Uses Zephyr SPI API on SPI3 to communicate with the ESP-PSRAM64.
 * The PSRAM is byte-addressable and requires no erase before write.
 *
 * SPI protocol (ESP-PSRAM64):
 *   Read:  [0x03][A23..A0] -> [data...]
 *   Write: [0x02][A23..A0][data...]
 *   RDID:  [0x9F][00][00][00] -> [MFG][KGD][EID...]
 *     Expected: MFG=0x0D, KGD=0x5D
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include "psram.h"

LOG_MODULE_REGISTER(psram, CONFIG_PSRAM_LOG_LEVEL);

#define PSRAM_NODE DT_NODELABEL(psram0)

static const struct spi_dt_spec psram_spi = SPI_DT_SPEC_GET(
	PSRAM_NODE,
	SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8),
	0);

/* Max bytes per SPI transaction (limited by EasyDMA maxcnt = 8191) */
#define PSRAM_MAX_XFER 8000

int psram_init(void)
{
	if (!spi_is_ready_dt(&psram_spi)) {
		LOG_ERR("PSRAM SPI device not ready");
		return -ENODEV;
	}

	/* Read ID to verify communication */
	uint8_t tx_data[8] = { PSRAM_CMD_RDID, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t rx_data[8] = { 0 };

	struct spi_buf tx_buf = { .buf = tx_data, .len = 8 };
	struct spi_buf rx_buf = { .buf = rx_data, .len = 8 };
	struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };

	int err = spi_transceive_dt(&psram_spi, &tx, &rx);
	if (err) {
		LOG_ERR("PSRAM RDID failed, err %d", err);
		return err;
	}

	/* ID bytes start at offset 4 (after 4-byte command phase) */
	uint8_t mfg = rx_data[4];
	uint8_t kgd = rx_data[5];

	LOG_INF("PSRAM ID: MFG=0x%02X KGD=0x%02X", mfg, kgd);

	if (mfg != 0x0D || kgd != 0x5D) {
		LOG_WRN("Unexpected PSRAM ID (expected MFG=0x0D KGD=0x5D)");
	}

	LOG_INF("PSRAM initialized (8MB on SPI3)");
	return 0;
}

int psram_write(uint32_t addr, const void *data, size_t len)
{
	if (addr + len > PSRAM_SIZE) {
		return -EINVAL;
	}

	const uint8_t *src = data;

	while (len > 0) {
		size_t chunk = (len > PSRAM_MAX_XFER) ? PSRAM_MAX_XFER : len;

		uint8_t cmd[4] = {
			PSRAM_CMD_WRITE,
			(addr >> 16) & 0xFF,
			(addr >> 8) & 0xFF,
			addr & 0xFF,
		};

		struct spi_buf tx_bufs[2] = {
			{ .buf = cmd, .len = sizeof(cmd) },
			{ .buf = (void *)src, .len = chunk },
		};
		struct spi_buf_set tx = { .buffers = tx_bufs, .count = 2 };

		int err = spi_write_dt(&psram_spi, &tx);
		if (err) {
			LOG_ERR("PSRAM write failed at 0x%06X, err %d", addr, err);
			return err;
		}

		src += chunk;
		addr += chunk;
		len -= chunk;
	}

	return 0;
}

int psram_read(uint32_t addr, void *buf, size_t len)
{
	if (addr + len > PSRAM_SIZE) {
		return -EINVAL;
	}

	uint8_t *dst = buf;
	uint8_t discard[4];

	while (len > 0) {
		size_t chunk = (len > PSRAM_MAX_XFER) ? PSRAM_MAX_XFER : len;

		uint8_t cmd[4] = {
			PSRAM_CMD_READ,
			(addr >> 16) & 0xFF,
			(addr >> 8) & 0xFF,
			addr & 0xFF,
		};

		/* TX: command + dst buffer as dummy (contents don't matter).
		 * RX: discard during cmd phase, read into dst during data phase. */
		struct spi_buf tx_bufs[2] = {
			{ .buf = cmd, .len = sizeof(cmd) },
			{ .buf = dst, .len = chunk },
		};
		struct spi_buf rx_bufs[2] = {
			{ .buf = discard, .len = sizeof(cmd) },
			{ .buf = dst, .len = chunk },
		};
		struct spi_buf_set tx = { .buffers = tx_bufs, .count = 2 };
		struct spi_buf_set rx = { .buffers = rx_bufs, .count = 2 };

		int err = spi_transceive_dt(&psram_spi, &tx, &rx);
		if (err) {
			LOG_ERR("PSRAM read failed at 0x%06X, err %d", addr, err);
			return err;
		}

		dst += chunk;
		addr += chunk;
		len -= chunk;
	}

	return 0;
}

void psram_test_read_write(void)
{
	int err;

	err = psram_init();
	if (err) {
		LOG_ERR("PSRAM init failed, err %d", err);
		return;
	}

	/* Write a known pattern to address 0 */
	uint8_t tx_data[16];
	for (int i = 0; i < sizeof(tx_data); i++) {
		tx_data[i] = i + 0xA0;
	}

	LOG_INF("PSRAM write: 16 bytes at addr 0x000000");
	err = psram_write(0x000000, tx_data, sizeof(tx_data));
	if (err) {
		LOG_ERR("PSRAM write failed, err %d", err);
		return;
	}

	/* Read back */
	uint8_t rx_data[16] = { 0 };
	LOG_INF("PSRAM read: 16 bytes at addr 0x000000");
	err = psram_read(0x000000, rx_data, sizeof(rx_data));
	if (err) {
		LOG_ERR("PSRAM read failed, err %d", err);
		return;
	}

	/* Compare */
	bool match = true;
	for (int i = 0; i < sizeof(tx_data); i++) {
		if (tx_data[i] != rx_data[i]) {
			LOG_ERR("Mismatch at [%d]: wrote 0x%02X, read 0x%02X",
				i, tx_data[i], rx_data[i]);
			match = false;
		}
	}

	if (match) {
		LOG_INF("PSRAM test PASSED — all 16 bytes match");
	} else {
		LOG_ERR("PSRAM test FAILED");
	}
}

void psram_test_write_only(void)
{
	if (!spi_is_ready_dt(&psram_spi)) {
		LOG_ERR("PSRAM SPI not ready");
		return;
	}

	LOG_INF("PSRAM write-only test running");
	k_sleep(K_MSEC(100));

	uint8_t data[20] = {
		PSRAM_CMD_WRITE, 0x00, 0x00, 0x00, /* cmd + 24-bit addr 0x000000 */
		0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04,
		0x05, 0x06, 0x07, 0x08, 0xCA, 0xFE, 0xBA, 0xBE
	};

	struct spi_buf tx_buf = { .buf = data, .len = sizeof(data) };
	struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };

	int count = 0;
	while (1) {
		int err = spi_write_dt(&psram_spi, &tx);
		LOG_INF("Write #%d, err=%d", count++, err);
		k_sleep(K_MSEC(1000));
	}
}
