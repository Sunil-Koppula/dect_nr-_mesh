/*
 * Sensor device logic for DECT NR+ mesh network
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include "sensor.h"
#include "../packet.h"
#include "../radio.h"
#include "../queue.h"
#include "../mesh.h"
#include "../state.h"
#include "../storage.h"

LOG_MODULE_DECLARE(app);

#define TX_HANDLE 1
#define RX_HANDLE 2

/* === Sensor storage helpers === */

int sensor_store_identity(const sensor_identity_t *id)
{
	return storage_write(SENSOR_IDENTITY_KEY, id, sizeof(*id));
}

int sensor_load_identity(sensor_identity_t *id)
{
	return storage_read(SENSOR_IDENTITY_KEY, id, sizeof(*id));
}

bool sensor_has_identity(void)
{
	return storage_exists(SENSOR_IDENTITY_KEY);
}

/* === Sensor main === */

void sensor_main(void)
{
	LOG_INF("Sensor mode started");

	/* TODO: implement sensor logic */
}
