/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/drivers/hwinfo.h>
#include <dk_buttons_and_leds.h>
#include "packet.h"
#include "radio.h"
#include "storage.h"
#include "gateway/gateway.h"
#include "anchor/anchor.h"
#include "sensor/sensor.h"

LOG_MODULE_REGISTER(app);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

/* Shared device state */
uint16_t device_id;
device_type_t my_device_type = (device_type_t)CONFIG_DEVICE_TYPE;
uint8_t my_hop_num;

/* Factory reset: hold button 1 for 2 seconds */
#define FACTORY_RESET_HOLD_MS 2000

static int64_t btn1_press_time;

static void button_handler(uint32_t button_state, uint32_t has_changed)
{
	if (has_changed & DK_BTN1_MSK) {
		if (button_state & DK_BTN1_MSK) {
			btn1_press_time = k_uptime_get();
		} else {
			int64_t held_ms = k_uptime_get() - btn1_press_time;

			if (held_ms >= FACTORY_RESET_HOLD_MS) {
				LOG_WRN("Factory reset! Clearing NVM...");
				int err = storage_clear_all();
				if (err) {
					LOG_ERR("NVM clear failed, err %d", err);
				} else {
					LOG_WRN("NVM cleared. Rebooting...");
				}
				k_sleep(K_MSEC(500));
				sys_reboot(SYS_REBOOT_COLD);
			}
		}
	}
}

int main(void)
{
	int err;

	if (my_device_type == DEVICE_TYPE_GATEWAY) {
		my_hop_num = 0;
	}

	LOG_INF("DECT NR+ Mesh [%s] started", device_type_str(my_device_type));

	/* Initialize NVS storage */
	err = storage_init();
	if (err) {
		LOG_ERR("storage init failed, err %d", err);
		return err;
	}

	/* Initialize buttons for factory reset */
	err = dk_buttons_init(button_handler);
	if (err) {
		LOG_ERR("buttons init failed, err %d", err);
		return err;
	}

	/* Initialize modem and PHY */
	err = nrf_modem_lib_init();
	if (err) {
		LOG_ERR("modem init failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_event_handler_set(dect_phy_event_handler);
	if (err) {
		LOG_ERR("event_handler_set failed, err %d", err);
		return err;
	}

	err = nrf_modem_dect_phy_init();
	if (err) {
		LOG_ERR("phy_init failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	if (exit_flag) {
		return -EIO;
	}

	err = nrf_modem_dect_phy_configure(&dect_phy_config_params);
	if (err) {
		LOG_ERR("phy_configure failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	if (exit_flag) {
		return -EIO;
	}

	err = nrf_modem_dect_phy_activate(NRF_MODEM_DECT_PHY_RADIO_MODE_LOW_LATENCY);
	if (err) {
		LOG_ERR("phy_activate failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
	if (exit_flag) {
		return -EIO;
	}

	hwinfo_get_device_id((void *)&device_id, sizeof(device_id));
	LOG_INF("PHY ready, device ID: %d", device_id);

	/* Dispatch to device-type entry point */
	switch (my_device_type) {
	case DEVICE_TYPE_GATEWAY:
		gateway_main();
		break;
	case DEVICE_TYPE_ANCHOR:
		anchor_main();
		break;
	case DEVICE_TYPE_SENSOR:
		sensor_main();
		break;
	default:
		LOG_ERR("Unknown device type");
		break;
	}

	/* Shutdown */
	nrf_modem_dect_phy_deactivate();
	k_sem_take(&deinit_sem, K_FOREVER);
	nrf_modem_dect_phy_deinit();
	k_sem_take(&deinit_sem, K_FOREVER);
	nrf_modem_lib_shutdown();

	LOG_INF("Bye!");
	return 0;
}
