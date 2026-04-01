/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/gpio.h>
#include <nrf_modem_dect_phy.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <zephyr/drivers/hwinfo.h>
#include <dk_buttons_and_leds.h>
#include "protocol.h"
#include "radio.h"
#include "nvs_store.h"
#include "identity.h"
#include "mesh.h"
#include "display.h"
#include "psram.h"
#include "large_data.h"
#include "at_cmd.h"
#include <zephyr/app_version.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, CONFIG_MAIN_LOG_LEVEL);

BUILD_ASSERT(CONFIG_CARRIER, "Carrier must be configured according to local regulations");

/* Device-type selection pins from devicetree */
#define DEVTYPE_PIN0_NODE DT_NODELABEL(devtype_pin0)
#define DEVTYPE_PIN1_NODE DT_NODELABEL(devtype_pin1)

static const struct gpio_dt_spec devtype_pin0 = GPIO_DT_SPEC_GET(DEVTYPE_PIN0_NODE, gpios);
static const struct gpio_dt_spec devtype_pin1 = GPIO_DT_SPEC_GET(DEVTYPE_PIN1_NODE, gpios);

/**
 * Read P0.21 and P0.23 to determine device type.
 *   P0.21=0, P0.23=0 → Gateway
 *   P0.21=0, P0.23=1 → Anchor
 *   P0.21=1, P0.23=0 → Sensor
 *   P0.21=1, P0.23=1 → Sensor (reserved)
 */
static device_type_t device_type_read_pins(void)
{
	int bit0, bit1;

	if (!gpio_is_ready_dt(&devtype_pin0) || !gpio_is_ready_dt(&devtype_pin1)) {
		LOG_ERR("Device-type GPIO not ready, defaulting to Sensor");
		return DEVICE_TYPE_SENSOR;
	}

	gpio_pin_configure_dt(&devtype_pin0, GPIO_INPUT);
	gpio_pin_configure_dt(&devtype_pin1, GPIO_INPUT);

	bit0 = gpio_pin_get_dt(&devtype_pin0);  /* P0.21 */
	bit1 = gpio_pin_get_dt(&devtype_pin1);  /* P0.23 */

	LOG_INF("Device-type pins: P0.21=%d P0.23=%d", bit0, bit1);

	if (bit0 == 0 && bit1 == 0) {
		return DEVICE_TYPE_GATEWAY;
	} else if (bit0 == 0 && bit1 == 1) {
		return DEVICE_TYPE_ANCHOR;
	} else {
		return DEVICE_TYPE_SENSOR;
	}
}

/* Shared device state */
uint16_t device_id;
device_type_t my_device_type;
uint8_t my_hop_num;

/* Factory reset: hold button 1 for 2 seconds */
#define FACTORY_RESET_HOLD_MS 2000

/* Button semaphores for sensor triggers */
K_SEM_DEFINE(btn2_sem, 0, 1);
K_SEM_DEFINE(btn3_sem, 0, 1);
K_SEM_DEFINE(btn4_sem, 0, 1);

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
				}
				LOG_WRN("Factory reset complete. Rebooting...");
				k_sleep(K_MSEC(500));
				sys_reboot(SYS_REBOOT_COLD);
			}
		}
	}

	/* Button 2 press triggers small data send */
	if ((has_changed & DK_BTN2_MSK) && (button_state & DK_BTN2_MSK)) {
		k_sem_give(&btn2_sem);
	}

	/* Button 3 press triggers 50KB large data send */
	if ((has_changed & DK_BTN3_MSK) && (button_state & DK_BTN3_MSK)) {
		k_sem_give(&btn3_sem);
	}

	/* Button 4 press triggers 75KB large data send */
	if ((has_changed & DK_BTN4_MSK) && (button_state & DK_BTN4_MSK)) {
		k_sem_give(&btn4_sem);
	}
}

int main(void)
{
	int err;

	my_device_type = device_type_read_pins();

	if (my_device_type == DEVICE_TYPE_GATEWAY) {
		my_hop_num = 0;
	}

	LOG_INF("DECT NR+ Mesh [%s] v%s started",
		device_type_str(my_device_type), APP_VERSION_STRING);

	/* Initialize NVS storage */
	err = storage_init();
	if (err) {
		LOG_ERR("storage init failed, err %d", err);
		return err;
	}

	/* Load RSSI threshold from NVM (or use default) */
	mesh_rssi_threshold_load();

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

	// /* --- CW TX power test (must run before phy_init) --- */
	// LOG_INF("CW test: +19 dBm on carrier %d for 5s — check SA at J1", CONFIG_CARRIER);
	// err = nrf_modem_dect_phy_test_rf_tx_cw_control(
	// 	NRF_MODEM_DECT_PHY_TEST_RF_TX_CW_OPER_ON,
	// 	CONFIG_CARRIER, 19);
	// if (err) {
	// 	LOG_ERR("CW start failed: %d", err);
	// } else {
	// 	k_sem_take(&operation_sem, K_FOREVER);
	// 	k_sleep(K_SECONDS(100));
	// 	nrf_modem_dect_phy_test_rf_tx_cw_control(
	// 		NRF_MODEM_DECT_PHY_TEST_RF_TX_CW_OPER_OFF,
	// 		CONFIG_CARRIER, 0);
	// 	k_sem_take(&operation_sem, K_FOREVER);
	// 	LOG_INF("CW test done");
	// }
	// /* --- End CW test --- */

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

	/* Initialize and show device type + ID on DFR0997 display */
	err = display_init();
	if (err) {
		LOG_WRN("display init failed, err %d (continuing)", err);
	} else {
		display_header(my_device_type, device_id);
	}

	/* Initialize PSRAM + large data transfer module */
	err = psram_init();
	if (err) {
		LOG_ERR("PSRAM init failed, err %d", err);
	} else {
		large_data_init();
	}

	/* Start AT command handler (reads from console UART) */
	at_cmd_init();

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
