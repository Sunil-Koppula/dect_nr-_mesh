/*
 * Custom AT command handler for DECT NR+ mesh network
 *
 * Uses Zephyr console_getline() API in a dedicated thread.
 * This coexists with MCUmgr SMP and logging on the same UART.
 *
 * Usage from serial terminal:
 *   AT              → OK
 *   AT+VERSION?     → +VERSION: 1.0.0
 *   AT+INFO?        → All device info
 *   AT+PAIR?        → List paired devices
 *   AT+OTA_STAGE    → Stage OTA + reboot
 *   AT+OTA_DIST     → Distribute OTA
 *   AT+RESET        → Reboot
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/console/console.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/app_version.h>
#include "at_cmd.h"
#include "identity.h"
#include "protocol.h"
#include "nvs_store.h"
#include "mesh.h"
#include "common.h"

/* Paired store pointers — set at runtime by gateway/anchor main.
 * NULL on sensor (no children). */
const void *gw_anchor_store_ptr;
const void *gw_sensor_store_ptr;

/* AT+SEND_*_DATA semaphores — signaled to gateway main loop */
K_SEM_DEFINE(send_small_data_sem, 0, 1);
K_SEM_DEFINE(send_large_data_sem, 0, 1);

/* AT+SENSOR_<ID> / AT+ANCHOR_<ID> — parent query for a specific device */
K_SEM_DEFINE(parent_query_sem, 0, 1);
uint16_t parent_query_sensor_id;

/* AT+SENSOR_ALL / AT+ANCHOR_ALL — parent query for all devices */
K_SEM_DEFINE(parent_query_all_sem, 0, 1);
uint8_t parent_query_all_filter;  /* DEVICE_TYPE_SENSOR or DEVICE_TYPE_ANCHOR */

/* AT+REPAIR — broadcast factory reset to all paired devices */
K_SEM_DEFINE(repair_sem, 0, 1);

/* AT+SET_RSSI_<dBm> — broadcast new RSSI threshold to all devices */
K_SEM_DEFINE(set_rssi_sem, 0, 1);
int16_t set_rssi_value;

/* Thread configuration */
#define AT_CMD_STACK_SIZE 2048
#define AT_CMD_PRIORITY   10

static K_THREAD_STACK_DEFINE(at_cmd_stack, AT_CMD_STACK_SIZE);
static struct k_thread at_cmd_thread;

/* ===== Command handlers ===== */

static void at_ok(void)
{
	printk("OK\r\n");
}

static void cmd_version(void)
{
	printk("+VERSION: %d.%d.%d\r\nOK\r\n",
	       APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_PATCHLEVEL);
}

static void cmd_devtype(void)
{
	printk("+DEVTYPE: %s\r\nOK\r\n", device_type_str(my_device_type));
}

static void cmd_devid(void)
{
	printk("+DEVID: %d\r\nOK\r\n", device_id);
}

static void cmd_hop(void)
{
	printk("+HOP: %d\r\nOK\r\n", my_hop_num);
}

static void cmd_rssi(void)
{
	printk("+RSSI: %d\r\nOK\r\n", last_rssi_dbm);
}

static void cmd_carrier(void)
{
	printk("+CARRIER: %d\r\nOK\r\n", CONFIG_CARRIER);
}

static void cmd_txpower(void)
{
	printk("+TXPOWER: %d\r\nOK\r\n", CONFIG_TX_POWER);
}

static void cmd_get_rssi(void)
{
	printk("+GET_RSSI: %d dBm\r\nOK\r\n", mesh_rssi_threshold_2 / 2);
}

static void cmd_info(void)
{
	printk("+INFO:\r\n");
	printk("  Device: %s ID:%d\r\n",
	       device_type_str(my_device_type), device_id);
	printk("  Version: %d.%d.%d\r\n",
	       APP_VERSION_MAJOR, APP_VERSION_MINOR, APP_PATCHLEVEL);
	printk("  Hop: %d\r\n", my_hop_num);
	printk("  Carrier: %d\r\n", CONFIG_CARRIER);
	printk("  TX Power: %d dBm\r\n", CONFIG_TX_POWER);
	printk("  RSSI: %d dBm\r\n", last_rssi_dbm);
	printk("OK\r\n");
}

static void cmd_pair(void)
{
	if (my_device_type == DEVICE_TYPE_SENSOR) {
		printk("+PAIR: sensor has no children\r\nOK\r\n");
		return;
	}

	if (!gw_anchor_store_ptr || !gw_sensor_store_ptr) {
		printk("ERROR: paired stores not available\r\n");
		return;
	}

	const paired_store_t *a_store = gw_anchor_store_ptr;
	const paired_store_t *s_store = gw_sensor_store_ptr;
	paired_device_info_t info;

	printk("+PAIR: Anchors\r\n");
	for (int i = 0; i < a_store->max_entries; i++) {
		if (paired_store_get_info(a_store, i, &info) == 0) {
			printk("  [%d] ID:%d v%d.%d.%d\r\n", i,
			       info.device_id, info.version_major,
			       info.version_minor, info.version_patch);
		}
	}

	printk("+PAIR: Sensors\r\n");
	for (int i = 0; i < s_store->max_entries; i++) {
		if (paired_store_get_info(s_store, i, &info) == 0) {
			printk("  [%d] ID:%d v%d.%d.%d\r\n", i,
			       info.device_id, info.version_major,
			       info.version_minor, info.version_patch);
		}
	}

	printk("OK\r\n");
}

static void cmd_factory_reset(void)
{
	if (my_device_type == DEVICE_TYPE_GATEWAY ||
	    my_device_type == DEVICE_TYPE_ANCHOR) {
		printk("+FACTORY_RESET: broadcasting to paired devices...\r\n");
		k_sem_give(&repair_sem);
		/* Gateway/anchor main loop handles broadcast + local reset */
		return;
	}

	/* Sensor: local-only reset */
	printk("+FACTORY_RESET: clearing NVM and rebooting...\r\n");
	factory_reset_reboot();
}

static void cmd_reset(void)
{
	printk("+RESET: rebooting...\r\n");
	k_sleep(K_MSEC(200));
	sys_reboot(SYS_REBOOT_COLD);
}

static void cmd_send_small_data(void)
{
	if (my_device_type != DEVICE_TYPE_GATEWAY) {
		printk("+SEND_SMALL_DATA: only available on gateway\r\n");
		printk("ERROR\r\n");
		return;
	}
	printk("+SEND_SMALL_DATA: requesting small data from all sensors...\r\n");
	k_sem_give(&send_small_data_sem);
	printk("OK\r\n");
}

static void cmd_send_large_data(void)
{
	if (my_device_type != DEVICE_TYPE_GATEWAY) {
		printk("+SEND_LARGE_DATA: only available on gateway\r\n");
		printk("ERROR\r\n");
		return;
	}
	printk("+SEND_LARGE_DATA: requesting large data from all sensors...\r\n");
	k_sem_give(&send_large_data_sem);
	printk("OK\r\n");
}

static void cmd_repair(void)
{
	if (my_device_type != DEVICE_TYPE_GATEWAY) {
		printk("+REPAIR: only available on gateway\r\n");
		printk("ERROR\r\n");
		return;
	}
	printk("+REPAIR: broadcasting factory reset to all devices...\r\n");
	k_sem_give(&repair_sem);
	printk("OK\r\n");
}

static void cmd_help(void)
{
	printk("+HELP:\r\n");
	printk("  AT              Test\r\n");
	printk("  AT+VERSION?     Firmware version\r\n");
	printk("  AT+DEVTYPE?     Device type\r\n");
	printk("  AT+DEVID?       Device ID\r\n");
	printk("  AT+HOP?         Hop number\r\n");
	printk("  AT+RSSI?        Last RSSI\r\n");
	printk("  AT+CARRIER?     Carrier freq\r\n");
	printk("  AT+TXPOWER?     TX power\r\n");
	printk("  AT+GET_RSSI?    RSSI threshold\r\n");
	printk("  AT+INFO?        All device info\r\n");
	printk("  AT+PAIR?        Paired devices\r\n");
	printk("  AT+SENSOR_<ID>  Query sensor parent info (gateway)\r\n");
	printk("  AT+SENSOR_ALL   Query all sensors parent info (gateway)\r\n");
	printk("  AT+ANCHOR_<ID>  Query anchor parent info (gateway)\r\n");
	printk("  AT+ANCHOR_ALL   Query all anchors parent info (gateway)\r\n");
	printk("  AT+SEND_SMALL_DATA  Request small data (gateway)\r\n");
	printk("  AT+SEND_LARGE_DATA  Request large data (gateway)\r\n");
	printk("  AT+SET_RSSI_<dBm> Set RSSI threshold (gateway)\r\n");
	printk("  AT+REPAIR       Factory reset all devices (gateway)\r\n");
	printk("  AT+FACTORY_RESET  Factory reset\r\n");
	printk("  AT+RESET        Reboot\r\n");
	printk("  AT+HELP         This help\r\n");
	printk("OK\r\n");
}

/* ===== Dispatch table ===== */

static const struct {
	const char *cmd;
	void (*handler)(void);
} at_commands[] = {
	{ "",              at_ok },
	{ "+VERSION?",     cmd_version },
	{ "+DEVTYPE?",     cmd_devtype },
	{ "+DEVID?",       cmd_devid },
	{ "+HOP?",         cmd_hop },
	{ "+RSSI?",        cmd_rssi },
	{ "+CARRIER?",     cmd_carrier },
	{ "+TXPOWER?",     cmd_txpower },
	{ "+GET_RSSI?",    cmd_get_rssi },
	{ "+INFO?",        cmd_info },
	{ "+PAIR?",        cmd_pair },
	{ "+SEND_SMALL_DATA", cmd_send_small_data },
	{ "+SEND_LARGE_DATA", cmd_send_large_data },
	{ "+REPAIR",        cmd_repair },
	{ "+FACTORY_RESET", cmd_factory_reset },
	{ "+RESET",        cmd_reset },
	{ "+HELP",         cmd_help },
};

#define AT_CMD_COUNT (sizeof(at_commands) / sizeof(at_commands[0]))

/* ===== Device query helper ===== */

static bool handle_device_query_cmd(const char *cmd_part, const char *prefix,
				    const char *label, uint8_t filter)
{
	size_t plen = strlen(prefix);
	if (strncmp(cmd_part, prefix, plen) != 0) {
		return false;
	}
	const char *suffix = cmd_part + plen;
	if (my_device_type != DEVICE_TYPE_GATEWAY) {
		printk("+%s: only available on gateway\r\n", label);
		printk("ERROR\r\n");
	} else if (strcmp(suffix, "ALL") == 0) {
		printk("+%s_ALL: querying parent of all...\r\n", label);
		parent_query_all_filter = filter;
		k_sem_give(&parent_query_all_sem);
		printk("OK\r\n");
	} else if (*suffix == '\0') {
		printk("ERROR: missing %s ID\r\n", label);
	} else {
		parent_query_sensor_id = (uint16_t)atoi(suffix);
		printk("+%s: querying parent of ID:%d...\r\n", label, parent_query_sensor_id);
		k_sem_give(&parent_query_sem);
		printk("OK\r\n");
	}
	return true;
}

/* ===== Console getline thread ===== */

static void at_cmd_thread_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	console_getline_init();

	while (true) {
		char *line = console_getline();

		if (!line || line[0] == '\0') {
			continue;
		}

		/* Convert to uppercase */
		for (int i = 0; line[i]; i++) {
			if (line[i] >= 'a' && line[i] <= 'z') {
				line[i] -= 32;
			}
		}

		/* Must start with "AT" */
		if (line[0] != 'A' || line[1] != 'T') {
			continue;
		}

		const char *cmd_part = line + 2;

		bool found = false;
		for (int i = 0; i < AT_CMD_COUNT; i++) {
			if (strcmp(cmd_part, at_commands[i].cmd) == 0) {
				at_commands[i].handler();
				found = true;
				break;
			}
		}

		if (!found) found = handle_device_query_cmd(cmd_part, "+SENSOR_", "SENSOR", DEVICE_TYPE_SENSOR);
		if (!found) found = handle_device_query_cmd(cmd_part, "+ANCHOR_", "ANCHOR", DEVICE_TYPE_ANCHOR);

		/* Check for AT+SET_RSSI_<dBm> */
		if (!found && strncmp(cmd_part, "+SET_RSSI_", 10) == 0) {
			const char *val_str = cmd_part + 10;

			if (my_device_type != DEVICE_TYPE_GATEWAY) {
				printk("+SET_RSSI: only available on gateway\r\n");
				printk("ERROR\r\n");
			} else if (*val_str == '\0') {
				printk("ERROR: missing RSSI value\r\n");
			} else {
				set_rssi_value = (int16_t)(-atoi(val_str));
				printk("+SET_RSSI: broadcasting threshold %d dBm to all devices...\r\n",
				       set_rssi_value);
				k_sem_give(&set_rssi_sem);
				printk("OK\r\n");
			}
			found = true;
		}

		if (!found) {
			printk("ERROR: unknown command\r\n");
		}
	}
}

/* ===== Public API ===== */

void at_cmd_init(void)
{
	k_thread_create(&at_cmd_thread, at_cmd_stack,
			AT_CMD_STACK_SIZE,
			at_cmd_thread_entry, NULL, NULL, NULL,
			AT_CMD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&at_cmd_thread, "at_cmd");
}
