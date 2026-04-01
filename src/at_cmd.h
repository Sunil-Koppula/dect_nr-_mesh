/*
 * Custom AT command handler for DECT NR+ mesh network
 *
 * Runs a UART RX thread on the console UART (uart0).
 * Lines starting with "AT" are parsed and executed.
 * Non-AT data (logging, SMP) passes through unaffected.
 *
 * Supported commands:
 *   AT                       → OK (basic test)
 *   AT+VERSION?              → +VERSION: <major>.<minor>.<patch>
 *   AT+DEVTYPE?              → +DEVTYPE: GATEWAY|ANCHOR|SENSOR
 *   AT+DEVID?                → +DEVID: <id>
 *   AT+HOP?                  → +HOP: <hop_num>
 *   AT+RSSI?                 → +RSSI: <dBm>
 *   AT+CARRIER?              → +CARRIER: <carrier>
 *   AT+TXPOWER?              → +TXPOWER: <dBm>
 *   AT+PAIR?                 → List all paired devices with version
 *   AT+OTA_STAGE             → Copy secondary → staging + reboot (gateway)
 *   AT+OTA_DIST              → Distribute OTA to mesh (gateway)
 *   AT+OTA_STATUS?           → +OTA_STATUS: staging version + validity
 *   AT+FACTORY_RESET         → Clear NVM + staging + reboot
 *   AT+RESET                 → Reboot
 *   AT+SEND_DATA             → Request all sensors to send data (gateway)
 */

#ifndef AT_CMD_H
#define AT_CMD_H

#include <zephyr/kernel.h>

/* Initialize the AT command handler.
 * Starts a UART RX thread that reads lines from the console UART.
 * Call once from main after all subsystems are initialized. */
void at_cmd_init(void);

/* Semaphores signaled by AT+SEND_SMALL_DATA / AT+SEND_LARGE_DATA.
 * Gateway main loop checks these to initiate data requests. */
extern struct k_sem send_small_data_sem;
extern struct k_sem send_large_data_sem;

/* AT+SENSOR_<ID> / AT+ANCHOR_<ID> — parent query for a specific device.
 * Gateway main loop checks parent_query_sem, reads parent_query_sensor_id,
 * sends PARENT_QUERY, and logs the PARENT_RESPONSE. */
extern struct k_sem parent_query_sem;
extern uint16_t parent_query_sensor_id;

/* AT+SENSOR_ALL / AT+ANCHOR_ALL — query parent info for all devices.
 * Sends PARENT_QUERY to all paired anchors and sensors.
 * parent_query_all_filter: DEVICE_TYPE_SENSOR or DEVICE_TYPE_ANCHOR */
extern struct k_sem parent_query_all_sem;
extern uint8_t parent_query_all_filter;

/* AT+REPAIR — broadcast factory reset to all paired devices, then self.
 * Gateway main loop checks this, sends REPAIR to all, waits 2s, resets. */
extern struct k_sem repair_sem;

/* AT+SET_RSSI_<dBm> — broadcast new RSSI threshold to all devices.
 * Gateway main loop checks set_rssi_sem, reads set_rssi_value,
 * broadcasts SET_RSSI, stores in NVM, then reboots. */
extern struct k_sem set_rssi_sem;
extern int16_t set_rssi_value;

#endif /* AT_CMD_H */
