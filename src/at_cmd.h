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
 */

#ifndef AT_CMD_H
#define AT_CMD_H

/* Initialize the AT command handler.
 * Starts a UART RX thread that reads lines from the console UART.
 * Call once from main after all subsystems are initialized. */
void at_cmd_init(void);

#endif /* AT_CMD_H */
