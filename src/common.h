/*
 * Shared helpers for DECT NR+ mesh network
 *
 * Functions used by gateway, anchor, and sensor to avoid duplication.
 */

#ifndef COMMON_H
#define COMMON_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "protocol.h"

/* Shared TX handle used by all device roles */
#define TX_HANDLE 1

/* Transmit data and block until the modem operation completes. */
int transmit_and_wait(void *data, size_t len);

/* Handle an incoming PAIR_REQUEST: compute hash, send PAIR_RESPONSE. */
void common_handle_pair_request(const pair_request_packet_t *pkt,
				int16_t rssi_2);

/*
 * Verify CRC on a DATA packet and send ACK back to sender.
 * Returns STATUS_SUCCESS (0) if CRC matched.
 * Returns STATUS_CRC_FAIL (2) if CRC failed.
 * Returns -1 if packet not addressed to us.
 * Returns -2 if ACK TX failed.
 */
int common_data_verify_and_ack(const data_packet_t *pkt, uint16_t len,
			       int16_t rssi_2);

/* Drain RX queue, collecting PAIR_RESPONSEs into discovery subsystem.
 * If log_each is true, logs each response as it is received. */
void drain_discovery_responses(bool log_each);

/* Factory reset: clear all NVM, sleep 500 ms, cold reboot. Does not return. */
void factory_reset_reboot(void);

/* Store RSSI threshold in NVM, sleep 500 ms, cold reboot. Does not return. */
void rssi_store_and_reboot(int16_t rssi_dbm);

#endif /* COMMON_H */
