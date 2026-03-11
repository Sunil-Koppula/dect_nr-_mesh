/*
 * RX message queue for DECT NR+ mesh network
 * Decouples PHY ISR context from main thread processing
 */

#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include "packet.h"

#define RX_QUEUE_SIZE 16

/* Received packet item */
struct rx_queue_item {
	uint8_t data[DATA_LEN_MAX];
	uint16_t len;
	int16_t rssi_2;
};

/* Enqueue a received packet (safe to call from ISR) */
int rx_queue_put(const void *data, uint16_t len, int16_t rssi_2);

/* Dequeue a received packet (blocks until available or timeout) */
int rx_queue_get(struct rx_queue_item *item, k_timeout_t timeout);

#endif /* QUEUE_H */
