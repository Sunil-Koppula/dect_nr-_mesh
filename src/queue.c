/*
 * RX message queue for DECT NR+ mesh network
 */

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "queue.h"

LOG_MODULE_REGISTER(queue, CONFIG_QUEUE_LOG_LEVEL);

K_MSGQ_DEFINE(rx_msgq, sizeof(struct rx_queue_item), RX_QUEUE_SIZE, 4);

int rx_queue_put(const void *data, uint16_t len, int16_t rssi_2)
{
	struct rx_queue_item item;

	if (len > DATA_LEN_MAX) {
		len = DATA_LEN_MAX;
	}

	memcpy(item.data, data, len);
	item.len = len;
	item.rssi_2 = rssi_2;

	int err = k_msgq_put(&rx_msgq, &item, K_NO_WAIT);

	if (err) {
		LOG_WRN("RX queue full, dropping packet");
	}

	return err;
}

int rx_queue_get(struct rx_queue_item *item, k_timeout_t timeout)
{
	return k_msgq_get(&rx_msgq, item, timeout);
}
