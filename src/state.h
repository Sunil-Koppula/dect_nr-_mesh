/*
 * Shared device state for DECT NR+ mesh network
 */

#ifndef STATE_H
#define STATE_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include "packet.h"

/* Device identity and role (defined in main.c) */
extern uint16_t device_id;
extern device_type_t my_device_type;
extern uint8_t my_hop_num;

/* Button 2 semaphore — triggers sensor data send (defined in main.c) */
extern struct k_sem btn2_sem;

#endif /* STATE_H */
