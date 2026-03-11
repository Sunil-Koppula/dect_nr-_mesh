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

/* Button semaphores (defined in main.c) */
extern struct k_sem btn2_sem;  /* small data send */
extern struct k_sem btn3_sem;  /* 25KB large data send */
extern struct k_sem btn4_sem;  /* 10KB large data send */

#endif /* STATE_H */
