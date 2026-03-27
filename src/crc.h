/*
 * CRC, hashing, and random utilities for DECT NR+ mesh network
 */

#ifndef CRC_H
#define CRC_H

#include <stdint.h>

/* CRC16 for data integrity */
uint16_t compute_crc16(const void *data, uint32_t len);
uint16_t compute_crc16_continue(uint16_t crc, const void *data, uint32_t len);

/* Compute pairing hash */
uint32_t compute_pair_hash(uint16_t dev_id, uint32_t random_num);

/* Random number */
uint32_t next_random(void);

#endif /* CRC_H */
