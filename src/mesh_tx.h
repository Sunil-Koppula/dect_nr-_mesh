/*
 * Mesh TX helpers for DECT NR+ mesh network
 */

#ifndef MESH_TX_H
#define MESH_TX_H

#include <stdint.h>
#include <stddef.h>

/* TX helpers */
int send_pair_request(uint32_t handle, uint32_t random_num);
int send_pair_response(uint32_t handle, uint16_t dst_id, uint32_t hash);
int send_pair_confirm(uint32_t handle, uint16_t dst_id, uint8_t status);
int send_data(uint32_t handle, uint16_t dst_id, const void *payload,
	      uint16_t payload_len);

#endif /* MESH_TX_H */
