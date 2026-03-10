/*
 * Packet definitions for DECT NR+ mesh network
 */

#ifndef PACKET_H
#define PACKET_H

#include <stdint.h>

/* Device Type */
typedef enum {
	DEVICE_TYPE_UNKNOWN = 0x00,
	DEVICE_TYPE_GATEWAY = 0x01,
	DEVICE_TYPE_ANCHOR  = 0x02,
	DEVICE_TYPE_SENSOR  = 0x03,
} device_type_t;

/* Packet Types Identifier */
typedef enum {
	PACKET_TYPE_UNKNOWN       = 0x00,
	PACKET_TYPE_PAIR_REQUEST  = 0x01,
	PACKET_TYPE_PAIR_RESPONSE = 0x02,
	PACKET_TYPE_PAIR_CONFIRM  = 0x03,
	PACKET_TYPE_DATA          = 0x04,
	PACKET_TYPE_DATA_ACK      = 0x05,
} packet_type_t;

/* Pairing confirm status codes */
#define PAIR_STATUS_SUCCESS 0x00
#define PAIR_STATUS_FAILURE 0x01

/* Max application data length per PHY subslot */
#define DATA_LEN_MAX 32

/********** Packet Structures **********/

/* Pairing Request Packet (8 bytes) */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint8_t device_type;        /* device_type_t */
	uint16_t device_id;
	uint32_t random_num;
} __attribute__((packed)) pair_request_packet_t;

#define PAIR_REQUEST_PACKET_SIZE sizeof(pair_request_packet_t)

/* Pairing Response Packet (11 bytes) — unicast to requester */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint8_t device_type;        /* device_type_t */
	uint16_t device_id;         /* responder's ID */
	uint16_t dst_device_id;     /* requester's ID (unicast target) */
	uint32_t hash;
	uint8_t hop_num;
} __attribute__((packed)) pair_response_packet_t;

#define PAIR_RESPONSE_PACKET_SIZE sizeof(pair_response_packet_t)

/* Pairing Confirm Packet (7 bytes) — unicast to responder */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint8_t device_type;        /* device_type_t */
	uint16_t device_id;         /* confirmer's ID */
	uint16_t dst_device_id;     /* responder's ID (unicast target) */
	uint8_t status;             /* PAIR_STATUS_SUCCESS / PAIR_STATUS_FAILURE */
} __attribute__((packed)) pair_confirm_packet_t;

#define PAIR_CONFIRM_PACKET_SIZE sizeof(pair_confirm_packet_t)

/* Data Packet (5 bytes header + variable payload) */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t payload[0];
} __attribute__((packed)) data_packet_t;

#define DATA_PACKET_SIZE sizeof(data_packet_t)

/* Data Acknowledgment Packet (7 bytes) */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t hop_num;
	uint8_t status;
} __attribute__((packed)) data_ack_packet_t;

#define DATA_ACK_PACKET_SIZE sizeof(data_ack_packet_t)

/* Get device type as string */
static inline const char *device_type_str(device_type_t type)
{
	switch (type) {
	case DEVICE_TYPE_GATEWAY: return "GATEWAY";
	case DEVICE_TYPE_ANCHOR:  return "ANCHOR";
	case DEVICE_TYPE_SENSOR:  return "SENSOR";
	default:                  return "UNKNOWN";
	}
}

#endif /* PACKET_H */
