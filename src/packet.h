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
	PACKET_TYPE_UNKNOWN       		= 0x00,
	PACKET_TYPE_PAIR_REQUEST  		= 0x01,
	PACKET_TYPE_PAIR_RESPONSE 		= 0x02,
	PACKET_TYPE_PAIR_CONFIRM  		= 0x03,
	PACKET_TYPE_DATA          		= 0x04,
	PACKET_TYPE_DATA_ACK      		= 0x05,
	PACKET_TYPE_LARGE_DATA_INIT     = 0x06,
	PACKET_TYPE_LARGE_DATA_TRANSFER = 0x07,
	PACKET_TYPE_LARGE_DATA_END	  	= 0x08,
	PACKET_TYPE_LARGE_DATA_ACK      = 0x09,
	PACKET_TYPE_LARGE_DATA_NACK     = 0x0A,
	PACKET_TYPE_STREAM_REQUEST      = 0x0B,
	PACKET_TYPE_OTA_INIT            = 0x0C,
	PACKET_TYPE_OTA_ACK             = 0x0D,
} packet_type_t;

/* Pairing confirm status codes */
#define PAIR_STATUS_SUCCESS 0x00
#define PAIR_STATUS_FAILURE 0x01

/* Data ACK status codes */
#define DATA_ACK_SUCCESS  0x00
#define DATA_ACK_CRC_FAIL 0x01

/* Large data file types */
#define LARGE_DATA_FILE_DATA  0x00
#define LARGE_DATA_FILE_OTA   0x01  /* OTA firmware image (unified, same for all devices) */

/* Large data ACK status codes */
#define LARGE_DATA_ACK_SUCCESS  0x00
#define LARGE_DATA_ACK_CRC_FAIL 0x01

/* Max application data length per PHY subslot */
#define DATA_LEN_MAX 32

/********** Packet Structures **********/

/* Pairing Request Packet (12 bytes) */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint8_t device_type;        /* device_type_t */
	uint16_t device_id;
	uint32_t random_num;
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t version_patch;
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

/* Pairing Confirm Packet (11 bytes) — unicast to responder */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint8_t device_type;        /* device_type_t */
	uint16_t device_id;         /* confirmer's ID */
	uint16_t dst_device_id;     /* responder's ID (unicast target) */
	uint8_t status;             /* PAIR_STATUS_SUCCESS / PAIR_STATUS_FAILURE */
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t version_patch;
} __attribute__((packed)) pair_confirm_packet_t;

#define PAIR_CONFIRM_PACKET_SIZE sizeof(pair_confirm_packet_t)

/* Data Packet (6 bytes header + variable payload + 2 bytes CRC16)
 * Wire format: [header][payload][crc16][padding...]
 * CRC16 is computed over the payload bytes only.
 * payload_len tells receiver the actual payload size (PHY pads to subslot).
 */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t payload_len;        /* actual payload length */
	uint8_t payload[0];         /* followed by uint16_t crc16 */
} __attribute__((packed)) data_packet_t;

#define DATA_PACKET_SIZE sizeof(data_packet_t)
#define DATA_CRC_SIZE    2  /* uint16_t CRC16 appended after payload */

/* Data Acknowledgment Packet (7 bytes) */
typedef struct {
	uint8_t packet_type;        /* packet_type_t */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t hop_num;
	uint8_t status;
} __attribute__((packed)) data_ack_packet_t;

#define DATA_ACK_PACKET_SIZE sizeof(data_ack_packet_t)

/********** Large Data Transfer Packets **********/

/* Large Data Init — announces a new transfer */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_LARGE_DATA_INIT */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t file_type;          /* LARGE_DATA_FILE_DATA / OTA */
	uint32_t total_size;        /* total data size in bytes */
	uint16_t frag_total;        /* total number of fragments */
	uint8_t last_frag_size;     /* payload size of last fragment */
	uint16_t crc16;             /* CRC16 over ALL data bytes */
} __attribute__((packed)) large_data_init_packet_t;

#define LARGE_DATA_INIT_PACKET_SIZE sizeof(large_data_init_packet_t)

/* Large Data Transfer — one fragment of data */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_LARGE_DATA_TRANSFER */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint16_t frag_num;          /* fragment index (0-based) */
	uint8_t payload[0];         /* up to LARGE_DATA_FRAG_SIZE bytes */
} __attribute__((packed)) large_data_transfer_packet_t;

#define LARGE_DATA_TRANSFER_PACKET_SIZE sizeof(large_data_transfer_packet_t)
#define LARGE_DATA_FRAG_SIZE (DATA_LEN_MAX - LARGE_DATA_TRANSFER_PACKET_SIZE)

/* Large Data End — last fragment (same layout as TRANSFER) */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_LARGE_DATA_END */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint16_t frag_num;          /* last fragment index */
	uint8_t payload[0];         /* last fragment data */
} __attribute__((packed)) large_data_end_packet_t;

#define LARGE_DATA_END_PACKET_SIZE sizeof(large_data_end_packet_t)
#define LARGE_DATA_END_FRAG_SIZE (DATA_LEN_MAX - LARGE_DATA_END_PACKET_SIZE)

/* Large Data ACK — receiver confirms whole transfer */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_LARGE_DATA_ACK */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t status;             /* LARGE_DATA_ACK_SUCCESS / CRC_FAIL */
} __attribute__((packed)) large_data_ack_packet_t;

#define LARGE_DATA_ACK_PACKET_SIZE sizeof(large_data_ack_packet_t)

/* Large Data NACK — request retransmission of missing fragments */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_LARGE_DATA_NACK */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t frag_count;         /* number of missing fragment numbers following */
	uint16_t frag_nums[0];      /* missing fragment numbers */
} __attribute__((packed)) large_data_nack_packet_t;

#define LARGE_DATA_NACK_PACKET_SIZE sizeof(large_data_nack_packet_t)
/* Max missing frags per NACK packet */
#define LARGE_DATA_NACK_MAX_FRAGS \
	((DATA_LEN_MAX - LARGE_DATA_NACK_PACKET_SIZE) / sizeof(uint16_t))

/* Stream Request — sensor asks gateway to stream data for 60s every 500ms */
typedef struct {
	uint8_t packet_type;
	uint16_t src_device_id;
	uint16_t dst_device_id;
} __attribute__((packed)) stream_request_packet_t;

#define STREAM_REQUEST_PACKET_SIZE sizeof(stream_request_packet_t)

/********** OTA Update Packets **********/

/* OTA ACK status codes */
#define OTA_ACK_ACCEPT   0x00  /* Version is newer, ready to receive */
#define OTA_ACK_REJECT   0x01  /* Version is same or older, skip */

/* OTA Init — announces firmware version to target */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_OTA_INIT */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t version_major;
	uint8_t version_minor;
	uint16_t version_patch;
	uint32_t image_size;        /* total OTA image size */
} __attribute__((packed)) ota_init_packet_t;

#define OTA_INIT_PACKET_SIZE sizeof(ota_init_packet_t)

/* OTA ACK — target responds with accept/reject */
typedef struct {
	uint8_t packet_type;        /* PACKET_TYPE_OTA_ACK */
	uint16_t src_device_id;
	uint16_t dst_device_id;
	uint8_t status;             /* OTA_ACK_ACCEPT / OTA_ACK_REJECT */
	uint8_t version_major;      /* target's current version */
	uint8_t version_minor;
	uint16_t version_patch;
} __attribute__((packed)) ota_ack_packet_t;

#define OTA_ACK_PACKET_SIZE sizeof(ota_ack_packet_t)

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
