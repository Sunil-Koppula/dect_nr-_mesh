/*
 * Large data transfer protocol for DECT NR+ mesh network
 *
 * Sender flow:
 *   1. Send LARGE_DATA_INIT (file type, total size, fragment count)
 *   2. Send LARGE_DATA_TRANSFER for each fragment (frag_num + payload)
 *   3. Send LARGE_DATA_END (last fragment + CRC16 over all data)
 *   4. Wait for LARGE_DATA_ACK (success / CRC fail)
 *
 * Receiver flow:
 *   - On INIT: allocate reassembly context for that sender
 *   - On TRANSFER: store fragment in buffer by frag_num
 *   - On END: store last fragment, verify CRC, send ACK
 *
 * Multiple senders are tracked by src_device_id.
 */

#ifndef LARGE_DATA_H
#define LARGE_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

/* Max concurrent large data senders we can reassemble */
#define LARGE_DATA_MAX_SESSIONS 4

/* Max total data size we can reassemble (80KB) */
#define LARGE_DATA_MAX_SIZE (80 * 1024)

/* Reassembly session state */
typedef enum {
	LARGE_DATA_STATE_IDLE = 0,
	LARGE_DATA_STATE_RECEIVING,
	LARGE_DATA_STATE_COMPLETE,
} large_data_state_t;

/* Reassembly context — one per sender */
typedef struct {
	large_data_state_t state;
	uint16_t src_device_id;     /* sender ID (key for lookup) */
	uint8_t file_type;
	uint32_t total_size;
	uint16_t frag_total;
	uint8_t last_frag_size;
	uint16_t crc16;             /* expected CRC (from INIT) */
	uint16_t frags_received;    /* count of fragments received */
	uint8_t *buffer;            /* reassembly buffer (heap allocated) */
} large_data_rx_session_t;

/* Initialize large data module */
void large_data_init(void);

/* --- Receiver API --- */

/* Handle LARGE_DATA_INIT packet (ISR-safe) */
void large_data_handle_init(const large_data_init_packet_t *pkt);

/* Handle LARGE_DATA_TRANSFER packet (ISR-safe) */
void large_data_handle_transfer(const large_data_transfer_packet_t *pkt,
				uint16_t len);

/* Handle LARGE_DATA_END packet (ISR-safe) — stores last fragment,
 * verifies CRC, queues a pending ACK for the main thread to send */
void large_data_handle_end(const large_data_end_packet_t *pkt, uint16_t len);

/* Check if there's a pending ACK to send (called from main thread).
 * If an END was processed in ISR, this sends the ACK and frees the session.
 * tx_handle: PHY handle for sending the ACK */
void large_data_send_pending_ack(uint32_t tx_handle);

/* Semaphore signaled when END is processed and ACK needs to be sent.
 * Main loop should check this to break out of RX early. */
extern struct k_sem large_data_end_sem;

/* --- Sender API --- */

/* Send large data to dst_id. Blocks until transfer complete + ACK received.
 * tx_handle: PHY handle for TX
 * rx_handle: PHY handle for RX (ACK listen)
 * Returns 0 on success (ACK received with SUCCESS), negative on error */
int large_data_send(uint32_t tx_handle, uint32_t rx_handle,
		    uint16_t dst_id, uint8_t file_type,
		    const void *data, uint32_t data_len);

#endif /* LARGE_DATA_H */
