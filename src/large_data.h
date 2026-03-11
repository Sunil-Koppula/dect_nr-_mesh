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
 *   - On INIT: erase flash slot, allocate bitmap in RAM
 *   - On TRANSFER: write fragment directly to external flash
 *   - On END: write last fragment, verify CRC from flash, send ACK
 *
 * Multiple senders are tracked by src_device_id.
 * Fragment data is stored on external SPI flash (not RAM).
 * Only the fragment bitmap is kept in RAM (~bytes per session).
 */

#ifndef LARGE_DATA_H
#define LARGE_DATA_H

#include <stdint.h>
#include <stdbool.h>
#include "packet.h"

/* Max concurrent large data senders we can reassemble */
#define LARGE_DATA_MAX_SESSIONS 4

/* Max total data size we can reassemble (256KB per slot) */
#define LARGE_DATA_MAX_SIZE (256 * 1024)

/* Max bitmap size in bytes (covers up to 256KB / 25-byte frags ~ 10486 frags) */
#define LARGE_DATA_BITMAP_MAX ((LARGE_DATA_MAX_SIZE / LARGE_DATA_FRAG_SIZE + 8) / 8)

/* Reassembly session state */
typedef enum {
	LARGE_DATA_STATE_IDLE = 0,
	LARGE_DATA_STATE_RECEIVING,
	LARGE_DATA_STATE_COMPLETE,
	LARGE_DATA_STATE_ERASE_PENDING,
} large_data_state_t;

/* Reassembly context — one per sender.
 * Fragment data is stored in external flash (one slot per session).
 * Only the bitmap is in RAM. */
typedef struct {
	large_data_state_t state;
	uint16_t src_device_id;     /* sender ID (key for lookup) */
	uint8_t file_type;
	uint8_t flash_slot;         /* flash slot index (0..MAX_SESSIONS-1) */
	uint32_t total_size;
	uint16_t frag_total;
	uint8_t last_frag_size;
	uint16_t crc16;             /* expected CRC (from INIT) */
	uint16_t frags_received;    /* count of fragments received */
	uint8_t frag_bitmap[LARGE_DATA_BITMAP_MAX]; /* received-fragment bitmap */
} large_data_rx_session_t;

/* Initialize large data module (call after flash_store_init) */
void large_data_init(void);

/* --- Receiver API --- */

/* Handle LARGE_DATA_INIT packet.
 * NOTE: no longer ISR-safe — flash erase is deferred to main thread.
 * INIT packets are queued via rx_queue like normal packets. */
void large_data_handle_init(const large_data_init_packet_t *pkt);

/* Handle LARGE_DATA_TRANSFER packet.
 * Called from ISR — queues fragment to ring buffer for flash writer thread. */
void large_data_handle_transfer(const large_data_transfer_packet_t *pkt,
				uint16_t len);

/* Handle LARGE_DATA_END packet.
 * Called from ISR — queues to ring buffer for flash write + CRC verify. */
void large_data_handle_end(const large_data_end_packet_t *pkt, uint16_t len);

/* Send pending ACK/NACK responses (called from main thread) */
void large_data_send_pending_ack(uint32_t tx_handle);

/* Process deferred INIT packets that need flash erase (main thread) */
void large_data_process_pending_init(void);

/* Semaphore signaled when END is processed and ACK needs to be sent.
 * Main loop should check this to break out of RX early. */
extern struct k_sem large_data_end_sem;

/* Semaphore signaled when INIT needs flash erase from main thread */
extern struct k_sem large_data_init_sem;

/* Retrieve a completed large data session.
 * Returns flash slot, size, file type, and sender ID.
 * Caller reads data from flash via flash_store_read().
 * Call large_data_free_completed() when done. */
bool large_data_get_completed(uint8_t *flash_slot, uint32_t *size,
			      uint8_t *file_type, uint16_t *src_id);

/* Free a completed session */
void large_data_free_completed(uint16_t src_id);

/* --- Sender API --- */

/* Send large data to dst_id. Blocks until transfer complete + ACK received.
 * tx_handle: PHY handle for TX
 * rx_handle: PHY handle for RX (ACK listen)
 * Returns 0 on success (ACK received with SUCCESS), negative on error */
int large_data_send(uint32_t tx_handle, uint32_t rx_handle,
		    uint16_t dst_id, uint8_t file_type,
		    const void *data, uint32_t data_len);

#endif /* LARGE_DATA_H */
