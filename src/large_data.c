/*
 * Large data transfer protocol for DECT NR+ mesh network
 *
 * Fragment data is stored in external SPI PSRAM (8MB) instead of internal RAM.
 * Only the fragment bitmap (~1.3KB per session) lives in internal RAM.
 * This allows 4 concurrent sessions of up to 256KB each.
 *
 * PSRAM layout (4 slots, 256KB each, starting at offset 0):
 *   Slot 0: 0x000000 - 0x03FFFF
 *   Slot 1: 0x040000 - 0x07FFFF
 *   Slot 2: 0x080000 - 0x0BFFFF
 *   Slot 3: 0x0C0000 - 0x0FFFFF
 *
 * INIT/TRANSFER/END: queued to writer thread via ring buffer
 *
 * SPI operations are NOT ISR-safe (they use mutexes/sleep),
 * so all PSRAM writes happen in a dedicated worker thread.
 */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/random.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/devicetree.h>
#include "large_data.h"
#include "psram.h"
#include "crc.h"
#include "mesh_tx.h"
#include "radio.h"
#include "queue.h"
#include "identity.h"
#include "display.h"
#include "log_all.h"

LOG_MODULE_REGISTER(large_data, CONFIG_LARGE_DATA_LOG_LEVEL);

BUILD_ASSERT(LARGE_DATA_MAX_SESSIONS * LARGE_DATA_SLOT_SIZE <= PSRAM_SIZE,
	     "PSRAM must have enough space for all session slots");

#define RETRANSMIT_MAX 5

/* ===== Fragment ring buffer for ISR -> thread offloading ===== */

/*
 * Ring buffer entries for queuing fragment writes from ISR to worker thread.
 * SPI PSRAM writes are fast but still not ISR-safe, so we buffer here.
 * At ~200 frags/sec arrival rate, 512 entries gives ~2.5 sec of buffering.
 */
#define FRAG_RING_SIZE 512
#define FRAG_RING_MASK (FRAG_RING_SIZE - 1)

BUILD_ASSERT((FRAG_RING_SIZE & FRAG_RING_MASK) == 0,
	     "FRAG_RING_SIZE must be power of 2");

enum frag_ring_type {
	FRAG_RING_TRANSFER = 0,
	FRAG_RING_END,
	FRAG_RING_INIT,
};

struct frag_ring_entry {
	uint8_t data[DATA_LEN_MAX];
	uint16_t len;
	uint8_t type;
};

static struct frag_ring_entry frag_ring[FRAG_RING_SIZE];
static volatile uint32_t frag_ring_head; /* written by ISR */
static volatile uint32_t frag_ring_tail; /* read by worker */

/* Semaphore to wake the PSRAM writer thread */
static K_SEM_DEFINE(frag_write_sem, 0, K_SEM_MAX_LIMIT);

/* PSRAM writer thread — high priority so it drains the ring
 * faster than fragments arrive, preventing overflow */
#define PSRAM_WRITER_STACK_SIZE 2048
#define PSRAM_WRITER_PRIORITY 2
static K_THREAD_STACK_DEFINE(psram_writer_stack, PSRAM_WRITER_STACK_SIZE);
static struct k_thread psram_writer_thread;

/* ===== Receiver: reassembly sessions ===== */

static large_data_rx_session_t sessions[LARGE_DATA_MAX_SESSIONS];

/* Pending responses from END packets (ACK or NACK) */
K_SEM_DEFINE(large_data_end_sem, 0, LARGE_DATA_MAX_SESSIONS);
static struct {
	uint8_t data[DATA_LEN_MAX];   /* ACK or NACK packet */
	uint16_t len;
	large_data_rx_session_t *session;
	bool free_session;
	volatile bool valid;
} pending_responses[LARGE_DATA_MAX_SESSIONS];

/* Kept for API compatibility — no longer used (INIT goes through ring) */
K_SEM_DEFINE(large_data_init_sem, 0, LARGE_DATA_MAX_SESSIONS);

static large_data_rx_session_t *find_session(uint16_t src_device_id)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (sessions[i].state != LARGE_DATA_STATE_IDLE &&
		    sessions[i].src_device_id == src_device_id) {
			return &sessions[i];
		}
	}
	return NULL;
}

static large_data_rx_session_t *alloc_session(uint8_t *slot_out)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (sessions[i].state == LARGE_DATA_STATE_IDLE) {
			*slot_out = (uint8_t)i;
			return &sessions[i];
		}
	}
	return NULL;
}

static void free_session(large_data_rx_session_t *s)
{
	s->state = LARGE_DATA_STATE_IDLE;
}

void large_data_cleanup_stale_sessions(void)
{
	int64_t now = k_uptime_get();

	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		large_data_rx_session_t *s = &sessions[i];

		if (s->state == LARGE_DATA_STATE_RECEIVING &&
		    (now - s->last_activity_ms) > LARGE_DATA_SESSION_TIMEOUT_MS) {
			ALL_WRN("Session for ID:%d timed out (%d/%d frags), freeing slot %d",
				s->src_device_id, s->frags_received,
				s->frag_total, s->psram_slot);
			s->state = LARGE_DATA_STATE_IDLE;
		}
	}
}

bool large_data_any_active_session(void)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (sessions[i].state == LARGE_DATA_STATE_RECEIVING) {
			return true;
		}
	}
	return false;
}

/* ===== PSRAM writer: processes fragment ring in thread context ===== */

static inline uint32_t psram_slot_offset(uint8_t slot, uint32_t offset)
{
	return (uint32_t)slot * LARGE_DATA_SLOT_SIZE + offset;
}

static void process_transfer(const large_data_transfer_packet_t *pkt,
			     uint16_t len)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	large_data_rx_session_t *s = find_session(pkt->src_device_id);

	if (!s || s->state != LARGE_DATA_STATE_RECEIVING) {
		return;
	}

	if (pkt->frag_num >= s->frag_total) {
		ALL_WRN("Fragment %d out of range (total:%d)",
			pkt->frag_num, s->frag_total);
		return;
	}

	uint16_t payload_len = len - LARGE_DATA_TRANSFER_PACKET_SIZE;
	uint32_t offset = (uint32_t)pkt->frag_num * LARGE_DATA_FRAG_SIZE;

	if (offset + payload_len > s->total_size) {
		ALL_WRN("Fragment %d would overflow buffer", pkt->frag_num);
		return;
	}

	/* Skip if already received */
	if (s->frag_bitmap[pkt->frag_num / 8] & (1 << (pkt->frag_num % 8))) {
		return;
	}

	/* Write fragment to PSRAM (safe — we're in thread context) */
	int err = psram_write(psram_slot_offset(s->psram_slot, offset),
			      pkt->payload, payload_len);
	if (err) {
		ALL_WRN("PSRAM write frag %d failed, err %d",
			pkt->frag_num, err);
		return;
	}

	s->frag_bitmap[pkt->frag_num / 8] |= (1 << (pkt->frag_num % 8));
	s->frags_received++;
	s->last_activity_ms = k_uptime_get();

	if (s->frags_received % 100 == 0) {
		LOG_INF("Received %d/%d fragments from ID:%d",
			s->frags_received, s->frag_total,
			pkt->src_device_id);
	}
}

/* Compute CRC-16 over data stored in PSRAM (reads in chunks) */
#define PSRAM_CRC_CHUNK 512

static uint16_t psram_compute_crc16(uint8_t slot, uint32_t total_size)
{
	uint8_t buf[PSRAM_CRC_CHUNK];
	uint16_t crc = 0xFFFF;
	uint32_t remaining = total_size;
	uint32_t offset = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > PSRAM_CRC_CHUNK) ?
				 PSRAM_CRC_CHUNK : (uint16_t)remaining;

		int err = psram_read(psram_slot_offset(slot, offset),
				     buf, chunk);
		if (err) {
			LOG_ERR("PSRAM read for CRC failed at offset %d, err %d",
				offset, err);
			return 0;
		}

		for (uint16_t i = 0; i < chunk; i++) {
			crc ^= (uint16_t)buf[i] << 8;
			for (int j = 0; j < 8; j++) {
				if (crc & 0x8000) {
					crc = (crc << 1) ^ 0x1021;
				} else {
					crc <<= 1;
				}
			}
		}

		offset += chunk;
		remaining -= chunk;
	}

	return crc;
}

static void process_end(const large_data_end_packet_t *pkt, uint16_t len)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	large_data_rx_session_t *s = find_session(pkt->src_device_id);

	if (!s) {
		ALL_WRN("No session for sender ID:%d", pkt->src_device_id);
		return;
	}

	/* If session already completed, re-queue the SUCCESS ACK
	 * (sender missed it and is re-requesting) */
	if (s->state == LARGE_DATA_STATE_COMPLETE) {
		ALL_INF("Re-sending ACK for completed session from ID:%d",
			pkt->src_device_id);
		for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
			if (!pending_responses[i].valid) {
				large_data_ack_packet_t *ack =
					(large_data_ack_packet_t *)
						pending_responses[i].data;
				ack->packet_type = PACKET_TYPE_LARGE_DATA_ACK;
				ack->src_device_id = device_id;
				ack->dst_device_id = pkt->src_device_id;
				ack->status = STATUS_SUCCESS;
				pending_responses[i].len =
					LARGE_DATA_ACK_PACKET_SIZE;
				pending_responses[i].session = s;
				pending_responses[i].free_session = false;
				pending_responses[i].valid = true;
				k_sem_give(&large_data_end_sem);
				return;
			}
		}
		return;
	}

	if (s->state != LARGE_DATA_STATE_RECEIVING) {
		ALL_WRN("Session for ID:%d in unexpected state %d",
			pkt->src_device_id, s->state);
		return;
	}

	/* Write last fragment to PSRAM — use last_frag_size from INIT. */
	if (!(s->frag_bitmap[pkt->frag_num / 8] & (1 << (pkt->frag_num % 8)))) {
		uint16_t payload_len = s->last_frag_size;
		uint32_t offset = (uint32_t)pkt->frag_num * LARGE_DATA_FRAG_SIZE;

		if (offset + payload_len > s->total_size) {
			ALL_WRN("Last fragment would overflow buffer");
			free_session(s);
			return;
		}

		int err = psram_write(psram_slot_offset(s->psram_slot, offset),
				      pkt->payload, payload_len);
		if (err) {
			ALL_WRN("PSRAM write last frag failed, err %d", err);
			free_session(s);
			return;
		}

		s->frag_bitmap[pkt->frag_num / 8] |= (1 << (pkt->frag_num % 8));
		s->frags_received++;
	}

	ALL_INF("Received %d/%d fragments from ID:%d, verifying CRC...",
		s->frags_received, s->frag_total, pkt->src_device_id);

	/* Verify CRC over entire reassembled data in PSRAM */
	uint16_t calc_crc = psram_compute_crc16(s->psram_slot, s->total_size);

	if (calc_crc == s->crc16) {
		ALL_INF("Large data CRC OK (%d bytes from ID:%d)",
			s->total_size, pkt->src_device_id);
		s->state = LARGE_DATA_STATE_COMPLETE;

		/* Queue SUCCESS ACK — keep session alive for relay */
		for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
			if (!pending_responses[i].valid) {
				large_data_ack_packet_t *ack =
					(large_data_ack_packet_t *)pending_responses[i].data;
				ack->packet_type = PACKET_TYPE_LARGE_DATA_ACK;
				ack->src_device_id = device_id;
				ack->dst_device_id = pkt->src_device_id;
				ack->status = STATUS_SUCCESS;
				pending_responses[i].len = LARGE_DATA_ACK_PACKET_SIZE;
				pending_responses[i].session = s;
				pending_responses[i].free_session = false;
				pending_responses[i].valid = true;
				k_sem_give(&large_data_end_sem);
				return;
			}
		}
		return;
	}

	/* CRC fail — count missing fragments */
	uint16_t missing_count = 0;

	for (uint16_t f = 0; f < s->frag_total; f++) {
		if (!(s->frag_bitmap[f / 8] & (1 << (f % 8)))) {
			missing_count++;
		}
	}

	ALL_WRN("Large data CRC FAIL missing %d/%d frags", missing_count, s->frag_total);

	/* Dump data at missing fragment offsets */
	for (uint16_t f = 0; f < s->frag_total; f++) {
		if (!(s->frag_bitmap[f / 8] & (1 << (f % 8)))) {
			uint32_t off = (uint32_t)f * LARGE_DATA_FRAG_SIZE;
			uint8_t dbg[8];
			psram_read(psram_slot_offset(s->psram_slot, off),
				   dbg, sizeof(dbg));
			LOG_WRN("  Missing frag %d @%d: %02x %02x %02x %02x %02x %02x %02x %02x",
				f, off, dbg[0], dbg[1], dbg[2], dbg[3],
				dbg[4], dbg[5], dbg[6], dbg[7]);
		}
	}

	/* If no missing frags, dump boundaries for manual inspection */
	if (missing_count == 0) {
		ALL_WRN("  All frags marked received — dumping boundaries");
		for (uint16_t f = 0; f < s->frag_total;
		     f += s->frag_total / 8) {
			uint32_t off = (uint32_t)f * LARGE_DATA_FRAG_SIZE;
			uint8_t dbg[8];
			psram_read(psram_slot_offset(s->psram_slot, off),
				   dbg, sizeof(dbg));
			LOG_WRN("  Frag %d @%d: %02x %02x %02x %02x %02x %02x %02x %02x",
				f, off, dbg[0], dbg[1], dbg[2],
				dbg[3], dbg[4], dbg[5], dbg[6],
				dbg[7]);
		}
	}

	if (missing_count > 0) {
		/* Queue multiple NACK packets to cover ALL missing fragments
		 * in one response cycle (each NACK holds up to 13 frag numbers) */
		uint16_t sent = 0;
		uint16_t frag_cursor = 0;
		int nack_count = 0;

		while (sent < missing_count) {
			/* Find a free response slot */
			int slot = -1;

			for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
				if (!pending_responses[i].valid) {
					slot = i;
					break;
				}
			}
			if (slot < 0) {
				ALL_WRN("No free response slots for NACK "
					"(%d/%d missing sent)",
					sent, missing_count);
				break;
			}

			uint16_t batch = missing_count - sent;

			if (batch > LARGE_DATA_NACK_MAX_FRAGS) {
				batch = LARGE_DATA_NACK_MAX_FRAGS;
			}

			large_data_nack_packet_t *nack =
				(large_data_nack_packet_t *)
					pending_responses[slot].data;
			nack->packet_type = PACKET_TYPE_LARGE_DATA_NACK;
			nack->src_device_id = device_id;
			nack->dst_device_id = pkt->src_device_id;
			nack->frag_count = (uint8_t)batch;

			uint8_t idx = 0;

			for (; frag_cursor < s->frag_total && idx < batch;
			     frag_cursor++) {
				if (!(s->frag_bitmap[frag_cursor / 8] &
				      (1 << (frag_cursor % 8)))) {
					nack->frag_nums[idx++] = frag_cursor;
				}
			}

			pending_responses[slot].len =
				LARGE_DATA_NACK_PACKET_SIZE +
				idx * sizeof(uint16_t);
			pending_responses[slot].session = s;
			pending_responses[slot].free_session = false;
			pending_responses[slot].valid = true;

			sent += idx;
			nack_count++;
		}

		s->state = LARGE_DATA_STATE_RECEIVING;
		s->frags_received--;
		ALL_INF("NACK: %d packets covering %d/%d missing frags",
			nack_count, sent, missing_count);
		k_sem_give(&large_data_end_sem);
	} else {
		/* 0 missing but CRC still wrong — data corruption */
		for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
			if (!pending_responses[i].valid) {
				large_data_ack_packet_t *ack =
					(large_data_ack_packet_t *)
						pending_responses[i].data;
				ack->packet_type = PACKET_TYPE_LARGE_DATA_ACK;
				ack->src_device_id = device_id;
				ack->dst_device_id = pkt->src_device_id;
				ack->status = STATUS_CRC_FAIL;
				pending_responses[i].len =
					LARGE_DATA_ACK_PACKET_SIZE;
				pending_responses[i].session = s;
				pending_responses[i].free_session = true;
				pending_responses[i].valid = true;
				k_sem_give(&large_data_end_sem);
				return;
			}
		}
	}
}

static void process_init_in_writer(const large_data_init_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	if (pkt->total_size > LARGE_DATA_MAX_SIZE) {
		ALL_ERR("Data too large (%d > %d)", pkt->total_size,
			LARGE_DATA_MAX_SIZE);
		return;
	}

	/* Find or allocate session */
	large_data_rx_session_t *s = find_session(pkt->src_device_id);
	uint8_t slot;

	if (s) {
		slot = s->psram_slot;
	} else {
		s = alloc_session(&slot);
	}

	if (!s) {
		ALL_ERR("No free reassembly sessions");
		return;
	}

	/* No erase needed for PSRAM — just overwrite.
	 * Initialize session. */
	s->state = LARGE_DATA_STATE_RECEIVING;
	s->src_device_id = pkt->src_device_id;
	s->file_type = pkt->file_type;
	s->psram_slot = slot;
	s->total_size = pkt->total_size;
	s->frag_total = pkt->frag_total;
	s->last_frag_size = pkt->last_frag_size;
	s->crc16 = pkt->crc16;
	s->frags_received = 0;
	s->last_activity_ms = k_uptime_get();
	memset(s->frag_bitmap, 0, sizeof(s->frag_bitmap));

	ALL_INF("Reassembly started for ID:%d (slot:%d, %d bytes, %d frags)",
		pkt->src_device_id, slot, pkt->total_size,
		pkt->frag_total);
}

static void psram_writer_entry(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		k_sem_take(&frag_write_sem, K_FOREVER);

		uint32_t tail = frag_ring_tail;

		if (tail == frag_ring_head) {
			continue;
		}

		struct frag_ring_entry *e = &frag_ring[tail & FRAG_RING_MASK];

		switch (e->type) {
		case FRAG_RING_INIT:
			process_init_in_writer(
				(const large_data_init_packet_t *)e->data);
			break;
		case FRAG_RING_END:
			process_end(
				(const large_data_end_packet_t *)e->data,
				e->len);
			break;
		case FRAG_RING_TRANSFER:
		default:
			process_transfer(
				(const large_data_transfer_packet_t *)e->data,
				e->len);
			break;
		}

		/* Advance tail (only this thread writes tail) */
		frag_ring_tail = tail + 1;
	}
}

/* ===== Public API ===== */

void large_data_init(void)
{
	memset(sessions, 0, sizeof(sessions));
	memset(pending_responses, 0, sizeof(pending_responses));
	frag_ring_head = 0;
	frag_ring_tail = 0;

	/* Start the PSRAM writer thread */
	k_thread_create(&psram_writer_thread, psram_writer_stack,
			PSRAM_WRITER_STACK_SIZE,
			psram_writer_entry, NULL, NULL, NULL,
			PSRAM_WRITER_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&psram_writer_thread, "psram_writer");
}

/*
 * INIT is now queued into the same ring buffer as TRANSFER/END.
 * The flash writer thread processes INIT (flash erase + session setup)
 * before any subsequent TRANSFER fragments, since they arrive in order.
 * Fragments arriving during the erase accumulate in the ring buffer.
 */
void large_data_handle_init(const large_data_init_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	ALL_INF("Large data INIT from ID:%d type:%d size:%d frags:%d last:%d",
		pkt->src_device_id, pkt->file_type, pkt->total_size,
		pkt->frag_total, pkt->last_frag_size);

	uint32_t head = frag_ring_head;
	uint32_t tail = frag_ring_tail;

	if (head - tail >= FRAG_RING_SIZE) {
		LOG_ERR("Fragment ring full, dropping INIT");
		return;
	}

	struct frag_ring_entry *e = &frag_ring[head & FRAG_RING_MASK];

	memcpy(e->data, pkt, sizeof(*pkt));
	e->len = sizeof(*pkt);
	e->type = FRAG_RING_INIT;

	frag_ring_head = head + 1;
	k_sem_give(&frag_write_sem);
}

void large_data_process_pending_init(void)
{
	/* No-op: INIT is now processed by the flash writer thread */
}

/*
 * Called from ISR (on_pdc). Copies fragment into ring buffer
 * and wakes the flash writer thread to do the actual flash write.
 */
void large_data_handle_transfer(const large_data_transfer_packet_t *pkt,
				uint16_t len)
{
	uint32_t head = frag_ring_head;
	uint32_t tail = frag_ring_tail;

	/* Check if ring is full */
	if (head - tail >= FRAG_RING_SIZE) {
		LOG_WRN("Fragment ring full, dropping frag %d", pkt->frag_num);
		return;
	}

	struct frag_ring_entry *e = &frag_ring[head & FRAG_RING_MASK];

	memcpy(e->data, pkt, len);
	e->len = len;
	e->type = FRAG_RING_TRANSFER;

	/* Advance head (only ISR writes head) */
	frag_ring_head = head + 1;
	k_sem_give(&frag_write_sem);
}

/*
 * Called from ISR (on_pdc). Queues END packet for thread processing.
 * END triggers CRC verification and ACK generation, which need thread context.
 */
void large_data_handle_end(const large_data_end_packet_t *pkt, uint16_t len)
{
	uint32_t head = frag_ring_head;
	uint32_t tail = frag_ring_tail;

	/* Check if ring is full */
	if (head - tail >= FRAG_RING_SIZE) {
		LOG_WRN("Fragment ring full, dropping END packet");
		return;
	}

	struct frag_ring_entry *e = &frag_ring[head & FRAG_RING_MASK];

	memcpy(e->data, pkt, len);
	e->len = len;
	e->type = FRAG_RING_END;

	frag_ring_head = head + 1;
	k_sem_give(&frag_write_sem);
}

void large_data_send_pending_ack(uint32_t tx_handle)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (!pending_responses[i].valid) {
			continue;
		}
		pending_responses[i].valid = false;

		uint8_t pkt_type = pending_responses[i].data[0];

		int err = transmit(tx_handle, pending_responses[i].data,
				   pending_responses[i].len);

		if (err) {
			ALL_ERR("Failed to send large data response, err %d", err);
		} else {
			k_sem_take(&operation_sem, K_FOREVER);
			if (pkt_type == PACKET_TYPE_LARGE_DATA_ACK) {
				large_data_ack_packet_t *ack =
					(large_data_ack_packet_t *)pending_responses[i].data;
				ALL_INF("Large data ACK sent to ID:%d (status:%s)",
					ack->dst_device_id,
					ack->status == STATUS_SUCCESS ?
						"OK" : "CRC_FAIL");
			} else if (pkt_type == PACKET_TYPE_LARGE_DATA_NACK) {
				large_data_nack_packet_t *nack =
					(large_data_nack_packet_t *)pending_responses[i].data;
				ALL_INF("Large data NACK sent to ID:%d (%d missing frags)",
					nack->dst_device_id, nack->frag_count);
			}
		}

		if (pending_responses[i].session) {
			if (pending_responses[i].free_session) {
				free_session(pending_responses[i].session);
				pending_responses[i].session = NULL;
			}
		}

		k_sleep(K_MSEC(10));
	}
}

bool large_data_get_completed(uint8_t *psram_slot, uint32_t *size,
			      uint8_t *file_type, uint16_t *src_id)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (sessions[i].state == LARGE_DATA_STATE_COMPLETE) {
			*psram_slot = sessions[i].psram_slot;
			*size = sessions[i].total_size;
			*file_type = sessions[i].file_type;
			*src_id = sessions[i].src_device_id;
			return true;
		}
	}
	return false;
}

void large_data_free_completed(uint16_t src_id)
{
	large_data_rx_session_t *s = find_session(src_id);

	if (s && s->state == LARGE_DATA_STATE_COMPLETE) {
		free_session(s);
	}
}

/* ===== Sender: fragment and transmit ===== */

static int tx_with_retry(uint32_t tx_handle, void *data, size_t len)
{
	for (int retries = 0; retries < 10; retries++) {
		last_op_err = 0;
		int err = transmit(tx_handle, data, len);

		if (err) {
			return err;
		}
		k_sem_take(&operation_sem, K_FOREVER);
		if (last_op_err == 0) {
			return 0;
		}
		/* Random backoff to avoid collision with other sensors */
		k_sleep(K_MSEC(10 + retries * 5 +
			       (sys_rand32_get() % 20)));
	}
	ALL_ERR("TX failed after retries, err %d", last_op_err);
	return -EIO;
}

int large_data_send(uint32_t tx_handle, uint32_t rx_handle,
		    uint16_t dst_id, uint8_t file_type,
		    const void *data, uint32_t data_len)
{
	if (data_len == 0 || data_len > LARGE_DATA_MAX_SIZE) {
		return -EINVAL;
	}

	const uint8_t *src = data;

	uint16_t frag_total = (data_len + LARGE_DATA_FRAG_SIZE - 1) /
			      LARGE_DATA_FRAG_SIZE;
	uint16_t transfer_count = frag_total - 1;
	uint8_t last_frag_size = (uint8_t)(data_len -
			(uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE);

	uint16_t crc = compute_crc16(data, data_len);
	int err;

	ALL_INF("Large data send: %d bytes, %d frags, last:%d, CRC:0x%04x",
		data_len, frag_total, last_frag_size, crc);

	/* Flush any stale packets in rx_queue before starting */
	{
		struct rx_queue_item flush_item;

		while (rx_queue_get(&flush_item, K_NO_WAIT) == 0) {
		}
	}

	/* Step 1: Send INIT */
	large_data_init_packet_t init_pkt = {
		.packet_type = PACKET_TYPE_LARGE_DATA_INIT,
		.src_device_id = device_id,
		.dst_device_id = dst_id,
		.file_type = file_type,
		.total_size = data_len,
		.frag_total = frag_total,
		.last_frag_size = last_frag_size,
		.crc16 = crc,
	};

	err = tx_with_retry(tx_handle, &init_pkt, sizeof(init_pkt));
	if (err) {
		return err;
	}
	k_sleep(K_MSEC(10));

	/* Step 2: Send TRANSFER fragments (all except last) */
	for (uint16_t i = 0; i < transfer_count; i++) {
		uint8_t buf[DATA_LEN_MAX];
		large_data_transfer_packet_t *t =
			(large_data_transfer_packet_t *)buf;

		t->packet_type = PACKET_TYPE_LARGE_DATA_TRANSFER;
		t->src_device_id = device_id;
		t->dst_device_id = dst_id;
		t->frag_num = i;
		memcpy(t->payload, &src[(uint32_t)i * LARGE_DATA_FRAG_SIZE],
		       LARGE_DATA_FRAG_SIZE);

		err = tx_with_retry(tx_handle, buf,
				    LARGE_DATA_TRANSFER_PACKET_SIZE +
				    LARGE_DATA_FRAG_SIZE);
		if (err) {
			return err;
		}

		if ((i + 1) % 100 == 0) {
			LOG_INF("Sent %d/%d fragments", i + 1, frag_total);
		}

		k_sleep(K_MSEC(3 + (sys_rand32_get() % 5)));
	}

	/* Step 3: Send END (last fragment) */
	{
		uint8_t buf[DATA_LEN_MAX];
		large_data_end_packet_t *e = (large_data_end_packet_t *)buf;
		uint32_t offset = (uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE;

		e->packet_type = PACKET_TYPE_LARGE_DATA_END;
		e->src_device_id = device_id;
		e->dst_device_id = dst_id;
		e->frag_num = frag_total - 1;
		memcpy(e->payload, &src[offset], last_frag_size);

		err = tx_with_retry(tx_handle, buf,
				    LARGE_DATA_END_PACKET_SIZE + last_frag_size);
		if (err) {
			return err;
		}
	}

	ALL_INF("All %d fragments sent, waiting for ACK...", frag_total);

	/* Flush rx_queue before listening — stale packets from during TX
	 * would otherwise fill the queue and cause drops */
	{
		struct rx_queue_item flush_item;

		while (rx_queue_get(&flush_item, K_NO_WAIT) == 0) {
		}
	}

	/* Step 4: Wait for ACK/NACK, retransmit on NACK.
	 * Receiver may send multiple NACK packets covering all missing
	 * fragments. We use a bitmap to accumulate them all. */
	uint8_t retx_bitmap[LARGE_DATA_BITMAP_MAX];

	for (int retransmit = 0; retransmit <= RETRANSMIT_MAX; retransmit++) {
		/* Listen in short windows (1s each), up to 10s total.
		 * Break out as soon as we receive an ACK or NACK. */
		bool got_nack = false;
		bool got_ack = false;
		uint8_t ack_status = 0;
		uint16_t total_missing = 0;

		memset(retx_bitmap, 0, sizeof(retx_bitmap));

		for (int poll = 0; poll < 15; poll++) {
			err = receive_ms(rx_handle, 1000);
			if (err) {
				ALL_ERR("Receive for ACK failed, err %d", err);
				return err;
			}
			k_sem_take(&operation_sem, K_FOREVER);

			struct rx_queue_item item;

			while (rx_queue_get(&item, K_NO_WAIT) == 0) {
				if (item.len < 1) {
					continue;
				}

				if (item.data[0] == PACKET_TYPE_LARGE_DATA_ACK &&
				    item.len >= LARGE_DATA_ACK_PACKET_SIZE) {
					const large_data_ack_packet_t *ack =
						(const large_data_ack_packet_t *)
							item.data;
					if (ack->dst_device_id == device_id) {
						got_ack = true;
						ack_status = ack->status;
					}
				}

				if (item.data[0] == PACKET_TYPE_LARGE_DATA_NACK &&
				    item.len >= LARGE_DATA_NACK_PACKET_SIZE) {
					const large_data_nack_packet_t *nack =
						(const large_data_nack_packet_t *)
							item.data;
					if (nack->dst_device_id == device_id) {
						uint8_t cnt = nack->frag_count;

						if (cnt > LARGE_DATA_NACK_MAX_FRAGS) {
							cnt = LARGE_DATA_NACK_MAX_FRAGS;
						}
						for (uint8_t f = 0; f < cnt; f++) {
							uint16_t fn =
								nack->frag_nums[f];
							if (fn < frag_total) {
								retx_bitmap[fn / 8] |=
									(1 << (fn % 8));
								total_missing++;
							}
						}
						got_nack = true;
					}
				}
			}

			/* Break out of poll loop as soon as we got a response */
			if (got_ack || got_nack) {
				break;
			}

			/* After 5s with no response, re-send END to prompt
			 * receiver to re-verify CRC and send ACK/NACK */
			if (poll == 4 || poll == 9) {
				ALL_INF("No response after %ds, re-sending END "
					"to request ACK/NACK",
					poll + 1);
				uint8_t end_buf[DATA_LEN_MAX];
				large_data_end_packet_t *re =
					(large_data_end_packet_t *)end_buf;
				re->packet_type = PACKET_TYPE_LARGE_DATA_END;
				re->src_device_id = device_id;
				re->dst_device_id = dst_id;
				re->frag_num = frag_total - 1;
				memcpy(re->payload,
				       &src[(uint32_t)transfer_count *
					    LARGE_DATA_FRAG_SIZE],
				       last_frag_size);
				tx_with_retry(tx_handle, end_buf,
					      LARGE_DATA_END_PACKET_SIZE +
					      last_frag_size);
			}
		}

		if (got_ack) {
			if (ack_status == STATUS_SUCCESS) {
				ALL_INF("Large data transfer SUCCESS");
				return 0;
			}
			ALL_WRN("Large data transfer FAILED: CRC mismatch");
			return -EIO;
		}

		if (got_nack && retransmit < RETRANSMIT_MAX) {
			ALL_INF("NACK: %d missing frags, retransmitting "
				"(attempt %d/%d)",
				total_missing, retransmit + 1, RETRANSMIT_MAX);

			k_sleep(K_MSEC(1000));

			bool need_end = false;

			for (uint16_t fn = 0; fn < frag_total; fn++) {
				if (!(retx_bitmap[fn / 8] &
				      (1 << (fn % 8)))) {
					continue;
				}

				uint8_t buf[DATA_LEN_MAX];
				uint32_t frag_offset =
					(uint32_t)fn * LARGE_DATA_FRAG_SIZE;

				if (fn == frag_total - 1) {
					large_data_end_packet_t *e =
						(large_data_end_packet_t *)buf;
					e->packet_type =
						PACKET_TYPE_LARGE_DATA_END;
					e->src_device_id = device_id;
					e->dst_device_id = dst_id;
					e->frag_num = fn;
					memcpy(e->payload, &src[frag_offset],
					       last_frag_size);
					err = tx_with_retry(tx_handle, buf,
						LARGE_DATA_END_PACKET_SIZE +
						last_frag_size);
					need_end = true;
				} else {
					large_data_transfer_packet_t *t =
						(large_data_transfer_packet_t *)
							buf;
					t->packet_type =
						PACKET_TYPE_LARGE_DATA_TRANSFER;
					t->src_device_id = device_id;
					t->dst_device_id = dst_id;
					t->frag_num = fn;
					memcpy(t->payload, &src[frag_offset],
					       LARGE_DATA_FRAG_SIZE);
					err = tx_with_retry(tx_handle, buf,
						LARGE_DATA_TRANSFER_PACKET_SIZE
						+ LARGE_DATA_FRAG_SIZE);
				}

				if (err) {
					return err;
				}
				k_sleep(K_MSEC(3 + (sys_rand32_get() % 5)));
			}

			/* Always re-send END to trigger CRC check */
			if (!need_end) {
				uint8_t buf[DATA_LEN_MAX];
				large_data_end_packet_t *e =
					(large_data_end_packet_t *)buf;
				uint32_t end_offset =
					(uint32_t)transfer_count *
					LARGE_DATA_FRAG_SIZE;
				e->packet_type = PACKET_TYPE_LARGE_DATA_END;
				e->src_device_id = device_id;
				e->dst_device_id = dst_id;
				e->frag_num = frag_total - 1;
				memcpy(e->payload, &src[end_offset],
				       last_frag_size);
				err = tx_with_retry(tx_handle, buf,
					LARGE_DATA_END_PACKET_SIZE +
					last_frag_size);
				if (err) {
					return err;
				}
			}

			ALL_INF("Retransmitted %d frags, waiting for ACK...",
				total_missing);
			continue;
		}

		if (!got_nack) {
			ALL_WRN("No ACK/NACK received");
			return -ETIMEDOUT;
		}
	}

	ALL_WRN("Large data transfer failed after %d retransmit attempts",
		RETRANSMIT_MAX);
	return -EIO;
}

/* --- Flash-based sender: reads fragments from external flash --- */

#define EXT_FLASH_NODE DT_NODELABEL(gd25wb256)

static int flash_read_frag(uint32_t flash_offset, uint32_t frag_offset,
			   void *buf, uint16_t len)
{
	const struct device *dev = DEVICE_DT_GET(EXT_FLASH_NODE);

	return flash_read(dev, flash_offset + frag_offset, buf, len);
}

int large_data_send_from_flash(uint32_t tx_handle, uint32_t rx_handle,
			       uint16_t dst_id, uint8_t file_type,
			       uint32_t flash_offset, uint32_t data_len)
{
	if (data_len == 0 || data_len > LARGE_DATA_MAX_SIZE) {
		return -EINVAL;
	}

	uint16_t frag_total = (data_len + LARGE_DATA_FRAG_SIZE - 1) /
			      LARGE_DATA_FRAG_SIZE;
	uint16_t transfer_count = frag_total - 1;
	uint8_t last_frag_size = (uint8_t)(data_len -
			(uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE);

	/* Compute CRC over flash data in chunks */
	uint16_t crc = 0xFFFF;
	uint8_t crc_buf[512];
	uint32_t remaining = data_len;
	uint32_t off = 0;

	while (remaining > 0) {
		uint16_t chunk = (remaining > sizeof(crc_buf)) ?
				 sizeof(crc_buf) : (uint16_t)remaining;
		int err = flash_read_frag(flash_offset, off, crc_buf, chunk);

		if (err) {
			LOG_ERR("Flash CRC read failed at 0x%x", flash_offset + off);
			return err;
		}
		crc = compute_crc16_continue(crc, crc_buf, chunk);
		off += chunk;
		remaining -= chunk;
	}

	int err;

	ALL_INF("Large data send (flash): %d bytes, %d frags, CRC:0x%04x",
		data_len, frag_total, crc);

	/* Step 1: Send INIT */
	large_data_init_packet_t init_pkt = {
		.packet_type = PACKET_TYPE_LARGE_DATA_INIT,
		.src_device_id = device_id,
		.dst_device_id = dst_id,
		.file_type = file_type,
		.total_size = data_len,
		.frag_total = frag_total,
		.last_frag_size = last_frag_size,
		.crc16 = crc,
	};

	err = tx_with_retry(tx_handle, &init_pkt, sizeof(init_pkt));
	if (err) {
		return err;
	}
	k_sleep(K_MSEC(10));

	/* Step 2: Send TRANSFER fragments (all except last) */
	for (uint16_t i = 0; i < transfer_count; i++) {
		uint8_t buf[DATA_LEN_MAX];
		large_data_transfer_packet_t *t =
			(large_data_transfer_packet_t *)buf;

		t->packet_type = PACKET_TYPE_LARGE_DATA_TRANSFER;
		t->src_device_id = device_id;
		t->dst_device_id = dst_id;
		t->frag_num = i;

		err = flash_read_frag(flash_offset,
				      (uint32_t)i * LARGE_DATA_FRAG_SIZE,
				      t->payload, LARGE_DATA_FRAG_SIZE);
		if (err) {
			return err;
		}

		err = tx_with_retry(tx_handle, buf,
				    LARGE_DATA_TRANSFER_PACKET_SIZE +
				    LARGE_DATA_FRAG_SIZE);
		if (err) {
			return err;
		}

		if ((i + 1) % 100 == 0) {
			LOG_INF("Sent %d/%d fragments", i + 1, frag_total);
		}

		k_sleep(K_MSEC(3 + (sys_rand32_get() % 5)));
	}

	/* Step 3: Send END (last fragment) */
	{
		uint8_t buf[DATA_LEN_MAX];
		large_data_end_packet_t *e = (large_data_end_packet_t *)buf;

		e->packet_type = PACKET_TYPE_LARGE_DATA_END;
		e->src_device_id = device_id;
		e->dst_device_id = dst_id;
		e->frag_num = frag_total - 1;

		err = flash_read_frag(flash_offset,
				      (uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE,
				      e->payload, last_frag_size);
		if (err) {
			return err;
		}

		err = tx_with_retry(tx_handle, buf,
				    LARGE_DATA_END_PACKET_SIZE + last_frag_size);
		if (err) {
			return err;
		}
	}

	ALL_INF("All %d fragments sent (flash), waiting for ACK...", frag_total);

	/* Step 4: Wait for ACK/NACK, retransmit on NACK */
	for (int retransmit = 0; retransmit <= RETRANSMIT_MAX; retransmit++) {
		err = receive_ms(rx_handle, 10000);
		if (err) {
			ALL_ERR("Receive for ACK failed, err %d", err);
			return err;
		}
		k_sem_take(&operation_sem, K_FOREVER);

		struct rx_queue_item item;
		bool got_nack = false;
		uint8_t nack_frag_count = 0;
		uint16_t nack_frags[LARGE_DATA_NACK_MAX_FRAGS];

		while (rx_queue_get(&item, K_NO_WAIT) == 0) {
			if (item.len < 1) {
				continue;
			}

			if (item.data[0] == PACKET_TYPE_LARGE_DATA_ACK &&
			    item.len >= LARGE_DATA_ACK_PACKET_SIZE) {
				const large_data_ack_packet_t *ack =
					(const large_data_ack_packet_t *)item.data;
				if (ack->dst_device_id == device_id) {
					if (ack->status == STATUS_SUCCESS) {
						ALL_INF("Large data transfer (flash) SUCCESS");
						return 0;
					}
					ALL_WRN("Large data transfer (flash) FAILED: CRC mismatch");
					return -EIO;
				}
			}

			if (item.data[0] == PACKET_TYPE_LARGE_DATA_NACK &&
			    item.len >= LARGE_DATA_NACK_PACKET_SIZE) {
				const large_data_nack_packet_t *nack =
					(const large_data_nack_packet_t *)item.data;
				if (nack->dst_device_id == device_id) {
					nack_frag_count = nack->frag_count;
					if (nack_frag_count > LARGE_DATA_NACK_MAX_FRAGS) {
						nack_frag_count = LARGE_DATA_NACK_MAX_FRAGS;
					}
					memcpy(nack_frags, nack->frag_nums,
					       nack_frag_count * sizeof(uint16_t));
					got_nack = true;
				}
			}
		}

		if (got_nack && retransmit < RETRANSMIT_MAX) {
			ALL_INF("NACK: %d missing frags, retransmit %d/%d",
				nack_frag_count, retransmit + 1, RETRANSMIT_MAX);

			k_sleep(K_MSEC(1000));

			for (uint8_t f = 0; f < nack_frag_count; f++) {
				uint16_t frag_num = nack_frags[f];

				if (frag_num >= frag_total) {
					continue;
				}

				uint8_t buf[DATA_LEN_MAX];
				uint32_t frag_off = (uint32_t)frag_num * LARGE_DATA_FRAG_SIZE;

				if (frag_num == frag_total - 1) {
					large_data_end_packet_t *e =
						(large_data_end_packet_t *)buf;
					e->packet_type = PACKET_TYPE_LARGE_DATA_END;
					e->src_device_id = device_id;
					e->dst_device_id = dst_id;
					e->frag_num = frag_num;
					flash_read_frag(flash_offset, frag_off,
							e->payload, last_frag_size);
					err = tx_with_retry(tx_handle, buf,
						    LARGE_DATA_END_PACKET_SIZE +
						    last_frag_size);
				} else {
					large_data_transfer_packet_t *t =
						(large_data_transfer_packet_t *)buf;
					t->packet_type = PACKET_TYPE_LARGE_DATA_TRANSFER;
					t->src_device_id = device_id;
					t->dst_device_id = dst_id;
					t->frag_num = frag_num;
					flash_read_frag(flash_offset, frag_off,
							t->payload, LARGE_DATA_FRAG_SIZE);
					err = tx_with_retry(tx_handle, buf,
						    LARGE_DATA_TRANSFER_PACKET_SIZE +
						    LARGE_DATA_FRAG_SIZE);
				}

				if (err) {
					return err;
				}

				LOG_INF("Retransmitted fragment %d", frag_num);
				k_sleep(K_MSEC(3 + (sys_rand32_get() % 5)));
			}

			/* Re-send END if not in NACK list */
			bool last_in_nack = false;

			for (uint8_t f = 0; f < nack_frag_count; f++) {
				if (nack_frags[f] == frag_total - 1) {
					last_in_nack = true;
					break;
				}
			}
			if (!last_in_nack) {
				uint8_t buf[DATA_LEN_MAX];
				large_data_end_packet_t *e =
					(large_data_end_packet_t *)buf;
				e->packet_type = PACKET_TYPE_LARGE_DATA_END;
				e->src_device_id = device_id;
				e->dst_device_id = dst_id;
				e->frag_num = frag_total - 1;
				flash_read_frag(flash_offset,
						(uint32_t)transfer_count *
						LARGE_DATA_FRAG_SIZE,
						e->payload, last_frag_size);
				err = tx_with_retry(tx_handle, buf,
					    LARGE_DATA_END_PACKET_SIZE +
					    last_frag_size);
				if (err) {
					return err;
				}
			}

			ALL_INF("Retransmission done, waiting for ACK...");
			continue;
		}

		if (!got_nack) {
			ALL_WRN("No ACK/NACK received for large data (flash)");
			return -ETIMEDOUT;
		}
	}

	ALL_WRN("Large data (flash) failed after %d retransmit attempts",
		RETRANSMIT_MAX);
	return -EIO;
}
