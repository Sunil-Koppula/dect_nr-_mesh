/*
 * Large data transfer protocol for DECT NR+ mesh network
 *
 * INIT and TRANSFER packets are processed directly in the PHY ISR callback
 * (on_pdc) to avoid RX queue overflow. Only END packets go through the queue
 * since they need to TX an ACK from thread context.
 */

#include <string.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "large_data.h"
#include "mesh.h"
#include "radio.h"
#include "queue.h"
#include "state.h"

LOG_MODULE_DECLARE(app);

/* ===== Receiver: reassembly sessions ===== */

/*
 * Dedicated heap for reassembly buffers.
 * Using k_heap_alloc with K_NO_WAIT makes it safe to call from ISR context
 * (on_pdc callback), unlike k_malloc which uses K_FOREVER.
 */
K_HEAP_DEFINE(large_data_heap, LARGE_DATA_MAX_SIZE + 256);

static large_data_rx_session_t sessions[LARGE_DATA_MAX_SESSIONS];

/* Pending responses from ISR-processed END packets (ACK or NACK) */
K_SEM_DEFINE(large_data_end_sem, 0, LARGE_DATA_MAX_SESSIONS);
static struct {
	uint8_t data[DATA_LEN_MAX];   /* ACK or NACK packet */
	uint16_t len;
	large_data_rx_session_t *session;
	bool free_session;             /* false = keep session for retransmit */
	volatile bool valid;
} pending_responses[LARGE_DATA_MAX_SESSIONS];

void large_data_init(void)
{
	memset(sessions, 0, sizeof(sessions));
	memset(pending_responses, 0, sizeof(pending_responses));
}

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

static large_data_rx_session_t *alloc_session(void)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (sessions[i].state == LARGE_DATA_STATE_IDLE) {
			return &sessions[i];
		}
	}
	return NULL;
}

static void free_session(large_data_rx_session_t *s)
{
	if (s->buffer) {
		k_heap_free(&large_data_heap, s->buffer);
		s->buffer = NULL;
	}
	if (s->frag_bitmap) {
		k_heap_free(&large_data_heap, s->frag_bitmap);
		s->frag_bitmap = NULL;
	}
	s->state = LARGE_DATA_STATE_IDLE;
}

void large_data_handle_init(const large_data_init_packet_t *pkt)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	LOG_INF("Large data INIT from ID:%d type:%d size:%d frags:%d last:%d",
		pkt->src_device_id, pkt->file_type, pkt->total_size,
		pkt->frag_total, pkt->last_frag_size);

	large_data_rx_session_t *s = find_session(pkt->src_device_id);

	if (s) {
		free_session(s);
	} else {
		s = alloc_session();
	}

	if (!s) {
		LOG_ERR("No free reassembly sessions");
		return;
	}

	if (pkt->total_size > LARGE_DATA_MAX_SIZE) {
		LOG_ERR("Data too large (%d > %d)", pkt->total_size,
			LARGE_DATA_MAX_SIZE);
		return;
	}

	s->buffer = k_heap_alloc(&large_data_heap, pkt->total_size, K_NO_WAIT);
	if (!s->buffer) {
		LOG_ERR("Failed to allocate %d bytes for reassembly",
			pkt->total_size);
		return;
	}
	memset(s->buffer, 0, pkt->total_size);

	uint16_t bitmap_size = (pkt->frag_total + 7) / 8;

	s->frag_bitmap = k_heap_alloc(&large_data_heap, bitmap_size, K_NO_WAIT);
	if (!s->frag_bitmap) {
		LOG_ERR("Failed to allocate frag bitmap");
		k_heap_free(&large_data_heap, s->buffer);
		s->buffer = NULL;
		return;
	}
	memset(s->frag_bitmap, 0, bitmap_size);

	s->state = LARGE_DATA_STATE_RECEIVING;
	s->src_device_id = pkt->src_device_id;
	s->file_type = pkt->file_type;
	s->total_size = pkt->total_size;
	s->frag_total = pkt->frag_total;
	s->last_frag_size = pkt->last_frag_size;
	s->crc16 = pkt->crc16;
	s->frags_received = 0;

	LOG_INF("Reassembly started for ID:%d (%d bytes, %d frags)",
		pkt->src_device_id, pkt->total_size, pkt->frag_total);
}

void large_data_handle_transfer(const large_data_transfer_packet_t *pkt,
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
		LOG_WRN("Fragment %d out of range (total:%d)",
			pkt->frag_num, s->frag_total);
		return;
	}

	uint16_t payload_len = len - LARGE_DATA_TRANSFER_PACKET_SIZE;
	uint32_t offset = (uint32_t)pkt->frag_num * LARGE_DATA_FRAG_SIZE;

	if (offset + payload_len > s->total_size) {
		LOG_WRN("Fragment %d would overflow buffer", pkt->frag_num);
		return;
	}

	memcpy(&s->buffer[offset], pkt->payload, payload_len);
	s->frag_bitmap[pkt->frag_num / 8] |= (1 << (pkt->frag_num % 8));
	s->frags_received++;

	if (s->frags_received % 100 == 0) {
		LOG_INF("Received %d/%d fragments from ID:%d",
			s->frags_received, s->frag_total,
			pkt->src_device_id);
	}
}

void large_data_handle_end(const large_data_end_packet_t *pkt, uint16_t len)
{
	if (pkt->dst_device_id != device_id) {
		return;
	}

	large_data_rx_session_t *s = find_session(pkt->src_device_id);

	if (!s || s->state != LARGE_DATA_STATE_RECEIVING) {
		LOG_WRN("No active session for sender ID:%d", pkt->src_device_id);
		return;
	}

	/* Store last fragment payload — use last_frag_size from INIT,
	 * not PHY length which is padded to DATA_LEN_MAX */
	uint16_t payload_len = s->last_frag_size;
	uint32_t offset = (uint32_t)pkt->frag_num * LARGE_DATA_FRAG_SIZE;

	if (offset + payload_len > s->total_size) {
		LOG_WRN("Last fragment would overflow buffer");
		free_session(s);
		return;
	}

	memcpy(&s->buffer[offset], pkt->payload, payload_len);
	s->frag_bitmap[pkt->frag_num / 8] |= (1 << (pkt->frag_num % 8));
	s->frags_received++;

	LOG_INF("Received %d/%d fragments from ID:%d, verifying CRC...",
		s->frags_received, s->frag_total, pkt->src_device_id);

	/* Verify CRC over entire reassembled data */
	uint16_t calc_crc = compute_crc16(s->buffer, s->total_size);

	if (calc_crc == s->crc16) {
		LOG_INF("Large data CRC OK (%d bytes from ID:%d)",
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
				ack->status = LARGE_DATA_ACK_SUCCESS;
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

	LOG_WRN("Large data CRC FAIL (expected:0x%04x got:0x%04x, "
		"missing %d/%d frags)",
		s->crc16, calc_crc, missing_count, s->frag_total);

	for (uint16_t f = 0; f < s->frag_total; f++) {
		if (!(s->frag_bitmap[f / 8] & (1 << (f % 8)))) {
			LOG_WRN("  Missing fragment %d", f);
		}
	}

	/* Scan buffer for corrupted regions */
	uint8_t expected_byte = s->buffer[0];
	bool all_same = true;

	for (uint32_t b = 1; b < s->total_size; b++) {
		if (s->buffer[b] != expected_byte) {
			all_same = false;
			break;
		}
	}
	if (all_same) {
		LOG_WRN("  Buffer is all 0x%02x but CRC mismatch", expected_byte);
	} else {
		/* Find first corrupted byte range */
		for (uint32_t b = 0; b < s->total_size; b++) {
			if (s->buffer[b] != expected_byte) {
				uint16_t frag = b / LARGE_DATA_FRAG_SIZE;

				LOG_WRN("  Corruption at byte %d (frag %d): "
					"expected 0x%02x got 0x%02x",
					b, frag, expected_byte, s->buffer[b]);
				break;
			}
		}
	}

	/* Queue response: NACK if few missing, ACK(CRC_FAIL) if too many */
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (!pending_responses[i].valid) {
			if (missing_count > 0 &&
			    missing_count <= LARGE_DATA_NACK_MAX_FRAGS) {
				/* Send NACK with missing frag list */
				large_data_nack_packet_t *nack =
					(large_data_nack_packet_t *)pending_responses[i].data;
				nack->packet_type = PACKET_TYPE_LARGE_DATA_NACK;
				nack->src_device_id = device_id;
				nack->dst_device_id = pkt->src_device_id;
				nack->frag_count = (uint8_t)missing_count;
				uint8_t idx = 0;

				for (uint16_t f = 0; f < s->frag_total; f++) {
					if (!(s->frag_bitmap[f / 8] & (1 << (f % 8)))) {
						nack->frag_nums[idx++] = f;
					}
				}
				pending_responses[i].len = LARGE_DATA_NACK_PACKET_SIZE +
					missing_count * sizeof(uint16_t);
				pending_responses[i].session = s;
				pending_responses[i].free_session = false; /* keep for retransmit */
				s->state = LARGE_DATA_STATE_RECEIVING; /* allow more frags */
				s->frags_received--; /* undo END frag count — will re-receive END */
			} else {
				/* Too many missing — send CRC_FAIL ACK */
				large_data_ack_packet_t *ack =
					(large_data_ack_packet_t *)pending_responses[i].data;
				ack->packet_type = PACKET_TYPE_LARGE_DATA_ACK;
				ack->src_device_id = device_id;
				ack->dst_device_id = pkt->src_device_id;
				ack->status = LARGE_DATA_ACK_CRC_FAIL;
				pending_responses[i].len = LARGE_DATA_ACK_PACKET_SIZE;
				pending_responses[i].session = s;
				pending_responses[i].free_session = true;
			}
			pending_responses[i].valid = true;
			k_sem_give(&large_data_end_sem);
			return;
		}
	}
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
			LOG_ERR("Failed to send large data response, err %d", err);
		} else {
			k_sem_take(&operation_sem, K_FOREVER);
			if (pkt_type == PACKET_TYPE_LARGE_DATA_ACK) {
				large_data_ack_packet_t *ack =
					(large_data_ack_packet_t *)pending_responses[i].data;
				LOG_INF("Large data ACK sent to ID:%d (status:%s)",
					ack->dst_device_id,
					ack->status == LARGE_DATA_ACK_SUCCESS ?
						"OK" : "CRC_FAIL");
			} else if (pkt_type == PACKET_TYPE_LARGE_DATA_NACK) {
				large_data_nack_packet_t *nack =
					(large_data_nack_packet_t *)pending_responses[i].data;
				LOG_INF("Large data NACK sent to ID:%d (%d missing frags)",
					nack->dst_device_id, nack->frag_count);
			}
		}

		if (pending_responses[i].session) {
			if (pending_responses[i].free_session) {
				free_session(pending_responses[i].session);
				pending_responses[i].session = NULL;
			}
			/* SUCCESS sessions stay in COMPLETE state for relay */
		}

		k_sleep(K_MSEC(10));
	}
}

bool large_data_get_completed(uint8_t **data, uint32_t *size,
			      uint8_t *file_type, uint16_t *src_id)
{
	for (int i = 0; i < LARGE_DATA_MAX_SESSIONS; i++) {
		if (sessions[i].state == LARGE_DATA_STATE_COMPLETE &&
		    sessions[i].buffer != NULL) {
			*data = sessions[i].buffer;
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

int large_data_send(uint32_t tx_handle, uint32_t rx_handle,
		    uint16_t dst_id, uint8_t file_type,
		    const void *data, uint32_t data_len)
{
	if (data_len == 0 || data_len > LARGE_DATA_MAX_SIZE) {
		return -EINVAL;
	}

	const uint8_t *src = data;

	/*
	 * Fragment calculation:
	 * - TRANSFER packets carry exactly LARGE_DATA_FRAG_SIZE bytes
	 * - END packet carries 1..LARGE_DATA_FRAG_SIZE bytes (+ CRC header)
	 * - All fragments except the last use TRANSFER
	 * - The last fragment uses END (which also carries the CRC)
	 *
	 * last_frag_size must be >= 1 (END always carries data).
	 * Use LARGE_DATA_FRAG_SIZE as the max END payload too — even
	 * though END has a larger header, the PHY subslot is the same.
	 * The END payload is simply whatever remains.
	 */
	/*
	 * Fragment calculation: TRANSFER and END have the same payload
	 * capacity (LARGE_DATA_FRAG_SIZE). All fragments except the last
	 * use TRANSFER; the last uses END.
	 */
	uint16_t frag_total = (data_len + LARGE_DATA_FRAG_SIZE - 1) /
			      LARGE_DATA_FRAG_SIZE;
	uint16_t transfer_count = frag_total - 1;
	uint8_t last_frag_size = (uint8_t)(data_len -
			(uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE);

	uint16_t crc = compute_crc16(data, data_len);
	int err;

	LOG_INF("Large data send: %d bytes, %d frags, last:%d, CRC:0x%04x",
		data_len, frag_total, last_frag_size, crc);

	/* Step 1: Send INIT (includes CRC for verification at END) */
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

	/* Helper macro: transmit with retry on op_complete error (e.g. LBT fail) */
#define TX_WITH_RETRY(tx_handle, data, len, label) \
	do { \
		int _retries = 0; \
		while (_retries < 10) { \
			last_op_err = 0; \
			err = transmit(tx_handle, data, len); \
			if (err) { \
				LOG_ERR("Failed to send " label ", err %d", err); \
				return err; \
			} \
			k_sem_take(&operation_sem, K_FOREVER); \
			if (last_op_err == 0) { \
				break; \
			} \
			_retries++; \
			k_sleep(K_MSEC(10 + _retries * 5)); \
		} \
		if (last_op_err != 0) { \
			LOG_ERR(label " failed after retries, err %d", last_op_err); \
			return -EIO; \
		} \
	} while (0)

	TX_WITH_RETRY(tx_handle, &init_pkt, sizeof(init_pkt), "INIT");
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

		TX_WITH_RETRY(tx_handle, buf,
			      LARGE_DATA_TRANSFER_PACKET_SIZE +
			      LARGE_DATA_FRAG_SIZE, "frag");

		if ((i + 1) % 100 == 0) {
			LOG_INF("Sent %d/%d fragments", i + 1, frag_total);
		}

		k_sleep(K_MSEC(5));
	}

	/* Step 3: Send END (last fragment — CRC was sent in INIT) */
	{
		uint8_t buf[DATA_LEN_MAX];
		large_data_end_packet_t *e = (large_data_end_packet_t *)buf;
		uint32_t offset = (uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE;

		e->packet_type = PACKET_TYPE_LARGE_DATA_END;
		e->src_device_id = device_id;
		e->dst_device_id = dst_id;
		e->frag_num = frag_total - 1;
		memcpy(e->payload, &src[offset], last_frag_size);

		TX_WITH_RETRY(tx_handle, buf,
			      LARGE_DATA_END_PACKET_SIZE + last_frag_size, "END");
	}

#define RETRANSMIT_MAX 3

	LOG_INF("All %d fragments sent, waiting for ACK...", frag_total);

	/* Step 4: Wait for ACK/NACK, retransmit on NACK */
	for (int retransmit = 0; retransmit <= RETRANSMIT_MAX; retransmit++) {
		err = receive_ms(rx_handle, 5000);
		if (err) {
			LOG_ERR("Receive for ACK failed, err %d", err);
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
					if (ack->status == LARGE_DATA_ACK_SUCCESS) {
						LOG_INF("Large data transfer SUCCESS");
						return 0;
					}
					LOG_WRN("Large data transfer FAILED: CRC mismatch");
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
			LOG_INF("NACK received: %d missing frags, retransmitting (attempt %d/%d)",
				nack_frag_count, retransmit + 1, RETRANSMIT_MAX);

			k_sleep(K_MSEC(10));

			/* Retransmit requested fragments */
			for (uint8_t f = 0; f < nack_frag_count; f++) {
				uint16_t frag_num = nack_frags[f];

				if (frag_num >= frag_total) {
					continue;
				}

				uint8_t buf[DATA_LEN_MAX];
				uint32_t frag_offset = (uint32_t)frag_num * LARGE_DATA_FRAG_SIZE;

				if (frag_num == frag_total - 1) {
					/* Last fragment — send as END */
					large_data_end_packet_t *e =
						(large_data_end_packet_t *)buf;
					e->packet_type = PACKET_TYPE_LARGE_DATA_END;
					e->src_device_id = device_id;
					e->dst_device_id = dst_id;
					e->frag_num = frag_num;
					memcpy(e->payload, &src[frag_offset],
					       last_frag_size);
					TX_WITH_RETRY(tx_handle, buf,
						      LARGE_DATA_END_PACKET_SIZE +
						      last_frag_size, "retx END");
				} else {
					/* Regular fragment — send as TRANSFER */
					large_data_transfer_packet_t *t =
						(large_data_transfer_packet_t *)buf;
					t->packet_type = PACKET_TYPE_LARGE_DATA_TRANSFER;
					t->src_device_id = device_id;
					t->dst_device_id = dst_id;
					t->frag_num = frag_num;
					memcpy(t->payload, &src[frag_offset],
					       LARGE_DATA_FRAG_SIZE);
					TX_WITH_RETRY(tx_handle, buf,
						      LARGE_DATA_TRANSFER_PACKET_SIZE +
						      LARGE_DATA_FRAG_SIZE, "retx frag");
				}

				LOG_INF("Retransmitted fragment %d", frag_num);
				k_sleep(K_MSEC(5));
			}

			/* If last frag wasn't in the NACK list, send END again
			 * to trigger receiver CRC check */
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
				uint32_t end_offset =
					(uint32_t)transfer_count * LARGE_DATA_FRAG_SIZE;
				e->packet_type = PACKET_TYPE_LARGE_DATA_END;
				e->src_device_id = device_id;
				e->dst_device_id = dst_id;
				e->frag_num = frag_total - 1;
				memcpy(e->payload, &src[end_offset], last_frag_size);
				TX_WITH_RETRY(tx_handle, buf,
					      LARGE_DATA_END_PACKET_SIZE +
					      last_frag_size, "retx END");
			}

			LOG_INF("Retransmission done, waiting for ACK...");
			continue; /* loop back to wait for ACK/NACK */
		}

		if (!got_nack) {
			LOG_WRN("No ACK/NACK received for large data transfer");
			return -ETIMEDOUT;
		}
	}

	LOG_WRN("Large data transfer failed after %d retransmit attempts",
		RETRANSMIT_MAX);
	return -EIO;

#undef TX_WITH_RETRY
#undef RETRANSMIT_MAX
}
