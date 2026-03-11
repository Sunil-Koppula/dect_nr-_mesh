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

/* Pending ACK from ISR-processed END packet */
K_SEM_DEFINE(large_data_end_sem, 0, 1);
static large_data_ack_packet_t pending_ack;
static volatile bool pending_ack_valid;
static large_data_rx_session_t *pending_ack_session;

void large_data_init(void)
{
	memset(sessions, 0, sizeof(sessions));
	pending_ack_valid = false;
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
	s->frags_received++;

	LOG_INF("Received %d/%d fragments from ID:%d, verifying CRC...",
		s->frags_received, s->frag_total, pkt->src_device_id);

	/* Verify CRC over entire reassembled data */
	uint16_t calc_crc = compute_crc16(s->buffer, s->total_size);
	uint8_t status;

	if (calc_crc == s->crc16) {
		status = LARGE_DATA_ACK_SUCCESS;
		LOG_INF("Large data CRC OK (%d bytes from ID:%d)",
			s->total_size, pkt->src_device_id);
		s->state = LARGE_DATA_STATE_COMPLETE;
	} else {
		status = LARGE_DATA_ACK_CRC_FAIL;
		LOG_WRN("Large data CRC FAIL (expected:0x%04x got:0x%04x, "
			"received %d/%d frags)",
			s->crc16, calc_crc,
			s->frags_received, s->frag_total);
	}

	/* Queue ACK for main thread to send (can't TX from ISR) */
	pending_ack.packet_type = PACKET_TYPE_LARGE_DATA_ACK;
	pending_ack.src_device_id = device_id;
	pending_ack.dst_device_id = pkt->src_device_id;
	pending_ack.status = status;
	pending_ack_session = s;
	pending_ack_valid = true;

	/* Signal main thread to break out of RX and send ACK */
	k_sem_give(&large_data_end_sem);
}

void large_data_send_pending_ack(uint32_t tx_handle)
{
	if (!pending_ack_valid) {
		return;
	}
	pending_ack_valid = false;

	int err = transmit(tx_handle, &pending_ack, sizeof(pending_ack));

	if (err) {
		LOG_ERR("Failed to send large data ACK, err %d", err);
	} else {
		k_sem_take(&operation_sem, K_FOREVER);
		LOG_INF("Large data ACK sent to ID:%d (status:%s)",
			pending_ack.dst_device_id,
			pending_ack.status == LARGE_DATA_ACK_SUCCESS ?
				"OK" : "CRC_FAIL");
	}

	if (pending_ack_session) {
		free_session(pending_ack_session);
		pending_ack_session = NULL;
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

	err = transmit(tx_handle, &init_pkt, sizeof(init_pkt));
	if (err) {
		LOG_ERR("Failed to send INIT, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);
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

		err = transmit(tx_handle, buf,
			       LARGE_DATA_TRANSFER_PACKET_SIZE +
			       LARGE_DATA_FRAG_SIZE);
		if (err) {
			LOG_ERR("Failed to send frag %d, err %d", i, err);
			return err;
		}
		k_sem_take(&operation_sem, K_FOREVER);

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

		err = transmit(tx_handle, buf,
			       LARGE_DATA_END_PACKET_SIZE + last_frag_size);
		if (err) {
			LOG_ERR("Failed to send END, err %d", err);
			return err;
		}
		k_sem_take(&operation_sem, K_FOREVER);
	}

	LOG_INF("All %d fragments sent, waiting for ACK...", frag_total);

	/* Step 4: Wait for ACK */
	err = receive_ms(rx_handle, 5000);
	if (err) {
		LOG_ERR("Receive for ACK failed, err %d", err);
		return err;
	}
	k_sem_take(&operation_sem, K_FOREVER);

	struct rx_queue_item item;

	while (rx_queue_get(&item, K_NO_WAIT) == 0) {
		if (item.len < LARGE_DATA_ACK_PACKET_SIZE) {
			continue;
		}
		if (item.data[0] != PACKET_TYPE_LARGE_DATA_ACK) {
			continue;
		}
		const large_data_ack_packet_t *ack =
			(const large_data_ack_packet_t *)item.data;
		if (ack->dst_device_id == device_id) {
			if (ack->status == LARGE_DATA_ACK_SUCCESS) {
				LOG_INF("Large data transfer SUCCESS");
				return 0;
			} else {
				LOG_WRN("Large data transfer FAILED: CRC mismatch");
				return -EIO;
			}
		}
	}

	LOG_WRN("No ACK received for large data transfer");
	return -ETIMEDOUT;
}
