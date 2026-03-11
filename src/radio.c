/*
 * PHY radio operations and callbacks for DECT NR+ mesh network
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_dect_phy.h>
#include "radio.h"
#include "queue.h"
#include "state.h"
#include "large_data.h"

LOG_MODULE_DECLARE(app);

K_SEM_DEFINE(operation_sem, 0, 1);
K_SEM_DEFINE(deinit_sem, 0, 1);

bool exit_flag;

struct nrf_modem_dect_phy_config_params dect_phy_config_params = {
	.band_group_index = ((CONFIG_CARRIER >= 525 && CONFIG_CARRIER <= 551)) ? 1 : 0,
	.harq_rx_process_count = 4,
	.harq_rx_expiry_time_us = 5000000,
};

/* ===== PHY Callbacks ===== */

static void on_init(const struct nrf_modem_dect_phy_init_event *evt)
{
	if (evt->err) {
		LOG_ERR("Init failed, err %d", evt->err);
		exit_flag = true;
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_deinit(const struct nrf_modem_dect_phy_deinit_event *evt)
{
	if (evt->err) {
		LOG_ERR("Deinit failed, err %d", evt->err);
		return;
	}
	k_sem_give(&deinit_sem);
}

static void on_activate(const struct nrf_modem_dect_phy_activate_event *evt)
{
	if (evt->err) {
		LOG_ERR("Activate failed, err %d", evt->err);
		exit_flag = true;
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_deactivate(const struct nrf_modem_dect_phy_deactivate_event *evt)
{
	if (evt->err) {
		LOG_ERR("Deactivate failed, err %d", evt->err);
		return;
	}
	k_sem_give(&deinit_sem);
}

static void on_configure(const struct nrf_modem_dect_phy_configure_event *evt)
{
	if (evt->err) {
		LOG_ERR("Configure failed, err %d", evt->err);
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_radio_config(const struct nrf_modem_dect_phy_radio_config_event *evt)
{
	if (evt->err) {
		LOG_ERR("Radio config failed, err %d", evt->err);
		return;
	}
	k_sem_give(&operation_sem);
}

static void on_op_complete(const struct nrf_modem_dect_phy_op_complete_event *evt)
{
	if (evt->err) {
		LOG_WRN("op_complete handle %d err %d", evt->handle, evt->err);
	}
	k_sem_give(&operation_sem);
}

static void on_cancel(const struct nrf_modem_dect_phy_cancel_event *evt)
{
	LOG_DBG("on_cancel status %d", evt->err);
	k_sem_give(&operation_sem);
}

static void on_pcc(const struct nrf_modem_dect_phy_pcc_event *evt)
{
	/* PCC received — nothing to do, PDC follows */
}

static void on_pcc_crc_err(const struct nrf_modem_dect_phy_pcc_crc_failure_event *evt)
{
	LOG_WRN("PCC CRC error");
}

static void on_pdc(const struct nrf_modem_dect_phy_pdc_event *evt)
{
	if (evt->len < 1) {
		return;
	}

	/*
	 * Process all large data packets directly in ISR context:
	 * - INIT: k_heap_alloc(K_NO_WAIT), ISR-safe
	 * - TRANSFER: memcpy into reassembly buffer, ISR-safe
	 * - END: CRC verify + queue pending ACK, signals main thread
	 *        via large_data_end_sem to cancel RX and send ACK
	 */
	uint8_t pkt_type = ((const uint8_t *)evt->data)[0];

	if (pkt_type == PACKET_TYPE_LARGE_DATA_INIT &&
	    evt->len >= LARGE_DATA_INIT_PACKET_SIZE) {
		large_data_handle_init(
			(const large_data_init_packet_t *)evt->data);
		return;
	}

	if (pkt_type == PACKET_TYPE_LARGE_DATA_TRANSFER &&
	    evt->len >= LARGE_DATA_TRANSFER_PACKET_SIZE) {
		large_data_handle_transfer(
			(const large_data_transfer_packet_t *)evt->data,
			evt->len);
		return;
	}

	if (pkt_type == PACKET_TYPE_LARGE_DATA_END &&
	    evt->len >= LARGE_DATA_END_PACKET_SIZE) {
		large_data_handle_end(
			(const large_data_end_packet_t *)evt->data,
			evt->len);
		return;
	}

	rx_queue_put(evt->data, evt->len, evt->rssi_2);
}

static void on_pdc_crc_err(const struct nrf_modem_dect_phy_pdc_crc_failure_event *evt)
{
	LOG_WRN("PDC CRC error");
}

static void on_rssi(const struct nrf_modem_dect_phy_rssi_event *evt)
{
	/* unused */
}

void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt)
{
	switch (evt->id) {
	case NRF_MODEM_DECT_PHY_EVT_INIT:
		on_init(&evt->init);
		break;
	case NRF_MODEM_DECT_PHY_EVT_DEINIT:
		on_deinit(&evt->deinit);
		break;
	case NRF_MODEM_DECT_PHY_EVT_ACTIVATE:
		on_activate(&evt->activate);
		break;
	case NRF_MODEM_DECT_PHY_EVT_DEACTIVATE:
		on_deactivate(&evt->deactivate);
		break;
	case NRF_MODEM_DECT_PHY_EVT_CONFIGURE:
		on_configure(&evt->configure);
		break;
	case NRF_MODEM_DECT_PHY_EVT_RADIO_CONFIG:
		on_radio_config(&evt->radio_config);
		break;
	case NRF_MODEM_DECT_PHY_EVT_COMPLETED:
		on_op_complete(&evt->op_complete);
		break;
	case NRF_MODEM_DECT_PHY_EVT_CANCELED:
		on_cancel(&evt->cancel);
		break;
	case NRF_MODEM_DECT_PHY_EVT_RSSI:
		on_rssi(&evt->rssi);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC:
		on_pcc(&evt->pcc);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PCC_ERROR:
		on_pcc_crc_err(&evt->pcc_crc_err);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC:
		on_pdc(&evt->pdc);
		break;
	case NRF_MODEM_DECT_PHY_EVT_PDC_ERROR:
		on_pdc_crc_err(&evt->pdc_crc_err);
		break;
	default:
		break;
	}
}

/* ===== TX/RX Functions ===== */

int transmit(uint32_t handle, void *data, size_t data_len)
{
	struct phy_ctrl_field_common header = {
		.header_format = 0x0,
		.packet_length_type = 0x0,
		.packet_length = 0x01,
		.short_network_id = (CONFIG_NETWORK_ID & 0xff),
		.transmitter_id_hi = (device_id >> 8),
		.transmitter_id_lo = (device_id & 0xff),
		.transmit_power = CONFIG_TX_POWER,
		.reserved = 0,
		.df_mcs = CONFIG_MCS,
	};

	struct nrf_modem_dect_phy_tx_params tx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.phy_type = 0,
		.lbt_rssi_threshold_max = -80,
		.carrier = CONFIG_CARRIER,
		.lbt_period = NRF_MODEM_DECT_LBT_PERIOD_MIN,
		.phy_header = (union nrf_modem_dect_phy_hdr *)&header,
		.data = data,
		.data_size = data_len,
	};

	return nrf_modem_dect_phy_tx(&tx_op_params);
}

int receive(uint32_t handle)
{
	struct nrf_modem_dect_phy_rx_params rx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
		.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
		.rssi_level = -60,
		.carrier = CONFIG_CARRIER,
		.duration = CONFIG_RX_PERIOD_S * MSEC_PER_SEC *
			    NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
		.filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
		.filter.is_short_network_id_used = 1,
		.filter.receiver_identity = 0,
	};

	return nrf_modem_dect_phy_rx(&rx_op_params);
}

int receive_ms(uint32_t handle, uint32_t duration_ms)
{
	struct nrf_modem_dect_phy_rx_params rx_op_params = {
		.start_time = 0,
		.handle = handle,
		.network_id = CONFIG_NETWORK_ID,
		.mode = NRF_MODEM_DECT_PHY_RX_MODE_CONTINUOUS,
		.rssi_interval = NRF_MODEM_DECT_PHY_RSSI_INTERVAL_OFF,
		.link_id = NRF_MODEM_DECT_PHY_LINK_UNSPECIFIED,
		.rssi_level = -60,
		.carrier = CONFIG_CARRIER,
		.duration = duration_ms * NRF_MODEM_DECT_MODEM_TIME_TICK_RATE_KHZ,
		.filter.short_network_id = CONFIG_NETWORK_ID & 0xff,
		.filter.is_short_network_id_used = 1,
		.filter.receiver_identity = 0,
	};

	return nrf_modem_dect_phy_rx(&rx_op_params);
}
