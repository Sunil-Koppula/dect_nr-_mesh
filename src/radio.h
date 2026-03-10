/*
 * PHY radio operations for DECT NR+ mesh network
 */

#ifndef RADIO_H
#define RADIO_H

#include <stdint.h>
#include <stddef.h>
#include <zephyr/kernel.h>
#include <nrf_modem_dect_phy.h>

/* Header type 1, due to endianness the order is different than in the specification. */
struct phy_ctrl_field_common {
	uint32_t packet_length : 4;
	uint32_t packet_length_type : 1;
	uint32_t header_format : 3;
	uint32_t short_network_id : 8;
	uint32_t transmitter_id_hi : 8;
	uint32_t transmitter_id_lo : 8;
	uint32_t df_mcs : 3;
	uint32_t reserved : 1;
	uint32_t transmit_power : 4;
	uint32_t pad : 24;
};

/* Semaphores for synchronizing modem calls */
extern struct k_sem operation_sem;
extern struct k_sem deinit_sem;

/* Set when PHY init/activate fails */
extern bool exit_flag;

/* PHY event handler */
void dect_phy_event_handler(const struct nrf_modem_dect_phy_event *evt);

/* PHY config parameters */
extern struct nrf_modem_dect_phy_config_params dect_phy_config_params;

/* Transmit data on the PHY */
int transmit(uint32_t handle, void *data, size_t data_len);

/* Start receiving on the PHY */
int receive(uint32_t handle);

#endif /* RADIO_H */
