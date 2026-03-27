/*
 * DFR0997 UART LCD Display Driver
 * 320x240 IPS TFT with ESP32-S3 controller
 * Serial mode: 9600 baud, 8N1
 * Protocol: 0x55 0xAA <len> <cmd> <payload...>
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"

/* Initialize UART1 for the DFR0997 display */
int display_init(void);

/* Clear all objects from the display */
void display_clear(void);

/* Set background color (RGB) */
void display_set_bg(uint8_t r, uint8_t g, uint8_t b);

/* Draw a text string at pixel position (x, y) with font size and color */
void display_draw_string(uint16_t x, uint16_t y, const char *text, uint8_t font_size, uint8_t r, uint8_t g, uint8_t b);

/* Draw device type and ID header (called on startup after PHY init) */
void display_header(device_type_t type, uint16_t device_id);

/* Update the RSSI value shown in the header area */
void display_update_rssi(int16_t rssi_dbm);

/* Log a message on the display */
void DISPLAY_LOG_INF(const char *fmt, ...);
void DISPLAY_LOG_WRN(const char *fmt, ...);
void DISPLAY_LOG_ERR(const char *fmt, ...);

#endif /* DISPLAY_H */
