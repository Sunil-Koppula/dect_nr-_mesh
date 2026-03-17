/*
 * DFR0997 UART LCD Display Driver
 *
 * 320x240 IPS TFT with onboard ESP32-S3 running LVGL.
 * UART packet: 0x55 0xAA <len> <cmd> <payload...>
 *   where len = total_packet_length - 3
 * Default baud: 9600, 8N1
 *
 * Reference: DFRobot_LcdDisplay library source
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include "display.h"
#include "packet.h"

LOG_MODULE_REGISTER(display, LOG_LEVEL_INF);

/* Protocol header */
#define HDR_H 0x55
#define HDR_L 0xAA

/* Command IDs */
#define CMD_DRAW_TEXT     0x18
#define CMD_SET_BG_COLOR  0x19
#define CMD_CLEAN_SCREEN  0x1D

/* Object ID counter (each drawn object gets a unique ID) */
static uint8_t next_obj_id = 1;

static const struct device *uart_dev;
static bool display_ready;

/* Scrolling log buffer */
#define DISPLAY_LOG_MAX_LINES 10
#define DISPLAY_LOG_LINE_LEN  35

struct log_entry {
	char text[DISPLAY_LOG_LINE_LEN];
	uint8_t r, g, b;
};

static struct log_entry log_buffer[DISPLAY_LOG_MAX_LINES];
static int log_count;

/* Cached header info for redraw */
static device_type_t header_type;
static uint16_t header_device_id;
static bool header_saved;

static void uart_send(const uint8_t *data, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		uart_poll_out(uart_dev, data[i]);
	}
}

int display_init(void)
{
	uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart1));
	if (!device_is_ready(uart_dev)) {
		LOG_ERR("UART1 device not ready");
		return -ENODEV;
	}

	/* Give the ESP32-S3 on the display time to boot */
	k_sleep(K_MSEC(2000));

	display_clear();
	display_set_bg(0, 0, 0);
	display_ready = true;

	LOG_INF("DFR0997 display initialized on UART1");
	return 0;
}

void display_clear(void)
{
	/* cleanScreen: 55 AA 01 1D */
	uint8_t pkt[] = { HDR_H, HDR_L, 0x01, CMD_CLEAN_SCREEN };

	uart_send(pkt, sizeof(pkt));
	k_sleep(K_MSEC(150));
	next_obj_id = 1;
}

void display_set_bg(uint8_t r, uint8_t g, uint8_t b)
{
	/* setBackgroundColor: 55 AA 04 19 RR GG BB */
	uint8_t pkt[] = { HDR_H, HDR_L, 0x04, CMD_SET_BG_COLOR, r, g, b };

	uart_send(pkt, sizeof(pkt));
	k_sleep(K_MSEC(25));
}

void display_draw_string(uint16_t x, uint16_t y, const char *text,
			 uint8_t font_size, uint8_t r, uint8_t g, uint8_t b)
{
	size_t text_len = strlen(text);

	if (text_len > 110) {
		text_len = 110;
	}

	/* Total packet = 13 + text_len bytes
	 * len byte = total - 3 = 10 + text_len
	 * Layout: [55] [AA] [len] [18] [id] [fontSize] [R] [G] [B]
	 *         [x_hi] [x_lo] [y_hi] [y_lo] [text...]
	 */
	uint8_t pkt[128];
	uint8_t id = next_obj_id++;

	pkt[0] = HDR_H;
	pkt[1] = HDR_L;
	pkt[2] = (uint8_t)(10 + text_len);   /* length = total - 3 */
	pkt[3] = CMD_DRAW_TEXT;
	pkt[4] = id;
	pkt[5] = font_size;
	pkt[6] = r;
	pkt[7] = g;
	pkt[8] = b;
	pkt[9] = (x >> 8) & 0xFF;
	pkt[10] = x & 0xFF;
	pkt[11] = (y >> 8) & 0xFF;
	pkt[12] = y & 0xFF;
	memcpy(&pkt[13], text, text_len);

	uart_send(pkt, 13 + text_len);
	k_sleep(K_MSEC(50));
}

static void display_redraw_header(void)
{
	char line[10];
	snprintf(line, sizeof(line), "ID: %d", header_device_id);

	switch (header_type)
	{
	case DEVICE_TYPE_GATEWAY:
		display_draw_string(60, 0, "GATEWAY", 15, 0, 255, 255);
		break;
	case DEVICE_TYPE_ANCHOR:
		display_draw_string(60, 0, "ANCHOR", 15, 0, 255, 255);
		break;
	case DEVICE_TYPE_SENSOR:
		display_draw_string(60, 0, "SENSOR", 15, 0, 255, 255);
		break;
	default:
		display_draw_string(60, 0, "UNKNOWN", 15, 0, 255, 255);
		break;
	}
	display_draw_string(160, 0, line, 15, 0, 255, 255);
	display_draw_string(0, 10, "----------------------------------------------------------------", 15, 255, 255, 255);
}

static void display_log_redraw(void)
{
	if (log_count >= DISPLAY_LOG_MAX_LINES) {
		display_clear();
		display_set_bg(0, 0, 0);

		if (header_saved) {
			display_redraw_header();
		}

		for (int i = 0; i < log_count; i++) {
			display_draw_string(0, 30 + i * 20,
						log_buffer[i].text, 15,
						log_buffer[i].r,
						log_buffer[i].g,
						log_buffer[i].b);
		}
	} else {
		/* Just draw the new line */
		int i = log_count - 1;
		display_draw_string(0, 30 + i * 20,
					log_buffer[i].text, 15,
					log_buffer[i].r,
					log_buffer[i].g,
					log_buffer[i].b);
	}
}

#define DISPLAY_LOG_WRAP_LEN 35

static void display_log_push_line(const char *text, size_t len,
				  uint8_t r, uint8_t g, uint8_t b)
{
	if (log_count >= DISPLAY_LOG_MAX_LINES) {
		memmove(&log_buffer[0], &log_buffer[1],
			sizeof(struct log_entry) * (DISPLAY_LOG_MAX_LINES - 1));
		log_count = DISPLAY_LOG_MAX_LINES - 1;
	}

	size_t copy_len = (len < DISPLAY_LOG_LINE_LEN - 1) ? len : (DISPLAY_LOG_LINE_LEN - 1);
	memcpy(log_buffer[log_count].text, text, copy_len);
	log_buffer[log_count].text[copy_len] = '\0';
	log_buffer[log_count].r = r;
	log_buffer[log_count].g = g;
	log_buffer[log_count].b = b;
	log_count++;
}

static void display_log_add(const char *text, uint8_t r, uint8_t g, uint8_t b)
{
	size_t total_len = strlen(text);
	const char *p = text;

	/* First line */
	size_t chunk = (total_len > DISPLAY_LOG_WRAP_LEN) ? DISPLAY_LOG_WRAP_LEN : total_len;
	display_log_push_line(p, chunk, r, g, b);
	p += chunk;
	total_len -= chunk;

	/* Remaining wrapped lines */
	while (total_len > 0) {
		chunk = (total_len > DISPLAY_LOG_WRAP_LEN) ? DISPLAY_LOG_WRAP_LEN : total_len;
		display_log_push_line(p, chunk, r, g, b);
		p += chunk;
		total_len -= chunk;
	}

	display_log_redraw();
}

void display_header(device_type_t type, uint16_t device_id)
{
	if (!display_ready) {
		LOG_WRN("Display not ready, cannot show header");
		return;
	}

	header_type = type;
	header_device_id = device_id;
	header_saved = true;

	display_redraw_header();
}

void DISPLAY_LOG_INF(const char *fmt, ...)
{
	if (!display_ready) {
		return;
	}

	char line[DISPLAY_LOG_LINE_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);

	display_log_add(line, 255, 255, 255);
}

void DISPLAY_LOG_WRN(const char *fmt, ...)
{
	if (!display_ready) {
		return;
	}

	char line[DISPLAY_LOG_LINE_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);

	display_log_add(line, 255, 255, 0);
}

void DISPLAY_LOG_ERR(const char *fmt, ...)
{
	if (!display_ready) {
		return;
	}

	char line[DISPLAY_LOG_LINE_LEN];
	va_list args;
	va_start(args, fmt);
	vsnprintf(line, sizeof(line), fmt, args);
	va_end(args);

	display_log_add(line, 255, 0, 0);
}