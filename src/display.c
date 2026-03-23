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

/* ---------- LCD log queue & worker thread ---------- */
#define DISPLAY_LOG_FMT_LEN      80
#define DISPLAY_LOG_QUEUE_DEPTH  32
#define DISPLAY_LOG_STACK_SIZE   1024
#define DISPLAY_LOG_THREAD_PRIO  10

/* Forward declaration — defined after the log buffer is set up */
static void display_log_add(const char *text, uint8_t r, uint8_t g, uint8_t b);

struct display_log_msg {
	char     text[DISPLAY_LOG_FMT_LEN];
	uint8_t  r, g, b;
};

K_MSGQ_DEFINE(display_log_q, sizeof(struct display_log_msg),
	      DISPLAY_LOG_QUEUE_DEPTH, 4);

static void display_log_thread_fn(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

	struct display_log_msg msg;

	while (true) {
		k_msgq_get(&display_log_q, &msg, K_FOREVER);
		display_log_add(msg.text, msg.r, msg.g, msg.b);
	}
}

K_THREAD_DEFINE(display_log_tid, DISPLAY_LOG_STACK_SIZE,
		display_log_thread_fn, NULL, NULL, NULL,
		DISPLAY_LOG_THREAD_PRIO, 0, 0);

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
static int16_t header_rssi_dbm = 1;  /* sentinel: 1 = no packet received yet */

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
		display_draw_string(10, 0, "GATEWAY", 15, 0, 255, 255);
		break;
	case DEVICE_TYPE_ANCHOR:
		display_draw_string(10, 0, "ANCHOR", 15, 0, 255, 255);
		break;
	case DEVICE_TYPE_SENSOR:
		display_draw_string(10, 0, "SENSOR", 15, 0, 255, 255);
		break;
	default:
		display_draw_string(10, 0, "UNKNOWN", 15, 0, 255, 255);
		break;
	}
	display_draw_string(100, 0, line, 15, 0, 255, 255);

	char rssi_str[16];
	if (header_rssi_dbm == 1) {
		snprintf(rssi_str, sizeof(rssi_str), "RSSI:");
	} else {
		snprintf(rssi_str, sizeof(rssi_str), "RSSI:%ddBm", header_rssi_dbm);
	}
	display_draw_string(180, 0, rssi_str, 15, 0, 255, 0);

	display_draw_string(0, 10, "----------------------------------------------------------------", 15, 255, 255, 255);
}

static bool need_full_redraw;

static void display_log_redraw(int new_lines)
{
	if (need_full_redraw) {
		need_full_redraw = false;
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
		/* Draw only the newly added lines */
		int start = log_count - new_lines;

		for (int i = start; i < log_count; i++) {
			display_draw_string(0, 30 + i * 20,
						log_buffer[i].text, 15,
						log_buffer[i].r,
						log_buffer[i].g,
						log_buffer[i].b);
		}
	}
}

#define DISPLAY_LOG_WRAP_LEN  35

static void display_log_push_line(const char *text, size_t len,
				  uint8_t r, uint8_t g, uint8_t b)
{
	if (log_count >= DISPLAY_LOG_MAX_LINES) {
		memmove(&log_buffer[0], &log_buffer[1],
			sizeof(struct log_entry) * (DISPLAY_LOG_MAX_LINES - 1));
		log_count = DISPLAY_LOG_MAX_LINES - 1;
		need_full_redraw = true;
	}

	size_t copy_len = (len < DISPLAY_LOG_LINE_LEN - 1) ? len : (DISPLAY_LOG_LINE_LEN - 1);
	memcpy(log_buffer[log_count].text, text, copy_len);
	log_buffer[log_count].text[copy_len] = '\0';
	log_buffer[log_count].r = r;
	log_buffer[log_count].g = g;
	log_buffer[log_count].b = b;
	log_count++;
}

static size_t find_wrap_point(const char *text, size_t len)
{
	if (len <= DISPLAY_LOG_WRAP_LEN) {
		return len;
	}

	/* Search backward from wrap limit for a space */
	for (size_t i = DISPLAY_LOG_WRAP_LEN; i > 0; i--) {
		if (text[i] == ' ') {
			return i;
		}
	}

	/* No space found, hard break */
	return DISPLAY_LOG_WRAP_LEN;
}

static void display_log_add(const char *text, uint8_t r, uint8_t g, uint8_t b)
{
	size_t total_len = strlen(text);
	const char *p = text;
	int new_lines = 0;

	while (total_len > 0) {
		size_t chunk = find_wrap_point(p, total_len);
		display_log_push_line(p, chunk, r, g, b);
		new_lines++;
		p += chunk;
		total_len -= chunk;

		/* Skip the space at the break point */
		if (total_len > 0 && *p == ' ') {
			p++;
			total_len--;
		}
	}

	display_log_redraw(new_lines);
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

void display_update_rssi(int16_t rssi_dbm)
{
	if (!display_ready || !header_saved) {
		return;
	}

	header_rssi_dbm = rssi_dbm;

	// char rssi_str[16];
	// snprintf(rssi_str, sizeof(rssi_str), "RSSI:%ddBm", rssi_dbm);
	// display_draw_string(180, 0, rssi_str, 15, 0, 255, 0);
}

static void display_log_enqueue(const char *fmt, va_list args,
				uint8_t r, uint8_t g, uint8_t b)
{
	if (!display_ready) {
		return;
	}

	struct display_log_msg msg = { .r = r, .g = g, .b = b };

	vsnprintf(msg.text, sizeof(msg.text), fmt, args);

	/* Non-blocking: drop the message if the queue is full */
	if (k_msgq_put(&display_log_q, &msg, K_NO_WAIT) != 0) {
		LOG_WRN("display log queue full, message dropped");
	}
}

void DISPLAY_LOG_INF(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	display_log_enqueue(fmt, args, 255, 255, 255);
	va_end(args);
}

void DISPLAY_LOG_WRN(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	display_log_enqueue(fmt, args, 255, 255, 0);
	va_end(args);
}

void DISPLAY_LOG_ERR(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	display_log_enqueue(fmt, args, 255, 0, 0);
	va_end(args);
}