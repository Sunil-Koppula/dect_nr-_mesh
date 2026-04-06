/*
 * ALL_INF / ALL_WRN / ALL_ERR
 *
 * Convenience macros that emit a Zephyr LOG message (using the calling
 * module's registered log context) AND queue the same message to the
 * LCD display via DISPLAY_LOG_*.
 *
 * Usage:
 *   #include "log_all.h"
 *   ALL_INF("Hello %s", "world");
 */

#ifndef LOG_ALL_H
#define LOG_ALL_H

#include <zephyr/logging/log.h>
#include "display.h"

#define ALL_INF(fmt, ...) \
	do { \
		LOG_INF(fmt, ##__VA_ARGS__); \
		DISPLAY_LOG_INF(fmt, ##__VA_ARGS__); \
	} while (0)

#define ALL_WRN(fmt, ...) \
	do { \
		LOG_WRN(fmt, ##__VA_ARGS__); \
		DISPLAY_LOG_WRN(fmt, ##__VA_ARGS__); \
	} while (0)

#define ALL_ERR(fmt, ...) \
	do { \
		LOG_ERR(fmt, ##__VA_ARGS__); \
		DISPLAY_LOG_ERR(fmt, ##__VA_ARGS__); \
	} while (0)

#endif /* LOG_ALL_H */