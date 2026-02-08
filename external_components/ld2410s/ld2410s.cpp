/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "ld2410s.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

// #include <stdio.h>
// #include <stdlib.h>

#include <zephyr/kernel.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
// #include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/devicetree.h>

#define REPORT_FREQUENCY_HZ 8
#define RESPONSE_SPEED	    0x0A /* 0x05 = Normal, 0x0A = Fast */
#define DETECTION_DISTANCE_TEST_VALUE 15
#define DISAPPEARANCE_DELAY_TEST_VALUE 30
#define DATA_FORMAT_TEST_VALUE 0

static const uint32_t THRESHOLDS_TEST_VALUES[16] = {
	1, 1, 1, 1,
	1, 1, 1, 1,
	1, 1, 1, 1,
	1, 1, 1, 1,
};

static const char *TAG = "ld2410s";

#define LOG_ERR(...) ESP_LOGE(TAG, __VA_ARGS__)
#define LOG_WRN(...) ESP_LOGW(TAG, __VA_ARGS__)
#define LOG_INF(...) ESP_LOGI(TAG, __VA_ARGS__)
#define LOG_DBG(...) ESP_LOGD(TAG, __VA_ARGS__)

static uint8_t minimal_data_header[] = {0x6E};
static uint8_t minimal_data_tail[] = {0x62};

/* UART */
#define UART_MSGQ_SIZE 1000
K_MSGQ_DEFINE(uart_msgq, sizeof(uint8_t), UART_MSGQ_SIZE, 1);

#define UART_THREAD_STACK_SIZE 2048
#define UART_THREAD_PRIORITY   5
#define UART_DELAY_MS	       100

K_THREAD_STACK_DEFINE(ld2410s_process_uart_stack, UART_THREAD_STACK_SIZE);
static struct k_thread uart_thread_data;

static const struct device *uart;
static void (*auto_threshold_complete_cb)(void) = NULL;
static void (*config_update_cb)(const struct ld2410s_config *) = NULL;

K_MSGQ_DEFINE(data_queue, sizeof(struct data_frame), 10, 1);
K_MSGQ_DEFINE(ack_queue, sizeof(struct ack_frame), 10, 1);

static uint8_t header_cmd[] = {0xFD, 0xFC, 0xFB, 0xFA};
static uint8_t tail_cmd[] = {0x04, 0x03, 0x02, 0x01};

static uint8_t header_auto_threshold[] = {0xF4, 0xF3, 0xF2, 0xF1};
static uint8_t tail_auto_threshold[] = {0xF8, 0xF7, 0xF6, 0xF5};

static uint8_t open_cmd[] = {0x04, 0x00, 0xFF, 0x00, 0x01, 0x00};
static uint8_t close_cmd[] = {0x02, 0x00, 0xFE, 0x00};

/* Sensor interrupt */
static struct gpio_dt_spec interrupt_pin = GPIO_DT_SPEC_GET(DT_NODELABEL(interrupt_pin), gpios);
static struct gpio_callback interrupt_callback;

/* Suspend the UART and rely on IRQ from sensor */
static bool suspend_uart = false;

static void uart_resume(void)
{
	int err;

	err = pm_device_action_run(uart, PM_DEVICE_ACTION_RESUME);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("Failed to resume UART: %d", err);
	}
}

static void uart_suspend(void)
{
	int err;

	err = pm_device_action_run(uart, PM_DEVICE_ACTION_SUSPEND);
	if (err < 0 && err != -EALREADY) {
		LOG_ERR("Failed to suspend UART: %d", err);
	}
}

// Public wrappers for UART power management
void ld2410s_uart_suspend(void)
{
	uart_suspend();
}

void ld2410s_uart_resume(void)
{
	uart_resume();
}

static void uart_send_only(const uint8_t *tx, const uint8_t len)
{
	for (int i = 0; i < sizeof(header_cmd); i++) {
		uart_poll_out(uart, header_cmd[i]);
	}

	for (int i = 0; i < len; i++) {
		uart_poll_out(uart, tx[i]);
	}

	for (int i = 0; i < sizeof(tail_cmd); i++) {
		uart_poll_out(uart, tail_cmd[i]);
	}
}

bool ld2410s_get_ack(struct ack_frame *out_ack)
{
	struct ack_frame ack_frame;
	int ret = k_msgq_get(&ack_queue, &ack_frame, K_NO_WAIT);

	if (ret == 0) {
		// Successfully got an ACK from queue
		if (out_ack) {
			*out_ack = ack_frame;
		}
		return true;
	}

	return false;
}

static void uart_send(const uint8_t *tx, const uint8_t len, struct ack_frame *out_ack)
{
	struct ack_frame ack_frame;

	for (int i = 0; i < sizeof(header_cmd); i++) {
		uart_poll_out(uart, header_cmd[i]);
	}

	for (int i = 0; i < len; i++) {
		uart_poll_out(uart, tx[i]);
	}

	for (int i = 0; i < sizeof(tail_cmd); i++) {
		uart_poll_out(uart, tail_cmd[i]);
	}

	LOG_DBG("Expecting ACK for command 0x%02X", tx[2]);
	k_msgq_get(&ack_queue, &ack_frame, K_FOREVER);
	if (ack_frame.cmd != tx[2]) {
		LOG_WRN("ACK mismatch (expected 0x%02X, got 0x%02X), retrying in %dms...", tx[2],
			ack_frame.cmd, UART_DELAY_MS);
		goto retry;
	} else if (!ack_frame.success) {
		LOG_WRN("ACK fail, retrying in %dms...", UART_DELAY_MS);
		goto retry;
	} else {
		LOG_DBG("ACK OK");
		k_msleep(UART_DELAY_MS);
		if (out_ack) {
			*out_ack = ack_frame;
		}
		return;
	}

retry:
	k_msleep(UART_DELAY_MS);
	uart_send(tx, len, out_ack);
}

static void uart_open(void)
{
	if (suspend_uart)
		uart_resume();

	uart_send_only(open_cmd, sizeof(open_cmd));
}

static void uart_close(bool remain_open)
{
	uart_send_only(close_cmd, sizeof(close_cmd));

	if (suspend_uart && !remain_open)
		uart_suspend();
}

static bool read_config(struct ld2410s_config *config)
{
	struct ack_frame ack_frame;

	const uint8_t read_config_cmd[] = {
		0x0C, 0x00, /* Frame length */
		0x71, 0x00, /* Command */
		0x05, 0x00, /* 05: Farthest distance gate */
		0x06, 0x00, /* 06: Disappearance delay */
		0x02, 0x00, /* 02: Frequency of status reporting */
		0x0C, 0x00, /* 0C: Frequency of distance reporting */
		0x0B, 0x00, /* 0B: Response speed */
	};

	uart_send(read_config_cmd, sizeof(read_config_cmd), &ack_frame);

	if (!ack_frame.success) {
		LOG_ERR("Failed to read configuration");
		return false;
	}

	config->detection_distance = ack_frame.data[0];
	config->disappearance_delay = ack_frame.data[4];
	config->status_reporting_frequency = ack_frame.data[8];
	config->distance_reporting_frequency = ack_frame.data[12];
	config->response_speed = ack_frame.data[16];

	if (config_update_cb) {
		config_update_cb(config);
	}

	return true;
}

bool ld2410s_get_thresholds(uint32_t *out_thresholds)
{
	struct ack_frame ack_frame;
	bool ret = false;

	const uint8_t get_thresholds_cmd[] = {
		0x22, 0x00, /* Frame length */
		0x73, 0x00, /* Command */
		0x00, 0x00,
		0x01, 0x00,
		0x02, 0x00,
		0x03, 0x00,
		0x04, 0x00,
		0x05, 0x00,
		0x06, 0x00,
		0x07, 0x00,
		0x08, 0x00,
		0x09, 0x00,
		0x0a, 0x00,
		0x0b, 0x00,
		0x0c, 0x00,
		0x0d, 0x00,
		0x0e, 0x00,
		0x0f, 0x00,
	};

	uart_open();
	uart_send(get_thresholds_cmd, sizeof(get_thresholds_cmd), &ack_frame);

	if (!ack_frame.success) {
		LOG_ERR("Failed to get thresholds");
		goto out;
	}

	if (ack_frame.data_size != 16 * 4) {
		LOG_ERR("Threshold data size mismatch: expected 16, got %u",
			ack_frame.data_size);
		goto out;
	}

	for (int i = 0; i < 8 * 4; i += 4) {
		uint32_t threshold = ack_frame.data[i] | (ack_frame.data[i + 1] << 8) |
				     (ack_frame.data[i + 2] << 16) | (ack_frame.data[i + 3] << 24);
		out_thresholds[i / 4] = threshold;
	}

	for (int i = 8 * 4; i < 16 * 4; i += 4) {
		uint32_t threshold = ack_frame.data[i] | (ack_frame.data[i + 1] << 8) |
				     (ack_frame.data[i + 2] << 16) | (ack_frame.data[i + 3] << 24);
		out_thresholds[i / 4] = threshold;
	}

	ret = true;

out:
	uart_close(false);
	return ret;
}

bool ld2410s_set_thresholds(const uint32_t *thresholds)
{
	struct ack_frame ack_frame;
	bool ret = false;

	uint8_t set_thresholds_cmd[2 + 2 + 16 * 6] = {
		0x62, 0x00, /* Frame length */
		0x72, 0x00, /* Command */
		/* Thresholds, 6 bytes each */
	};

	for (int i = 0; i < 16; i++) {
		set_thresholds_cmd[4 + i * 6 + 0] = i; /* Threshold index */
		set_thresholds_cmd[4 + i * 6 + 1] = 0x00;
		set_thresholds_cmd[4 + i * 6 + 2] = (uint8_t)(thresholds[i] & 0xFF);
		set_thresholds_cmd[4 + i * 6 + 3] = (uint8_t)((thresholds[i] >> 8) & 0xFF);
		set_thresholds_cmd[4 + i * 6 + 4] = (uint8_t)((thresholds[i] >> 16) & 0xFF);
		set_thresholds_cmd[4 + i * 6 + 5] = (uint8_t)((thresholds[i] >> 24) & 0xFF);
	}

	LOG_INF("0x%02X 0x%02X ", set_thresholds_cmd[0], set_thresholds_cmd[1]);
	LOG_INF("0x%02X 0x%02X ", set_thresholds_cmd[2], set_thresholds_cmd[3]);
	for (int i = 4; i < 16 * 6; i += 6) {
		LOG_INF("0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X", set_thresholds_cmd[i], set_thresholds_cmd[i + 1], set_thresholds_cmd[i + 2], set_thresholds_cmd[i + 3], set_thresholds_cmd[i + 4], set_thresholds_cmd[i + 5]);
	}

	uart_open();
	uart_send(set_thresholds_cmd, sizeof(set_thresholds_cmd), &ack_frame);

	if (!ack_frame.success) {
		LOG_ERR("Failed to set thresholds");
		goto out;
	}

	ret = true;

out:
	uart_close(false);
	return ret;
}

void ld2410s_set_detection_distance(uint8_t distance)
{
	struct ld2410s_config config;
	bool success;

	distance = distance > 15 ? 15 : distance;
	distance = distance < 0 ? 0 : distance;

	const uint8_t detection_distance_cmd[] = {
		0x08,	  0x00,		    /* Frame length */
		0x70,	  0x00,		    /* Command */
		0x05,	  0x00,		    /* 05: Farthest distance gate */
		distance, 0x00, 0x00, 0x00, /* Distance */
	};

	uart_open();
	uart_send(detection_distance_cmd, sizeof(detection_distance_cmd), NULL);
	success = read_config(&config);
	uart_close(false);

	if (!success) {
		LOG_ERR("Failed to read configuration after setting detection distance");
		return;
	}

	LOG_INF("Config detection distance: %u", config.detection_distance);
	if (config.detection_distance != distance) {
		LOG_ERR("Detection distance mismatch: expected %u, got %u", distance,
			config.detection_distance);
		return;
	}

	LOG_INF("Set detection distance to %u", distance);
}

void ld2410s_set_disappearance_delay(uint8_t delay)
{
	struct ld2410s_config config;
	bool success;

	delay = delay > 120 ? 120 : delay;
	delay = delay < 10 ? 10 : delay;

	const uint8_t disappearance_delay_cmd[] = {
		0x08,  0x00,		 /* Frame length */
		0x70,  0x00,		 /* Command */
		0x06,  0x00,		 /* 06: Unmanned delay time */
		delay, 0x00, 0x00, 0x00, /* Delay */
	};

	uart_open();
	uart_send(disappearance_delay_cmd, sizeof(disappearance_delay_cmd), NULL);
	success = read_config(&config);
	uart_close(false);

	if (!success) {
		LOG_ERR("Failed to read configuration after setting disappearance delay");
		return;
	}

	if (config.disappearance_delay != delay) {
		LOG_ERR("Disappearance delay mismatch: expected %u, got %u", delay,
			config.disappearance_delay);
		return;
	}

	LOG_INF("Set disappearance delay to %u", delay);
}

// Individual command functions for async request-response pattern
void ld2410s_send_open_cmd()
{
	if (suspend_uart)
		uart_resume();
	uart_send_only(open_cmd, sizeof(open_cmd));
}

void ld2410s_send_general_config_cmd()
{
	uint8_t freq = REPORT_FREQUENCY_HZ * 10;

	const uint8_t general_config_cmd[] = {
		0x14,		0x00,		  /* Frame length */
		0x70,		0x00,		  /* Command */
		0x02,		0x00,		  /* 02: Frequency of status reporting */
		freq,		0x00, 0x00, 0x00, /* Frequency */
		0x0C,		0x00,		  /* 0C: Frequency of distance reporting */
		freq,		0x00, 0x00, 0x00, /* Frequency */
		0x0B,		0x00,		  /* 0B: Response speed */
		RESPONSE_SPEED, 0x00, 0x00, 0x00, /* Normal/Fast */
	};

	uart_send_only(general_config_cmd, sizeof(general_config_cmd));
}

void ld2410s_send_detection_distance_cmd()
{
	uint8_t distance = DETECTION_DISTANCE_TEST_VALUE;

	if (distance > 15) {
		distance = 15;
	}

	const uint8_t detection_distance_cmd[] = {
		0x08,	  0x00,
		0x70,	  0x00,
		0x05,	  0x00,
		distance, 0x00, 0x00, 0x00,
	};

	uart_send_only(detection_distance_cmd, sizeof(detection_distance_cmd));
}

void ld2410s_send_disappearance_delay_cmd()
{
	uint8_t delay = DISAPPEARANCE_DELAY_TEST_VALUE;

	if (delay > 120) {
		delay = 120;
	}
	if (delay < 10) {
		delay = 10;
	}

	const uint8_t disappearance_delay_cmd[] = {
		0x08,  0x00,
		0x70,  0x00,
		0x06,  0x00,
		delay, 0x00, 0x00, 0x00,
	};

	uart_send_only(disappearance_delay_cmd, sizeof(disappearance_delay_cmd));
}

void ld2410s_send_data_format_cmd()
{
	uint8_t format = DATA_FORMAT_TEST_VALUE;

	if (format > 1) {
		format = 1;
	}

	const uint8_t data_format_cmd[] = {
		0x08,   0x00,
		0x7A,   0x00,
		format, 0x00, 0x00, 0x00, 0x00, 0x00,
	};

	uart_send_only(data_format_cmd, sizeof(data_format_cmd));
}

void ld2410s_send_read_config_cmd()
{
	const uint8_t read_config_cmd[] = {
		0x0C, 0x00, /* Frame length */
		0x71, 0x00, /* Command */
		0x05, 0x00, /* 05: Farthest distance gate */
		0x06, 0x00, /* 06: Disappearance delay */
		0x02, 0x00, /* 02: Frequency of status reporting */
		0x0C, 0x00, /* 0C: Frequency of distance reporting */
		0x0B, 0x00, /* 0B: Response speed */
	};

	uart_send_only(read_config_cmd, sizeof(read_config_cmd));
}

void ld2410s_send_get_thresholds_cmd()
{
	const uint8_t get_thresholds_cmd[] = {
		0x22, 0x00,
		0x73, 0x00,
		0x00, 0x00,
		0x01, 0x00,
		0x02, 0x00,
		0x03, 0x00,
		0x04, 0x00,
		0x05, 0x00,
		0x06, 0x00,
		0x07, 0x00,
		0x08, 0x00,
		0x09, 0x00,
		0x0a, 0x00,
		0x0b, 0x00,
		0x0c, 0x00,
		0x0d, 0x00,
		0x0e, 0x00,
		0x0f, 0x00,
	};

	uart_send_only(get_thresholds_cmd, sizeof(get_thresholds_cmd));
}

void ld2410s_send_set_thresholds_cmd()
{
	uint8_t set_thresholds_cmd[2 + 2 + 16 * 6] = {
		0x62, 0x00,
		0x72, 0x00,
	};

	for (int i = 0; i < 16; i++) {
		set_thresholds_cmd[4 + i * 6 + 0] = i;
		set_thresholds_cmd[4 + i * 6 + 1] = 0x00;
		set_thresholds_cmd[4 + i * 6 + 2] = (uint8_t)(THRESHOLDS_TEST_VALUES[i] & 0xFF);
		set_thresholds_cmd[4 + i * 6 + 3] = (uint8_t)((THRESHOLDS_TEST_VALUES[i] >> 8) & 0xFF);
		set_thresholds_cmd[4 + i * 6 + 4] = (uint8_t)((THRESHOLDS_TEST_VALUES[i] >> 16) & 0xFF);
		set_thresholds_cmd[4 + i * 6 + 5] = (uint8_t)((THRESHOLDS_TEST_VALUES[i] >> 24) & 0xFF);
	}

	uart_send_only(set_thresholds_cmd, sizeof(set_thresholds_cmd));
}

void ld2410s_send_close_cmd()
{
	uart_send_only(close_cmd, sizeof(close_cmd));
	if (suspend_uart)
		uart_suspend();
}

// Legacy function - sends all three commands at once (not recommended)
void ld2410s_set_general_config()
{
	uart_open();
	uint8_t freq = REPORT_FREQUENCY_HZ * 10;

	const uint8_t general_config_cmd[] = {
		0x14,		0x00,		  /* Frame length */
		0x70,		0x00,		  /* Command */
		0x02,		0x00,		  /* 02: Frequency of status reporting */
		freq,		0x00, 0x00, 0x00, /* Frequency */
		0x0C,		0x00,		  /* 0C: Frequency of distance reporting */
		freq,		0x00, 0x00, 0x00, /* Frequency */
		0x0B,		0x00,		  /* 0B: Response speed */
		RESPONSE_SPEED, 0x00, 0x00, 0x00, /* Normal/Fast */
	};

	uart_send_only(general_config_cmd, sizeof(general_config_cmd));
	uart_close(false);
}

static void ld2410s_set_general_config_old()
{
	uint8_t freq = REPORT_FREQUENCY_HZ * 10;
	struct ld2410s_config config;
	bool success;

	const uint8_t general_config_cmd[] = {
		0x14,		0x00,		  /* Frame length */
		0x70,		0x00,		  /* Command */
		0x02,		0x00,		  /* 02: Frequency of status reporting */
		freq,		0x00, 0x00, 0x00, /* Frequency */
		0x0C,		0x00,		  /* 0C: Frequency of distance reporting */
		freq,		0x00, 0x00, 0x00, /* Frequency */
		0x0B,		0x00,		  /* 0B: Response speed */
		RESPONSE_SPEED, 0x00, 0x00, 0x00, /* Normal/Fast */
	};

	const uint8_t data_format_cmd[] = {
		0x08, 0x00,			    /* Frame length */
		0x7A, 0x00,			    /* Command */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, /* Minimal output format */
	};

	uart_open();
	uart_send(general_config_cmd, sizeof(general_config_cmd), NULL);
	uart_send(data_format_cmd, sizeof(data_format_cmd), NULL);
	success = read_config(&config);
	uart_close(false);

	if (!success) {
		LOG_ERR("Failed to read configuration after setting general config");
		return;
	}

	if (config.status_reporting_frequency != freq) {
		LOG_ERR("Status reporting frequency mismatch: expected %u, got %u", freq,
			config.status_reporting_frequency);
		return;
	}
	if (config.distance_reporting_frequency != freq) {
		LOG_ERR("Distance reporting frequency mismatch: expected %u, got %u", freq,
			config.distance_reporting_frequency);
		return;
	}
	if (config.response_speed != RESPONSE_SPEED) {
		LOG_ERR("Response speed mismatch: expected 0x%02X, got 0x%02X", RESPONSE_SPEED,
			config.response_speed);
		return;
	}

	LOG_INF("Set general configuration");
}

void ld2410s_set_automatic_threshold(void (*cb)(void))
{
	auto_threshold_complete_cb = cb;
	const uint8_t automatic_threshold_cmd[] = {
		0x08, 0x00, /* Frame length */
		0x09, 0x00, /* Command */
		0x02, 0x00, /* Trigger factor */
		0x01, 0x00, /* Retention factor */
		0x78, 0x00, /* Scanning time (120 seconds) */
	};

	uart_open();
	uart_send(automatic_threshold_cmd, sizeof(automatic_threshold_cmd), NULL);
	uart_close(true);

	LOG_INF("Set automatic threshold");
}

static void parse_frame(const uint8_t *buf)
{
	struct data_frame data_frame;

	data_frame.detection = buf[1];
	data_frame.distance = buf[2] | (buf[3] << 8);

	if (k_msgq_put(&data_queue, &data_frame, K_NO_WAIT) != 0) {
		struct data_frame tmp;

		LOG_WRN("data_queue full, dropping oldest frame");
		k_msgq_get(&data_queue, &tmp, K_NO_WAIT);
		k_msgq_put(&data_queue, &data_frame, K_NO_WAIT);
	}
}

static void parse_ack(const uint8_t *buf, uint8_t size)
{
	struct ack_frame ack_frame;

	ack_frame.cmd = buf[6];
	ack_frame.success = (buf[8] == 0 && buf[9] == 0);
	ack_frame.data_size = (buf[4] | (buf[5] << 8)) - 4; /* -4 to remove cmd and success bytes */

	for (int i = 0; i < ack_frame.data_size && i < sizeof(ack_frame.data); i++) {
		ack_frame.data[i] = buf[10 + i];
	}

	if (k_msgq_put(&ack_queue, &ack_frame, K_NO_WAIT) != 0) {
		struct ack_frame tmp;

		LOG_WRN("ack_queue full, dropping oldest ack");
		k_msgq_get(&ack_queue, &tmp, K_NO_WAIT);
		k_msgq_put(&ack_queue, &ack_frame, K_NO_WAIT);
	}
}

static void parse_auto_threshold(const uint8_t *buf, uint8_t size)
{
	uint16_t progress = buf[7] | (buf[8] << 8);

	LOG_INF("Parsed auto threshold data: progress = %u", progress);

	if (progress == 100 && auto_threshold_complete_cb) {
		auto_threshold_complete_cb();

		/* UART is not suspended during auto threshold, suspend it now if configured */
		if (suspend_uart) {
			uart_suspend();
		}
	}
}

static void uart_cb(const struct device *uart, void *user_data)
{
	uint8_t c;

	while (uart_irq_update(uart) && uart_irq_is_pending(uart)) {
		if (!uart_irq_rx_ready(uart)) {
			LOG_WRN("UART RX not ready, skipping");
			return;
		}

		while (uart_fifo_read(uart, &c, 1) == 1) {
			if (k_msgq_put(&uart_msgq, &c, K_NO_WAIT) != 0) {
				uint8_t tmp;

				LOG_WRN("UART msgq full, dropping oldest byte");
				k_msgq_get(&uart_msgq, &tmp, K_NO_WAIT);
				k_msgq_put(&uart_msgq, &c, K_NO_WAIT);
			}
		}
	}
}

enum {
    WAIT_HEADER,
    WAIT_FRAME_TAIL,
    WAIT_ACK_TAIL,
    WAIT_AUTO_THRESHOLD_TAIL,
} state = WAIT_HEADER;
uint8_t data[UART_MSGQ_SIZE] = {0};
uint8_t idx = 0;

void ld2410s_process_uart_data()
{
    uint8_t byte;
    int ret;

    ret = k_msgq_get(&uart_msgq, &byte, K_NO_WAIT);

    if (ret != 0) {
        return;
    }

    LOG_DBG("0x%02X", byte);

    switch (state) {
    case WAIT_HEADER:
        data[0] = data[1];
        data[1] = data[2];
        data[2] = data[3];
        data[3] = byte;
        if (byte == minimal_data_header[0]) {
            state = WAIT_FRAME_TAIL;
            data[0] = byte;
            idx = 1;
            LOG_DBG("Data header");
        } else if (memcmp(data, header_cmd, sizeof(header_cmd)) == 0) {
            state = WAIT_ACK_TAIL;
            idx = 4;
            LOG_DBG("ACK header");
        } else if (memcmp(data, header_auto_threshold,
                  sizeof(header_auto_threshold)) == 0) {
            state = WAIT_AUTO_THRESHOLD_TAIL;
            idx = 4;
            LOG_DBG("Auto threshold header");
        }
        break;
    case WAIT_FRAME_TAIL:
    case WAIT_ACK_TAIL:
    case WAIT_AUTO_THRESHOLD_TAIL:
        if (idx < UART_MSGQ_SIZE) {
            data[idx++] = byte;
        } else {
            LOG_WRN("UART msgq buffer overflow, resetting state");
            idx = 0;
            state = WAIT_HEADER;
            break;
        }
        if (state == WAIT_FRAME_TAIL) {
            if (idx == 5 && byte == minimal_data_tail[0]) {
                state = WAIT_HEADER;
                data[0] = 0;
                parse_frame(data);
                LOG_DBG("Data tail");
            } else if (idx >= 4 && memcmp(data + idx - 4, header_cmd,
                              sizeof(header_cmd)) == 0) {
                state = WAIT_ACK_TAIL;
                memcpy(data, data + idx - 4, 4);
                idx = 4;
                LOG_WRN("Got ACK header in data frame, switching state");
            } else if (idx >= 4 && memcmp(data + idx - 4, header_auto_threshold,
                              sizeof(header_auto_threshold)) == 0) {
                state = WAIT_AUTO_THRESHOLD_TAIL;
                memcpy(data, data + idx - 4, 4);
                idx = 4;
                LOG_WRN("Got auto threshold header in data frame, "
                    "switching state");
            }
        } else if (state == WAIT_ACK_TAIL &&
               memcmp(data + idx - 4, tail_cmd, sizeof(tail_cmd)) == 0) {
            state = WAIT_HEADER;
            memset(data, 0, 4);
            parse_ack(data, idx);
            LOG_DBG("ACK tail");
        } else if (state == WAIT_AUTO_THRESHOLD_TAIL) {
            if (memcmp(data + idx - 4, tail_auto_threshold,
                   sizeof(tail_auto_threshold)) == 0) {
                state = WAIT_HEADER;
                memset(data, 0, 4);
                parse_auto_threshold(data, idx);
                LOG_DBG("Auto threshold tail");
            } else if (idx >= 4 && memcmp(data + idx - 4, header_cmd,
                              sizeof(header_cmd)) == 0) {
                state = WAIT_ACK_TAIL;
                memcpy(data, data + idx - 4, 4);
                idx = 4;
                LOG_WRN("Got ACK header in auto threshold frame, switching "
                    "state");
            }
        }
        break;
    }
}

void interrupt_handler(const struct device *dev, struct gpio_callback *cb, gpio_port_pins_t pins)
{
	struct data_frame data_frame;

	data_frame.detection = gpio_pin_get_dt(&interrupt_pin) ? 2 : 0;
	data_frame.distance = 999;

	LOG_INF("Sensor interrupt: %d", data_frame.detection);

	if (suspend_uart) {
		k_msgq_put(&data_queue, &data_frame, K_NO_WAIT);
	}
}

int ld2410s_power_on()
{
	const struct gpio_dt_spec ldo_enable = GPIO_DT_SPEC_GET(DT_ALIAS(ldo_enable), gpios);

	/* Enable LDO for sensor power */
	if (!gpio_is_ready_dt(&ldo_enable)) {
		LOG_ERR("LDO enable GPIO device not ready");
		return -1;
	}

	LOG_ERR("Milestone 1");

	int ret = gpio_pin_configure_dt(&ldo_enable, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LDO enable GPIO: %d", ret);
		return -1;
	}

	LOG_ERR("Milestone 2");

	/* Turn off sensor first, to make sure configuration is cleared */
	ret = gpio_pin_set_dt(&ldo_enable, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set LDO enable GPIO low: %d", ret);
		return -1;
	}

	LOG_ERR("Milestone 3");

	/* Give LDO time to power off */
	esphome::delay(1000);

	ret = gpio_pin_set_dt(&ldo_enable, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set LDO enable GPIO high: %d", ret);
		return -1;
	}

	LOG_ERR("Milestone 4");

	/* Wait for sensor to start up */
	esphome::delay(4000);

	LOG_ERR("Milestone 5");

	return 0;
}

void ld2410s_uart_init()
{
	uart = DEVICE_DT_GET(DT_ALIAS(uart_mmwave));

	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return;
	}

	uart_irq_rx_disable(uart);
	uart_irq_tx_disable(uart);
	uart_irq_callback_set(uart, uart_cb);
	uart_irq_rx_enable(uart);
}

void ld2410s_init()
{
	LOG_ERR("Powering on sensor...");

	if (ld2410s_power_on() != 0) {
		LOG_ERR("Failed to power on sensor");
		return;
	}

	LOG_ERR("Sensor powered on");

	ld2410s_uart_init();

	if (!gpio_is_ready_dt(&interrupt_pin)) {
		LOG_ERR("Interrupt GPIO not ready");
		return;
	}

	if (gpio_pin_configure_dt(&interrupt_pin, GPIO_INPUT) != 0) {
		LOG_ERR("Failed to configure interrupt GPIO");
		return;
	}

	if (gpio_pin_interrupt_configure_dt(&interrupt_pin, GPIO_INT_EDGE_BOTH) != 0) {
		LOG_ERR("Failed to configure interrupt GPIO for edge detection");
		return;
	}

	gpio_init_callback(&interrupt_callback, interrupt_handler, BIT(interrupt_pin.pin));
	if (gpio_add_callback(interrupt_pin.port, &interrupt_callback) != 0) {
		LOG_ERR("Failed to add GPIO callback");
		return;
	}

    LOG_ERR("Initialization complete");
}


void ld2410s_get_frame(struct data_frame *out_frame)
{
	k_msgq_get(&data_queue, out_frame, K_FOREVER);
}

void ld2410s_register_config_update_cb(void (*cb)(const struct ld2410s_config *))
{
	config_update_cb = cb;
}
