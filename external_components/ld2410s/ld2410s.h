/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef LD2410S_H
#define LD2410S_H

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>

struct data_frame {
	uint8_t detection;
	uint16_t distance;
};

struct ack_frame {
	uint8_t cmd;
	uint8_t success;
	uint8_t data[100];
	uint8_t data_size;
};

struct ld2410s_config {
	uint8_t detection_distance;
	uint8_t disappearance_delay;
	uint8_t status_reporting_frequency;
	uint8_t distance_reporting_frequency;
	uint8_t response_speed;
};

void ld2410s_init();
void ld2410s_get_frame(struct data_frame *out_frame);
void ld2410s_set_detection_distance(uint8_t distance);
void ld2410s_set_disappearance_delay(uint8_t delay);
void ld2410s_set_automatic_threshold(void (*cb)(void));
void ld2410s_register_config_update_cb(void (*cb)(const struct ld2410s_config *));
bool ld2410s_get_thresholds(uint32_t *out_thresholds);
bool ld2410s_set_thresholds(const uint32_t *thresholds);
void ld2410s_process_uart_data();
void ld2410s_set_general_config();
bool ld2410s_get_ack(struct ack_frame *out_ack);

// Individual command functions for async request-response pattern
void ld2410s_send_open_cmd();
void ld2410s_send_general_config_cmd();
void ld2410s_send_detection_distance_cmd();
void ld2410s_send_disappearance_delay_cmd();
void ld2410s_send_data_format_cmd();
void ld2410s_send_read_config_cmd();
void ld2410s_send_get_thresholds_cmd();
void ld2410s_send_set_thresholds_cmd();
void ld2410s_send_close_cmd();

// UART power management
void ld2410s_uart_suspend();
void ld2410s_uart_resume();

#ifdef __cplusplus
}
#endif

#endif  // LD2410S_H