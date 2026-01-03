/*
 * SPDX-License-Identifier: Apache-2.0
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/kernel.h>

struct data_frame {
	uint8_t detection;
	uint16_t distance;
};

struct ld2410s_config {
	uint8_t detection_distance;
	uint8_t disappearance_delay;
	uint8_t status_reporting_frequency;
	uint8_t distance_reporting_frequency;
	uint8_t response_speed;
};

void ld2410s_init(const struct device *_uart);
void ld2410s_get_frame(struct data_frame *out_frame);
void ld2410s_set_detection_distance(uint8_t distance);
void ld2410s_set_disappearance_delay(uint8_t delay);
void ld2410s_set_automatic_threshold(void (*cb)(void));
void ld2410s_register_config_update_cb(void (*cb)(const struct ld2410s_config *));
bool ld2410s_get_thresholds(uint32_t *out_thresholds);
bool ld2410s_set_thresholds(const uint32_t *thresholds);

#ifdef __cplusplus
}
#endif