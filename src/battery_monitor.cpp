/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "battery_monitor.h"

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/device.h>

LOG_MODULE_REGISTER(battery_monitor, LOG_LEVEL_INF);

K_THREAD_STACK_DEFINE(batteryMonitorStack, 1024);

#if !DT_NODE_EXISTS(DT_NODELABEL(xiao_battery))
#error "Overlay for xiao_battery node not properly defined."
#endif

#define BATTERY_NODE DT_NODELABEL(xiao_battery)

int BatteryMonitor::Init(VoltageChangedCallback cb)
{
	int err;

	voltageCb = cb;

	gpioCharging = GPIO_DT_SPEC_GET(BATTERY_NODE, charging_gpios);
	gpioReadSink = GPIO_DT_SPEC_GET(BATTERY_NODE, read_sink_gpios);
	gpioChargeSpeed = GPIO_DT_SPEC_GET(BATTERY_NODE, charge_speed_gpios);

	if (!gpio_is_ready_dt(&gpioReadSink)) {
		LOG_ERR("read_sink GPIO device not ready");
		return -1;
	}

	if (!gpio_is_ready_dt(&gpioChargeSpeed)) {
		LOG_ERR("charge_speed GPIO not ready");
		return -1;
	}

	if (!gpio_is_ready_dt(&gpioCharging)) {
		LOG_ERR("charging GPIO not ready");
		return -1;
	}

	err = gpio_pin_configure_dt(&gpioReadSink, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure read_sink GPIO: %d", err);
		return -1;
	}

	err = gpio_pin_configure_dt(&gpioChargeSpeed, GPIO_OUTPUT_ACTIVE);
	if (err < 0) {
		LOG_ERR("Failed to configure charge_speed GPIO: %d", err);
		return -1;
	}

	err = gpio_pin_configure_dt(&gpioCharging, GPIO_INPUT);
	if (err < 0) {
		LOG_ERR("Failed to configure charging GPIO: %d", err);
		return -1;
	}

	bat_adc = ADC_DT_SPEC_GET(BATTERY_NODE);

	if (!adc_is_ready_dt(&bat_adc)) {
		LOG_ERR("ADC controller device %s not ready", bat_adc.dev->name);
		return -1;
	}

	err = adc_channel_setup_dt(&bat_adc);
	if (err < 0) {
		LOG_ERR("Could not setup channel #%d (%d)", 0, err);
		return -1;
	}

	err = adc_sequence_init_dt(&bat_adc, &sequence);
	if (err < 0) {
		LOG_ERR("Could not initalize sequnce");
		return -1;
	}

	k_thread_create(
		&batteryMonitorThread, batteryMonitorStack,
		K_THREAD_STACK_SIZEOF(batteryMonitorStack),
		[](void *arg1, void *arg2, void *arg3) {
			static_cast<BatteryMonitor *>(arg1)->MonitorThread();
		},
		this, nullptr, nullptr, K_PRIO_COOP(1), 0, K_NO_WAIT);

	return 0;
}

/*
 * Reads the battery voltage using the ADC and calls the voltage callback
 */
void BatteryMonitor::MonitorThread()
{
	int32_t bat_mv;
	int err;

	while (true) {
		int32_t adc_sum = 0;

		/* Resume ADC */
		err = pm_device_action_run(bat_adc.dev, PM_DEVICE_ACTION_RESUME);
		if (err < 0 && err != -EALREADY) {
			LOG_ERR("Failed to resume ADC: %d", err);
			k_msleep(MonitorIntervalSec * 1000);
			continue;
		}

		/* Wait for ADC to stabilize */
		k_msleep(10);

		for (int i = 0; i < NumVoltageSamples; i++) {
			err = adc_read(bat_adc.dev, &sequence);

			if (err < 0) {
				LOG_ERR("Error when reading ADC: %d", err);
				continue;
			}

			adc_sum += adcVal;

			k_msleep(2);
		}

		/* Suspend ADC */
		err = pm_device_action_run(bat_adc.dev, PM_DEVICE_ACTION_SUSPEND);
		if (err < 0 && err != -EALREADY) {
			LOG_ERR("Failed to suspend ADC: %d", err);
			k_msleep(MonitorIntervalSec * 1000);
			continue;
		}

		bat_mv = adc_sum / NumVoltageSamples;
		err = adc_raw_to_millivolts_dt(&bat_adc, &bat_mv);

		if (err < 0) {
			LOG_WRN("ADC raw to millivolts conversion failed: %d", err);
			bat_mv = 0;
		}

		/* Convert to voltage at battery side */
		bat_mv *= BatteryVoltageDivider;
		LOG_INF("Battery voltage: %d mV", bat_mv);

		if (voltageCb) {
			voltageCb(bat_mv);
		}

		k_msleep(MonitorIntervalSec * 1000);
	}
}
