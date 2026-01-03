/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <functional>

#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/gpio.h>

class BatteryMonitor
{
public:
	using VoltageChangedCallback = std::function<void(int mvNew)>;

	static BatteryMonitor &Instance()
	{
		static BatteryMonitor batteryMonitor;
		return batteryMonitor;
	};

	int Init(VoltageChangedCallback cb);

private:
	static constexpr uint32_t MonitorIntervalSec = 600;
	static constexpr uint8_t NumVoltageSamples = 10;
	static constexpr uint32_t BatteryVoltageDivider = (1000 + 500) / 500;

	/* ADC for monitoring*/
	int16_t adcVal;
	struct adc_sequence sequence = {
		.buffer = &adcVal,
		.buffer_size = sizeof(adcVal),
	};
	struct adc_dt_spec bat_adc;

	/* GPIOs for charging */
	struct gpio_dt_spec gpioCharging;
	struct gpio_dt_spec gpioReadSink;
	struct gpio_dt_spec gpioChargeSpeed;

	VoltageChangedCallback voltageCb;

	struct k_thread batteryMonitorThread;

	void MonitorThread();
};
