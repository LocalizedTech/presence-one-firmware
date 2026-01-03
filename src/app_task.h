/*
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <platform/CHIPDeviceLayer.h>

class AppTask
{
public:
	static AppTask &Instance()
	{
		static AppTask sAppTask;
		return sAppTask;
	};

	CHIP_ERROR StartApp();

	uint8_t hold_time = 0;

private:
	static constexpr int SensorStartupDelayMs = 4000;
	static constexpr uint8_t DetectionOccupiedMask = 0x2;

	struct k_thread sensorPollingThread;
	const struct device *uart = DEVICE_DT_GET(DT_ALIAS(uart_mmwave));

	CHIP_ERROR InitMatter();
	void UpdateOccupancy(bool occupied);
	void InitMatterAttributes();
	static void ConfigUpdateCallback(const struct ld2410s_config *config);
	int PowerOnSensor();

	void PollingThread();
};
