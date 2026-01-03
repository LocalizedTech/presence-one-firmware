/*
 * SPDX-License-Identifier: Apache-2.0
 */

#include "app_task.h"
#include "battery_monitor.h"
#include "ld2410s.h"

#include "app/matter_init.h"
#include "app/task_executor.h"
#include "lib/core/CHIPError.h"
#include "lib/support/CodeUtils.h"

#include <setup_payload/OnboardingCodesUtil.h>
#include <app-common/zap-generated/attributes/Accessors.h>

#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

LOG_MODULE_DECLARE(app, CONFIG_CHIP_APP_LOG_LEVEL);

K_THREAD_STACK_DEFINE(sensorPollingStack, 1024);

/* GPIO device for LDO enable */
static const struct gpio_dt_spec ldo_enable = GPIO_DT_SPEC_GET(DT_ALIAS(ldo_enable), gpios);

using namespace ::chip;
using namespace ::chip::app;
using namespace ::chip::DeviceLayer;
using namespace ::Clusters;

/* Matter endpoints */
constexpr EndpointId endpointOccupancy = 1;
constexpr EndpointId endpointDistance = 2;
constexpr EndpointId endpointAutoThreshold = 3;
constexpr EndpointId endpointTriggerOffset = 4;
constexpr EndpointId endpointHoldingOffset = 5;

bool autoThresholdRunning = false;

static void AutoThresholdCompleteCallback()
{
	Nrf::PostTask([]() {
		autoThresholdRunning = false;
		OnOff::Attributes::OnOff::Set(endpointAutoThreshold, false);
		LOG_INF("Auto threshold complete, set OnOff to OFF");
	});
}

chip::Protocols::InteractionModel::Status
emberAfExternalAttributeReadCallback(chip::EndpointId endpoint, chip::ClusterId clusterId,
				     const EmberAfAttributeMetadata *attributeMetadata,
				     uint8_t *buffer, uint16_t maxReadLength)
{
	switch (endpoint) {
	case endpointOccupancy:
		if (clusterId == OccupancySensing::Id &&
		    attributeMetadata->attributeId == OccupancySensing::Attributes::HoldTime::Id) {
			if (AppTask::Instance().hold_time == 0) {
				return chip::Protocols::InteractionModel::Status::Busy;
			}

			buffer[0] = AppTask::Instance().hold_time;
			return chip::Protocols::InteractionModel::Status::Success;
		}
		break;
	}

	return chip::Protocols::InteractionModel::Status::UnsupportedAttribute;
}

chip::Protocols::InteractionModel::Status
emberAfExternalAttributeWriteCallback(chip::EndpointId endpoint, chip::ClusterId clusterId,
				      const EmberAfAttributeMetadata *attributeMetadata,
				      uint8_t *buffer)
{
	/* Ignore all Matter attribute changes if auto threshold is running */
	if (autoThresholdRunning) {
		LOG_INF("Auto threshold is running, ignore attribute write");
		return chip::Protocols::InteractionModel::Status::Busy;
	}

	switch (endpoint) {
	case endpointOccupancy:
		if (clusterId == OccupancySensing::Id &&
		    attributeMetadata->attributeId == OccupancySensing::Attributes::HoldTime::Id) {
			uint8_t delay = buffer[0];

			AppTask::Instance().hold_time = delay;
			Nrf::PostTask([delay]() { ld2410s_set_disappearance_delay(delay); });

			LOG_INF("HoldTime written: %u", delay);
			return chip::Protocols::InteractionModel::Status::Success;
		}
		break;
	}

	return chip::Protocols::InteractionModel::Status::UnsupportedAttribute;
}

chip::Protocols::InteractionModel::Status
MatterPreAttributeChangeCallback(const chip::app::ConcreteAttributePath &attributePath,
				 uint8_t type, uint16_t size, uint8_t *value)
{
	/* Ignore all Matter attribute changes if auto threshold is running */
	if (autoThresholdRunning) {
		LOG_INF("Auto threshold is running, ignore attribute change");
		return chip::Protocols::InteractionModel::Status::Busy;
	}

	switch (attributePath.mEndpointId) {
	case endpointTriggerOffset:
	case endpointHoldingOffset:
		if (attributePath.mClusterId == LevelControl::Id &&
		    attributePath.mAttributeId == LevelControl::Attributes::OnLevel::Id) {
			chip::app::DataModel::Nullable<uint8_t> oldValue;
			int32_t delta = 0;
			int32_t triggerDelta = 0;
			int32_t holdingDelta = 0;

			LevelControl::Attributes::OnLevel::Get(attributePath.mEndpointId, oldValue);

			if (oldValue.IsNull()) {
				delta = *value;
			} else {
				delta = *value - oldValue.Value();
			}

			if (attributePath.mEndpointId == endpointTriggerOffset) {
				triggerDelta = delta;
			} else if (attributePath.mEndpointId == endpointHoldingOffset) {
				holdingDelta = delta;
			}

			/* UART operation could take time, so we post it to the task queue */
			Nrf::PostTask([triggerDelta, holdingDelta]() {
				uint32_t thresholds[16];

				bool success = ld2410s_get_thresholds(thresholds);
				if (!success) {
					LOG_ERR("Failed to get thresholds");
					return;
				}

				for (int i = 0; i < 8; i++) {
					int32_t new_threshold = thresholds[i] + triggerDelta;

					/* TODO: Need to check what the highest threshold value
					 * actually is. Doing it like this also makes us lose the
					 * original value if it gets clamped and then we decrease it
					 * the same amount */
					new_threshold =
						std::clamp(new_threshold, int32_t{0}, int32_t{100});

					LOG_INF("Trigger %d: %u -> %u", i, thresholds[i],
						new_threshold);
					thresholds[i] = new_threshold;
				}

				for (int i = 8; i < 16; i++) {
					int32_t new_threshold = thresholds[i] + holdingDelta;

					new_threshold =
						std::clamp(new_threshold, int32_t{0}, int32_t{100});

					LOG_INF("Holding %d: %u -> %u", i, thresholds[i],
						new_threshold);
					thresholds[i] = new_threshold;
				}

				success = ld2410s_set_thresholds(thresholds);
				if (!success) {
					LOG_ERR("Failed to set thresholds");
				}
			});
		}
		break;
	}

	return chip::Protocols::InteractionModel::Status::Success;
}

void MatterPostAttributeChangeCallback(const chip::app::ConcreteAttributePath &attributePath,
				       uint8_t type, uint16_t size, uint8_t *value)
{
	switch (attributePath.mEndpointId) {
	case endpointDistance:
		if (attributePath.mClusterId == LevelControl::Id &&
		    attributePath.mAttributeId == LevelControl::Attributes::OnLevel::Id) {
			uint8_t level = *value;

			Nrf::PostTask([level]() { ld2410s_set_detection_distance(level); });
			LOG_INF("Distance changed: %u", level);
		}
		break;
	case endpointAutoThreshold:
		if (attributePath.mClusterId == OnOff::Id &&
		    attributePath.mAttributeId == OnOff::Attributes::OnOff::Id) {
			bool on = *value;

			LOG_INF("AutoThreshold changed: %s", on ? "On" : "Off");

			if (!on) {
				return;
			}

			/* Reset offsets to 100, can be done immediately since we're in Matter
			 * context */
			LevelControl::Attributes::OnLevel::Set(endpointTriggerOffset, 100);
			LevelControl::Attributes::OnLevel::Set(endpointHoldingOffset, 100);

			/* Schedule automatic threshold, UART can block so put it in a task */
			Nrf::PostTask([]() {
				autoThresholdRunning = true;
				ld2410s_set_automatic_threshold(AutoThresholdCompleteCallback);
			});
		}
	}
}

void AppTask::UpdateOccupancy(bool occupied)
{
	Nrf::PostTask([occupied]() {
		BitMask<OccupancySensing::OccupancyBitmap> occupancyBitmap;
		Protocols::InteractionModel::Status status;

		occupancyBitmap.Set(OccupancySensing::OccupancyBitmap::kOccupied, occupied);
		status = OccupancySensing::Attributes::Occupancy::Set(endpointOccupancy,
								      occupancyBitmap);

		if (status != Protocols::InteractionModel::Status::Success) {
			LOG_ERR("Setting occupancy cluster failed: %x", to_underlying(status));
			return;
		}

		LOG_INF("Occupancy state updated to: %s", occupied ? "Occupied" : "Unoccupied");
	});
}

void AppTask::InitMatterAttributes()
{
	BitMask<OccupancySensing::OccupancySensorTypeBitmap> occupancySensorTypeBitmap;

	occupancySensorTypeBitmap.Set(OccupancySensing::OccupancySensorTypeBitmap::kPir);
	OccupancySensing::Attributes::OccupancySensorTypeBitmap::Set(endpointOccupancy,
								     occupancySensorTypeBitmap);
	OccupancySensing::Attributes::OccupancySensorType::Set(
		endpointOccupancy, OccupancySensing::OccupancySensorTypeEnum::kPir);

	DeviceLayer::AttributeList<UserLabel::Structs::LabelStruct::Type,
				   DeviceLayer::kMaxUserLabelListLength>
		detectionDistanceList;
	detectionDistanceList.add(UserLabel::Structs::LabelStruct::Type{
		CharSpan::fromCharString("Label"), CharSpan::fromCharString("Detection Distance")});

	DeviceLayer::AttributeList<UserLabel::Structs::LabelStruct::Type,
				   DeviceLayer::kMaxUserLabelListLength>
		autoThresholdList;
	autoThresholdList.add(UserLabel::Structs::LabelStruct::Type{
		CharSpan::fromCharString("Label"), CharSpan::fromCharString("Auto Threshold")});

	DeviceLayer::AttributeList<UserLabel::Structs::LabelStruct::Type,
				   DeviceLayer::kMaxUserLabelListLength>
		triggerOffsetList;
	triggerOffsetList.add(UserLabel::Structs::LabelStruct::Type{
		CharSpan::fromCharString("Label"), CharSpan::fromCharString("Trigger Offset")});

	DeviceLayer::AttributeList<UserLabel::Structs::LabelStruct::Type,
				   DeviceLayer::kMaxUserLabelListLength>
		holdingOffsetList;
	holdingOffsetList.add(UserLabel::Structs::LabelStruct::Type{
		CharSpan::fromCharString("Label"), CharSpan::fromCharString("Holding Offset")});

	DeviceLayer::GetDeviceInfoProvider()->SetUserLabelList(endpointDistance,
							       detectionDistanceList);
	DeviceLayer::GetDeviceInfoProvider()->SetUserLabelList(endpointAutoThreshold,
							       autoThresholdList);
	DeviceLayer::GetDeviceInfoProvider()->SetUserLabelList(endpointTriggerOffset,
							       triggerOffsetList);
	DeviceLayer::GetDeviceInfoProvider()->SetUserLabelList(endpointHoldingOffset,
							       holdingOffsetList);
}

void AppTask::PollingThread()
{
	uint16_t distance = 0;
	uint8_t detection = 0;

	LOG_INF("Sensor polling thread started");

	while (true) {
		struct data_frame frame;

		ld2410s_get_frame(&frame);
		if (frame.detection != detection) {
			detection = frame.detection;
			UpdateOccupancy((detection & DetectionOccupiedMask) != 0);
		}

		distance = frame.distance;
		LOG_DBG("Distance: %u", distance);
	}
}

void AppTask::ConfigUpdateCallback(const struct ld2410s_config *config)
{
	uint8_t disappearance_delay = config->disappearance_delay;
	uint8_t detection_distance = config->detection_distance;

	Nrf::PostTask([disappearance_delay, detection_distance]() {
		AppTask::Instance().hold_time = disappearance_delay;
		LevelControl::Attributes::OnLevel::Set(endpointDistance, detection_distance);

		LOG_INF("Matter attributes updated: disappearance_delay=%u, detection_distance=%u",
			disappearance_delay, detection_distance);
	});
}

CHIP_ERROR AppTask::InitMatter()
{
	static DeviceLayer::DeviceInfoProviderImpl defaultProvider;
	uint64_t serial =
		(static_cast<uint64_t>(NRF_FICR->DEVICEID[0]) << 32) | NRF_FICR->DEVICEID[1];

	/* Write serial number into config, this used in the Basic Information cluster */
	DeviceLayer::Internal::ZephyrConfig::WriteConfigValueStr(
		DeviceLayer::Internal::ZephyrConfig::kConfigKey_SerialNum,
		std::to_string(serial).c_str());

	/* Set DeviceInfoProvider, this is needed for setting UserLabel attributes */
	Nrf::Matter::InitData initData = {};
	initData.mDeviceInfoProvider = &defaultProvider;

	ReturnErrorOnFailure(Nrf::Matter::PrepareServer(initData));
	return Nrf::Matter::StartServer();
}

int AppTask::PowerOnSensor()
{
	/* Enable LDO for sensor power */
	if (!gpio_is_ready_dt(&ldo_enable)) {
		LOG_ERR("LDO enable GPIO device not ready");
		return -1;
	}

	int ret = gpio_pin_configure_dt(&ldo_enable, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		LOG_ERR("Failed to configure LDO enable GPIO: %d", ret);
		return -1;
	}

	/* Turn off sensor first, to make sure configuration is cleared */
	ret = gpio_pin_set_dt(&ldo_enable, 0);
	if (ret < 0) {
		LOG_ERR("Failed to set LDO enable GPIO low: %d", ret);
		return -1;
	}

	/* Give LDO time to power off */
	k_msleep(1000);

	ret = gpio_pin_set_dt(&ldo_enable, 1);
	if (ret < 0) {
		LOG_ERR("Failed to set LDO enable GPIO high: %d", ret);
		return -1;
	}

	/* Wait for sensor to start up */
	k_msleep(SensorStartupDelayMs);

	return 0;
}

CHIP_ERROR AppTask::StartApp()
{
	ReturnErrorOnFailure(InitMatter());

	BatteryMonitor::Instance().Init([](int mvNew) {
		Nrf::PostTask([mvNew]() { PowerSource::Attributes::BatVoltage::Set(0, mvNew); });
	});

	InitMatterAttributes();

	PowerOnSensor();

	if (!device_is_ready(uart)) {
		LOG_ERR("UART device not ready");
		return CHIP_ERROR_INCORRECT_STATE;
	}

	ld2410s_register_config_update_cb(ConfigUpdateCallback);
	ld2410s_init(uart);

	k_thread_create(
		&sensorPollingThread, sensorPollingStack, K_THREAD_STACK_SIZEOF(sensorPollingStack),
		[](void *arg1, void *arg2, void *arg3) {
			static_cast<AppTask *>(arg1)->PollingThread();
		},
		this, nullptr, nullptr, K_PRIO_COOP(1), 0, K_NO_WAIT);

	while (true) {
		Nrf::DispatchNextTask();
	}

	return CHIP_NO_ERROR;
}
