/*
 * STMicroelectronics Magnetometer Uncalibrated Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWMagnetometerUncalibrated.h"

SWMagnetometerUncalibrated::SWMagnetometerUncalibrated(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED,
			pipe_data_fd, true, true, true, true)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_MAGNETIC_FIELD;
	type_sensor_need_trigger = SENSOR_TYPE_MAGNETIC_FIELD;
}

SWMagnetometerUncalibrated::~SWMagnetometerUncalibrated()
{

}

void SWMagnetometerUncalibrated::TriggerEventReceived()
{
	int data_remaining;
	SensorBaseData magn_data;

	do {
		data_remaining = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &magn_data);
		if (data_remaining < 0)
			return;

		memcpy(sensor_event.uncalibrated_magnetic.uncalib, magn_data.raw, num_data_axis * sizeof(float));
		memcpy(sensor_event.uncalibrated_magnetic.bias, magn_data.offset, num_data_axis * sizeof(float));
		sensor_event.timestamp = magn_data.timestamp;

		SWSensorBaseWithPollrate::WriteDataToPipe();
		SWSensorBaseWithPollrate::ProcessData(&magn_data);
	} while (data_remaining > 0);
}
