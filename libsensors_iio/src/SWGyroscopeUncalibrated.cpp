/*
 * STMicroelectronics Gyroscope Uncalibrated Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWGyroscopeUncalibrated.h"

SWGyroscopeUncalibrated::SWGyroscopeUncalibrated(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_GYROSCOPE_UNCALIBRATED,
			pipe_data_fd, true, true, true, true)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_GYROSCOPE_UNCALIBRATED;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_GYROSCOPE;
	type_sensor_need_trigger = SENSOR_TYPE_GYROSCOPE;
}

SWGyroscopeUncalibrated::~SWGyroscopeUncalibrated()
{

}

void SWGyroscopeUncalibrated::TriggerEventReceived()
{
	int data_remaining;
	SensorBaseData gyro_data;

	do {
		data_remaining = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &gyro_data);
		if (data_remaining < 0)
			return;

		memcpy(sensor_event.uncalibrated_gyro.uncalib, gyro_data.raw, num_data_axis * sizeof(float));
		memcpy(sensor_event.uncalibrated_gyro.bias, gyro_data.offset, num_data_axis * sizeof(float));
		sensor_event.timestamp = gyro_data.timestamp;

		SWSensorBaseWithPollrate::WriteDataToPipe();
		SWSensorBaseWithPollrate::ProcessData(&gyro_data);
	} while (data_remaining > 0);
}
