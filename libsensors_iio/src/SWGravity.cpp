/*
 * STMicroelectronics SW Gravity Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWGravity.h"

SWGravity::SWGravity(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_GRAVITY,
			pipe_data_fd, true, false, true, false)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_GRAVITY;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	sensor_t_data.maxRange = CONFIG_ST_HAL_ACCEL_RANGE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X;
	type_sensor_need_trigger = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X;
}

SWGravity::~SWGravity()
{

}

void SWGravity::TriggerEventReceived()
{
	int data_remaining;
	SensorBaseData gravity;

	do {
		data_remaining = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &gravity);
		if (data_remaining < 0)
			return;

		memcpy(sensor_event.data, gravity.processed, num_data_axis * sizeof(float));
		sensor_event.timestamp = gravity.timestamp;

		SWSensorBaseWithPollrate::WriteDataToPipe();
		SWSensorBaseWithPollrate::ProcessData(&gravity);
	} while (data_remaining > 0);
}
