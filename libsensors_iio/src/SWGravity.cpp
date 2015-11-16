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
#include <stdlib.h>
#include <string.h>

#include "SWGravity.h"

SWGravity::SWGravity(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_GRAVITY,
			pipe_data_fd, true, false, true, false)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_GRAVITY;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X;
	type_sensor_need_trigger = SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X;

	num_data_axis = SENSOR_BASE_4AXIS;
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
