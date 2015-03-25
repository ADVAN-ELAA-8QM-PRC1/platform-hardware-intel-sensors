/*
 * STMicroelectronics SW Linear Acceleration Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWLinearAccel.h"

SWLinearAccel::SWLinearAccel(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_LINEAR_ACCELERATION,
			pipe_data_fd, true, false, true, false)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_LINEAR_ACCELERATION;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	sensor_t_data.maxRange = CONFIG_ST_HAL_ACCEL_RANGE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X;
	type_sensor_need_trigger = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X;
}

SWLinearAccel::~SWLinearAccel()
{

}

void SWLinearAccel::TriggerEventReceived()
{
	int data_remaining;
	SensorBaseData linear_accel;

	do {
		data_remaining = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &linear_accel);
		if (data_remaining < 0)
			return;

		memcpy(sensor_event.data, linear_accel.processed, num_data_axis * sizeof(float));
		sensor_event.timestamp = linear_accel.timestamp;

		SWSensorBaseWithPollrate::WriteDataToPipe();
		SWSensorBaseWithPollrate::ProcessData(&linear_accel);
	} while (data_remaining > 0);
}
