/*
 * STMicroelectronics SW Game Rotation Vector Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWGameRotationVector.h"

SWGameRotationVector::SWGameRotationVector(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_GAME_ROTATION_VECTOR,
			pipe_data_fd, true, true, true, false)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_GAME_ROTATION_VECTOR;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X;
	type_sensor_need_trigger = SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X;

	num_data_axis = SENSOR_BASE_4AXIS;
}

SWGameRotationVector::~SWGameRotationVector()
{

}

void SWGameRotationVector::TriggerEventReceived()
{
	int data_remaining;
	SensorBaseData game_quaternion;

	do {
		data_remaining = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &game_quaternion);
		if (data_remaining < 0)
			return;

		memcpy(sensor_event.data, game_quaternion.processed, num_data_axis * sizeof(float));
		sensor_event.timestamp = game_quaternion.timestamp;

		SWSensorBaseWithPollrate::WriteDataToPipe();
		SWSensorBaseWithPollrate::ProcessData(&game_quaternion);
	} while (data_remaining > 0);
}
