/*
 * STMicroelectronics SW Rotation Vector Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWRotationVector.h"

SWRotationVector::SWRotationVector(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_ROTATION_VECTOR,
			pipe_data_fd, true, true, true, false)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_ROTATION_VECTOR;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X;
	type_sensor_need_trigger = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X;

	num_data_axis = SENSOR_BASE_4AXIS;
}

SWRotationVector::~SWRotationVector()
{

}

void SWRotationVector::TriggerEventReceived()
{
	int data_remaining;
	SensorBaseData quaternion;

	do {
		data_remaining = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &quaternion);
		if (data_remaining < 0)
			return;

		memcpy(sensor_event.data, quaternion.processed, num_data_axis * sizeof(float));
		sensor_event.timestamp = quaternion.timestamp;

		SWSensorBaseWithPollrate::WriteDataToPipe();
		SWSensorBaseWithPollrate::ProcessData(&quaternion);
	} while (data_remaining > 0);
}
