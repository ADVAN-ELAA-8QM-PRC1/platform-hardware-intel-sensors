/*
 * STMicroelectronics SignificantMotion Base Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SignificantMotion.h"

SignMotion::SignMotion(HWSensorBaseCommonData *data, const char *name, int handle,
		int pipe_data_fd, float power_consumption) :
			HWSensorBase(data, name, handle,
			SENSOR_TYPE_SIGNIFICANT_MOTION, 0, pipe_data_fd, power_consumption)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_SIGNIFICANT_MOTION;
	sensor_t_data.flags = SENSOR_FLAG_ONE_SHOT_MODE | SENSOR_FLAG_WAKE_UP;
	sensor_t_data.minDelay = -1;
	sensor_t_data.resolution = 1.0f;
	sensor_t_data.maxRange = 1.0f;

	num_data_axis = SENSOR_BASE_0AXIS;
}

SignMotion::~SignMotion()
{

}

int SignMotion::SetDelay(int __attribute__((unused))handle,
				int64_t __attribute__((unused))period_ns,
				int64_t __attribute__((unused))timeout)
{
	return 0;
}

void SignMotion::ProcessEvent(struct iio_event_data *event_data)
{
	sensor_event.data[0] = 1.0f;
	sensor_event.timestamp = event_data->timestamp;

	HWSensorBase::WriteDataToPipe();
	HWSensorBase::ProcessEvent(event_data);
	Enable(sensor_t_data.handle, false);
}
