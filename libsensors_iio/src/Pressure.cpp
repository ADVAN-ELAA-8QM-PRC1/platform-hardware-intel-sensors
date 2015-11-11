/*
 * STMicroelectronics Pressure Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "Pressure.h"

Pressure::Pressure(HWSensorBaseCommonData *data, const char *name,
		struct iio_sampling_frequency_available *sfa, int handle,
		unsigned int hw_fifo_len, int pipe_data_fd, float power_consumption, bool wakeup) :
			HWSensorBaseWithPollrate(data, name, sfa, handle,
			SENSOR_TYPE_PRESSURE, hw_fifo_len, pipe_data_fd, power_consumption)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_PRESSURE;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	if (wakeup)
		sensor_t_data.flags |= SENSOR_FLAG_WAKE_UP;

	sensor_t_data.resolution = data->channels[0].scale;
	sensor_t_data.maxRange = sensor_t_data.resolution * (pow(2.0, (double)data->channels[0].bits_used) - 1);

	num_data_axis = SENSOR_BASE_1AXIS;
}

Pressure::~Pressure()
{

}

void Pressure::ProcessData(SensorBaseData *data)
{
	sensor_event.pressure = data->raw[0];
	sensor_event.timestamp = data->timestamp;

	HWSensorBaseWithPollrate::WriteDataToPipe();
	HWSensorBaseWithPollrate::ProcessData(data);
}
