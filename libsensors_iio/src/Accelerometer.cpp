/*
 * STMicroelectronics Accelerometer Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "Accelerometer.h"

Accelerometer::Accelerometer(HWSensorBaseCommonData *data, const char *name,
		struct iio_sampling_frequency_available *sfa, int handle,
		unsigned int hw_fifo_len, int pipe_data_fd, float power_consumption, bool wakeup) :
			HWSensorBaseWithPollrate(data, name, sfa, handle,
			SENSOR_TYPE_ACCELEROMETER, hw_fifo_len, pipe_data_fd, power_consumption)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_ACCELEROMETER;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	if (wakeup)
		sensor_t_data.flags |= SENSOR_FLAG_WAKE_UP;

	sensor_t_data.resolution = data->channels[0].scale;
	sensor_t_data.maxRange = sensor_t_data.resolution * (pow(2, data->channels[0].bits_used - 1) - 1);
}

Accelerometer::~Accelerometer()
{

}

void Accelerometer::ProcessData(SensorBaseData *data)
{
	float tmp_raw_data[num_data_axis];

	memcpy(tmp_raw_data, data->raw, num_data_axis * sizeof(float));

	data->raw[0] = SENSOR_X_DATA(tmp_raw_data[0], tmp_raw_data[1], tmp_raw_data[2], CONFIG_ST_HAL_ACCEL_ROT_MATRIX);
	data->raw[1] = SENSOR_Y_DATA(tmp_raw_data[0], tmp_raw_data[1], tmp_raw_data[2], CONFIG_ST_HAL_ACCEL_ROT_MATRIX);
	data->raw[2] = SENSOR_Z_DATA(tmp_raw_data[0], tmp_raw_data[1], tmp_raw_data[2], CONFIG_ST_HAL_ACCEL_ROT_MATRIX);

	sensor_event.acceleration.x = data->raw[0];
	sensor_event.acceleration.y = data->raw[1];
	sensor_event.acceleration.z = data->raw[2];
	sensor_event.acceleration.status = SENSOR_STATUS_UNRELIABLE;
	sensor_event.timestamp = data->timestamp;

	HWSensorBaseWithPollrate::WriteDataToPipe();
	HWSensorBaseWithPollrate::ProcessData(data);
}
