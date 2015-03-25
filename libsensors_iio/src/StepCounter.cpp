/*
 * STMicroelectronics Step Counter Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "StepCounter.h"

StepCounter::StepCounter(HWSensorBaseCommonData *data, const char *name,
		int handle, unsigned int hw_fifo_len, int pipe_data_fd,
		float power_consumption, bool wakeup) :
			HWSensorBase(data, name, handle,
			SENSOR_TYPE_STEP_COUNTER, hw_fifo_len, pipe_data_fd, power_consumption)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_STEP_COUNTER;
	sensor_t_data.flags = SENSOR_FLAG_ON_CHANGE_MODE;

	if (wakeup)
		sensor_t_data.flags |= SENSOR_FLAG_WAKE_UP;

	sensor_t_data.resolution = 1.0f;
	sensor_t_data.maxRange = pow(2, data->channels[0].bits_used) - 1;

	num_data_axis = SENSOR_BASE_1AXIS;
}

StepCounter::~StepCounter()
{

}

int StepCounter::Enable(int handle, bool enable)
{
	int err;

	err = HWSensorBase::Enable(handle, enable);
	if (err < 0)
		return err;

	if (!GetStatus())
		last_data_timestamp = 0;

	return 0;
}

int StepCounter::SetDelay(int handle, int64_t period_ns, int64_t timeout)
{
	int err;
	int64_t min_pollrate_ns;

	err = SensorBase::SetDelay(handle, period_ns, timeout);
	if (err < 0)
		return err;

	min_pollrate_ns = GetMinPeriod();

	err = write_sysfs_int((char *)FILENAME_MAX_DELIVERY_RATE,
			common_data.iio_sysfs_path, (int)NS_TO_MS(min_pollrate_ns));
	if (err < 0) {
		ALOGE("%s: Failed to write max rate delivery \"%s/%s\".",
				common_data.device_name, common_data.iio_sysfs_path,
				FILENAME_MAX_DELIVERY_RATE);
		return err;
	}

	return 0;
}

void StepCounter::ProcessData(SensorBaseData *data)
{
	sensor_event.u64.step_counter = (uint64_t)data->raw[0];
	sensor_event.timestamp = data->timestamp;

	HWSensorBase::WriteDataToPipe();
	HWSensorBase::ProcessData(data);
}
