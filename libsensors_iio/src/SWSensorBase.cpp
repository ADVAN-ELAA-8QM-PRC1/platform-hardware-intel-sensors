/*
 * STMicroelectronics SW Sensor Base Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <string.h>

#include "SWSensorBase.h"

SWSensorBase::SWSensorBase(const char *name, int handle, int sensor_type, int pipe_data_fd,
		bool use_dependency_resolution, bool use_dependency_range, bool use_dependency_delay,
		bool use_dependency_name) : SensorBase(name, handle, sensor_type, pipe_data_fd)
{
	dependency_resolution = use_dependency_resolution;
	dependency_range = use_dependency_range;
	dependency_delay = use_dependency_delay;
	dependency_name = use_dependency_name;

	return;
}

SWSensorBase::~SWSensorBase()
{
	return;
}

int SWSensorBase::AddSensorDependency(SensorBase *p)
{
	struct sensor_t dependecy_data;

	if (sensor_t_data.fifoMaxEventCount == 0)
		sensor_t_data.fifoMaxEventCount = p->GetMaxFifoLenght();
	else {
		if (p->GetMaxFifoLenght() < (int)sensor_t_data.fifoMaxEventCount)
			sensor_t_data.fifoMaxEventCount = p->GetMaxFifoLenght();
	}

	p->FillSensor_tData(&dependecy_data);

	if (dependency_resolution)
		sensor_t_data.resolution = dependecy_data.resolution;

	if (dependency_range)
		sensor_t_data.maxRange = dependecy_data.maxRange;

	if (dependency_delay) {
		if (sensor_t_data.minDelay == 0)
			sensor_t_data.minDelay = dependecy_data.minDelay;
		else {
			if (dependecy_data.minDelay > sensor_t_data.minDelay)
				sensor_t_data.minDelay = dependecy_data.minDelay;
		}

		if (sensor_t_data.maxDelay == 0)
			sensor_t_data.maxDelay = dependecy_data.maxDelay;
		else {
			if (dependecy_data.maxDelay < sensor_t_data.maxDelay)
				sensor_t_data.maxDelay = dependecy_data.maxDelay;
		}
	}

	if (dependency_name) {
		memcpy((char *)sensor_t_data.name, dependecy_data.name, strlen(dependecy_data.name) + 1);
	}

	return SensorBase::AddSensorDependency(p);
}

int SWSensorBase::FlushData()
{
	int err, i;

	if (GetStatus() && (GetMinTimeout() > 0)) {
		for (i = 0; i < (int)dependencies_num; i++) {
			err = dependencies[i]->FlushData();
			if (err < 0)
				return -EINVAL;
		}
	} else
		return -EINVAL;

	return SensorBase::FlushData();
}

void SWSensorBase::ThreadTask()
{
	while (true) {
		pthread_mutex_lock(&mutext.trigger_mutex);
		pthread_cond_wait(&mutext.trigger_data_cond, &mutext.trigger_mutex);
		TriggerEventReceived();
		pthread_mutex_unlock(&mutext.trigger_mutex);
	}
}


SWSensorBaseWithPollrate::SWSensorBaseWithPollrate(const char *name, int handle, int sensor_type, int pipe_data_fd,
		bool use_dependency_resolution, bool use_dependency_range, bool use_dependency_delay,
		bool use_dependency_name) : SWSensorBase(name, handle, sensor_type, pipe_data_fd,
		use_dependency_resolution, use_dependency_range, use_dependency_delay, use_dependency_name)
{

}

SWSensorBaseWithPollrate::~SWSensorBaseWithPollrate()
{

}

int SWSensorBaseWithPollrate::SetDelay(int handle, int64_t period_ns, int64_t timeout)
{
	int i, err;
	int64_t temp_real_pollrate = 0;

	err = SWSensorBase::SetDelay(handle, period_ns, timeout);
	if (err < 0)
		return err;

	for (i = 0; i < (int)dependencies_num; i++) {
		if (temp_real_pollrate < GetMinPeriod())
			temp_real_pollrate = GetMinPeriod();
	}

	return 0;
}

void SWSensorBaseWithPollrate::WriteDataToPipe()
{
	int err;

	if (!GetStatusOfHandle(sensor_t_data.handle))
		return;

	if (sensor_event.timestamp > (last_data_timestamp + GetDelay())) {
		err = write(android_pipe_fd, &sensor_event, sizeof(sensor_event));
		if (err < 0) {
			ALOGE("%s: Failed to write sensor data to pipe.", android_name);
			return;
		}

		last_data_timestamp = sensor_event.timestamp;
	}
}
