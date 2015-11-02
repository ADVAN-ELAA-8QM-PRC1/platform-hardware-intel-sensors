/*
 * STMicroelectronics Sensor Base Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <signal.h>

#include "SensorBase.h"

#define ST_SENSOR_BASE_WAIT_US_BEFORE_SEND_FLUSH		(200000)

SensorBase::SensorBase(const char *name, int handle, int type, int pipe_data_fd)
{
	int i;

	if (strlen(name) + 1 > SENSOR_BASE_ANDROID_NAME_MAX) {
		memcpy(android_name, name, SENSOR_BASE_ANDROID_NAME_MAX - 1);
		android_name[SENSOR_BASE_ANDROID_NAME_MAX - 1] = '\0';
	} else
		memcpy(android_name, name, strlen(name) + 1);

	memset(&sensor_t_data, 0, sizeof(sensor_t));
	memset(&sensor_event, 0, sizeof(sensors_event_t));
	memset(type_dependencies, 0, SENSOR_BASE_MAX_DEPENDENCY * sizeof(int));
	memset(sensors_pollrates, 0, ST_HAL_IIO_MAX_DEVICES * sizeof(int64_t));
	memset(last_timestap_pushed, 0, ST_HAL_IIO_MAX_DEVICES * sizeof(int64_t));

	for (i = 0; i < ST_HAL_IIO_MAX_DEVICES; i++)
		sensors_timeout[i] = INT64_MAX;

	sensor_event.version = sizeof(sensors_event_t);
	sensor_event.sensor = handle;
	sensor_event.type = type;

	sensor_t_data.name = android_name;
	sensor_t_data.handle = handle;
	sensor_t_data.type = type;
	sensor_t_data.vendor = "STMicroelectronics";
	sensor_t_data.version = 1;

	real_pollrate = 0;
	dependencies_num = 0;
	last_data_timestamp = 0;
	enabled_sensors_mask = 0;
	sensors_to_trigger_num = 0;
	type_sensor_need_trigger = -ENODEV;
	sensors_to_push_data_num = 0;
	num_data_axis = SENSOR_BASE_3AXIS;

	android_pipe_fd = pipe_data_fd;

	pthread_mutex_init(&mutext.trigger_mutex, NULL);
	pthread_cond_init(&mutext.trigger_data_cond, NULL);

	valid_class = true;
}

SensorBase::~SensorBase()
{
	int i;

	for (i = 0; i < (int)dependencies_num; i++)
		DeAllocateBufferForDependencyData(i);
}

bool SensorBase::IsValidClass()
{
	return valid_class;
}

int SensorBase::GetHandle()
{
	return sensor_t_data.handle;
}

int SensorBase::GetType()
{
	return sensor_t_data.type;
}

int SensorBase::GetMaxFifoLenght()
{
	return sensor_t_data.fifoMaxEventCount;
}

void SensorBase::SetBitEnableMask(int handle)
{
	enabled_sensors_mask |= (1ULL << handle);
}

void SensorBase::ResetBitEnableMask(int handle)
{
	enabled_sensors_mask &= ~(1ULL << handle);
}

char* SensorBase::GetName()
{
	return (char *)sensor_t_data.name;
}

int SensorBase::Enable(int handle, bool enable)
{
	int err, i = 0;
#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_INFO)
	bool old_status = GetStatus();
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

	if ((enable && !GetStatus()) || (!enable && !GetStatusExcludeHandle(handle))) {
		for (i = 0; i < (int)dependencies_num; i++) {
			err = dependencies[i]->Enable(sensor_event.sensor, enable);
			if (err < 0)
				goto restore_enable_dependencies;
		}
	}

	if (enable)
		SetBitEnableMask(handle);
	else {
		err = SetDelay(handle, 0, INT64_MAX);
		if (err < 0)
			goto restore_enable_dependencies;

		ResetBitEnableMask(handle);
	}

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_INFO)
	if (((old_status && !GetStatus()) || (!old_status && GetStatus())) && (sensor_t_data.type < SENSOR_TYPE_ST_CUSTOM_NO_SENSOR)) {
		if (GetStatus())
			ALOGI("\"%s\": power-on (sensor type: %d).", sensor_t_data.name, sensor_t_data.type);
		else
			ALOGI("\"%s\": power-off (sensor type: %d).", sensor_t_data.name, sensor_t_data.type);
	}
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

	return 0;

restore_enable_dependencies:
	for (i--; i >= 0; i--)
		dependencies[i]->Enable(sensor_event.sensor, !enable);

	return err;
}

bool SensorBase::GetStatusExcludeHandle(int handle)
{
	return (enabled_sensors_mask & ~(1ULL << handle)) > 0 ? true : false;
}

bool SensorBase::GetStatusOfHandle(int handle)
{
	return (enabled_sensors_mask & (1ULL << handle)) > 0 ? true : false;
}

bool SensorBase::GetStatus()
{
	return enabled_sensors_mask > 0 ? true : false;
}

int SensorBase::SetDelay(int handle, int64_t period_ns, int64_t timeout)
{
	int err, i;
	int64_t restore_min_timeout, restore_min_period_ms;

	restore_min_timeout = sensors_timeout[handle];
	restore_min_period_ms = sensors_pollrates[handle];

	sensors_pollrates[handle] = period_ns;
	sensors_timeout[handle] = timeout;

	for (i = 0; i < (int)dependencies_num; i++) {
		err = dependencies[i]->SetDelay(sensor_event.sensor, GetMinPeriod(), GetMinTimeout());
		if (err < 0)
			goto restore_delay_dependencies;
	}

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_INFO)
	if ((handle == sensor_t_data.handle) && (period_ns > 0))
		ALOGI("\"%s\": changed pollrate to %.2f Hz, timeout %lld ms (sensor type: %d).",
				sensor_t_data.name, NS_TO_FREQUENCY((float)(uint64_t)period_ns),
				(uint64_t)NS_TO_MS((uint64_t)timeout), sensor_t_data.type);
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

	return 0;

restore_delay_dependencies:
	sensors_pollrates[handle] = restore_min_period_ms;
	sensors_timeout[handle] = restore_min_timeout;

	for (i--; i >= 0; i--)
		dependencies[i]->SetDelay(sensor_event.sensor, GetMinPeriod(), GetMinTimeout());

	return err;
}

int64_t SensorBase::GetDelay()
{
	return sensors_pollrates[sensor_event.sensor];
}

int64_t SensorBase::GetRealPollrate()
{
	return real_pollrate;
}

void SensorBase::GetDepenciesTypeList(int type[SENSOR_BASE_MAX_DEPENDENCY])
{
	memcpy(type, type_dependencies, SENSOR_BASE_MAX_DEPENDENCY * sizeof(int));
}

trigger_mutex* SensorBase::GetMutexForTrigger()
{
	return &mutext;
}

int SensorBase::GetSensorNeedTriggerType()
{
	return type_sensor_need_trigger;
}

int SensorBase::AddSensorDependency(SensorBase *p)
{
	int err;
	uint32_t sensor_wake_flag;
	struct sensor_t dependecy_data;

	if (dependencies_num >= SENSOR_BASE_MAX_DEPENDENCY) {
		ALOGE("%s: Failed to add dependency, too many dependencies.", android_name);
		return -ENOMEM;
	}

	err = AllocateBufferForDependencyData(dependencies_num, p->GetMaxFifoLenght());
	if (err < 0)
		return err;

	err = p->AddSensorToDataPush(this);
	if (err < 0) {
		DeAllocateBufferForDependencyData(dependencies_num);
		return err;
	}

	p->FillSensor_tData(&dependecy_data);
	sensor_t_data.power += dependecy_data.power;

	sensor_wake_flag = (dependecy_data.flags & SENSOR_FLAG_WAKE_UP);
	if (dependencies_num == 0)
		sensor_t_data.flags |= sensor_wake_flag;
	else {
		if (!sensor_wake_flag)
			sensor_t_data.flags &= ~sensor_wake_flag;
	}

	handle_remapping[p->GetHandle()] = dependencies_num;
	dependencies[dependencies_num] = p;
	dependencies_num++;

	return 0;
}

void SensorBase::RemoveSensorDependency(SensorBase *p)
{
	int i;

	for (i = 0; i < (int)dependencies_num; i++) {
		if (p == dependencies[i])
			break;
	}
	if (i == (int)dependencies_num)
		return;

	DeAllocateBufferForDependencyData(i);
	p->RemoveSensorToDataPush(this);

	for (; i < (int)dependencies_num - 1; i++)
		dependencies[i] = dependencies[i + 1];

	dependencies_num--;
}

int SensorBase::AllocateBufferForDependencyData(int dependency_id, unsigned int max_fifo_len)
{
	circular_buffer_data[dependency_id] = new CircularBuffer(max_fifo_len < 2 ? 2 : max_fifo_len);
	if (!circular_buffer_data[dependency_id])
		return -ENOMEM;

	return 0;
}

void SensorBase::DeAllocateBufferForDependencyData(int dependency_id)
{
	delete circular_buffer_data[dependency_id];
}

int SensorBase::AddSensorToDataPush(SensorBase *t)
{
	int err;

	if (sensors_to_push_data_num >= SENSOR_BASE_MAX_DEPENDENCY) {
		ALOGE("%s: Failed to add dependency data, too many sensors to push data.", android_name);
		return -ENOMEM;
	}

	sensors_to_push_data_type[sensors_to_push_data_num] = t->GetType();
	sensors_to_push_data[sensors_to_push_data_num] = t;
	sensors_to_push_data_num++;

	return 0;
}

void SensorBase::RemoveSensorToDataPush(SensorBase *t)
{
	int i;

	for (i = 0; i < (int)sensors_to_push_data_num; i++) {
		if (t == sensors_to_push_data[i])
			break;
	}
	if (i == (int)sensors_to_push_data_num)
		return;

	for (; i < (int)sensors_to_push_data_num - 1; i++)
		sensors_to_push_data[i] = sensors_to_push_data[i + 1];

	sensors_to_push_data_num--;
}

int SensorBase::AddSensorToTrigger(SensorBase *t)
{
	int err;

	if (sensors_to_trigger_num >= SENSOR_BASE_MAX_DEPENDENCY) {
		ALOGE("%s: Failed to add dependency, too many sensors to trigger.", android_name);
		return -ENOMEM;
	}

	sensors_to_trigger[sensors_to_trigger_num] = t;
	sensors_to_trigger_num++;

	return 0;
}

bool SensorBase::FillSensor_tData(struct sensor_t *data)
{
	memcpy(data, &sensor_t_data, sizeof(struct sensor_t));

	if (sensor_t_data.type >= SENSOR_TYPE_ST_CUSTOM_NO_SENSOR)
		return false;

	return true;
}

int SensorBase::FlushData(int)
{
	int err;
	sensors_event_t flush_event_data;

	flush_event_data.sensor = 0;
	flush_event_data.timestamp = 0;
	flush_event_data.meta_data.sensor = sensor_t_data.handle;
	flush_event_data.meta_data.what = META_DATA_FLUSH_COMPLETE;
	flush_event_data.type = SENSOR_TYPE_META_DATA;
	flush_event_data.version = META_DATA_VERSION;

	err = write(android_pipe_fd, &flush_event_data, sizeof(sensor_event));
	if (err < 0) {
		ALOGE("%s: Failed to write flush event data to pipe.", android_name);
		return err;
	}

	return 0;
}

void SensorBase::WriteDataToPipe()
{
	int err;

	if (!GetStatusOfHandle(sensor_t_data.handle))
		return;

	if (sensor_event.timestamp > last_data_timestamp) {
		err = write(android_pipe_fd, &sensor_event, sizeof(sensor_event));
		if (err < 0) {
			ALOGE("%s: Failed to write sensor data to pipe.", android_name);
			return;
		}

		last_data_timestamp = sensor_event.timestamp;
	}
}

void SensorBase::ProcessData(SensorBaseData *data)
{
	int i;
	trigger_mutex *dep_mutex;

	for (i = 0; i < (int)sensors_to_push_data_num; i++) {
		if (sensors_to_push_data[i]->GetStatus())
			sensors_to_push_data[i]->ReceiveDataFromDependency(sensor_t_data.handle, data);
	}

	for (i = 0; i < (int)sensors_to_trigger_num; i++) {
		if (sensors_to_trigger[i]->GetStatus()) {
			dep_mutex = sensors_to_trigger[i]->GetMutexForTrigger();
			pthread_mutex_lock(&dep_mutex->trigger_mutex);
			pthread_cond_signal(&dep_mutex->trigger_data_cond);
			pthread_mutex_unlock(&dep_mutex->trigger_mutex);
		}
	}
}

void SensorBase::ProcessEvent(struct iio_event_data __attribute__((unused))*event_data)
{
	return;
}

void SensorBase::TriggerEventReceived()
{
	return;
}

void SensorBase::ReceiveDataFromDependency(int handle, SensorBaseData *data)
{
	if (data->timestamp >= last_timestap_pushed[handle]) {
		circular_buffer_data[handle_remapping[handle]]->writeElement(data);
		last_timestap_pushed[handle] = data->timestamp;
	}

	return;
}

int SensorBase::GetLatestValidDataFromDependency(int dependency_id, SensorBaseData *data)
{
	return circular_buffer_data[dependency_id]->readElement(data);
}


int64_t SensorBase::GetMinTimeout()
{
	int i;
	int64_t min = INT64_MAX;

	for (i = 0; i < ST_HAL_IIO_MAX_DEVICES; i++) {
		if (sensors_timeout[i] < min)
			min = sensors_timeout[i];
	}

	return min;
}

int64_t SensorBase::GetMinPeriod()
{
	int i;
	int64_t min = INT64_MAX;

	for (i = 0; i < ST_HAL_IIO_MAX_DEVICES; i++) {
		if ((sensors_pollrates[i] < min) && (sensors_pollrates[i] > 0))
			min = sensors_pollrates[i];
	}

	return min;
}

void *SensorBase::ThreadWork(void *context)
{
	SensorBase *mypointer = (SensorBase *)context;

	mypointer->ThreadTask();

	return mypointer;
}

void SensorBase::ThreadTask()
{
	pthread_exit(NULL);
}
