/*
 * STMicroelectronics Accel-Magn-Gyro Fusion 9X Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "SWAccelMagnGyroFusion9X.h"

extern "C" {
	#include "iNemoEngineAPI.h"
}

SWAccelMagnGyroFusion9X::SWAccelMagnGyroFusion9X(const char *name, int handle, int pipe_data_fd) :
		SWSensorBaseWithPollrate(name, handle, SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X,
			pipe_data_fd, false, false, true, false)
{
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	sensor_t_data.resolution = ST_SENSOR_FUSION_RESOLUTION(1.0f);
	sensor_t_data.maxRange = 1.0f;

	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ACCELEROMETER;
	type_dependencies[SENSOR_BASE_DEPENDENCY_1] = SENSOR_TYPE_GEOMAGNETIC_FIELD;
	type_dependencies[SENSOR_BASE_DEPENDENCY_2] = SENSOR_TYPE_GYROSCOPE;
	type_sensor_need_trigger = SENSOR_TYPE_GYROSCOPE;

	iNemoEngine_API_Initialization_9X(NULL);
}

SWAccelMagnGyroFusion9X::~SWAccelMagnGyroFusion9X()
{

}

int SWAccelMagnGyroFusion9X::Enable(int handle, bool enable)
{
	int err;
	bool old_status;

	old_status = GetStatus();

	err = SWSensorBaseWithPollrate::Enable(handle, enable);
	if (err < 0)
		return err;

	if ((GetStatus() && !old_status) || (!GetStatus() && old_status)) {
		sensor_event.timestamp = 0;
		iNemoEngine_API_enable_9X(enable);
	}

	return 0;
}

int SWAccelMagnGyroFusion9X::SetDelay(int handle, int64_t period_ns, int64_t timeout)
{
	int err;

	if ((period_ns > FREQUENCY_TO_NS(CONFIG_ST_HAL_MIN_FUSION_POLLRATE) && period_ns != INT64_MAX))
		period_ns = FREQUENCY_TO_NS(CONFIG_ST_HAL_MIN_FUSION_POLLRATE);

	err = SWSensorBaseWithPollrate::SetDelay(handle, period_ns, timeout);
	if (err < 0)
		return err;

	real_pollrate = dependencies[SENSOR_BASE_DEPENDENCY_2]->GetRealPollrate();

	return 0;
}

void SWAccelMagnGyroFusion9X::SplitAndProcessData(SensorBaseData data[ST_ACCEL_MAGN_GYRO_MAX_OUT_ID])
{
	int i, id, sensor_type;
	trigger_mutex *dep_mutex;

	for (i = 0; i < (int)sensors_to_push_data_num; i++) {
		if (sensors_to_push_data[i]->GetStatus()) {
			switch (sensors_to_push_data_type[i]) {
			case SENSOR_TYPE_ROTATION_VECTOR:
				id = ST_ACCEL_MAGN_GYRO_ROTATION_VECTOR_OUT_ID;
				break;
			case SENSOR_TYPE_ORIENTATION:
				id = ST_ACCEL_MAGN_GYRO_ORIENTATION_OUT_ID;
				break;
			case SENSOR_TYPE_GRAVITY:
				id = ST_ACCEL_MAGN_GYRO_GRAVITY_OUT_ID;
				break;
			case SENSOR_TYPE_LINEAR_ACCELERATION:
				id = ST_ACCEL_MAGN_GYRO_LINEAR_ACCEL__OUT_ID;
				break;
			default:
				continue;
			}

			sensors_to_push_data[i]->ReceiveDataFromDependency(sensor_t_data.handle, &data[id]);
		}
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

void SWAccelMagnGyroFusion9X::TriggerEventReceived()
{
	int64_t time_diff = 0;
	SensorBaseData accel_data, magn_data, gyro_data;
	int err, err2, data_remaining_gyro, nomaxdata_accel = 10, nomaxdata_magn = 10;

	do {
		data_remaining_gyro = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_2, &gyro_data);
		if (data_remaining_gyro < 0)
			return;

		do {
			err = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &accel_data);
			if (err < 0) {
				nomaxdata_accel--;
				usleep(200);
				continue;
			}

			time_diff = gyro_data.timestamp - accel_data.timestamp;

		} while ((time_diff >= GetRealPollrate()) && (nomaxdata_accel > 0));

		do {
			err2 = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_1, &magn_data);
			if (err2 < 0) {
				nomaxdata_magn--;
				usleep(200);
				continue;
			}

			time_diff = gyro_data.timestamp - magn_data.timestamp;

		} while ((time_diff >= GetRealPollrate()) && (nomaxdata_magn > 0));

		if ((err >= 0) && (err2 >= 0))
			iNemoEngine_API_Run_9X(accel_data.raw, magn_data.processed, gyro_data.processed, gyro_data.timestamp);

		sensor_event.timestamp = gyro_data.timestamp;

		err = iNemoEngine_API_Get_Quaternion_9X(outdata[ST_ACCEL_MAGN_GYRO_ROTATION_VECTOR_OUT_ID].processed);
		if (err < 0)
			return;

		err = iNemoEngine_API_Get_Euler_Angles_9X(outdata[ST_ACCEL_MAGN_GYRO_ORIENTATION_OUT_ID].processed);
		if (err < 0)
			return;

		err = iNemoEngine_API_Get_Gravity_9X(outdata[ST_ACCEL_MAGN_GYRO_GRAVITY_OUT_ID].processed);
		if (err < 0)
			return;

		err = iNemoEngine_API_Get_Linear_Acceleration_9X(outdata[ST_ACCEL_MAGN_GYRO_LINEAR_ACCEL__OUT_ID].processed);
		if (err < 0)
			return;

		outdata[ST_ACCEL_MAGN_GYRO_ROTATION_VECTOR_OUT_ID].timestamp = sensor_event.timestamp;
		outdata[ST_ACCEL_MAGN_GYRO_ORIENTATION_OUT_ID].timestamp = sensor_event.timestamp;
		outdata[ST_ACCEL_MAGN_GYRO_GRAVITY_OUT_ID].timestamp = sensor_event.timestamp;
		outdata[ST_ACCEL_MAGN_GYRO_LINEAR_ACCEL__OUT_ID].timestamp = sensor_event.timestamp;

		SplitAndProcessData(outdata);
	} while (data_remaining_gyro > 0);
}
