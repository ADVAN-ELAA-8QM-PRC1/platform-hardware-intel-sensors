/*
 * Copyright (C) 2013-2015 STMicroelectronics
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ST_SENSOR_BASE_H
#define ST_SENSOR_BASE_H

#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>
#include <pthread.h>
#include <errno.h>
#include <poll.h>
#include <vector>

#include <hardware/sensors.h>
#include <cutils/log.h>

#include "common_data.h"
#include <CircularBuffer.h>

#define SENSOR_BASE_0AXIS			(0)
#define SENSOR_BASE_1AXIS			(1)
#define SENSOR_BASE_3AXIS			(3)
#define SENSOR_BASE_4AXIS			(4)

#define SENSOR_BASE_DEPENDENCY_0		(0)
#define SENSOR_BASE_DEPENDENCY_1		(1)
#define SENSOR_BASE_DEPENDENCY_2		(2)
#define SENSOR_BASE_DEPENDENCY_3		(3)
#define SENSOR_BASE_DEPENDENCY_4		(4)
#define SENSOR_BASE_MAX_DEPENDENCY		(5)

#define SENSOR_BASE_ANDROID_NAME_MAX		(40)

#define POLL_TIMEOUT_FLUSH_EVENT		(-1)
#define POLL_TIMEOUT_DATA_EVENT		(1000)

#define GAUSS_TO_UTESLA(x)			((x) * 100.0f)
#define NS_TO_FREQUENCY(x)			(1E9 / x)
#define FREQUENCY_TO_NS(x)			(1E9 / x)
#define FREQUENCY_TO_US(x)			(1E6 / x)
#define NS_TO_MS(x)				(x / 1E6)
#define NS_TO_S(x)				(x / 1E9)

typedef struct trigger_mutex {
	pthread_mutex_t trigger_mutex;
	pthread_cond_t trigger_data_cond;
} trigger_mutex;

class SensorBase;

/*
 * class SensorBase
 */
class SensorBase {
private:
	int64_t enabled_sensors_mask;
	int64_t sensors_timeout[ST_HAL_IIO_MAX_DEVICES];
	int64_t sensors_pollrates[ST_HAL_IIO_MAX_DEVICES];
	int64_t last_timestap_pushed[ST_HAL_IIO_MAX_DEVICES];

	CircularBuffer *circular_buffer_data[SENSOR_BASE_MAX_DEPENDENCY];

protected:
	bool valid_class;
	char android_name[SENSOR_BASE_ANDROID_NAME_MAX];

	int android_pipe_fd;
	int type_sensor_need_trigger;
	int handle_remapping[ST_HAL_IIO_MAX_DEVICES];
	int type_dependencies[SENSOR_BASE_MAX_DEPENDENCY];
	int sensors_to_push_data_type[SENSOR_BASE_MAX_DEPENDENCY];

	unsigned int num_data_axis;
	unsigned int dependencies_num;
	unsigned int sensors_to_trigger_num;
	unsigned int sensors_to_push_data_num;

	int64_t real_pollrate;
	int64_t last_data_timestamp;

	SensorBase *sensors_to_push_data[SENSOR_BASE_MAX_DEPENDENCY];
	SensorBase *sensors_to_trigger[SENSOR_BASE_MAX_DEPENDENCY];
	SensorBase *dependencies[SENSOR_BASE_MAX_DEPENDENCY];
	sensors_event_t sensor_event;
	struct sensor_t sensor_t_data;

	trigger_mutex mutext;

	int64_t GetMinTimeout();
	int64_t GetMinPeriod();

	void SetBitEnableMask(int handle);
	void ResetBitEnableMask(int handle);

	bool GetStatusExcludeHandle(int handle);
	bool GetStatusOfHandle(int handle);

	int AllocateBufferForDependencyData(int dependency_id, unsigned int max_fifo_len);
	void DeAllocateBufferForDependencyData(int dependency_id);

	int AddSensorToDataPush(SensorBase *t);
	void RemoveSensorToDataPush(SensorBase *t);

public:
	SensorBase(const char *name, int handle, int type, int pipe_data_fd);
	virtual ~SensorBase();
	bool IsValidClass();

	int GetHandle();
	int GetType();
	int GetMaxFifoLenght();

	std::vector<int64_t> timestamp;
	char* GetName();

	virtual int Enable(int handle, bool enable);
	bool GetStatus();

	virtual int SetDelay(int handle, int64_t period_ns, int64_t timeout);
	int64_t GetDelay();
	int64_t GetRealPollrate();

	void GetDepenciesTypeList(int type[SENSOR_BASE_MAX_DEPENDENCY]);
	int AddSensorDependency(SensorBase *p);
	void RemoveSensorDependency(SensorBase *p);

	trigger_mutex* GetMutexForTrigger();
	int GetSensorNeedTriggerType();
	int AddSensorToTrigger(SensorBase *t);

	bool FillSensor_tData(struct sensor_t *data);

	virtual int WritePipeWithPoll(sensors_event_t *event_data, int size, int timeout);
	virtual int FlushData(bool);

	virtual void ProcessData(SensorBaseData *data);
	virtual void ProcessEvent(struct iio_event_data *event_data);
	virtual void TriggerEventReceived();
	virtual void WriteDataToPipe();
	virtual void ReceiveDataFromDependency(int handle, SensorBaseData *data);

	virtual int GetLatestValidDataFromDependency(int dependency_id, SensorBaseData *data);

	static void *ThreadWork(void *context);
	virtual void ThreadTask();
};

int64_t get_monotonic_time(void);

#endif /* ST_SENSOR_BASE_H */
