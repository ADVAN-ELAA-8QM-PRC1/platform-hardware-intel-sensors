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

#ifndef ST_SWSENSOR_BASE_H
#define ST_SWSENSOR_BASE_H


#include "SensorBase.h"

#define ST_SENSOR_FUSION_RESOLUTION(maxRange)		(maxRange / (1 << 24))

class SWSensorBase;

/*
 * class SWSensorBase
 */
class SWSensorBase : public SensorBase {
protected:
	bool dependency_resolution;
	bool dependency_range;
	bool dependency_delay;
	bool dependency_name;

public:
	SWSensorBase(const char *name, int handle, int sensor_type, int pipe_data_fd,
			bool use_dependency_resolution, bool use_dependency_range,
			bool use_dependency_delay, bool use_dependency_name);
	virtual ~SWSensorBase();

	int AddSensorDependency(SensorBase *p);
	virtual int FlushData();
	virtual void ThreadTask();
};


/*
 * class SWSensorBaseWithPollrate
 */
class SWSensorBaseWithPollrate : public SWSensorBase {
public:
	SWSensorBaseWithPollrate(const char *name, int handle, int sensor_type, int pipe_data_fd,
			bool use_dependency_resolution, bool use_dependency_range,
			bool use_dependency_delay, bool use_dependency_name);
	virtual ~SWSensorBaseWithPollrate();

	virtual int SetDelay(int handle, int64_t period_ns, int64_t timeout);
	virtual void WriteDataToPipe();
};

#endif /* ST_SWSENSOR_BASE_H */
