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

#ifndef ST_HWSENSOR_BASE_H
#define ST_HWSENSOR_BASE_H

#include <poll.h>
#include <math.h>

#include "SensorBase.h"

extern "C" {
	#include "iio_utils.h"
	#include "events.h"
	#include "sensor_cal.h"
};

#define HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN	(2)
#define HW_SENSOR_BASE_IIO_SYSFS_PATH_MAX	(50)
#define HW_SENSOR_BASE_IIO_DEVICE_NAME_MAX	(30)
#define HW_SENSOR_BASE_MAX_CHANNELS		(8)

#define FILENAME_BUFFER_ENABLE			"buffer/enable"
#define FILENAME_BUFFER_LENGTH			"buffer/length"
#define FILENAME_SAMPLING_FREQ			"sampling_frequency"
#define FILENAME_MAX_RATE_DELIVERY		"max_delivery_rate"
#define FILENAME_HRTIMER_TRIGGER_FREQ		"frequency"
#define FILENAME_FLUSH				"flush"

struct HWSensorBaseCommonData {
	char iio_sysfs_path[HW_SENSOR_BASE_IIO_SYSFS_PATH_MAX];
	char device_name[HW_SENSOR_BASE_IIO_DEVICE_NAME_MAX];
	unsigned int iio_dev_num;

	int num_channels;
	struct iio_channel_info channels[HW_SENSOR_BASE_MAX_CHANNELS];

	struct iio_scale_available sa;
} typedef HWSensorBaseCommonData;


class HWSensorBase;
class HWSensorBaseWithPollrate;

/*
 * class HWSensorBase
 */
class HWSensorBase : public SensorBase {
protected:
	struct pollfd pollfd_iio[2];
	ssize_t scan_size;
	uint8_t *sensor_data;
	unsigned int current_fifo_len;
	HWSensorBaseCommonData common_data;

	int WriteBufferLenght(unsigned int buf_len);

public:
	HWSensorBase(HWSensorBaseCommonData *data, const char *name,
				int handle, int sensor_type, unsigned int hw_fifo_len,
				int pipe_data_fd, float power_consumption);
	virtual ~HWSensorBase();

	virtual int Enable(int handle, bool enable);
	virtual int FlushData(int base);
	virtual void ThreadTask();
};


/*
 * class HWSensorBaseWithPollrate
 */
class HWSensorBaseWithPollrate : public HWSensorBase {
private:
	struct iio_sampling_frequency_available sampling_frequency_available;

public:
	HWSensorBaseWithPollrate(HWSensorBaseCommonData *data, const char *name,
			struct iio_sampling_frequency_available *sfa, int handle,
			int sensor_type, unsigned int hw_fifo_len, int pipe_data_fd,
			float power_consumption);
	virtual ~HWSensorBaseWithPollrate();

	virtual int SetDelay(int handle, int64_t period_ns, int64_t timeout);
	virtual void WriteDataToPipe();
};

#endif /* ST_HWSENSOR_BASE_H */
