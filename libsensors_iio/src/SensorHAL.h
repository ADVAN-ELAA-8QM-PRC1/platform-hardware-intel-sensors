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

#ifndef ST_SENSOR_HAL_H
#define ST_SENSOR_HAL_H

#include <hardware/hardware.h>
#include <hardware/sensors.h>
#include <poll.h>

#include "SWSensorBase.h"
#include "common_data.h"

#define ARRAY_SIZE(a)		(int)((sizeof(a) / sizeof(*(a))) / \
					static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

/*
 * Android string version
 */
#define ST_HAL_KITKAT_VERSION				0
#define ST_HAL_LOLLIPOP_VERSION				1

/*
 * IIO driver sensors names
 */
#define ST_SENSORS_LIST_1				"lsm6ds3"

/*
 * IIO driver sensors suffix for sensors
 */
#define ACCEL_NAME_SUFFIX_IIO				"_accel"
#define MAGN_NAME_SUFFIX_IIO				"_magn"
#define GYRO_NAME_SUFFIX_IIO				"_gyro"
#define SIGN_MOTION_NAME_SUFFIX_IIO			"_sign_motion"
#define STEP_DETECTOR_NAME_SUFFIX_IIO			"_step_d"
#define STEP_COUNTER_NAME_SUFFIX_IIO			"_step_c"
#define TILT_NAME_SUFFIX_IIO				"_tilt"
#define PRESSURE_NAME_SUFFIX_IIO			"_press"

#define ST_HAL_WAKEUP_SUFFIX_IIO			"_wk"

#define CONCATENATE_STRING(x, y)			(x y)

#if (CONFIG_ST_HAL_ANDROID_VERSION == ST_HAL_LOLLIPOP_VERSION)
#define ST_HAL_IIO_DEVICE_API_VERSION			SENSORS_DEVICE_API_VERSION_1_3
#else /* CONFIG_ST_HAL_ANDROID_VERSION */
#define ST_HAL_IIO_DEVICE_API_VERSION			SENSORS_DEVICE_API_VERSION_1_1
#endif /* CONFIG_ST_HAL_ANDROID_VERSION */

#if defined(CONFIG_ST_HAL_HAS_GEOMAG_FUSION) && \
				(defined(CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED))
#define ST_HAL_NEEDS_GEOMAG_FUSION			1
#endif /* CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED */

#if defined(CONFIG_ST_HAL_HAS_6AX_FUSION) && \
				(defined(CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED) || \
				defined(CONFIG_ST_HAL_GRAVITY_AP_ENABLED) || \
				defined(CONFIG_ST_HAL_LINEAR_AP_ENABLED))
#define ST_HAL_NEEDS_6AX_FUSION				1
#endif /* CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED */

#if defined(CONFIG_ST_HAL_HAS_9AX_FUSION) && \
				(defined(CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED) || \
				defined(CONFIG_ST_HAL_ORIENTATION_AP_ENABLED) || \
				defined(CONFIG_ST_HAL_GRAVITY_AP_ENABLED) || \
				defined(CONFIG_ST_HAL_LINEAR_AP_ENABLED))
#define ST_HAL_NEEDS_9AX_FUSION				1
#endif /* CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED */


struct tmp_dicovery_data {
	char *driver_name;
	char *trigger_name;
	char *dev_buffer_path;

	char *iio_sysfs_path;
	char *iio_sysfs_custom_trigger_path;
};

struct STSensorHAL_data {
	struct sensors_poll_device_1 poll_device;

	pthread_t *threads;
	SensorBase *sensor_classes[ST_HAL_IIO_MAX_DEVICES];

	unsigned int sensor_available;
	struct sensor_t *sensor_t_list;

	struct pollfd android_pollfd;
} typedef STSensorHAL_data;

#endif /* ST_SENSOR_HAL_H */
