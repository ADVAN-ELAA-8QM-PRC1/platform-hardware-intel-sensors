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

#ifndef ANDROID_SENSOR_HAL_COMMON_DATA
#define ANDROID_SENSOR_HAL_COMMON_DATA

#include <hardware/sensors.h>

#include "../configuration.h"

#define SENSOR_TYPE_ST_CUSTOM_NO_SENSOR			(SENSOR_TYPE_DEVICE_PRIVATE_BASE + 20)
#define SENSOR_TYPE_ST_ACCEL_MAGN_FUSION6X		(SENSOR_TYPE_ST_CUSTOM_NO_SENSOR + 0)
#define SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X		(SENSOR_TYPE_ST_CUSTOM_NO_SENSOR + 1)
#define SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X		(SENSOR_TYPE_ST_CUSTOM_NO_SENSOR + 2)

#define ST_HAL_IIO_MAX_DEVICES				(50)

#define SENSOR_DATA_X(datax, datay, dataz, x1, y1, z1, x2, y2, z2, x3, y3, z3) \
							((x1 == 1 ? datax : (x1 == -1 ? -datax : 0)) + \
							(x2 == 1 ? datay : (x2 == -1 ? -datay : 0)) + \
							(x3 == 1 ? dataz : (x3 == -1 ? -dataz : 0)))

#define SENSOR_DATA_Y(datax, datay, dataz, x1, y1, z1, x2, y2, z2, x3, y3, z3) \
							((y1 == 1 ? datax : (y1 == -1 ? -datax : 0)) + \
							(y2 == 1 ? datay : (y2 == -1 ? -datay : 0)) + \
							(y3 == 1 ? dataz : (y3 == -1 ? -dataz : 0)))

#define SENSOR_DATA_Z(datax, datay, dataz, x1, y1, z1, x2, y2, z2, x3, y3, z3) \
							((z1 == 1 ? datax : (z1 == -1 ? -datax : 0)) + \
							(z2 == 1 ? datay : (z2 == -1 ? -datay : 0)) + \
							(z3 == 1 ? dataz : (z3 == -1 ? -dataz : 0)))

#define SENSOR_X_DATA(...)				SENSOR_DATA_X(__VA_ARGS__)
#define SENSOR_Y_DATA(...)				SENSOR_DATA_Y(__VA_ARGS__)
#define SENSOR_Z_DATA(...)				SENSOR_DATA_Z(__VA_ARGS__)

#define ST_HAL_DEBUG_INFO				(1)
#define ST_HAL_DEBUG_VERBOSE				(2)
#define ST_HAL_DEBUG_EXTRA_VERBOSE			(3)

#endif /* ANDROID_SENSOR_HAL_COMMON_DATA */
