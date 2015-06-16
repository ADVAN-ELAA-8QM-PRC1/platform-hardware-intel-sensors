/*
 * Copyright (C) 2015 Intel Corp
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

#ifndef __SENSOR_CAL_H__
#define __SENSOR_CAL_H__

#include <stdio.h>
#include <stdbool.h>

#define NON_WAKEUP    0
#define WAKEUP    1

enum SENSOR_INDEX {
    ACCEL_SINDEX = 0,
    GYRO_SINDEX,
};

enum SENSOR_AXIS_INDEX {
    X_AXIS_INDEX = 0,
    Y_AXIS_INDEX,
    Z_AXIS_INDEX,
};

static const char * const sensor_cali_data_path[] = {
    [ACCEL_SINDEX] = "/config/sensor/accel_cali",
    [GYRO_SINDEX] = "/config/sensor/gyro_cali",
};

static const char * const sensor_sysfs_dir[][2] = {
    {
        [NON_WAKEUP] = "/sys/devices/iio:device1",
        [WAKEUP] = "/sys/devices/iio:device2",
    },
    {
        [NON_WAKEUP] = "/sys/devices/iio:device3",
        [WAKEUP] = "/sys/devices/iio:device4",
    },
};

static const char * const sensor_offset[][3] = {
    {
        [X_AXIS_INDEX] = "in_accel_x_offset",
        [Y_AXIS_INDEX] = "in_accel_y_offset",
        [Z_AXIS_INDEX] = "in_accel_z_offset",
    },
    {
        [X_AXIS_INDEX] = "in_anglvel_x_offset",
        [Y_AXIS_INDEX] = "in_anglvel_y_offset",
        [Z_AXIS_INDEX] = "in_anglvel_z_offset",
    },
};

static bool accl_cal_data_loaded;
static bool gyro_cal_data_loaded;
static int cal_data[2][3];

void do_cal_data_loading(const int sindex, const bool wakeup);

#endif
