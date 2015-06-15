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

#include <cutils/log.h>
#include "sensor_cal.h"

/* Load sensor calibration data */
int load_cali_data(const int sindex)
{
    int err = 0, num, sum;
    FILE *filp;

    filp = fopen(sensor_cali_data_path[sindex], "r");
    if (filp == NULL) {
        err = -errno;
        ALOGE("failed to open %s, errno=%d", sensor_cali_data_path[sindex], err);
        return err;
    }

    num = fscanf(filp, "%d %d %d %d",
                    &cal_data[sindex][0], &cal_data[sindex][1],
                    &cal_data[sindex][2], &sum);
    if (num != 4) {
        err = -EINVAL;
        ALOGE("read %s data failed, num=%d", sensor_cali_data_path[sindex], num);
        goto out;
    }

    if ((cal_data[sindex][0] + cal_data[sindex][1] + cal_data[sindex][2]) != sum) {
        err = -EINVAL;
        ALOGE("%s check sum error", sensor_cali_data_path[sindex]);
        goto out;
    }

out:
    fclose(filp);
    return err;
}

/* Write calibration data to iio channel offset */
void set_cali_offset(const int sindex)
{
#define MAX_BUF_LEN 64
    int err, j;
    FILE *filp;
    char buf[MAX_BUF_LEN];

    for (j = 0; j < (Z_AXIS_INDEX + 1); j++) {
        err = snprintf(buf, MAX_BUF_LEN, "%s/%s",
                sensor_sysfs_dir[sindex], sensor_offset[sindex][j]);
        if (err < 0) {
            ALOGE("%s/%s snprintf err=%d",
                sensor_sysfs_dir[sindex], sensor_offset[sindex][j], err);
            return;
        }
        filp = fopen(buf, "w");
        if (filp == NULL) {
            ALOGE("failed to open %s, errno=%d", buf, errno);
            return;
        }
        fprintf(filp, "%d", cal_data[sindex][j]);
        fclose(filp);
    }

    if (sindex == ACCEL_SINDEX)
        accl_cal_data_loaded = true;
    if (sindex == GYRO_SINDEX)
        gyro_cal_data_loaded = true;
}


void do_cal_data_loading(const int sindex)
{
    int err;

    err = load_cali_data(sindex);
    if (err != 0)
        return;

    set_cali_offset(sindex);

}
