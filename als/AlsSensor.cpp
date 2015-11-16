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

#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>
#include <cutils/properties.h>

#include "AlsSensor.h"

#define ALS_DATA_NAME "tsl2584 ambient light sensor"
#define SYSFS_ENABLE_FILE "/sys/class/i2c-adapter/i2c-6/6-0029/als_enable"
#define SYSFS_POLL_FILE "/sys/class/i2c-adapter/i2c-6/6-0029/als_delay"

/*****************************************************************************/

#define FIRST_GOOD_EVENT    1

LightSensor::LightSensor()
    : SensorBase(NULL, ALS_DATA_NAME),
      mEnabled(0),
      mEventsSinceEnable(0),
      mInputReader(4),
      mHasPendingEvent(false)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_L;
    mPendingEvent.type = SENSOR_TYPE_LIGHT;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

LightSensor::~LightSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
}

int LightSensor::setDelay(int32_t /* handle */, int64_t ns)
{
    int fd;
    int len, ms, ret = -1;
    char buf[6];

    ms = ns / 1000000;

    fd = open(SYSFS_POLL_FILE, O_RDWR);
    if (fd) {
        len = 6;
        memset(buf, 0, len);
        snprintf(buf, len, "%d", ms);
        write(fd, buf, sizeof(buf));
        close(fd);
        ret = 0;
    } else
        ALOGE("file  open failure\n");

    return ret;
}
int LightSensor::enable(int32_t /* handle */, int en)
{
    int flags = en ? 1 : 0;

    mEventsSinceEnable = 0;
    mPreviousLight = -1;
    if (flags != mEnabled) {
        int fd;
        fd = open(SYSFS_ENABLE_FILE, O_RDWR);
        if (fd >= 0) {
            char buf[2];
            buf[1] = 0;
            if (flags) {
                buf[0] = '1';
            } else {
                buf[0] = '0';
            }
            write(fd, buf, sizeof(buf));
            close(fd);
            mEnabled = flags;
            return 0;
        }
        return -1;
    }

    if (en)
        mEnabled=1;
    else
        mEnabled=0;

    return 0;
}

int LightSensor::isActivated(int /* handle */)
{
    return mEnabled;
}

#ifdef HAL_VERSION_GT_1_0
int LightSensor::batch(int /* handle */, int /* flags */, int64_t period_ns, int64_t /* timeout */)
{
    int fd;
    int len, ms, ret = -1;
    char buf[6];

    ms = period_ns / 1000000;

    fd = open(SYSFS_POLL_FILE, O_RDWR);
    if (fd) {
       len = 6;
       memset(buf, 0, len);
       snprintf(buf, len, "%d", ms);
       write(fd, buf, sizeof(buf));
       close(fd);
       ret = 0;
    }
    else
        ALOGE("file  open failure\n");

    return ret;
}
#endif

bool LightSensor::hasPendingEvents() const {
    return mHasPendingEvent;
}

int LightSensor::readEvents(sensors_event_t* data, int count)
{
    if (count < 1)
        return -EINVAL;

    if (mHasPendingEvent) {
        mHasPendingEvent = false;
        mPendingEvent.timestamp = getTimestamp();
        *data = mPendingEvent;
        return mEnabled ? 1 : 0;
    }

    ssize_t n = mInputReader.fill(data_fd);
    if (n < 0)
        return n;

    int numEventReceived = 0;
    input_event const* event;

    while (count && mInputReader.readEvent(&event)) {
        int type = event->type;
        if (type == EV_MSC) {
            if (event->code == EVENT_TYPE_LIGHT) {
                 mPendingEvent.light = event->value;
                 if (mEventsSinceEnable < FIRST_GOOD_EVENT)
                    mEventsSinceEnable++;
            }
        } else if (type == EV_SYN) {
            mPendingEvent.timestamp = timevalToNano(event->time);
            if (mEnabled && (mPendingEvent.light != mPreviousLight) &&
                    mEventsSinceEnable >= FIRST_GOOD_EVENT) {
                *data++ = mPendingEvent;
                count--;
                numEventReceived++;
                mPreviousLight = mPendingEvent.light;
            }
        } else {
            ALOGE("LightSensor: unknown event (type=%d, code=%d)",
                    type, event->code);
        }
        mInputReader.next();
    }

    return numEventReceived;
}
