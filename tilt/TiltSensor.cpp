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

#include "TiltSensor.h"

TiltSensor::TiltSensor()
    : SensorBase(NULL, "lis3dsh_acc"),
      mEnabled(0),
      mInputReader(4)
{
    mPendingEvent.version = sizeof(sensors_event_t);
    mPendingEvent.sensor = ID_T;
    mPendingEvent.type = SENSOR_TYPE_WRIST_TILT_GESTURE;
    memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));
}

TiltSensor::~TiltSensor() {
    if (mEnabled) {
        enable(0, 0);
    }
}

int TiltSensor::enable(int32_t /* handle */, int en)
{
	int ret = 0;
	int flags = en ? 1 : 0;
	char sysfs_path[SYSFS_MAX_PATH_LEN];

	if (flags != mEnabled) {
		int fd;
		char buf[2];

		snprintf(sysfs_path, SYSFS_MAX_PATH_LEN, "%s/%s", INPUT_SYSFS_BASE, ENABLE_DEVICE);
		fd = open(sysfs_path, O_RDWR);
		if (fd > 0) {
			buf[1] = 0;
			if (flags)
				buf[0] = '1';
			else
				buf[0] = '0';
			write(fd, buf, sizeof(buf));
			close(fd);
		} else {
			ret = -1;
			goto out;
		}

		snprintf(sysfs_path, SYSFS_MAX_PATH_LEN, "%s/%s", INPUT_SYSFS_BASE, ENABLE_INTERRUPT_OUTPUT);
		fd = open(sysfs_path, O_RDWR);
		if (fd > 0) {
			buf[1] = 0;
			if (flags)
				buf[0] = '2';	/* 3: enable int1 and int2; 2: enable int1; 1: enable int2; 0: disbale */
			else
				buf[0] = '0';
			write(fd, buf, sizeof(buf));
			close(fd);
		} else {
			ret = -1;
			goto out;
		}

		snprintf(sysfs_path, SYSFS_MAX_PATH_LEN, "%s/%s", INPUT_SYSFS_BASE, ENABLE_STATE_PROG);
		fd = open(sysfs_path, O_RDWR);
		if(fd > 0) {
			buf[1] = 0;
			if (flags)
				buf[0] = '2';	/* 3: enable SM1 and SM2; 2: enable SM1; 1: enable SM2; 0: disbale */
			else
				buf[0] = '0';
			write(fd, buf, sizeof(buf));
			close(fd);
			mEnabled = flags;
		} else {
			ret = -1;
		}
	}
out:
	return ret;
}

int TiltSensor::isActivated(int /* handle */)
{
	return mEnabled;
}

int TiltSensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_MSC) {
			mPendingEvent.data[0] = 1.0f;
			/*mPendingEvent.data[event->code] = event->value;*/
		} else if (type == EV_SYN) {
			mPendingEvent.timestamp = timevalToNano(event->time);
			if (mEnabled) {
				*data++ = mPendingEvent;
				count--;
				numEventReceived++;
			}
		} else {
            ALOGE("TiltSensor: unknown event (type=%d, code=%d)",
							type, event->code);
		}
		mInputReader.next();
	}

	return numEventReceived;
}
