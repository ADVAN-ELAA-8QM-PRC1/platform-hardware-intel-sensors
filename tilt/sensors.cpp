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

#include <hardware/sensors.h>
#include <fcntl.h>
#include <errno.h>
#include <dirent.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <linux/input.h>
#include <utils/Atomic.h>
#include <utils/Log.h>
#include "sensors.h"
#include "TiltSensor.h"

#define DELAY_OUT_TIME          0x7FFFFFFF
#define LIGHT_SENSOR_POLLTIME   2000000000

#define SENSORS_LIGHT_HANDLE    (ID_L)
#define SENSORS_TILT_HANDLE     (ID_T)


/* The SENSORS Module */
static struct sensor_t sSensorList[] = {
	{ "TILT sensor",
	  "STMicroelectronics",
	  1, SENSORS_TILT_HANDLE,
	  SENSOR_TYPE_WRIST_TILT, 1.0f, 1.0f, 1.0f, 0, 0, 0,
	#ifdef ANDROID_L
	  SENSOR_STRING_TYPE_WRIST_TILT, NULL, 0, SENSOR_FLAG_SPECIAL_REPORTING_MODE | SENSOR_FLAG_WAKE_UP,
	#else
	  NULL, NULL, 0, 0,
	#endif
	  { } },
};

static int open_sensors(const struct hw_module_t* module, const char* id,
			struct hw_device_t** device);



static int sensors__get_sensors_list(struct sensors_module_t* module,
			struct sensor_t const** list)
{
	*list = sSensorList;
	return ARRAY_SIZE(sSensorList);
}

static struct hw_module_methods_t sensors_module_methods = {
	open: open_sensors
};

struct sensors_module_t HAL_MODULE_INFO_SYM = {
	common: {
		tag: HARDWARE_MODULE_TAG,
		version_major: 1,
		version_minor: 0,
		id: SENSORS_HARDWARE_MODULE_ID,
		name: "Intel MVN Sensor module",
		author: "Intel MVN Company",
		methods: &sensors_module_methods,
		dso: 0,
		reserved: {},
	},
	get_sensors_list: sensors__get_sensors_list,
};

struct sensors_poll_context_t {
#ifdef HAL_VERSION_GE_1_0
	struct sensors_poll_device_1 device; /* must be first */
#else
	struct sensors_poll_device_t device; /* must be first */
#endif
	sensors_poll_context_t();
	~sensors_poll_context_t();
	int activate(int handle, int enabled);
	int setDelay(int handle, int64_t ns);
#ifdef HAL_VERSION_GT_1_0
	int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
#endif
	int pollEvents(sensors_event_t* data, int count);
	bool getInitialized() { return mInitialized; };

private:
	bool mInitialized;

	enum {
		TILT = 0,
		numSensorDrivers,       /* wake pipe goes here */
		numFds,
	};

	static const size_t wake = numFds - 1;
	static const char WAKE_MESSAGE = 'W';
	struct pollfd mPollFds[numFds];
	int mWritePipeFd;
	SensorBase* mSensors[numSensorDrivers];

	int handleToDriver(int handle) const {
		switch (handle) {
		case ID_T:
			return TILT;
		}
		return -EINVAL;
	}
};



sensors_poll_context_t::sensors_poll_context_t()
{
	FUNC_LOG;
	mInitialized = false;
	/* Must clean this up early or else the destructor will make a mess */
	memset(mSensors, 0, sizeof(mSensors));

	mSensors[TILT] = new TiltSensor();
	mPollFds[TILT].fd = mSensors[TILT]->getFd();
	mPollFds[TILT].events = POLLIN;
	mPollFds[TILT].revents = 0;

	int wakeFds[2];
	int result = pipe(wakeFds);
	ALOGE_IF(result<0, "error creating wake pipe (%s)", strerror(errno));
	fcntl(wakeFds[0], F_SETFL, O_NONBLOCK);
	fcntl(wakeFds[1], F_SETFL, O_NONBLOCK);
	mWritePipeFd = wakeFds[1];

	mPollFds[wake].fd = wakeFds[0];
	mPollFds[wake].events = POLLIN;
	mPollFds[wake].revents = 0;
	mInitialized = true;
}

sensors_poll_context_t::~sensors_poll_context_t()
{
	FUNC_LOG;
	for (int i=0 ; i<numSensorDrivers ; i++)
		delete mSensors[i];

	close(mPollFds[wake].fd);
	close(mWritePipeFd);
	mInitialized = false;
}

int sensors_poll_context_t::activate(int handle, int enabled)
{
	FUNC_LOG;
	if (!mInitialized)
		return -EINVAL;

	int index = handleToDriver(handle);
	if (index < 0)
		return index;
	int err =  mSensors[index]->enable(handle, enabled);
	if (!err) {
		const char wakeMessage(WAKE_MESSAGE);
		int result = write(mWritePipeFd, &wakeMessage, 1);
		ALOGE_IF(result<0, "error sending wake message (%s)", strerror(errno));
	}
	return err;
}

int sensors_poll_context_t::setDelay(int handle, int64_t ns)
{
	FUNC_LOG;
	int index = handleToDriver(handle);
	if (index < 0)
		return index;

	return mSensors[index]->setDelay(handle, ns);
}

#ifdef HAL_VERSION_GT_1_0
int sensors_poll_context_t::batch(int handle, int flags, int64_t period_ns, int64_t timeout)
{
	FUNC_LOG;
	int index = handleToDriver(handle);
	if (index < 0)
		return index;

	return mSensors[index]->batch(handle, flags, period_ns, timeout);
}
#endif

int sensors_poll_context_t::pollEvents(sensors_event_t* data, int count)
{
	FUNC_LOG;
	int nbEvents = 0;
	int n = 0;

	do {
		for (int i=0 ; count && i<numSensorDrivers; i++) {
			SensorBase* const sensor(mSensors[i]);
			/* See if we have some pending events from the last poll() */
			if ((mPollFds[i].revents & POLLIN) || (sensor->hasPendingEvents())) {
				int nb = sensor->readEvents(data, count);

				/* no more data for this sensor */
				if (nb < count)
					mPollFds[i].revents = 0;

				count -= nb;
				nbEvents += nb;
				data += nb;
			}
		}

		if (count) {
			ALOGV("%s: start poll syscall to kernel", __func__);
			n = poll(mPollFds, numFds, nbEvents ? 0 : -1);
			if (n < 0) {
				ALOGE("poll() failed (%s)", strerror(errno));
				return -errno;
			}
			if (mPollFds[wake].revents & POLLIN) {
				char msg;
				int result = read(mPollFds[wake].fd, &msg, 1);
				ALOGE_IF(result<0, "error reading from wake pipe (%s)", strerror(errno));
				ALOGE_IF(msg != WAKE_MESSAGE, "unknown message on wake queue (0x%02x)", int(msg));
				mPollFds[wake].revents = 0;
			}
		}
		/* if we have events and space, go read them */
	} while (n && count);

	return nbEvents;
}

static int poll__close(struct hw_device_t *dev)
{
	FUNC_LOG;
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	if (ctx)
		delete ctx;

	return 0;
}

static int poll__activate(struct sensors_poll_device_t *dev,
			int handle, int enabled)
{
	FUNC_LOG;
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->activate(handle, enabled);
}

static int poll__setDelay(struct sensors_poll_device_t *dev,
			int handle, int64_t ns)
{
	FUNC_LOG;
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->setDelay(handle, ns);
}

#ifdef HAL_VERSION_GT_1_0
static int poll__batch(struct sensors_poll_device_1 *dev,
			int handle, int flags, int64_t period_ns, int64_t timeout)
{
	FUNC_LOG;
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->batch(handle, flags, period_ns, timeout);
}
#endif

static int poll__poll(struct sensors_poll_device_t *dev,
			sensors_event_t* data, int count)
{
	FUNC_LOG;
	sensors_poll_context_t *ctx = (sensors_poll_context_t *)dev;
	return ctx->pollEvents(data, count);
}

/* Open a new instance of a sensor device using name */
static int open_sensors(const struct hw_module_t* module, const char* id,
			struct hw_device_t** device)
{
	FUNC_LOG;
	int status = -EINVAL;
	sensors_poll_context_t *dev = new sensors_poll_context_t();

	if (!dev->getInitialized()) {
		ALOGE("Failed to open the sensors (%s)", id);
		return status;
	}
#ifdef HAL_VERSION_GE_1_0
	memset(&dev->device, 0, sizeof(sensors_poll_device_1));
#else
	memset(&dev->device, 0, sizeof(sensors_poll_device_t));
#endif

	dev->device.common.tag = HARDWARE_DEVICE_TAG;
#ifdef ANDROID_L
	dev->device.common.version  = SENSORS_DEVICE_API_VERSION_1_3;
#endif
#ifdef ANDROID_KK
	dev->device.common.version  = SENSORS_DEVICE_API_VERSION_1_1;
#endif
#ifdef ANDROID_JBMR2
	dev->device.common.version  = SENSORS_DEVICE_API_VERSION_1_0;
#endif
#ifdef ANDROID_JB
	dev->device.common.version  = 0;
#endif
	dev->device.common.module   = const_cast<hw_module_t*>(module);
	dev->device.common.close    = poll__close;
	dev->device.activate        = poll__activate;
	dev->device.setDelay        = poll__setDelay;
#ifdef HAL_VERSION_GT_1_0
	dev->device.batch           = poll__batch;
#endif
	dev->device.poll            = poll__poll;

	*device = &dev->device.common;
	status = 0;

	return status;
}
