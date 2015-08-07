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

#ifndef ANDROID_TILT_SENSOR_H
#define ANDROID_TILT_SENSOR_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include "sensors.h"
#include "SensorBase.h"
#include "InputEventReader.h"

#define SYSFS_MAX_PATH_LEN	64
#define INPUT_SYSFS_BASE	"/sys/class/i2c-adapter/i2c-4/4-001e"
#define POLL_PERIOD_MS		"poll_period_ms"
#define RANGE				"range"
#define ENABLE_DEVICE		"enable_device"
#define ENABLE_INTERRUPT_OUTPUT	"enable_interrupt_output"
#define ENABLE_STATE_PROG	"enable_state_prog"
#ifdef TILT_DEBUG
#define REG_VALUE			"reg_value"
#define REG_ADDR			"reg_addr"
#endif


struct input_event;

class TiltSensor : public SensorBase {
public:
    TiltSensor();
    virtual ~TiltSensor();

    virtual int readEvents(sensors_event_t* data, int count);
    virtual int setDelay(int32_t handle, int64_t ns);
    virtual int enable(int32_t handle, int enabled);
    virtual int isActivated(int handle);
#ifdef HAL_VERSION_GT_1_0
    virtual int batch(int handle, int flags, int64_t period_ns, int64_t timeout);
#endif

private:
    int mEnabled;
    InputEventCircularReader mInputReader;
    sensors_event_t mPendingEvent;
};
#endif  // ANDROID_TILT_SENSOR_H
