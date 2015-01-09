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

#ifndef ANDROID_INTELNDG_SENSORS_H
#define ANDROID_INTELNDG_SENSORS_H

#include <stdint.h>
#include <errno.h>
#include <sys/cdefs.h>
#include <sys/types.h>

#include <linux/input.h>

#include <hardware/hardware.h>
#include <hardware/sensors.h>

__BEGIN_DECLS

/*****************************************************************************/

#define ARRAY_SIZE(a)		(sizeof(a) / sizeof(a[0]))

#define ID_INTELNDG_BASE	(0x1000)
/* light sensor ID */
#define ID_L				(ID_INTELNDG_BASE)
/* tilt sensor ID */
#define ID_T				(ID_L + 1)

/*****************************************************************************/

/*
 * The SENSORS Module
 */

/*****************************************************************************/
/* the GP2A is a binary proximity sensor that triggers around 5 cm on
 * this hardware */
#define PROXIMITY_THRESHOLD_GP2A  	5.0f

/* input event code for light sensor */
#define EVENT_TYPE_LIGHT		MSC_RAW
/*****************************************************************************/



__END_DECLS

#endif  /* ANDROID_INTELNDG_SENSORS_H */
