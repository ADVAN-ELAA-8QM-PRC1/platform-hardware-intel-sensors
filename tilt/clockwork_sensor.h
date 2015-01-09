/*
 * Copyright (C) 2015 The Android Open Source Project
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

#ifndef ANDROID_CLOCKWORK_SENSOR_INTERFACE_H
#define ANDROID_CLOCKWORK_SENSOR_INTERFACE_H

#include <hardware/sensors.h>

__BEGIN_DECLS

/*
 * SENSOR_TYPE_WRIST_TILT
 * trigger-mode: special
 * wake-up sensor: yes
 *
 * A sensor of this type triggers an event each time a tilt of the device
 * is detected.
 *
 * Upon detecting a tilt event, a single event is returned containing the
 * value of the 3 accelerometer axes when the tilt event was detected.
 *
 *  All values are in SI units (m/s^2) and measure the acceleration of the
 *  device minus the force of gravity.
 *
 *  x: Acceleration on the x-axis
 *  y: Acceleration on the y-axis
 *  z: Acceleration on the z-axis
 *
 * Note that the readings from the accelerometer include the acceleration
 * due to gravity (which is opposite to the direction of the gravity vector).
 *
 * See "SENSOR_TYPE_ACCELEROMETER" for more detail on returned accelerometer
 * data.
 *
 * setDelay() has no impact on this sensor type
 *
 *  IMPORTANT NOTE: this sensor type is very different from other types
 *  in that it must work when the screen is off without the need of
 *  holding a partial wake-lock and MUST allow the SoC to go into suspend.
 *  When a tilt event is detected, the sensor must awaken the SoC and
 *  the event be reported.
 *
 *  When the sensor is not activated, it must also be deactivated in the
 *  hardware: it must not wake up the SoC anymore, even in case of
 *  a tilt event.
 */
#define SENSOR_TYPE_WRIST_TILT        (SENSOR_TYPE_DEVICE_PRIVATE_BASE)
#define SENSOR_STRING_TYPE_WRIST_TILT "com.google.android_wear.wrist.tilt"

__END_DECLS

#endif  /* ANDROID_CLOCKWORK_SENSOR_INTERFACE_H */
