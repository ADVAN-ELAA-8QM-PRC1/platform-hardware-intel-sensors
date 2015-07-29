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

#ifndef __VSENSOR_API_H__
#define __VSENSOR_API_H__

#include <sys/types.h>
#include <stdbool.h>

static int init_flag;
static float grav[3];
static float lin_acc[3];

void vSensor_API_Initialization_6X(void *);
void vSensor_API_enable_6X(bool enable);
int vSensor_API_Run_6X(float *accel,
            float *gyro, long long timestamp);
int vSensor_API_Get_Quaternion_6X(float *rotation_vector);
int vSensor_API_Get_LinAcc_6X(float *lin_accel);
int vSensor_API_Get_Gravity_6X(float *garvity);

#endif  /* __VSENSOR_API_H__ */
