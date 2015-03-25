#
# Copyright (C) 2013-2015 STMicroelectronics
# Denis Ciocca - Motion MEMS Product Div.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#/

ifneq ($(TARGET_SIMULATOR),true)

LOCAL_PATH := $(call my-dir)
ST_HAL_ROOT_PATH := $(call my-dir)

include $(CLEAR_VARS)
include $(ST_HAL_ROOT_PATH)/../.config

LOCAL_PRELINK_MODULE := false

ifdef TARGET_DEVICE
LOCAL_MODULE := libsensors_iio.$(TARGET_DEVICE)
else
LOCAL_MODULE := libsensors_iio.default
endif

LOCAL_MODULE_RELATIVE_PATH := hw

LOCAL_MODULE_OWNER := STMicroelectronics

LOCAL_C_INCLUDES := $(LOCAL_PATH)/

LOCAL_CFLAGS += -DLOG_TAG=\"SensorHAL\"


ifeq ($(DEBUG),y)
LOCAL_CFLAGS += -g -O0
endif


ifdef CONFIG_ST_HAL_HAS_6AX_FUSION
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../lib/iNemoEngine_SensorFusion
LOCAL_STATIC_LIBRARIES += iNemoEngine_SensorFusion
else
ifdef CONFIG_ST_HAL_HAS_9AX_FUSION
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../lib/iNemoEngine_SensorFusion
LOCAL_STATIC_LIBRARIES += iNemoEngine_SensorFusion
endif
endif

ifdef CONFIG_ST_HAL_HAS_GYRO_GBIAS_ESTIMATION
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../lib/iNemoEngine_gbias_Estimation
LOCAL_STATIC_LIBRARIES += iNemoEngine_gbias_Estimation
endif

ifdef CONFIG_ST_HAL_HAS_TILT_FU_FD
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../lib/FUFD_CustomTilt
LOCAL_STATIC_LIBRARIES += FUFD_CustomTilt
endif

ifdef CONFIG_ST_HAL_HAS_GEOMAG_FUSION
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../lib/iNemoEngine_GeoMag_Fusion
LOCAL_STATIC_LIBRARIES += iNemoEngine_GeoMag_Fusion
endif

ifdef CONFIG_ST_HAL_HAS_MAGN_CALIB
LOCAL_C_INCLUDES += $(LOCAL_PATH)/../lib/STCompass
LOCAL_STATIC_LIBRARIES += STCompass
endif


LOCAL_SRC_FILES := \
		iio_utils.c \
		SensorHAL.cpp \
		CircularBuffer.cpp \
		SensorBase.cpp \
		HWSensorBase.cpp \
		SWSensorBase.cpp

ifdef CONFIG_ST_HAL_ACCEL_ENABLED
LOCAL_SRC_FILES += Accelerometer.cpp
endif

ifdef CONFIG_ST_HAL_MAGN_ENABLED
LOCAL_SRC_FILES += Magnetometer.cpp
endif

ifdef CONFIG_ST_HAL_GYRO_ENABLED
LOCAL_SRC_FILES += Gyroscope.cpp
endif

ifdef CONFIG_ST_HAL_STEP_DETECTOR_ENABLED
LOCAL_SRC_FILES += StepDetector.cpp
endif

ifdef CONFIG_ST_HAL_STEP_COUNTER_ENABLED
LOCAL_SRC_FILES += StepCounter.cpp
endif

ifdef CONFIG_ST_HAL_SIGN_MOTION_ENABLED
LOCAL_SRC_FILES += SignificantMotion.cpp
endif

ifdef CONFIG_ST_HAL_TILT_ENABLED
LOCAL_SRC_FILES += TiltSensor.cpp
endif

ifdef CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED
LOCAL_SRC_FILES += SWMagnetometerUncalibrated.cpp
endif

ifdef CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED
LOCAL_SRC_FILES += SWGyroscopeUncalibrated.cpp
endif

ifdef CONFIG_ST_HAL_PRESSURE_ENABLED
LOCAL_SRC_FILES += Pressure.cpp
endif

ifdef CONFIG_ST_HAL_HAS_GEOMAG_FUSION
LOCAL_SRC_FILES += SWAccelMagnFusion6X.cpp
endif

ifdef CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED
LOCAL_SRC_FILES += SWGeoMagRotationVector.cpp
endif

ifdef CONFIG_ST_HAL_HAS_6AX_FUSION
LOCAL_SRC_FILES += SWAccelGyroFusion6X.cpp
endif

ifdef CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED
LOCAL_SRC_FILES += SWGameRotationVector.cpp
endif

ifdef CONFIG_ST_HAL_HAS_9AX_FUSION
LOCAL_SRC_FILES += SWAccelMagnGyroFusion9X.cpp
endif

ifdef CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED
LOCAL_SRC_FILES += SWRotationVector.cpp
endif

ifdef CONFIG_ST_HAL_ORIENTATION_AP_ENABLED
LOCAL_SRC_FILES += SWOrientation.cpp
endif

ifdef CONFIG_ST_HAL_GRAVITY_AP_ENABLED
LOCAL_SRC_FILES += SWGravity.cpp
endif

ifdef CONFIG_ST_HAL_LINEAR_AP_ENABLED
LOCAL_SRC_FILES += SWLinearAccel.cpp
endif


LOCAL_SHARED_LIBRARIES := liblog libcutils libutils libdl libc

LOCAL_MODULE_TAGS := optional

include $(BUILD_SHARED_LIBRARY)

include $(call all-makefiles-under, $(LOCAL_PATH))

endif # !TARGET_SIMULATOR
