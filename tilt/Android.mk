# Copyright (C) 2015 Intel Corp
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
#

LOCAL_PATH := $(call my-dir)

# HAL module implemenation, not prelinked, and stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.board>.so
include $(CLEAR_VARS)

# ANDROID version check
MAJOR_VERSION := $(shell echo $(PLATFORM_VERSION) | cut -f1 -d.)
MINOR_VERSION := $(shell echo $(PLATFORM_VERSION) | cut -f2 -d.)

VERSION_JB := $(shell test $(MAJOR_VERSION) -eq 4 -a $(MINOR_VERSION) -eq 1 && echo true)
VERSION_JB := $(shell test $(MAJOR_VERSION) -eq 4 -a $(MINOR_VERSION) -eq 2 && echo true)
VERSION_JB_MR2 := $(shell test $(MAJOR_VERSION) -eq 4 -a $(MINOR_VERSION) -eq 3 && echo true)
VERSION_KK := $(shell test $(MAJOR_VERSION) -eq 4 -a $(MINOR_VERSION) -eq 4 && echo true)
VERSION_L := $(shell test $(MAJOR_VERSION) -eq 5 && echo true)
VERSION_M := $(shell test $(MAJOR_VERSION) -eq 6 && echo true)
#ANDROID version check END

LOCAL_MODULE := sensor_tilt.robby

LOCAL_PRELINK_MODULE := false
LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional
# TODO: remove LOG_NDEBUG=0 for production builds, keep it during integration
LOCAL_CFLAGS := -DLOG_TAG=\"TiltSensor\" -DANDROID_VERSION=$(MAJOR_VERSION)

ifeq ($(VERSION_JB),true)
LOCAL_CFLAGS += -DANDROID_JB
endif

ifeq ($(VERSION_JBMR2),true)
LOCAL_CFLAGS += -DANDROID_JBMR2
#hal version is greater than and equal 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GE_1_0
endif

ifeq ($(VERSION_KK),true)
LOCAL_CFLAGS += -DANDROID_KK
#hal version is greater than and equal 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GE_1_0
#hal version is greater than 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GT_1_0
endif

ifeq ($(VERSION_L),true)
LOCAL_CFLAGS += -DANDROID_L
#hal version is greater than and equal 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GE_1_0
#hal version is greater than 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GT_1_0
endif

ifeq ($(VERSION_M),true)
LOCAL_CFLAGS += -DANDROID_M
#hal version is greater than and equal 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GE_1_0
#hal version is greater than 1_0
LOCAL_CFLAGS += -DHAL_VERSION_GT_1_0
endif

##LOCAL_C_INCLUDES += hardware/invensense/libsensors_iio
LOCAL_SRC_FILES := \
    sensors.cpp \
    InputEventReader.cpp \
    TiltSensor.cpp \
    SensorBase.cpp

LOCAL_SHARED_LIBRARIES := liblog libutils libdl

include $(BUILD_SHARED_LIBRARY)

