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

CURRENT_DIRECTORY := $(call my-dir)

include $(CLEAR_VARS)

MAJOR_VERSION := $(shell echo $(PLATFORM_VERSION) | cut -f1 -d.)
MINOR_VERSION := $(shell echo $(PLATFORM_VERSION) | cut -f2 -d.)

VERSION_KK := $(shell test $(MAJOR_VERSION) -eq 4 -a $(MINOR_VERSION) -eq 4 && echo true)
VERSION_L := $(shell test $(MAJOR_VERSION) -eq 5 && echo true)

ifeq ($(VERSION_KK),true)
export ST_HAL_ANDROID_VERSION=0
DEFCONFIG := android_KK_defconfig
endif
ifeq ($(VERSION_L),true)
export ST_HAL_ANDROID_VERSION=1
DEFCONFIG := android_L_defconfig
endif

ifneq ("$(wildcard $(CURRENT_DIRECTORY)/lib/FUFD_CustomTilt/FUFD_CustomTilt*)","")
export ST_HAL_HAS_FDFD_LIB=y
else
export ST_HAL_HAS_FDFD_LIB=n
endif

ifneq ("$(wildcard $(CURRENT_DIRECTORY)/lib/iNemoEngine_gbias_Estimation/iNemoEngine_gbias_Estimation*)","")
export ST_HAL_HAS_GBIAS_LIB=y
else
export ST_HAL_HAS_GBIAS_LIB=n
endif

ifneq ("$(wildcard $(CURRENT_DIRECTORY)/lib/iNemoEngine_GeoMag_Fusion/iNemoEngine_GeoMag_Fusion*)","")
export ST_HAL_HAS_GEOMAG_LIB=y
else
export ST_HAL_HAS_GEOMAG_LIB=n
endif

ifneq ("$(wildcard $(CURRENT_DIRECTORY)/lib/iNemoEngine_SensorFusion/iNemoEngine_SensorFusion*)","")
export ST_HAL_HAS_9X_6X_LIB=y
else
export ST_HAL_HAS_9X_6X_LIB=n
endif

ifneq ("$(wildcard $(CURRENT_DIRECTORY)/lib/STCompass/STCompass*)","")
export ST_HAL_HAS_COMPASS_LIB=y
else
export ST_HAL_HAS_COMPASS_LIB=n
endif

export ST_HAL_PATH=$(CURRENT_DIRECTORY)

all_modules:
ifeq (${menuconfig},y)
ifeq ("$(wildcard $(CURRENT_DIRECTORY)/.config)","")
$(error .config file not found! use `mm defconfig=y` first.)
endif

	gcc -o $(CURRENT_DIRECTORY)/tools/mkconfig $(CURRENT_DIRECTORY)/tools/mkconfig.c $(CURRENT_DIRECTORY)/tools/cfgdefine.c
	$(CURRENT_DIRECTORY)/tools/kconfig-mconf $(CURRENT_DIRECTORY)/Kconfig
	$(CURRENT_DIRECTORY)/tools/mkconfig $(CURRENT_DIRECTORY)/ > $(CURRENT_DIRECTORY)/configuration.h
else
ifeq (${defconfig},y)
	cp $(CURRENT_DIRECTORY)/src/$(DEFCONFIG) $(CURRENT_DIRECTORY)/.config
	$(CURRENT_DIRECTORY)/tools/mkconfig $(CURRENT_DIRECTORY)/ > $(CURRENT_DIRECTORY)/configuration.h
else
ifeq (${clean},y)
ifneq ("$(wildcard $(CURRENT_DIRECTORY)/.config)","")
	rm $(CURRENT_DIRECTORY)/.config
endif
ifneq ("$(wildcard $(CURRENT_DIRECTORY)/configuration.h)","")
	rm $(CURRENT_DIRECTORY)/configuration.h
endif
ifneq ("$(wildcard $(CURRENT_DIRECTORY)/.config.old)","")
	rm $(CURRENT_DIRECTORY)/.config.old
endif
else
ifeq ("$(wildcard $(CURRENT_DIRECTORY)/.config)","")
$(error .config file not found! use `mm defconfig=y` first.)
endif

$(shell $(CURRENT_DIRECTORY)/tools/mkconfig $(CURRENT_DIRECTORY)/ > $(CURRENT_DIRECTORY)/configuration.h)
include $(call all-makefiles-under, $(CURRENT_DIRECTORY))

endif
endif
endif

endif # !TARGET_SIMULATOR
