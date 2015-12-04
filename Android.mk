# Copyright (C) 2008 The Android Open Source Project
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
# Modified 2015 by HARDKERNEL Co,. Ltd

LOCAL_PATH := $(call my-dir)

# HAL module implementation stored in
# hw/<SENSORS_HARDWARE_MODULE_ID>.<ro.product.borad>.so
include $(CLEAR_VARS)

ifeq ($(BOARD_HAVE_ODROID_SENSOR), true)

LOCAL_MODULE := sensors.$(TARGET_PRODUCT)

LOCAL_MODULE_PATH := $(TARGET_OUT_SHARED_LIBRARIES)/hw

LOCAL_MODULE_TAGS := optional

LOCAL_CFLAGS := -DLOG_TAG=\"Sensors\"

LOCAL_SRC_FILES :=	sensors.cpp		\
			SensorBase.cpp		\
			OdroidSensor.cpp	\
			InputEventReader.cpp

LOCAL_SHARED_LIBRARIES := liblog
LOCAL_SHARED_LIBRARIES += libcutils
LOCAL_SHARED_LIBRARIES += libutils
LOCAL_SHARED_LIBRARIES += libdl

include $(BUILD_SHARED_LIBRARY)

endif
