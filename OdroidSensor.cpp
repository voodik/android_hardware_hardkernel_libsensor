/*
 * Copyright (C) 2008 The Android Open Source Project
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
#include <poll.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/select.h>
#include <cutils/log.h>

#include "OdroidSensor.h"

/*****************************************************************************/

OdroidSensor::OdroidSensor()
	: SensorBase(NULL, "ODROIDSensor"),
	  mEnabled(0),
	  mInputReader(4),
	  mHasPendingEvent(false)
{
	mPendingEvent.version = sizeof(sensors_event_t);
	mPendingEvent.sensor = ID_A;
	mPendingEvent.type = SENSOR_TYPE_ACCELEROMETER;
	memset(mPendingEvent.data, 0, sizeof(mPendingEvent.data));

	if (data_fd) {
		strcpy(input_sysfs_path, "/sys/class/input/");
		strcat(input_sysfs_path, input_name);
		strcat(input_sysfs_path, "/device/");
		input_sysfs_path_len = strlen(input_sysfs_path);
		enable(0, 1);
	}
}

OdroidSensor::~OdroidSensor() {
	if (mEnabled) {
		enable(0, 0);
	}
}

int OdroidSensor::setDelay(int32_t handle, int64_t ns)
{
	int fd;

	strcpy(&input_sysfs_path[input_sysfs_path_len], "poll_delay");

	fd = open(input_sysfs_path, O_RDWR);
	if (fd >= 0) {
		char buf[80];
		sprintf(buf, "%lld", ns);
		write(fd, buf, strlen(buf)+1);
		close(fd);
		return 0;
	}
	ALOGE("OdroidSensor::setDelay sysfs = %s, fd = %d", input_sysfs_path, fd);
	return -1;
}

int OdroidSensor::enable(int32_t handle, int en)
{
	int flags = en ? 1 : 0;

	if (flags != mEnabled) {
		int fd;

		strcpy(&input_sysfs_path[input_sysfs_path_len], "enable");

		fd = open(input_sysfs_path, O_RDWR);
		if (fd >= 0) {
			char buf[2];
			int err;

			buf[1] = 0;
			if (flags)	buf[0] = '1';
			else		buf[0] = '0';

			err = write(fd, buf, sizeof(buf));

			close(fd);

			mEnabled = flags;
			return 0;
		}
		ALOGE("OdroidSensor::enable sysfs = %s, fd = %d", input_sysfs_path, fd);
		return -1;
	}
	return 0;
}

bool OdroidSensor::hasPendingEvents() const {
	return mHasPendingEvent;
}

int OdroidSensor::readEvents(sensors_event_t* data, int count)
{
	if (count < 1)
		return -EINVAL;

	if (mHasPendingEvent) {
		mHasPendingEvent = false;
		mPendingEvent.timestamp = getTimestamp();
		*data = mPendingEvent;
		return mEnabled ? 1 : 0;
	}

	ssize_t n = mInputReader.fill(data_fd);
	if (n < 0)
		return n;

	int numEventReceived = 0;
	input_event const* event;

	while (count && mInputReader.readEvent(&event)) {
		int type = event->type;
		if (type == EV_ABS) {
			float value = event->value;
			if (event->code == EVENT_TYPE_ACCEL_X) {
				mPendingEvent.acceleration.x = (value * CONVERT_A_X);
			} else if (event->code == EVENT_TYPE_ACCEL_Y) {
				mPendingEvent.acceleration.y = (value * CONVERT_A_Y);
			} else if (event->code == EVENT_TYPE_ACCEL_Z) {
				mPendingEvent.acceleration.z = (value * CONVERT_A_Z);
			}
		} else if (type == EV_SYN) {
			mPendingEvent.timestamp = timevalToNano(event->time);
			if (mEnabled) {
				*data++ = mPendingEvent;
				numEventReceived++;
				count--;
ALOGE("event->x = %f, event->y = %f, event->z = %f\n",  mPendingEvent.acceleration.x,
							mPendingEvent.acceleration.y,
							mPendingEvent.acceleration.z);
			}
		} else {
			ALOGE("OdroidSensor: unknown event (type=%d, code=%d)",
			type, event->code);
		}
		mInputReader.next();
	}

	return numEventReceived;
}
