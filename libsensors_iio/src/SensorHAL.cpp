/*
 * STMicroelectronics SensorHAL core
 *
 * Version 3.1.0
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <endian.h>

#include "SensorHAL.h"
#include "Accelerometer.h"
#include "Magnetometer.h"
#include "Gyroscope.h"

#ifdef CONFIG_ST_HAL_STEP_DETECTOR_ENABLED
#include "StepDetector.h"
#endif /* CONFIG_ST_HAL_STEP_DETECTOR_ENABLED */

#ifdef CONFIG_ST_HAL_STEP_COUNTER_ENABLED
#include "StepCounter.h"
#endif /* CONFIG_ST_HAL_STEP_COUNTER_ENABLED */

#ifdef CONFIG_ST_HAL_SIGN_MOTION_ENABLED
#include "SignificantMotion.h"
#endif /* CONFIG_ST_HAL_SIGN_MOTION_ENABLED */

#ifdef CONFIG_ST_HAL_TILT_ENABLED
#include "TiltSensor.h"
#endif /* CONFIG_ST_HAL_TILT_ENABLED */

#ifdef CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED
#include "SWMagnetometerUncalibrated.h"
#endif /* CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED */

#ifdef CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED
#include "SWGyroscopeUncalibrated.h"
#endif /* CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED */

#ifdef CONFIG_ST_HAL_PRESSURE_ENABLED
#include "Pressure.h"
#endif /* CONFIG_ST_HAL_PRESSURE_ENABLED */

#ifdef ST_HAL_NEEDS_GEOMAG_FUSION
#include "SWAccelMagnFusion6X.h"
#endif /* ST_HAL_NEEDS_GEOMAG_FUSION */

#ifdef CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED
#include "SWGeoMagRotationVector.h"
#endif /* CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED */

#ifdef ST_HAL_NEEDS_6AX_FUSION
#include "SWAccelGyroFusion6X.h"
#endif /* ST_HAL_NEEDS_6AX_FUSION */

#ifdef CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED
#include "SWGameRotationVector.h"
#endif /* CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED */

#ifdef ST_HAL_NEEDS_9AX_FUSION
#include "SWAccelMagnGyroFusion9X.h"
#endif /* ST_HAL_NEEDS_9AX_FUSION */

#ifdef CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED
#include "SWRotationVector.h"
#endif /* CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED */

#ifdef CONFIG_ST_HAL_ORIENTATION_AP_ENABLED
#include "SWOrientation.h"
#endif /* CONFIG_ST_HAL_ORIENTATION_AP_ENABLED */

#ifdef CONFIG_ST_HAL_GRAVITY_AP_ENABLED
#include "SWGravity.h"
#endif /* CONFIG_ST_HAL_GRAVITY_AP_ENABLED */

#ifdef CONFIG_ST_HAL_LINEAR_AP_ENABLED
#include "SWLinearAccel.h"
#endif /* CONFIG_ST_HAL_LINEAR_AP_ENABLED */

struct STSensorHAL_iio_devices_data {
	char *iio_sysfs_path;
	char *device_name;
	char *android_name;
	unsigned int dev_id;
	int sensor_type;

	bool wake_up_sensor;

	int num_channels;
	struct iio_channel_info *channels;
	struct iio_scale_available sa;

	unsigned int hw_fifo_len;
	float power_consumption;

	struct iio_sampling_frequency_available sfa;
} typedef STSensorHAL_iio_devices_data;

/*
 * ST_sensors_supported: ST sensors data used for discovery procedure
 * @driver_name: IIO device name.
 * @android_name: name showed in Android OS.
 * @sensor_type: Android sensor type.
 * @power_consumption: sensor power consumption in mA.
 */
static const struct ST_sensors_supported {
	const char *driver_name;
	const char *android_name;
	int sensor_type;
	float power_consumption;
} ST_sensors_supported[] = {
#ifdef CONFIG_ST_HAL_ACCEL_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, ACCEL_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Accelerometer Sensor",
	.sensor_type = SENSOR_TYPE_ACCELEROMETER,
	.power_consumption = 240E-3f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_ACCEL_ENABLED */
#ifdef CONFIG_ST_HAL_MAGN_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, MAGN_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Magnetometer Sensor",
	.sensor_type = SENSOR_TYPE_GEOMAGNETIC_FIELD,
	.power_consumption = 2.0f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_MAGN_ENABLED */
#ifdef CONFIG_ST_HAL_GYRO_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, GYRO_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Gyroscope Sensor",
	.sensor_type = SENSOR_TYPE_GYROSCOPE,
	.power_consumption = 1.25f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_GYRO_ENABLED */
#ifdef CONFIG_ST_HAL_STEP_DETECTOR_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, STEP_DETECTOR_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Step Detector Sensor",
	.sensor_type = SENSOR_TYPE_STEP_DETECTOR,
	.power_consumption = 240E-3f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_STEP_DETECTOR_ENABLED */
#ifdef CONFIG_ST_HAL_STEP_COUNTER_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, STEP_COUNTER_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Step Counter Sensor",
	.sensor_type = SENSOR_TYPE_STEP_COUNTER,
	.power_consumption = 240E-3f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_STEP_COUNTER_ENABLED */
#ifdef CONFIG_ST_HAL_SIGN_MOTION_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, SIGN_MOTION_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Significant Motion Sensor",
	.sensor_type = SENSOR_TYPE_SIGNIFICANT_MOTION,
	.power_consumption = 240E-3f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_SIGN_MOTION_ENABLED */
#ifdef CONFIG_ST_HAL_TILT_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, TILT_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Tilt Sensor",
	.sensor_type = SENSOR_TYPE_TILT_DETECTOR,
	.power_consumption = 240E-3f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_TILT_ENABLED */
#ifdef CONFIG_ST_HAL_PRESSURE_ENABLED
#ifdef CONFIG_ST_HAL_LSM6DS3_ENABLED
	{
	.driver_name = CONCATENATE_STRING(ST_SENSORS_LIST_1, PRESSURE_NAME_SUFFIX_IIO),
	.android_name = "LSM6DS3 Pressure Sensor",
	.sensor_type = SENSOR_TYPE_PRESSURE,
	.power_consumption = 40E-3f,
	},
#endif /* CONFIG_ST_HAL_LSM6DS3_ENABLED */
#endif /* CONFIG_ST_HAL_PRESSURE_ENABLED */
};

static const struct ST_virtual_sensors_list {
	int sensor_type;
} ST_virtual_sensors_list[] = {
#ifdef ST_HAL_NEEDS_GEOMAG_FUSION
	{ .sensor_type = SENSOR_TYPE_ST_ACCEL_MAGN_FUSION6X },
#endif /* ST_HAL_NEEDS_GEOMAG_FUSION */
#ifdef CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED },
#endif /* CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED */
#ifdef CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED },
#endif /* CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED */
#ifdef CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR },
#endif /* CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED */
#ifdef ST_HAL_NEEDS_6AX_FUSION
	{ .sensor_type = SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X },
#endif /* ST_HAL_NEEDS_6AX_FUSION */
#ifdef CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_GAME_ROTATION_VECTOR },
#endif /* CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED */
#ifdef ST_HAL_NEEDS_9AX_FUSION
	{ .sensor_type = SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X },
#endif /* ST_HAL_NEEDS_9AX_FUSION */
#ifdef CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_ROTATION_VECTOR },
#endif /* CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED */
#ifdef CONFIG_ST_HAL_ORIENTATION_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_ORIENTATION },
#endif /* CONFIG_ST_HAL_ORIENTATION_AP_ENABLED */
#ifdef CONFIG_ST_HAL_GRAVITY_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_GRAVITY },
#endif /* CONFIG_ST_HAL_GRAVITY_AP_ENABLED */
#ifdef CONFIG_ST_HAL_LINEAR_AP_ENABLED
	{ .sensor_type = SENSOR_TYPE_LINEAR_ACCELERATION },
#endif /* CONFIG_ST_HAL_LINEAR_AP_ENABLED */
};

/*
 * st_hal_create_virtual_class_sensor: instantiate virtual sensor class.
 * @sensor_type: android sensor type.
 * @handle: android handle number.
 * @android_pipe_fd: file descriptor used to push new data.
 *
 * Return value: sensor class pointer on success, NULL pointer if fail.
 */
static SensorBase* st_hal_create_virtual_class_sensor(int sensor_type, int handle, int android_pipe_fd)
{
	SensorBase *sb = NULL;

	switch (sensor_type) {
#ifdef CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED
	case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
		sb = new SWMagnetometerUncalibrated("Magnetometer Uncalibrated Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_MAGN_UNCALIB_AP_ENABLED */
#ifdef CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED
	case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
		sb = new SWGyroscopeUncalibrated("Gyroscope Uncalibrated Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_GYRO_UNCALIB_AP_ENABLED */
#ifdef ST_HAL_NEEDS_GEOMAG_FUSION
	case SENSOR_TYPE_ST_ACCEL_MAGN_FUSION6X:
		sb = new SWAccelMagnFusion6X("Accel-Magn Fusion 6X", handle, android_pipe_fd);
		break;
#endif /* ST_HAL_NEEDS_GEOMAG_FUSION */
#ifdef CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED
	case SENSOR_TYPE_GEOMAGNETIC_ROTATION_VECTOR:
		sb = new SWGeoMagRotationVector("iNemoEngine GeoMagnetic Rotation Vector Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_GEOMAG_ROT_VECTOR_AP_ENABLED */
#ifdef ST_HAL_NEEDS_6AX_FUSION
	case SENSOR_TYPE_ST_ACCEL_GYRO_FUSION6X:
		sb = new SWAccelGyroFusion6X("Accel-Gyro Fusion 6X", handle, android_pipe_fd);
		break;
#endif /* ST_HAL_NEEDS_6AX_FUSION */
#ifdef CONFIG_ST_HAL_GAME_ROT_VECTOR_AP_ENABLED
	case SENSOR_TYPE_GAME_ROTATION_VECTOR:
		sb = new SWGameRotationVector("iNemoEngine Game Rotation Vector Sensor", handle, android_pipe_fd);
		break;
#endif /* SENSOR_TYPE_GAME_ROTATION_VECTOR */
#ifdef ST_HAL_NEEDS_9AX_FUSION
	case SENSOR_TYPE_ST_ACCEL_MAGN_GYRO_FUSION9X:
		sb = new SWAccelMagnGyroFusion9X("Accel-Magn-Gyro Fusion 9X", handle, android_pipe_fd);
		break;
#endif /* ST_HAL_NEEDS_9AX_FUSION */
#ifdef CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED
	case SENSOR_TYPE_ROTATION_VECTOR:
		sb = new SWRotationVector("iNemoEngine Rotation Vector Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_ROT_VECTOR_AP_ENABLED */
#ifdef CONFIG_ST_HAL_ORIENTATION_AP_ENABLED
	case SENSOR_TYPE_ORIENTATION:
		sb = new SWOrientation("iNemoEngine Orientation Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_ORIENTATION_AP_ENABLED */
#ifdef CONFIG_ST_HAL_GRAVITY_AP_ENABLED
	case SENSOR_TYPE_GRAVITY:
		sb = new SWGravity("iNemoEngine Gravity Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_GRAVITY_AP_ENABLED */
#ifdef CONFIG_ST_HAL_LINEAR_AP_ENABLED
	case SENSOR_TYPE_LINEAR_ACCELERATION:
		sb = new SWLinearAccel("iNemoEngine Linear Acceleration Sensor", handle, android_pipe_fd);
		break;
#endif /* CONFIG_ST_HAL_LINEAR_AP_ENABLED */
	default:
		(int)handle;
		(int)android_pipe_fd;
		return NULL;
	}

	return sb->IsValidClass() ? sb : NULL;
}

/*
 * st_hal_create_class_sensor: instantiate sensor class.
 * @data: device data.
 * @handle: android handle number.
 * @android_pipe_fd: file descriptor used to push new data.
 *
 * Return value: sensor class pointer on success, NULL pointer if fail.
 */
static SensorBase* st_hal_create_class_sensor(STSensorHAL_iio_devices_data *data, int handle, int android_pipe_fd)
{
	SensorBase *sb = NULL;
	struct HWSensorBaseCommonData class_data;

	if ((strlen(data->iio_sysfs_path) + 1 > HW_SENSOR_BASE_IIO_SYSFS_PATH_MAX) ||
			(strlen(data->device_name) + 1 > HW_SENSOR_BASE_IIO_DEVICE_NAME_MAX) ||
			(data->num_channels > HW_SENSOR_BASE_MAX_CHANNELS))
		return NULL;

	memcpy(class_data.device_name, data->device_name, strlen(data->device_name) + 1);
	memcpy(class_data.iio_sysfs_path, data->iio_sysfs_path, strlen(data->iio_sysfs_path) + 1);
	memcpy(&class_data.sa, &data->sa, sizeof(class_data.sa));
	memcpy(class_data.channels, data->channels, data->num_channels * sizeof(class_data.channels[0]));

	class_data.iio_dev_num = data->dev_id;
	class_data.num_channels = data->num_channels;

	switch (data->sensor_type) {
#ifdef CONFIG_ST_HAL_ACCEL_ENABLED
	case SENSOR_TYPE_ACCELEROMETER:
		sb = new Accelerometer(&class_data, data->android_name, &data->sfa,
				handle, data->hw_fifo_len, android_pipe_fd,
				data->power_consumption, data->wake_up_sensor);
		break;
#endif /* CONFIG_ST_HAL_ACCEL_ENABLED */
#ifdef CONFIG_ST_HAL_MAGN_ENABLED
	case SENSOR_TYPE_MAGNETIC_FIELD:
		sb = new Magnetometer(&class_data, data->android_name, &data->sfa,
				handle, data->hw_fifo_len, android_pipe_fd,
				data->power_consumption, data->wake_up_sensor);
		break;
#endif /* CONFIG_ST_HAL_MAGN_ENABLED */
#ifdef CONFIG_ST_HAL_GYRO_ENABLED
	case SENSOR_TYPE_GYROSCOPE:
		sb = new Gyroscope(&class_data, data->android_name, &data->sfa,
				handle, data->hw_fifo_len, android_pipe_fd,
				data->power_consumption, data->wake_up_sensor);
		break;
#endif /* CONFIG_ST_HAL_GYRO_ENABLED */
#ifdef CONFIG_ST_HAL_STEP_DETECTOR_ENABLED
	case SENSOR_TYPE_STEP_DETECTOR:
		sb = new StepDetector(&class_data, data->android_name,
				handle, data->hw_fifo_len, android_pipe_fd,
				data->power_consumption, data->wake_up_sensor);
		break;
#endif /* CONFIG_ST_HAL_STEP_DETECTOR_ENABLED */
#ifdef CONFIG_ST_HAL_STEP_COUNTER_ENABLED
	case SENSOR_TYPE_STEP_COUNTER:
		sb = new StepCounter(&class_data, data->android_name,
				handle, data->hw_fifo_len, android_pipe_fd,
				data->power_consumption, data->wake_up_sensor);
		break;
#endif /* CONFIG_ST_HAL_STEP_COUNTER_ENABLED */
#ifdef CONFIG_ST_HAL_SIGN_MOTION_ENABLED
	case SENSOR_TYPE_SIGNIFICANT_MOTION:
		sb = new SignMotion(&class_data, data->android_name,
				handle, android_pipe_fd, data->power_consumption);
		break;
#endif /* CONFIG_ST_HAL_SIGN_MOTION_ENABLED */
#ifdef CONFIG_ST_HAL_TILT_ENABLED
	case SENSOR_TYPE_TILT_DETECTOR:
		sb = new TiltSensor(&class_data, data->android_name,
				handle, android_pipe_fd, data->power_consumption);
		break;
#endif /* CONFIG_ST_HAL_TILT_ENABLED */
#ifdef CONFIG_ST_HAL_PRESSURE_ENABLED
	case SENSOR_TYPE_PRESSURE:
		sb = new Pressure(&class_data, data->android_name, &data->sfa,
				handle, data->hw_fifo_len, android_pipe_fd,
				data->power_consumption, data->wake_up_sensor);
		break;
#endif /* CONFIG_ST_HAL_PRESSURE_ENABLED */
	default:
		return NULL;
	}

	return sb->IsValidClass() ? sb : NULL;
}

/*
 * st_hal_set_fullscale: change fullscale of iio device sensor.
 * @iio_sysfs_path: iio device driver sysfs path.
 * @sensor_type: android sensor type.
 * @sa: scale available structure.
 * @channels: iio channels informations.
 * @num_channels: number of iio channels.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_set_fullscale(char *iio_sysfs_path, int sensor_type,
		struct iio_scale_available *sa, struct iio_channel_info *channels, int num_channels)
{
	int err, i, c, max_value;

	switch (sensor_type) {
#ifdef CONFIG_ST_HAL_ACCEL_ENABLED
	case SENSOR_TYPE_ACCELEROMETER:
		max_value = CONFIG_ST_HAL_ACCEL_RANGE;
		break;
#endif /* CONFIG_ST_HAL_ACCEL_ENABLED */
#ifdef CONFIG_ST_HAL_MAGN_ENABLED
	case SENSOR_TYPE_MAGNETIC_FIELD:
		max_value = CONFIG_ST_HAL_MAGN_RANGE;
		break;
#endif /* CONFIG_ST_HAL_MAGN_ENABLED */
#ifdef CONFIG_ST_HAL_GYRO_ENABLED
	case SENSOR_TYPE_GYROSCOPE:
		max_value = CONFIG_ST_HAL_GYRO_RANGE;
		break;
#endif /* CONFIG_ST_HAL_GYRO_ENABLED */
	default:
		return -EINVAL;
	}

	for (i = 0; i < (int)sa->num_available; i++) {
		if ((sa->values[i] * (pow(2, channels[0].bits_used - 1) - 1)) >= max_value)
			break;
	}
	if (i == (int)sa->num_available)
		i = sa->num_available - 1;

	err = iio_utils_set_scale(iio_sysfs_path, sa->values[i], sensor_type);
	if (err < 0)
		return err;

	for (c = 0; c < num_channels - 1; c++)
		channels[c].scale = sa->values[i];

	return 0;
}

/*
 * st_hal_load_iio_devices_data: read iio devices data.
 * @data: iio device data.
 *
 * Return value: number of sensors found on success, negative number if fail.
 */
static int st_hal_load_iio_devices_data(STSensorHAL_iio_devices_data *data)
{
	unsigned int index = 0;
	int err, iio_devices_num, i, n;
	struct iio_device iio_devices[ST_HAL_IIO_MAX_DEVICES];

	iio_devices_num = iio_utils_get_devices_name(iio_devices, ST_HAL_IIO_MAX_DEVICES);
	if (iio_devices_num <= 0) {
		ALOGE("Failed to read iio devices available into /sys/bus/iio/devices/ folder (errno: %d).", iio_devices_num);
		return iio_devices_num;
	}

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_VERBOSE)
	ALOGD("%d IIO devices available into /sys/bus/iio/devices/ folder.", iio_devices_num);
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

	for (i = 0; i < iio_devices_num; i++) {
		for (n = 0; n < ARRAY_SIZE(ST_sensors_supported); n++) {
			err = strncmp(iio_devices[i].name, ST_sensors_supported[n].driver_name,
							strlen(ST_sensors_supported[n].driver_name));
			if (err == 0)
				break;
		}
		if (n == ARRAY_SIZE(ST_sensors_supported)) {
#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_EXTRA_VERBOSE)
			ALOGD("\"%s\": IIO device not supported by sensor HAL.", iio_devices[i].name);
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */
			continue;
		}

		if (strcmp(&iio_devices[i].name[strlen(iio_devices[i].name) -
				strlen(ST_HAL_WAKEUP_SUFFIX_IIO)], ST_HAL_WAKEUP_SUFFIX_IIO) == 0)
			data[index].wake_up_sensor = true;
		else
			data[index].wake_up_sensor = false;

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_VERBOSE)
		ALOGD("\"%s\": IIO device found and supported. Wake-up sensor: %s", iio_devices[i].name, data[index].wake_up_sensor ? "yes" : "no" );
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

		err = asprintf(&data[index].iio_sysfs_path, "%siio:device%d",
						"/sys/bus/iio/devices/", iio_devices[i].dev_num);
		if (err < 0)
			continue;

		data[index].power_consumption = ST_sensors_supported[n].power_consumption;

		err = iio_utils_build_channel_array(data[index].iio_sysfs_path, &data[index].channels,
								&data[index].num_channels, true);
		if (err < 0) {
			ALOGE("\"%s\": failed to read IIO channels informations. (errno: %d)", iio_devices[i].name, err);
			goto st_hal_load_free_iio_sysfs_path;
		}

		err = iio_utils_enable_sensor(data[index].iio_sysfs_path, false);
		if (err < 0) {
			ALOGE("\"%s\": failed to disable sensor. (errno: %d)", iio_devices[i].name, err);
			goto st_hal_load_free_iio_channels;
		}

		if ((ST_sensors_supported[n].sensor_type != SENSOR_TYPE_STEP_DETECTOR) &&
				(ST_sensors_supported[n].sensor_type != SENSOR_TYPE_STEP_COUNTER) &&
					(ST_sensors_supported[n].sensor_type != SENSOR_TYPE_SIGNIFICANT_MOTION) &&
						(ST_sensors_supported[n].sensor_type != SENSOR_TYPE_TILT_DETECTOR)) {
			err = iio_utils_get_sampling_frequency_available(data[index].iio_sysfs_path, &data[index].sfa);
			if (err < 0)
				goto st_hal_load_free_iio_channels;

			err = iio_utils_get_scale_available(data[index].iio_sysfs_path, &data[index].sa,
									ST_sensors_supported[n].sensor_type);
			if (err < 0)
				goto st_hal_load_free_iio_channels;

			if (data[index].sa.num_available > 0) {
				err = st_hal_set_fullscale(data[index].iio_sysfs_path, ST_sensors_supported[n].sensor_type,
								&data[index].sa, data[index].channels, data[index].num_channels);
				if (err < 0) {
					ALOGE("\"%s\": failed to set device full-scale. (errno: %d)", iio_devices[i].name, err);
					goto st_hal_load_free_iio_channels;
				}
			}
		}

		err = asprintf(&data[index].device_name, "%s", iio_devices[i].name);
		if (err < 0)
			goto st_hal_load_free_iio_channels;

		err = asprintf(&data[index].android_name, "%s", ST_sensors_supported[n].android_name);
		if (err < 0)
			goto st_hal_load_free_device_name;

		data[index].hw_fifo_len = iio_utils_get_hw_fifo_lenght(data[index].iio_sysfs_path);
		data[index].sensor_type = ST_sensors_supported[n].sensor_type;
		data[index].dev_id = iio_devices[i].dev_num;

		index++;

		continue;

st_hal_load_free_device_name:
	free(data[index].device_name);
st_hal_load_free_iio_channels:
	free(data[index].channels);
st_hal_load_free_iio_sysfs_path:
	free(data[index].iio_sysfs_path);
	}

	if (index == 0)
		ALOGE("No IIO sensors found into /sys/bus/iio/devices/ folder.");

	return index;
}

/**
 * st_hal_dev_flush() - Android call this function to flush sensor batch data.
 * @dev: sensors device.
 * @handle: android sensor handle.
 *
 * Return value: 0 on success, negative number if fail.
 **/
static int st_hal_dev_flush(struct sensors_poll_device_1 *dev, int handle)
{
	STSensorHAL_data *hal_data = (STSensorHAL_data *)dev;
	ALOGD("st_hal_dev_flush type=%d", ((struct sensor_t) hal_data->sensor_t_list[handle]).type);
	/* One-shot sensor must return -EINVAL and not generate any flush complete metadata event */
	if (SENSOR_TYPE_SIGNIFICANT_MOTION == ((struct sensor_t) hal_data->sensor_t_list[handle]).type)
		return -EINVAL;

	return hal_data->sensor_classes[handle]->FlushData();
}

/**
 * st_hal_dev_batch() - Android O.S. calls this function to check and set batch mode
 * @dev: sensors device structure.
 * @handle: android sensor handle.
 * @flags: used for test the availability of batch mode.
 * @period_ns: time to batch (like setDelay(...)).
 * @timeout: 0 to disable batch mode.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_dev_batch(struct sensors_poll_device_1 *dev, int handle,
		int __attribute__((unused))flags, int64_t period_ns, int64_t timeout)
{
	STSensorHAL_data *hal_data = (STSensorHAL_data *)dev;

	return hal_data->sensor_classes[handle]->SetDelay(handle, period_ns, timeout);
}

/**
 * st_hal_dev_poll() - Android O.S. calls this function and waits until when new data are available
 * @dev: sensors device structure.
 * @data: data structure used to push data to the upper layer.
 * @count: maximum number of events in the same time.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_dev_poll(struct sensors_poll_device_t *dev,
						sensors_event_t *data, int count)
{
	int i, err, read_size;
	STSensorHAL_data *hal_data = (STSensorHAL_data *)dev;

	err = poll(&hal_data->android_pollfd, 1, -1);
	if (err <= 0)
		return 0;

	if (hal_data->android_pollfd.revents > 0) {
		read_size = read(hal_data->android_pollfd.fd, data, count * sizeof(struct sensors_event_t));
		if (read_size <= 0)
			return 0;
	} else
		return 0;

	return (read_size / sizeof(struct sensors_event_t));
}

/**
 * st_hal_dev_setDelay() - Set sensor polling rate
 * @dev: sensors device structure.
 * @handle: android sensor handle.
 * @ns: polling rate value expressed in nanoseconds.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_dev_setDelay(struct sensors_poll_device_t *dev, int handle, int64_t ns)
{
	STSensorHAL_data *hal_data = (STSensorHAL_data *)dev;

	return hal_data->sensor_classes[handle]->SetDelay(handle, ns, 0);
}

/**
 * st_hal_dev_activate() - Enable or Disable sensors
 * @dev: sensors device structure.
 * @handle: android sensor handle.
 * @enable: enable/ disable flag.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_dev_activate(struct sensors_poll_device_t *dev, int handle, int enabled)
{
	STSensorHAL_data *hal_data = (STSensorHAL_data *)dev;

	return hal_data->sensor_classes[handle]->Enable(handle, (bool)enabled);
}

/**
 * st_hal_dev_close() - Close device sensors module
 * @dev: sensors device structure.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_dev_close(struct hw_device_t *dev)
{
	int i;
	STSensorHAL_data *hal_data = (STSensorHAL_data *)dev;

	for (i = 0; i < (int)hal_data->sensor_available; i++)
		delete hal_data->sensor_classes[i];

	free(hal_data->threads);
	close(hal_data->android_pollfd.fd);
	free(hal_data->sensor_t_list);
	free(hal_data);

	return 0;
}

/**
 * st_hal_create_android_pipe() - Create dev_poll pipe
 * @hal_data: hal common data.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_create_android_pipe(STSensorHAL_data *hal_data)
{
	int err, pipe_fd[2];

	err = pipe(pipe_fd);
	if (err < 0)
		return err;

	fcntl(pipe_fd[0], F_SETFL, O_NONBLOCK);
	fcntl(pipe_fd[1], F_SETFL, O_NONBLOCK);

	hal_data->android_pollfd.events = POLLIN;
	hal_data->android_pollfd.fd = pipe_fd[0];

	return pipe_fd[1];
}

/**
 * open_sensors() - Open sensor device
 * see Android documentation.
 *
 * Return value: 0 on success, negative number if fail.
 */
static int st_hal_open_sensors(const struct hw_module_t *module,
				const char __attribute__((unused))*id,
						struct hw_device_t **device)
{
	bool real_sensor_class;
	STSensorHAL_data *hal_data;
	int sensor_class_valid_num =0 ;
	bool sensor_class_valid[ST_HAL_IIO_MAX_DEVICES];
	int type_dependencies[SENSOR_BASE_MAX_DEPENDENCY], type_index, type_sensor_trigger;
	SensorBase *sensor_class, *temp_sensor_class[ST_HAL_IIO_MAX_DEVICES];
	STSensorHAL_iio_devices_data iio_devices_data[ST_HAL_IIO_MAX_DEVICES];
	int err, i, c, android_write_pipe_fd, device_found_num, classes_available = 0, n = 0;
	bool temp_sensor_class_virtual[ST_HAL_IIO_MAX_DEVICES];

	hal_data = (STSensorHAL_data *)malloc(sizeof(STSensorHAL_data));
	if (!hal_data)
		return -ENOMEM;

	hal_data->sensor_available = 0;
	hal_data->poll_device.common.tag = HARDWARE_DEVICE_TAG;
	hal_data->poll_device.common.version = ST_HAL_IIO_DEVICE_API_VERSION;
	hal_data->poll_device.common.module = const_cast<hw_module_t*>(module);
	hal_data->poll_device.common.close = st_hal_dev_close;
	hal_data->poll_device.common.module->dso = hal_data;
	hal_data->poll_device.activate = st_hal_dev_activate;
	hal_data->poll_device.setDelay = st_hal_dev_setDelay;
	hal_data->poll_device.poll = st_hal_dev_poll;
	hal_data->poll_device.batch = st_hal_dev_batch;
	hal_data->poll_device.flush = st_hal_dev_flush;

	do_cal_data_loading(ACCEL_SINDEX, NON_WAKEUP);
	do_cal_data_loading(ACCEL_SINDEX, WAKEUP);
	do_cal_data_loading(GYRO_SINDEX, NON_WAKEUP);
	do_cal_data_loading(GYRO_SINDEX, WAKEUP);

	*device = &hal_data->poll_device.common;

	device_found_num = st_hal_load_iio_devices_data(iio_devices_data);
	if (device_found_num <= 0) {
		err = device_found_num;
		goto free_hal_data;
	}

	android_write_pipe_fd = st_hal_create_android_pipe(hal_data);
	if (android_write_pipe_fd < 0) {
		ALOGE("Failed to create Android pipe file.");
		err = device_found_num;
		goto free_hal_data;
	}

	for (i = 0; i < device_found_num; i++) {
		sensor_class = st_hal_create_class_sensor(&iio_devices_data[i], classes_available + 1, android_write_pipe_fd);

		free(iio_devices_data[i].iio_sysfs_path);
		free(iio_devices_data[i].android_name);
		free(iio_devices_data[i].channels);

		if (!sensor_class) {
			ALOGE("\"%s\": failed to create HW sensor class.", iio_devices_data[i].device_name);
			free(iio_devices_data[i].device_name);
			continue;
		}

		free(iio_devices_data[i].device_name);

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_VERBOSE)
	ALOGD("\"%s\": created HW class instance (sensor type: %d).", sensor_class->GetName(), sensor_class->GetType());
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

		temp_sensor_class[classes_available] = sensor_class;
		temp_sensor_class_virtual[classes_available] = false;
		sensor_class_valid[classes_available] = true;
		sensor_class_valid_num++;
		classes_available++;
	}
	if (classes_available == 0) {
		ALOGE("Failed to create HW sensors classes.");
		err = -ENODEV;
		goto close_android_pipe_fd;
	}

	for (i = 0; i < ARRAY_SIZE(ST_virtual_sensors_list); i++) {
		sensor_class = st_hal_create_virtual_class_sensor(ST_virtual_sensors_list[i].sensor_type, classes_available + 1, android_write_pipe_fd);
		if (!sensor_class) {
			ALOGE("Failed to create SW sensor class (sensor type: %d).", ST_virtual_sensors_list[i].sensor_type);
			continue;
		}

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_VERBOSE)
	if (sensor_class->GetType() < SENSOR_TYPE_ST_CUSTOM_NO_SENSOR)
		ALOGD("\"%s\": created SW class instance (sensor type: %d).", sensor_class->GetName(), sensor_class->GetType());
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

		temp_sensor_class[classes_available] = sensor_class;
		temp_sensor_class_virtual[classes_available] = true;
		sensor_class_valid[classes_available] = true;
		sensor_class_valid_num++;
		classes_available++;
	}

	for (i = 0; i < classes_available; i ++) {
		temp_sensor_class[i]->GetDepenciesTypeList(type_dependencies);
		type_index = 0;

		while((type_dependencies[type_index] > 0) && (type_index < SENSOR_BASE_MAX_DEPENDENCY)) {
			err = 0;

			for (c = 0; c < classes_available; c++) {
				if ((type_dependencies[type_index] == temp_sensor_class[c]->GetType()) && (sensor_class_valid[c])) {
					if (temp_sensor_class_virtual[i])
						err = ((SWSensorBase *)temp_sensor_class[i])->AddSensorDependency(temp_sensor_class[c]);
					else
						err = ((HWSensorBase *)temp_sensor_class[i])->AddSensorDependency(temp_sensor_class[c]);

					break;
				}
			}
			if ((c == classes_available) || (err < 0)) {
				ALOGE("\"%s\": failed to add dependency (sensor type dependency: %d).", temp_sensor_class[i]->GetName(), type_dependencies[type_index]);

				while (type_index > 0) {
					type_index--;

					for (c = 0; c < classes_available; c++) {
						if ((type_dependencies[type_index] == temp_sensor_class[c]->GetType()) && (sensor_class_valid[c])) {
							if (temp_sensor_class_virtual[i])
								((SWSensorBase *)temp_sensor_class[i])->RemoveSensorDependency(temp_sensor_class[c]);
							else
								((HWSensorBase *)temp_sensor_class[i])->RemoveSensorDependency(temp_sensor_class[c]);

							break;
						}
					}
				}

				sensor_class_valid_num--;
				sensor_class_valid[i] = false;
				goto failed_to_check_dependency;
			}

			type_index++;
		}

		type_sensor_trigger = temp_sensor_class[i]->GetSensorNeedTriggerType();
		if (type_sensor_trigger < 0)
			continue;

		err = 0;

		for (c = 0; c < classes_available; c++) {
			if (type_sensor_trigger == temp_sensor_class[c]->GetType()) {
				err = temp_sensor_class[c]->AddSensorToTrigger(temp_sensor_class[i]);
				break;
			}
		}
		if ((c == classes_available) || (err < 0)) {
			ALOGE("\"%s\": failed to add trigger (sensor trigger type: %d).", temp_sensor_class[i]->GetName(), type_sensor_trigger);
			sensor_class_valid_num--;
			sensor_class_valid[i] = false;
			break;
		}

failed_to_check_dependency:
		continue;
	}

	for (i = 0; i < classes_available; i++) {
		if (sensor_class_valid[i])
			hal_data->sensor_classes[temp_sensor_class[i]->GetHandle()] = temp_sensor_class[i];
	}

	hal_data->sensor_t_list = (struct sensor_t *)malloc(sensor_class_valid_num * sizeof(struct sensor_t));
	if (!hal_data->sensor_t_list) {
		err = -ENOMEM;
		goto destroy_classes;
	}

	hal_data->threads = (pthread_t *)malloc(sensor_class_valid_num * sizeof(pthread_t *));
	if (!hal_data->threads) {
		err = -ENOMEM;
		goto free_sensor_t_list;
	}

	for (i = 0; i < classes_available; i++) {
		if (sensor_class_valid[i]) {
			err = pthread_create(&hal_data->threads[i], NULL, &SensorBase::ThreadWork, (void *)temp_sensor_class[i]);
			if (err < 0)
				continue;

			real_sensor_class = hal_data->sensor_classes[temp_sensor_class[i]->GetHandle()]->FillSensor_tData(&hal_data->sensor_t_list[n]);
			if (!real_sensor_class)
				continue;

			n++;
		} else
			delete temp_sensor_class[i];
	}

	hal_data->sensor_available = n;

#if (CONFIG_ST_HAL_DEBUG_LEVEL >= ST_HAL_DEBUG_INFO)
	ALOGD("%d sensors available and ready.", n);
#endif /* CONFIG_ST_HAL_DEBUG_LEVEL */

	return 0;

free_sensor_t_list:
	free(hal_data->sensor_t_list);
destroy_classes:
	for (i = 0; i < classes_available; i ++)
		delete temp_sensor_class[i];

close_android_pipe_fd:
	close(android_write_pipe_fd);
	close(hal_data->android_pollfd.fd);
free_hal_data:
	free(hal_data);

	return err;
}

/**
 * get_sensors_list() - Get sensors list
 * @module: hardware specific informations.
 * @list: sensors list.
 *
 * Return value: number of sensors available.
 */
static int st_hal_get_sensors_list(struct sensors_module_t *module,
						struct sensor_t const **list)
{
	STSensorHAL_data *hal_data = (STSensorHAL_data *)module->common.dso;

	*list = (struct sensor_t const *)hal_data->sensor_t_list;

	return hal_data->sensor_available;
};

/*
 * struct hw_module_methods_t - Hardware module functions
 * see Android documentation.
 */
static struct hw_module_methods_t st_hal_sensors_module_methods = {
	open: st_hal_open_sensors
};

/*
 * struct sensors_module_t - Hardware module info
 * see Android documentation.
 */
struct sensors_module_t HAL_MODULE_INFO_SYM = {
	.common = {
		.tag = HARDWARE_MODULE_TAG,
		.module_api_version = SENSORS_MODULE_API_VERSION_0_1,
		.hal_api_version = 0,
		.id = SENSORS_HARDWARE_MODULE_ID,
		.name = "STMicroelectronics Sensors Module",
		.author = "STMicroelectronics",
		.methods = &st_hal_sensors_module_methods,
		.dso = NULL,
		.reserved = { },
	},
	.get_sensors_list = st_hal_get_sensors_list,
};
