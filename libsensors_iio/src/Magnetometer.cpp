/*
 * STMicroelectronics Magnetometer Sensor Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include <fcntl.h>
#include <assert.h>
#include <signal.h>

#include "Magnetometer.h"

#ifdef CONFIG_ST_HAL_MAGN_CALIB_ENABLED
extern "C" {
	#include "STCompass_API.h"
}
#endif /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */

Magnetometer::Magnetometer(HWSensorBaseCommonData *data, const char *name,
		struct iio_sampling_frequency_available *sfa, int handle,
		unsigned int hw_fifo_len, int pipe_data_fd, float power_consumption, bool wakeup) :
			HWSensorBaseWithPollrate(data, name, sfa, handle,
			SENSOR_TYPE_MAGNETIC_FIELD, hw_fifo_len, pipe_data_fd, power_consumption)
{
	sensor_t_data.stringType = SENSOR_STRING_TYPE_MAGNETIC_FIELD;
	sensor_t_data.flags = SENSOR_FLAG_CONTINUOUS_MODE;

	if (wakeup)
		sensor_t_data.flags |= SENSOR_FLAG_WAKE_UP;

	sensor_t_data.resolution = GAUSS_TO_UTESLA(data->channels[0].scale);
	sensor_t_data.maxRange = sensor_t_data.resolution * (pow(2, data->channels[0].bits_used - 1) - 1);

#ifdef CONFIG_ST_HAL_MAGN_CALIB_ENABLED
	type_dependencies[SENSOR_BASE_DEPENDENCY_0] = SENSOR_TYPE_ACCELEROMETER;
#endif /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */
}

Magnetometer::~Magnetometer()
{

}

int Magnetometer::Enable(int handle, bool enable)
{
#ifdef CONFIG_ST_HAL_MAGN_CALIB_ENABLED
	int err;
	bool old_status;

	old_status = GetStatus();

	err = HWSensorBaseWithPollrate::Enable(handle, enable);
	if (err < 0)
		return err;

	if (GetStatus() && !old_status)
		STCompass_API_Init(NULL);

	return 0;
#else /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */
	return HWSensorBaseWithPollrate::Enable(handle, enable);
#endif /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */
}

void Magnetometer::ProcessData(SensorBaseData *data)
{
	float tmp_raw_data[num_data_axis];
#ifdef CONFIG_ST_HAL_MAGN_CALIB_ENABLED
	int64_t time_diff = 0;
	int err, nomaxdata = 10;
	SensorBaseData accel_data;
#endif /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */

	memcpy(tmp_raw_data, data->raw, num_data_axis * sizeof(float));

	data->raw[0] = GAUSS_TO_UTESLA(SENSOR_X_DATA(tmp_raw_data[0], tmp_raw_data[1], tmp_raw_data[2], CONFIG_ST_HAL_MAGN_ROT_MATRIX));
	data->raw[1] = GAUSS_TO_UTESLA(SENSOR_Y_DATA(tmp_raw_data[0], tmp_raw_data[1], tmp_raw_data[2], CONFIG_ST_HAL_MAGN_ROT_MATRIX));
	data->raw[2] = GAUSS_TO_UTESLA(SENSOR_Z_DATA(tmp_raw_data[0], tmp_raw_data[1], tmp_raw_data[2], CONFIG_ST_HAL_MAGN_ROT_MATRIX));

#ifdef CONFIG_ST_HAL_MAGN_CALIB_ENABLED
	do {
		err = GetLatestValidDataFromDependency(SENSOR_BASE_DEPENDENCY_0, &accel_data);
		if (err < 0) {
			nomaxdata--;
			usleep(200);
			continue;
		}

		time_diff = data->timestamp - accel_data.timestamp;

	} while ((time_diff >= GetRealPollrate()) && (nomaxdata > 0));

	if (err >= 0)
		STCompass_API_Run(accel_data.raw, data->raw);

	sensor_event.magnetic.status = STCompass_API_Get_Calibration_Data(data->offset);
#else /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */
	sensor_event.magnetic.status = SENSOR_STATUS_UNRELIABLE;
#endif /* CONFIG_ST_HAL_MAGN_CALIB_ENABLED */

	data->processed[0] = data->raw[0] - data->offset[0];
	data->processed[1] = data->raw[1] - data->offset[1];
	data->processed[2] = data->raw[2] - data->offset[2];

	sensor_event.magnetic.azimuth = data->processed[0];
	sensor_event.magnetic.pitch = data->processed[1];
	sensor_event.magnetic.roll = data->processed[2];
	sensor_event.timestamp = data->timestamp;

	HWSensorBaseWithPollrate::WriteDataToPipe();
	HWSensorBaseWithPollrate::ProcessData(data);
}
