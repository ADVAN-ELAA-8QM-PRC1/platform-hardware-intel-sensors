/*
 * STMicroelectronics HW Sensor Base With Pollrate Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#define __STDC_LIMIT_MACROS

#include <fcntl.h>
#include <assert.h>
#include <string.h>
#include <signal.h>
#include <stdint.h>
#include <endian.h>

#include "HWSensorBase.h"

#define DEFAULT_HRTIMER_PERIOD_NS		(200000000)
#define LOCAL_REPORTING_MODE_MASK		6


/**
 * size_from_channelarray() - Calculate the storage size of a scan
 * @channels: the channel info array.
 * @num_channels: number of channels.
 **/
static int size_from_channelarray(struct iio_channel_info *channels, int num_channels)
{
	int bytes = 0, i;

	for (i = 0; i < num_channels; i++) {
		if (channels[i].bytes == 0)
			continue;

		if (bytes % channels[i].bytes == 0)
			channels[i].location = bytes;
		else
			channels[i].location = bytes -
				(bytes % channels[i].bytes) + channels[i].bytes;

		bytes = channels[i].location + channels[i].bytes;
	}

	return bytes;
}

/**
 * process_2byte_received() - Return channel data from 2 byte
 * @input: 2 byte of data received from buffer channel.
 * @info: information about channel structure.
 * @multi_data: 2byte is part of multiple data.
 **/
static float process_2byte_received(int input,
				struct iio_channel_info *info, bool multi_data)
{
	int16_t val;
	float offset = 0;

	if (info->be)
		input = be16toh((uint16_t)input);
	else
		input = le16toh((uint16_t)input);

	if (!multi_data) {
		offset = info->offset;
		val = input >> info->shift;
		if (info->is_signed) {
			val &= (1 << info->bits_used) - 1;
			val = (int16_t)(val << (16 - info->bits_used)) >>
							(16 - info->bits_used);
		} else
			val &= (1 << info->bits_used) - 1;
	} else
		val = input;

	return (((float)val + offset) * info->scale);
}

static float process_3byte_received(int input, struct iio_channel_info *info)
{
	int32_t val;

	if (info->be)
		input = be32toh((uint32_t)input);
	else
		input = le32toh((uint32_t)input);

	val = input >> info->shift;
	if (info->is_signed) {
		val &= (1 << info->bits_used) - 1;
		val = (int32_t)(val << (24 - info->bits_used)) >>
						(24 - info->bits_used);
	} else
		val &= (1 << info->bits_used) - 1;

	return (((float)val + info->offset) * info->scale);
}

/**
 * process_scan() - This functions use channels device information to build data
 * @hw_sensor: pointer to current hardware sensor.
 * @data: sensor data of all channels read from buffer.
 * @channels: information about channel structure.
 * @num_channels: number of channels of the sensor.
 **/
static int ProcessScanData(uint8_t *data, struct iio_channel_info *channels, int num_channels, SensorBaseData *sensor_out_data)
{
	int k;

	for (k = 0; k < num_channels; k++) {

		sensor_out_data->offset[k] = 0;

		switch (channels[k].bytes) {
		case 1:
			sensor_out_data->raw[k] = *(uint8_t *)(data + channels[k].location);
			break;
		case 2:
			sensor_out_data->raw[k] = process_2byte_received(*(uint16_t *)
					(data + channels[k].location), &channels[k], false);
			break;
		case 3:
			sensor_out_data->raw[k] = process_3byte_received(*(uint32_t *)
					(data + channels[k].location), &channels[k]);
			break;
		case 4:
			if (channels->multi_data) {
				sensor_out_data->raw[k] = process_2byte_received(*(uint16_t *)
					(data + channels[k].location), &channels[k], true);
				sensor_out_data->offset[k] = process_2byte_received(*(uint16_t *)
					(data + channels[k].location + sizeof(uint16_t)),
					&channels[k], true);
			} else {
				uint32_t val;

				if (channels[k].be)
					val = be32toh(*(uint32_t *)
							(data + channels[k].location));
				else
					val = le32toh(*(uint32_t *)
							(data + channels[k].location));

				if (channels->isfloat)
					sensor_out_data->raw[k] = (*((float *)((void *)&val)) +
						channels[k].offset) * channels[k].scale;
				else
					sensor_out_data->raw[k] = ((float)val +
						channels[k].offset) * channels[k].scale;
			}
			break;
		case 8:
			if (channels[k].is_signed) {
				int64_t val = *(int64_t *)(data + channels[k].location);
				if ((val >> channels[k].bits_used) & 1)
					val = (val & channels[k].mask) | ~channels[k].mask;

				if ((channels[k].scale == 1.0f) &&
						(channels[k].offset == 0.0f)) {
					sensor_out_data->timestamp = val;
				} else {
					sensor_out_data->raw[k] = (((float)val +
						channels[k].offset) * channels[k].scale);
				}
			}
			break;
		default:
			return -EINVAL;
		}
	}

	return num_channels;
}

HWSensorBase::HWSensorBase(HWSensorBaseCommonData *data, const char *name,
		int handle, int sensor_type, unsigned int hw_fifo_len, int pipe_data_fd,
		float power_consumption) : SensorBase(name, handle, sensor_type, pipe_data_fd)
{
	int err;
	char *buffer_path;

	memcpy(&common_data, data, sizeof(common_data));

	sensor_t_data.power = power_consumption;
	sensor_t_data.fifoMaxEventCount = hw_fifo_len;
	current_fifo_len = HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN;

	scan_size = size_from_channelarray(common_data.channels, common_data.num_channels);

	sensor_data = (uint8_t *)malloc(scan_size * HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN * hw_fifo_len * sizeof(uint8_t));
	if (!sensor_data)
		goto failed_creation;

	err = asprintf(&buffer_path, "/dev/iio:device%d", data->iio_dev_num);
	if (err <= 0)
		goto free_sensor_data;

	pollfd_iio[0].fd = open(buffer_path, O_RDONLY | O_NONBLOCK);
	if (pollfd_iio[0].fd < 0)
		goto free_buffer_path;

	err = ioctl(pollfd_iio[0].fd, IIO_GET_EVENT_FD_IOCTL, &pollfd_iio[1].fd);
	if (err < 0)
		goto close_iio_buffer;

	pollfd_iio[0].events = POLLIN;
	pollfd_iio[1].events = POLLIN;

	free(buffer_path);

	return;

close_iio_buffer:
	close(pollfd_iio[0].fd);
free_buffer_path:
	free(buffer_path);
free_sensor_data:
	free(sensor_data);
failed_creation:
	valid_class = false;
}

HWSensorBase::~HWSensorBase()
{
	if (!valid_class)
		return;

	free(sensor_data);
	close(pollfd_iio[0].fd);
	close(pollfd_iio[1].fd);
}

int HWSensorBase::WriteBufferLenght(unsigned int buf_len, unsigned int store_len)
{
	unsigned int hw_buf_fifo_len, hw_store_fifo_len;
	int err, current_len, cur_store_len, buff_enable;

	hw_buf_fifo_len = buf_len * HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN;
	if (hw_buf_fifo_len == 0)
		hw_buf_fifo_len = HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN;

	hw_store_fifo_len = store_len * HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN;
	if (hw_store_fifo_len == 0)
		hw_store_fifo_len = HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN;

	current_len = read_sysfs_posint((char *)FILENAME_BUFFER_LENGTH,
							common_data.iio_sysfs_path);
	if (current_len < 0)
		return current_len;

	cur_store_len = read_sysfs_posint((char *)FILENAME_BUFFER_STORE_LENGTH,
							common_data.iio_sysfs_path);
	if (cur_store_len < 0)
		return cur_store_len;

	if ((current_len == (int)hw_buf_fifo_len) && (cur_store_len == (int)hw_store_fifo_len))
		return 0;

	buff_enable = read_sysfs_posint((char *)FILENAME_BUFFER_ENABLE,
							common_data.iio_sysfs_path);
	if (buff_enable < 0)
		return buff_enable;

	if (buff_enable == 1) {
		err = write_sysfs_int_and_verify((char *)FILENAME_BUFFER_ENABLE,
							common_data.iio_sysfs_path, 0);
		if (err < 0)
			return err;
	}

	err = write_sysfs_int_and_verify((char *)FILENAME_BUFFER_LENGTH,
						common_data.iio_sysfs_path, hw_buf_fifo_len);
	if (err < 0)
		return err;

	err = write_sysfs_int_and_verify((char *)FILENAME_BUFFER_STORE_LENGTH,
						common_data.iio_sysfs_path, hw_store_fifo_len);
	if (err < 0)
		return err;

	current_fifo_len = hw_buf_fifo_len;

	if (buff_enable > 0) {
		err = write_sysfs_int_and_verify((char *)FILENAME_BUFFER_ENABLE,
							common_data.iio_sysfs_path, 1);
		if (err < 0)
			return err;
	}

	return 0;
}

int HWSensorBase::Enable(int handle, bool enable)
{
	int err;

	err = SensorBase::Enable(handle, enable);
	if (err < 0)
		return err;

	err = write_sysfs_int_and_verify((char *)FILENAME_BUFFER_ENABLE,
					common_data.iio_sysfs_path, GetStatus());
	if (err < 0) {
		ALOGE("%s: Failed to write buffer file \"%s/%s\".",
			common_data.device_name, common_data.iio_sysfs_path, FILENAME_BUFFER_ENABLE);
		goto restore_status_enable;
	}

	return 0;

restore_status_enable:
	SensorBase::Enable(handle, !enable);
	return err;
}

int HWSensorBase::FlushData(bool need_report_event)
{
	bool report_at_once = false;
	int err = -1;
	int32_t type = SensorBase::sensor_t_data.type;
#ifdef __LP64__
       uint64_t flags;
#else
       uint32_t flags;
#endif

	flags = SensorBase::sensor_t_data.flags;
	ALOGD("HWSensorBase::FlushData type=%d, flags=%lld", type, flags);

	/* No flush events for One-shot sensors */
	if (SENSOR_FLAG_ONE_SHOT_MODE == (flags & LOCAL_REPORTING_MODE_MASK))
		return -EINVAL;

	if (GetStatus()) {
		/* Sensors used fifo would report flush complete event after data in fifo reported,
		 * so we store the flush timestamp here. When we write fifo data to pipe,
		 * we compare them. If the fifo data wrote done, then write the flush complete event.
		 * One exception: if the flush call come after a fifo parse, there would no data
		 * in the fifo. If we still sent flush complete event like before, that should wait for
		 * another fifo parse, that would be a very long time (unit seconds). In this case,
		 * we would sent the flush complete event here.
		 */
		if (need_report_event && ((type == SENSOR_TYPE_ACCELEROMETER) ||
				(type == SENSOR_TYPE_GYROSCOPE))) {
			int64_t flush_timestamp = get_monotonic_time();
			if (flush_timestamp <= real_pollrate) {
				ALOGE("HWSensorBase get flush base timestamp failed");
				return err;
			}
			ALOGD("hw flush timestamp %lld", flush_timestamp);
			/* Scale the real_pollrate by 11/10 because LSM6DS3 ODR has +/-10% skew */
			if (flush_timestamp <= (last_data_timestamp + real_pollrate * 11 / 10))
				report_at_once = true;
			else {
				flush_timestamp -= real_pollrate * 11 /10;
				(SensorBase::timestamp).push_back(flush_timestamp);
			}
		}

		/* Sensors used fifo would trigger read fifo here */
		if (current_fifo_len > HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN) {
			err = write_sysfs_int((char *)FILENAME_FLUSH, common_data.iio_sysfs_path, 1);
			if (err < 0) {
				ALOGE("%s: Failed to write flush file \"%s/%s\".",
						common_data.device_name, common_data.iio_sysfs_path, FILENAME_FLUSH);
				return -EINVAL;
			}
		}

		/* Sensors which not use fifo would sent a flush complete event here.
		 * Sensors used fifo would sent a flush complete event here too if the fifo
		 * is empty now.
		 */
		if (need_report_event && (report_at_once || ((type != SENSOR_TYPE_ACCELEROMETER) &&
				(type != SENSOR_TYPE_GYROSCOPE))))
			SensorBase::FlushData(true);

		return 0;

	} else
		return -EINVAL;

}

void HWSensorBase::ThreadTask()
{
	uint8_t *data;
	int err, i, read_size;
	unsigned int hw_fifo_len;
	SensorBaseData sensor_data;
	struct iio_event_data event_data;

	if (sensor_t_data.fifoMaxEventCount > 0)
		hw_fifo_len = sensor_t_data.fifoMaxEventCount;
	else
		hw_fifo_len = 1;

	data = (uint8_t *)malloc(hw_fifo_len * scan_size * HW_SENSOR_BASE_DEFAULT_IIO_BUFFER_LEN * sizeof(uint8_t));
	if (!data)
		return;

	while (true) {
		err = poll(pollfd_iio, 2, -1);
		if (err <= 0)
			continue;

		if (pollfd_iio[0].revents > 0) {
			read_size = read(pollfd_iio[0].fd, data, current_fifo_len * scan_size);
			if (read_size <= 0)
				continue;

			for (i = 0; i < (read_size / scan_size); i++) {
				err = ProcessScanData(data + (i * scan_size), common_data.channels, common_data.num_channels, &sensor_data);
				if (err < 0)
					continue;

				ProcessData(&sensor_data);
			}
		}

		if (pollfd_iio[1].revents > 0) {
			read_size = read(pollfd_iio[1].fd, &event_data, sizeof(event_data));
			if (read_size <= 0)
				continue;

			ProcessEvent(&event_data);
		}
	}
}


HWSensorBaseWithPollrate::HWSensorBaseWithPollrate(HWSensorBaseCommonData *data, const char *name,
		struct iio_sampling_frequency_available *sfa, int handle,
		int sensor_type, unsigned int hw_fifo_len, int pipe_data_fd, float power_consumption) :
		HWSensorBase(data, name, handle, sensor_type, hw_fifo_len, pipe_data_fd, power_consumption)
{
	int i;
	unsigned int max_sampling_frequency = 0, min_sampling_frequency = UINT_MAX;

	memcpy(&sampling_frequency_available, sfa, sizeof(sampling_frequency_available));

	for (i = 0; i < (int)sfa->num_available; i++) {
		if ((max_sampling_frequency < sfa->hz[i]) &&
					(sfa->hz[i] <= CONFIG_ST_HAL_MAX_SAMPLING_FREQUENCY))
			max_sampling_frequency = sfa->hz[i];

		if (min_sampling_frequency > sfa->hz[i])
			min_sampling_frequency = sfa->hz[i];
	}

	sensor_t_data.minDelay = FREQUENCY_TO_US(max_sampling_frequency);
	sensor_t_data.maxDelay = FREQUENCY_TO_US(min_sampling_frequency);
}

HWSensorBaseWithPollrate::~HWSensorBaseWithPollrate()
{

}

int HWSensorBaseWithPollrate::SetDelay(int handle, int64_t period_ns, int64_t timeout)
{
#define	MIN_BUF_TIMEOUT	2500000000ULL
	int err, i;
	int64_t min_pollrate_ns, tmp_real_pollrate;
	unsigned int sampling_frequency, buf_len, store_len;

	err = HWSensorBase::SetDelay(handle, period_ns, timeout);
	if (err < 0)
		return err;

	min_pollrate_ns = GetMinPeriod();

	sampling_frequency = NS_TO_FREQUENCY(min_pollrate_ns);
	for (i = 0; i < (int)sampling_frequency_available.num_available; i++) {
		if (sampling_frequency_available.hz[i] >= sampling_frequency)
			break;
	}
	if (i == (int)sampling_frequency_available.num_available)
		i--;

	err = write_sysfs_int_and_verify((char *)FILENAME_SAMPLING_FREQ,
				common_data.iio_sysfs_path, sampling_frequency_available.hz[i]);
	if (err < 0) {
		ALOGE("%s: Failed to write sampling frequency file \"%s/%s\".",
				common_data.device_name, common_data.iio_sysfs_path, FILENAME_SAMPLING_FREQ);
		return err;
	}

	tmp_real_pollrate = real_pollrate = FREQUENCY_TO_NS(sampling_frequency_available.hz[i]);

	if (sensor_t_data.fifoMaxEventCount > 0) {
		store_len = GetMinTimeout() / tmp_real_pollrate;
		if (store_len > sensor_t_data.fifoMaxEventCount)
			store_len = sensor_t_data.fifoMaxEventCount;

		buf_len = MIN_BUF_TIMEOUT / tmp_real_pollrate;
		if (buf_len < store_len)
			buf_len = store_len;

		err = WriteBufferLenght(buf_len, store_len);
		if (err < 0)
			return err;
	}

	return 0;
}

void HWSensorBaseWithPollrate::WriteDataToPipe()
{
	int err, retry = 3;
	std::vector<int64_t>::iterator it;

	if (!GetStatusOfHandle(sensor_t_data.handle))
		return;

	if (!(SensorBase::timestamp.empty())) {
		int64_t last_timestamp = 0;
		for (it = SensorBase::timestamp.begin(); it != SensorBase::timestamp.end(); ) {
			/* If two flush event come within 1 odr, there may not have data in hw fifo,
			 * so report corresponding flush complete events here.
			 */
			if ((sensor_event.timestamp >= *it) ||
					(last_timestamp != 0 && (*it - last_timestamp < real_pollrate * 11 / 10))) {
				sensors_event_t flush_event_data;

				flush_event_data.sensor = 0;
				flush_event_data.timestamp = 0;
				flush_event_data.meta_data.sensor = sensor_t_data.handle;
				flush_event_data.meta_data.what = META_DATA_FLUSH_COMPLETE;
				flush_event_data.type = SENSOR_TYPE_META_DATA;
				flush_event_data.version = META_DATA_VERSION;

				while (retry) {
					err = SensorBase::WritePipeWithPoll(&flush_event_data, sizeof(sensor_event),
											POLL_TIMEOUT_FLUSH_EVENT);
					if (err > 0)
						break;

					retry--;
					ALOGI("%s: Retry writing flush event data to pipe, retry_cnt: %d.", android_name, 3-retry);
				}

				if (retry ==  0)
					ALOGE("%s: Failed to write HW flush_complete, err=%d", android_name, err);
				else
					ALOGD("write hw flush complete event to pipe succeed.");

				last_timestamp = *it;
				it = SensorBase::timestamp.erase(it);
			} else
				break;
		}
	}

	/* Scale the real_pollrate by 9/10 because LSM6DS3 ODR has +/-10% skew */
	if (sensor_event.timestamp >= (last_data_timestamp + real_pollrate * 9 / 10)) {
		err =  SensorBase::WritePipeWithPoll(&sensor_event, sizeof(sensor_event), POLL_TIMEOUT_DATA_EVENT);
		if (err <= 0) {
			ALOGE("%s: Write sensor data failed.", android_name);
			return;
		}

		last_data_timestamp = sensor_event.timestamp;
	} else
		ALOGE("%s: Dropping event type=%d because ts %lld < %lld (%lld + %lld * 9 / 10)", android_name,
					(last_data_timestamp + real_pollrate * 9 / 10), last_data_timestamp, real_pollrate);
}
