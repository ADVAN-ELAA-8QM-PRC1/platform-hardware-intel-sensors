/* IIO - useful set of util functionality
 *
 * Copyright (c) 2008 Jonathan Cameron
 * Modified by Denis Ciocca <denis.ciocca@st.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#ifndef IIO_UTILS
#define IIO_UTILS

#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include <stdio.h>
#include <stdint.h>
#include <dirent.h>
#include <errno.h>
#include <stdbool.h>


/* Made up value to limit allocation sizes */
#define IIO_MAX_NAME_LENGTH		(70)

#define FORMAT_SCAN_ELEMENTS_DIR	"%s/scan_elements"
#define FORMAT_TYPE_FILE		"%s_type"

#define IIO_DEFAULT_BUFFER_LEN		(2)

/**
 * struct iio_channel_info - information about a given channel
 * @name: channel name
 * @generic_name: general name for channel type
 * @scale: scale factor to be applied for conversion to si units
 * @offset: offset to be applied for conversion to si units
 * @index: the channel index in the buffer output
 * @bytes: number of bytes occupied in buffer output
 * @mask: a bit mask for the raw output
 * @is_signed: is the raw value stored signed
 * @enabled: is this channel enabled
 **/
struct iio_channel_info {
	char *name;
	char *generic_name;
	float scale;
	float offset;
	unsigned index;
	unsigned bytes;
	unsigned bits_used;
	unsigned shift;
	uint64_t mask;
	unsigned be;
	unsigned is_signed;
	unsigned enabled;
	unsigned location;
	bool isfloat;
	bool multi_data;
};


/**
 * build_channel_array() - function to figure out what channels are present
 * @device_dir: the IIO device directory in sysfs
 * @
 **/
int iio_utils_build_channel_array(const char *device_dir,
	struct iio_channel_info **ci_array, int *counter, bool read_offset);

/**
 * find_type_by_name() - function to match top level types by name
 * @name: top level type instance name
 * @type: the type of top level instance being sort
 *
 * Typical types this is used for are device and trigger.
 **/
int find_type_by_name(char *name, const char *type);

int write_sysfs_int(char *filename, char *basedir, int val);

int write_sysfs_int_and_verify(char *filename, char *basedir, int val);

int write_sysfs_ulong_and_verify(char *filename, char *basedir, unsigned long val);

int write_sysfs_float(char *filename, char *basedir, float val);

int write_sysfs_float_and_verify(char *filename, char *basedir, float val);





#define IIO_UTILS_MAX_SAMP_FREQ_AVAILABLE		(10)
#define IIO_UTILS_SCALE_AVAILABLE			(10)

struct iio_scale_available {
	float values[IIO_UTILS_SCALE_AVAILABLE];
	unsigned int num_available;
};

struct iio_sampling_frequency_available {
	unsigned int hz[IIO_UTILS_MAX_SAMP_FREQ_AVAILABLE];
	unsigned int num_available;
};

struct iio_device {
	unsigned int dev_num;
	char name[IIO_MAX_NAME_LENGTH];
};

int iio_utils_get_devices_name(struct iio_device devices[], unsigned int max_list);

int iio_utils_get_sampling_frequency_available(const char *device_dir,
					struct iio_sampling_frequency_available *sfa);

int iio_utils_get_scale_available(const char *device_dir, struct iio_scale_available *sa, int device_type);

int iio_utils_set_scale(const char *device_dir, float value, int device_type);

int iio_utils_get_hw_fifo_lenght(const char *device_dir);


int iio_utils_enable_sensor(const char *device_dir, bool enable);

/**
 * write_sysfs_string_and_verify() - string write, readback and verify
 * @filename: name of file to write to
 * @basedir: the sysfs directory in which the file is to be found
 * @val: the string to write
 **/
int write_sysfs_string_and_verify(char *filename, char *basedir, char *val);

int write_sysfs_string(char *filename, char *basedir, char *val);

int read_sysfs_posint(char *filename, char *basedir);

int read_sysfs_float(char *filename, char *basedir, float *val);

int read_sysfs_string(char *filename, char *basedir, char *str);

int read_sysfs_byte(char *filename, char *basedir, uint8_t *data, size_t len);

int write_sysfs_byte(char *filename, char *basedir, uint8_t *data, size_t len);

#endif /* IIO_UTILS */
