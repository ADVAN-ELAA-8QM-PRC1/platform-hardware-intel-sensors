/* IIO - useful set of util functionality
 *
 * Copyright (c) 2008 Jonathan Cameron
 * Modified by Denis Ciocca <denis.ciocca@st.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <utils/Log.h>

#include "iio_utils.h"
#include "common_data.h"


const char *iio_dir = "/sys/bus/iio/devices/";
const char *iio_sampling_frequency_available_filename = "sampling_frequency_available";
const char *iio_hw_fifo_lenght = "hw_fifo_lenght";
const char *iio_buffer_enable = "buffer/enable";


/**
 * iioutils_break_up_name() - extract generic name from full channel name
 * @full_name: the full channel name
 * @generic_name: the output generic channel name
 **/
inline int iioutils_break_up_name(const char *full_name, char **generic_name)
{
	char *current;
	char *w, *r;
	char *working;

	current = strdup(full_name);
	working = strtok(current, "_\0");
	w = working;
	r = working;

	while (*r != '\0') {
		if (!isdigit(*r)) {
			*w = *r;
			w++;
		}
		r++;
	}
	*w = '\0';
	*generic_name = strdup(working);
	free(current);

	return 0;
}

/**
 * iioutils_get_type() - find and process _type attribute data
 * @is_signed: output whether channel is signed
 * @bytes: output how many bytes the channel storage occupies
 * @mask: output a bit mask for the raw data
 * @be: big endian
 * @device_dir: the iio device directory
 * @name: the channel name
 * @generic_name: the channel type name
 **/
inline int iioutils_get_type(unsigned *is_signed, unsigned *bytes,
	unsigned *bits_used, unsigned *shift, uint64_t *mask, unsigned *be,
			const char *device_dir, const char *name, const char *generic_name)
{
	char *scan_el_dir, *builtname, *builtname_generic, *filename = 0;
	char signchar, endianchar;
	const struct dirent *ent;
	unsigned padint;
	FILE *sysfsfp;
	int ret;
	DIR *dp;

	ret = asprintf(&scan_el_dir, FORMAT_SCAN_ELEMENTS_DIR, device_dir);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_ret;
	}

	ret = asprintf(&builtname, FORMAT_TYPE_FILE, name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_scan_el_dir;
	}

	ret = asprintf(&builtname_generic, FORMAT_TYPE_FILE, generic_name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_builtname;
	}

	dp = opendir(scan_el_dir);
	if (dp == NULL) {
		ret = -errno;
		goto error_free_builtname_generic;
	}

	while (ent = readdir(dp), ent != NULL)
		/*
		 * Do we allow devices to override a generic name with
		 * a specific one?
		 */
		if ((strcmp(builtname, ent->d_name) == 0) ||
				(strcmp(builtname_generic, ent->d_name) == 0)) {
			ret = asprintf(&filename, "%s/%s", scan_el_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				goto error_closedir;
			}

			sysfsfp = fopen(filename, "r");
			if (sysfsfp == NULL) {
				printf("failed to open %s\n", filename);
				ret = -errno;
				goto error_free_filename;
			}

			ret = fscanf(sysfsfp, "%ce:%c%u/%u>>%u", &endianchar,
					&signchar, bits_used, &padint, shift);
			if (ret < 0) {
				ret = -errno;
				goto error_close_sysfsfp;
			}

			*be = (endianchar == 'b');
			*bytes = padint / 8;
			if (*bits_used == 64)
				*mask = ~0;
			else
				*mask = (1 << *bits_used) - 1;

			if (signchar == 's')
				*is_signed = 1;
			else
				*is_signed = 0;

			fclose(sysfsfp);
			free(filename);

			filename = 0;
			sysfsfp = 0;
		}

error_close_sysfsfp:
	if (sysfsfp)
		fclose(sysfsfp);
error_free_filename:
	if (filename)
		free(filename);
error_closedir:
	closedir(dp);
error_free_builtname_generic:
	free(builtname_generic);
error_free_builtname:
	free(builtname);
error_free_scan_el_dir:
	free(scan_el_dir);
error_ret:
	return ret;
}

inline int iioutils_get_param_float(float *output, const char *param_name,
		const char *device_dir, const char *name, const char *generic_name)
{
	char *builtname, *builtname_generic;
	const struct dirent *ent;
	char *filename = NULL;
	FILE *sysfsfp;
	int ret;
	DIR *dp;

	ret = asprintf(&builtname, "%s_%s", name, param_name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_ret;
	}

	ret = asprintf(&builtname_generic, "%s_%s", generic_name, param_name);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_free_builtname;
	}

	dp = opendir(device_dir);
	if (dp == NULL) {
		ret = -errno;
		goto error_free_builtname_generic;
	}

	while (ent = readdir(dp), ent != NULL)
		if ((strcmp(builtname, ent->d_name) == 0) ||
				(strcmp(builtname_generic, ent->d_name) == 0)) {
			ret = asprintf(&filename, "%s/%s", device_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				goto error_closedir;
			}

			sysfsfp = fopen(filename, "r");
			if (!sysfsfp) {
				ret = -errno;
				goto error_free_filename;
			}
			fscanf(sysfsfp, "%f", output);
			break;
		}
error_free_filename:
	if (filename)
		free(filename);
error_closedir:
	closedir(dp);
error_free_builtname_generic:
	free(builtname_generic);
error_free_builtname:
	free(builtname);
error_ret:
	return ret;
}

/**
 * bsort_channel_array_by_index() - reorder so that the array is in index order
 *
 **/
inline void bsort_channel_array_by_index(struct iio_channel_info **ci_array,
					 int cnt)
{

	struct iio_channel_info temp;
	int x, y;

	for (x = 0; x < cnt; x++) {
		for (y = 0; y < (cnt - 1); y++) {
			if ((*ci_array)[y].index > (*ci_array)[y+1].index) {
				temp = (*ci_array)[y + 1];
				(*ci_array)[y + 1] = (*ci_array)[y];
				(*ci_array)[y] = temp;
			}
		}
	}
}

int iio_utils_enable_sensor(const char *device_dir, bool enable)
{
	return write_sysfs_int_and_verify((char *)iio_buffer_enable, (char *)device_dir, (int)enable);
}

int iio_utils_get_hw_fifo_lenght(const char *device_dir)
{
	int len;

	len = read_sysfs_posint((char *)iio_hw_fifo_lenght, (char *)device_dir);
	if (len < 0)
		return 0;

	return len;
}

int iio_utils_set_scale(const char *device_dir, float value, int device_type)
{
	int err;
	char *scale_file_name;

	switch (device_type) {
		case SENSOR_TYPE_ACCELEROMETER:
			scale_file_name = (char *)"in_accel_x_scale";
			break;
		case SENSOR_TYPE_MAGNETIC_FIELD:
			scale_file_name = (char *)"in_magn_x_scale";
			break;
		case SENSOR_TYPE_GYROSCOPE:
			scale_file_name = (char *)"in_anglvel_x_scale";
			break;
		case SENSOR_TYPE_PRESSURE:
			scale_file_name = (char *)"in_press_scale";
			break;
		default:
			return -EINVAL;
	}

	err = write_sysfs_float_and_verify(scale_file_name, (char *)device_dir, value);
	if (err < 0)
		return err;

	return 0;
}

int iio_utils_get_scale_available(const char *device_dir, struct iio_scale_available *sa, int device_type)
{
	int err;
	FILE *fp;
	char *tmp_name, *avl_name, *pch, line[200];

	sa->num_available = 0;

	switch (device_type) {
		case SENSOR_TYPE_ACCELEROMETER:
			avl_name = (char *)"in_accel_scale_available";
			break;
		case SENSOR_TYPE_MAGNETIC_FIELD:
			avl_name = (char *)"in_magn_scale_available";
			break;
		case SENSOR_TYPE_GYROSCOPE:
			avl_name = (char *)"in_anglvel_scale_available";
			break;
		case SENSOR_TYPE_PRESSURE:
			avl_name = (char *)"in_press_scale_available";
			break;
		default:
			return -EINVAL;
	}

	err = asprintf(&tmp_name, "%s/%s", device_dir, avl_name);
	if (err < 0)
		return err;

	fp = fopen(tmp_name, "r");
	if (fp == NULL) {
		err = 0;
		goto open_file_error;
	}

	fgets(line, sizeof(line), fp);
	if (line == NULL) {
		ALOGE("Scale available file format error: \"%s\".", tmp_name);
		err = -EINVAL;
		goto read_error;
	}

	pch = strtok(line," ");
	while (pch != NULL) {
		sa->values[sa->num_available] = atof(pch);
		pch = strtok(NULL, " ");
		sa->num_available++;

		if (sa->num_available >= IIO_UTILS_SCALE_AVAILABLE)
			break;
	}

read_error:
	fclose(fp);
open_file_error:
	free(tmp_name);
	return err < 0 ? err : 0;
}

int iio_utils_get_sampling_frequency_available(const char *device_dir,
					struct iio_sampling_frequency_available *sfa)
{
	int err;
	FILE *fp;
	char *tmp_name, *pch, line[200];

	sfa->num_available = 0;

	err = asprintf(&tmp_name, "%s/%s", device_dir, iio_sampling_frequency_available_filename);
	if (err < 0)
		return err;

	fp = fopen(tmp_name, "r");
	if (fp == NULL) {
		ALOGE("Failed to open sampling frequency available file: \"%s\".", tmp_name);
		err = -errno;
		goto tmp_name_free;
	}

	fgets(line, sizeof(line), fp);
	if (line == NULL) {
		ALOGE("Sampling frequency file format error: \"%s\".", tmp_name);
		err = -EINVAL;
		goto close_file;
	}

	pch = strtok(line," ,.");
	while (pch != NULL) {
		sfa->hz[sfa->num_available] = atoi(pch);
		pch = strtok(NULL, " ,.");
		sfa->num_available++;

		if (sfa->num_available >= IIO_UTILS_MAX_SAMP_FREQ_AVAILABLE)
			break;
	}

close_file:
	fclose(fp);
tmp_name_free:
	free(tmp_name);
	return err < 0 ? err : 0;
}

int iio_utils_build_channel_array(const char *device_dir,
		struct iio_channel_info **ci_array, int *counter, bool read_offset)
{
	DIR *dp;
	FILE *sysfsfp;
	int count, i;
	struct iio_channel_info *current;
	int ret;
	const struct dirent *ent;
	char *scan_el_dir;
	char *filename;

	*counter = 0;
	ret = asprintf(&scan_el_dir, FORMAT_SCAN_ELEMENTS_DIR, device_dir);
	if (ret < 0) {
		ret = -ENOMEM;
		goto error_ret;
	}

	dp = opendir(scan_el_dir);
	if (dp == NULL) {
		ret = -errno;
		goto error_free_name;
	}

	while (ent = readdir(dp), ent != NULL)
		if (strcmp(ent->d_name + strlen(ent->d_name)
						- strlen("_en"), "_en") == 0) {
			ret = asprintf(&filename,
					"%s/%s", scan_el_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				goto error_close_dir;
			}

			sysfsfp = fopen(filename, "r+");
			if (sysfsfp == NULL) {
				ret = -errno;
				free(filename);
				goto error_close_dir;
			}

			fprintf(sysfsfp, "%d", 1);
			rewind(sysfsfp);

			fscanf(sysfsfp, "%u", &ret);
			if (ret == 1)
				(*counter)++;

			fclose(sysfsfp);
			free(filename);
		}
	*ci_array = malloc(sizeof(**ci_array) * (*counter));
	if (*ci_array == NULL) {
		ret = -ENOMEM;
		goto error_close_dir;
	}

	rewinddir(dp);
	count = 0;
	while (ent = readdir(dp), ent != NULL) {
		if (strcmp(ent->d_name + strlen(ent->d_name)
						- strlen("_en"), "_en") == 0) {
			current = &(*ci_array)[count++];
			ret = asprintf(&filename, "%s/%s", scan_el_dir, ent->d_name);
			if (ret < 0) {
				ret = -ENOMEM;
				/* decrement count to avoid freeing name */
				count--;
				goto error_cleanup_array;
			}

			sysfsfp = fopen(filename, "r");
			if (sysfsfp == NULL) {
				free(filename);
				ret = -errno;
				goto error_cleanup_array;
			}

			fscanf(sysfsfp, "%u", &current->enabled);
			fclose(sysfsfp);

			if (!current->enabled) {
				free(filename);
				count--;
				continue;
			}

			current->scale = 1.0f;
			current->offset = 0.0f;
			current->name = strndup(ent->d_name,
					strlen(ent->d_name) - strlen("_en"));
			if (current->name == NULL) {
				free(filename);
				ret = -ENOMEM;
				goto error_cleanup_array;
			}

			/* Get the generic and specific name elements */
			ret = iioutils_break_up_name(current->name,
							&current->generic_name);
			if (ret) {
				free(filename);
				goto error_cleanup_array;
			}

			ret = asprintf(&filename, "%s/%s_index",
						scan_el_dir, current->name);
			if (ret < 0) {
				free(filename);
				ret = -ENOMEM;
				goto error_cleanup_array;
			}

			sysfsfp = fopen(filename, "r");
			fscanf(sysfsfp, "%u", &current->index);
			fclose(sysfsfp);
			free(filename);

			/* Find the scale */
			ret = iioutils_get_param_float(&current->scale, "scale",
				device_dir, current->name, current->generic_name);
			if (ret < 0)
				goto error_cleanup_array;

			if (read_offset) {
				ret = iioutils_get_param_float(&current->offset,
						"offset", device_dir, current->name,
								current->generic_name);
				if (ret < 0)
					goto error_cleanup_array;
			}

			ret = iioutils_get_type(&current->is_signed,
						&current->bytes,
						&current->bits_used,
						&current->shift,
						&current->mask,
						&current->be,
						device_dir,
						current->name,
						current->generic_name);
		}
	}

	closedir(dp);
	/* reorder so that the array is in index order */
	bsort_channel_array_by_index(ci_array, *counter);

	return 1;

error_cleanup_array:
	for (i = count - 1; i >= 0; i--)
		free((*ci_array)[i].name);
	free(*ci_array);
error_close_dir:
	closedir(dp);
error_free_name:
	free(scan_el_dir);
error_ret:
	return ret;
}

int find_type_by_name(char *name, const char *type)
{
	char thisname[IIO_MAX_NAME_LENGTH];
	const struct dirent *ent;
	int number, numstrlen;
	char *filename;
	FILE *nameFile;
	DIR *dp;

	dp = opendir(iio_dir);
	if (dp == NULL)
		return -ENODEV;

	while (ent = readdir(dp), ent != NULL) {
		if (strcmp(ent->d_name, ".") != 0 &&
				strcmp(ent->d_name, "..") != 0 &&
				strlen(ent->d_name) > strlen(type) &&
				strncmp(ent->d_name, type, strlen(type)) == 0) {

			numstrlen = sscanf(ent->d_name + strlen(type), "%d", &number);

			/* verify the next character is not a colon */
			if (strncmp(ent->d_name + strlen(type) + numstrlen,
								":", 1) != 0) {
				filename = (char *)malloc(strlen(iio_dir)
						+ strlen(type)
						+ numstrlen
						+ 6);
				if (filename == NULL) {
					closedir(dp);
					return -ENOMEM;
				}

				sprintf(filename, "%s%s%d/name", iio_dir,
								type, number);
				nameFile = fopen(filename, "r");
				if (!nameFile) {
					free(filename);
					continue;
				}

				free(filename);
				fscanf(nameFile, "%s", thisname);
				fclose(nameFile);
				if (strcmp(name, thisname) == 0) {
					closedir(dp);
					return number;
				}
			}
		}
	}
	closedir(dp);

	return -ENODEV;
}

int iio_utils_get_devices_name(struct iio_device devices[], unsigned int max_list)
{
	unsigned int device_num = 0;
	char thisname[IIO_MAX_NAME_LENGTH];
	const struct dirent *ent;
	int number, numstrlen;
	char *filename;
	FILE *nameFile;
	DIR *dp;

	dp = opendir(iio_dir);
	if (dp == NULL)
		return -ENODEV;

	while (ent = readdir(dp), ent != NULL) {
		if (strcmp(ent->d_name, ".") != 0 &&
				strcmp(ent->d_name, "..") != 0 &&
				strlen(ent->d_name) > strlen("iio:device") &&
				strncmp(ent->d_name, "iio:device", strlen("iio:device")) == 0) {

			numstrlen = sscanf(ent->d_name + strlen("iio:device"), "%d", &number);

			/* verify the next character is not a colon */
			if (strncmp(ent->d_name + strlen("iio:device") + numstrlen,
								":", 1) != 0) {
				filename = (char *)malloc(strlen(iio_dir)
						+ strlen("iio:device")
						+ numstrlen
						+ 6);
				if (filename == NULL) {
					closedir(dp);
					return -ENOMEM;
				}

				sprintf(filename, "%s%s%d/name", iio_dir,
							"iio:device", number);
				nameFile = fopen(filename, "r");
				if (!nameFile) {
					free(filename);
					continue;
				}

				free(filename);
				fscanf(nameFile, "%s", thisname);
				fclose(nameFile);

				memcpy(devices[device_num].name, thisname, strlen(thisname));
				devices[device_num].name[strlen(thisname)] = '\0';

				devices[device_num].dev_num = number;
				device_num++;

				if (device_num >= max_list) {
					closedir(dp);
					return (int)device_num;
				}
			}
		}
	}
	closedir(dp);

	return (int)device_num;
}

inline int _write_sysfs_int(char *filename, char *basedir, int val, int verify)
{
	int ret = 0;
	FILE *sysfsfp;
	int test;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	fprintf(sysfsfp, "%d", val);
	fclose(sysfsfp);

	if (verify) {
		sysfsfp = fopen(temp, "r");
		if (sysfsfp == NULL) {
			ret = -errno;
			goto error_free;
		}
		fscanf(sysfsfp, "%d", &test);
		fclose(sysfsfp);

		if (test != val) {
			ALOGE("Failed to write \"%d\" to \"%s/%s\" file.",
							val, basedir, filename);
			ret = -1;
		}
	}

error_free:
	free(temp);
	return ret;
}

int write_sysfs_int(char *filename, char *basedir, int val)
{
	return _write_sysfs_int(filename, basedir, val, 0);
}

int write_sysfs_int_and_verify(char *filename, char *basedir, int val)
{
	return _write_sysfs_int(filename, basedir, val, 1);
}

inline int _write_sysfs_ulong_and_verify(char *filename, char *basedir,
						unsigned long val, int verify)
{
	int ret = 0;
	FILE *sysfsfp;
	unsigned long test;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	fprintf(sysfsfp, "%lu", val);
	fclose(sysfsfp);

	if (verify) {
		sysfsfp = fopen(temp, "r");
		if (sysfsfp == NULL) {
			ret = -errno;
			goto error_free;
		}
		fscanf(sysfsfp, "%lu", &test);
		fclose(sysfsfp);

		if (test != val) {
			ALOGE("Failed to write \"%lu\" to \"%s/%s\" file.",
							val, basedir, filename);
			ret = -1;
		}
	}

error_free:
	free(temp);
	return ret;
}

int write_sysfs_ulong_and_verify(char *filename, char *basedir, unsigned long val)
{
	return _write_sysfs_ulong_and_verify(filename, basedir, val, 1);
}

inline int _write_sysfs_float(char *filename, char *basedir, float val, int verify)
{
	int ret = 0;
	FILE *sysfsfp;
	float test;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	fprintf(sysfsfp, "%f", val);
	fclose(sysfsfp);

	if (verify) {
		sysfsfp = fopen(temp, "r");
		if (sysfsfp == NULL) {
			ret = -errno;
			goto error_free;
		}
		fscanf(sysfsfp, "%f", &test);
		fclose(sysfsfp);

		if (test != val) {
			ALOGE("Failed to write \"%f\" to \"%s/%s\" file.",
							val, basedir, filename);
			ret = -1;
		}
	}

error_free:
	free(temp);
	return ret;
}

int write_sysfs_float(char *filename, char *basedir, float val)
{
	return _write_sysfs_float(filename, basedir, val, 0);
}

int write_sysfs_float_and_verify(char *filename, char *basedir, float val)
{
	return _write_sysfs_float(filename, basedir, val, 1);
}

int _write_sysfs_string(char *filename, char *basedir, char *val, int verify)
{
	FILE  *sysfsfp;
	int ret = 0;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		ret = -EIO;
		goto error_free;
	}

	fprintf(sysfsfp, "%s", val);
	fclose(sysfsfp);
	if (verify) {
		sysfsfp = fopen(temp, "r");
		if (sysfsfp == NULL) {
			ret = -EIO;
			goto error_free;
		}

		fscanf(sysfsfp, "%s", temp);
		fclose(sysfsfp);
		if (strcmp(temp, val) != 0)
			ret = -1;
	}
error_free:
	free(temp);

	return ret;
}

int write_sysfs_string_and_verify(char *filename, char *basedir, char *val)
{
	return _write_sysfs_string(filename, basedir, val, 1);
}

int write_sysfs_string(char *filename, char *basedir, char *val)
{
	return _write_sysfs_string(filename, basedir, val, 0);
}

int read_sysfs_posint(char *filename, char *basedir)
{
	int ret;
	FILE  *sysfsfp;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "r");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	fscanf(sysfsfp, "%d\n", &ret);
	fclose(sysfsfp);

error_free:
	free(temp);
	return ret;
}

int read_sysfs_float(char *filename, char *basedir, float *val)
{
	float ret = 0;
	FILE  *sysfsfp;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "r");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	fscanf(sysfsfp, "%f\n", val);
	fclose(sysfsfp);

error_free:
	free(temp);
	return ret;
}

int read_sysfs_string(char *filename, char *basedir, char *str)
{
	int ret = 0;
	FILE  *sysfsfp;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "r");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	fscanf(sysfsfp, "%s\n", str);
	fclose(sysfsfp);

error_free:
	free(temp);
	return ret;
}

int read_sysfs_byte(char *filename, char *basedir, uint8_t *data, size_t len)
{
	int ret = 0;
	FILE  *sysfsfp;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "r");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	ret = fread(data, len, 1, sysfsfp);
	fclose(sysfsfp);

error_free:
	free(temp);
	return ret <= 0 ? -errno : (int)len;
}

int write_sysfs_byte(char *filename, char *basedir, uint8_t *data, size_t len)
{
	int ret = 0;
	FILE  *sysfsfp;

	char *temp = (char *)malloc(strlen(basedir) + strlen(filename) + 2);
	if (temp == NULL)
		return -ENOMEM;

	sprintf(temp, "%s/%s", basedir, filename);
	sysfsfp = fopen(temp, "w");
	if (sysfsfp == NULL) {
		ret = -errno;
		goto error_free;
	}

	ret = fwrite(data, len, 1, sysfsfp);
	fclose(sysfsfp);

error_free:
	free(temp);
	return ret <= 0 ? -errno : (int)len;
}
