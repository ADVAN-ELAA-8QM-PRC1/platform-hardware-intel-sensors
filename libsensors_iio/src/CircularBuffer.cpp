/*
 * STMicroelectronics Circular Buffer Class
 *
 * Copyright 2013-2015 STMicroelectronics Inc.
 * Author: Denis Ciocca - <denis.ciocca@st.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 */

#include "CircularBuffer.h"
#include <cutils/log.h>
CircularBuffer::CircularBuffer(unsigned int num_elements)
{
	data_sensor = (SensorBaseData *)malloc(num_elements * sizeof(SensorBaseData));

	pthread_mutex_init(&data_mutex, NULL);

	lenght = num_elements;
	elements_available = 0;
	first_free_element = &data_sensor[0];
	first_available_element = &data_sensor[0];
}

CircularBuffer::~CircularBuffer()
{
	delete data_sensor;
}

void CircularBuffer::writeElement(SensorBaseData *data)
{
	pthread_mutex_lock(&data_mutex);

	if (elements_available == lenght) {
		first_available_element++;

		if (first_available_element == (&data_sensor[0] + lenght))
			first_available_element = &data_sensor[0];
	}

	memcpy(first_free_element, data, sizeof(SensorBaseData));
	first_free_element++;

	if (first_free_element == (&data_sensor[0] + lenght))
		first_free_element = &data_sensor[0];

	if (elements_available < lenght)
		elements_available++;

	pthread_mutex_unlock(&data_mutex);
}

int CircularBuffer::readElement(SensorBaseData *data)
{
	int num_remaining_elements;

	pthread_mutex_lock(&data_mutex);

	if (elements_available == 0) {
		pthread_mutex_unlock(&data_mutex);
		return -EFAULT;
	}

	memcpy(data, first_available_element, sizeof(SensorBaseData));
	first_available_element++;

	if (first_available_element == (&data_sensor[0] + lenght))
		first_available_element = &data_sensor[0];

	elements_available--;
	num_remaining_elements = elements_available;

	pthread_mutex_unlock(&data_mutex);

	return num_remaining_elements;
}
