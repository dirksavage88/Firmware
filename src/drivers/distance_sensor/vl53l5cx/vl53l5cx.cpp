/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**********
 * Based on the driver from ST: Ultra lite driver. UM2884;
 ***********/

#include "vl53l5cx.hpp"
#include "vl53l5cx_buffers.h"
#include <stdlib.h>
#include <string.h>

#define VL53L5CX_DELAY 10000 // us
#define VL53L5CX_UNSHIFTED_I2C_ADDR 0x29

VL53L5CX::VL53L5CX(const I2CSPIDriverConfig &config)
	: I2C(config), I2CSPIDriver(config),
	  _px4_rangefinder(get_device_id(), config.rotation)
{

	// VL53L5CX typical range 0-4 meters with 60 degree field of view
	_px4_rangefinder.set_min_distance(0.f);
	_px4_rangefinder.set_max_distance(4.f);

	_px4_rangefinder.set_fov(math::radians(61.f));

	// Allow 3 retries as the device typically misses the first measure attempts.
	I2C::_retries = 3;

	// VL53L5CX_Configuration l5_config;
	// VL53L5CX_ResultsData l5_results;


	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_VL53L5CX);
}

VL53L5CX::~VL53L5CX()
{
	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int VL53L5CX::collect()
{
	uint8_t ret = 0;
	uint8_t rangeStatus = 0;
	uint16_t distance_mm = 0;

	perf_begin(_sample_perf);

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	ret |= vl53l5cx_get_ranging_data(&l5_config, &l5_results);
	rangeStatus = VL53L5CX_GetDistance(&distance_mm);

	switch (rangeStatus) {

	case 5:
		ret |= PX4_OK;
		break;

	case 9:
		ret |= PX4_OK;
		break;

	default:
		PX4_INFO("Bad range status");
	}

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return PX4_ERROR;
	}

	perf_end(_sample_perf);

	float distance_m = distance_mm / 1000.f;

	_px4_rangefinder.update(timestamp_sample, distance_m);

	return PX4_OK;
}

void VL53L5CX::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_comms_errors);
	perf_print_counter(_sample_perf);
}

int VL53L5CX::probe()
{
	uint8_t ret, active = 0;

	l5_config.platform.address = VL53L5CX_UNSHIFTED_I2C_ADDR;
	ret = vl53l5cx_is_alive(&l5_config, &active);

	if (!active || ret) {

		PX4_INFO("Error: sensor not connected");
		return -EIO;
	}

	_retries = 1;

	return PX4_OK;
}

void VL53L5CX::RunImpl()
{
	uint8_t dataReady = 0;
	VL53L5CX_CheckForDataReady(&dataReady);

	if (dataReady == 1) {
		collect();
	}

	ScheduleDelayed(VL53L5CX_DELAY);
}

int VL53L5CX::init()
{
	int ret = PX4_OK;

	ret = device::I2C::init();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	ret |= VL53L5CX_SensorInit();
	PX4_INFO("Sensor init complete");
	ret |= VL53L5CX_StartRanging();

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return PX4_ERROR;
	}

	PX4_DEBUG("vl53l5cx init success");
	ScheduleNow();
	return PX4_OK;
}

void VL53L5CX::print_usage()
{
	PRINT_MODULE_USAGE_NAME("vl53l5cx", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x29);
	PRINT_MODULE_USAGE_PARAM_INT(
		'R', 25, 1, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int8_t VL53L5CX::VL53L5CX_SensorInit()
{
	int8_t status = 0;



	status |= probe();

	if (status != PX4_OK) {

		return status;
	}

	status |= vl53l5cx_init(&l5_config);

	if (status) {

		PX4_INFO("VL53L5CX ULD load failure");
		return status;
	}

	PX4_INFO("API version: %s", VL53L5CX_API_REVISION);
	/* Set ranging as continous mode for fast ranging */
	// status |=
	// 	vl53l5cx_set_ranging_mode(&l5_config, VL53L5CX_RANGING_MODE_CONTINUOUS);

	return status;
}

int8_t VL53L5CX::VL53L5CX_StartRanging()
{
	int8_t status = 0;

	status |= vl53l5cx_start_ranging(&l5_config); /* Enable VL53L5CX */
	return status;
}

/**
 * @brief This function checks if the new ranging data is available by polling
 * the dedicated register.
 * @param : isDataReady==0 -> not ready; isDataReady==1 -> ready
 */
int8_t VL53L5CX::VL53L5CX_CheckForDataReady(uint8_t *isDataReady)
{
	uint8_t status = 0;

	status = vl53l5cx_check_data_ready(&l5_config, isDataReady);

	/* Read in the register to check if a new value is available */
	if (!*isDataReady) {
		PX4_INFO("Data not ready");
	}

	return status;
}

int8_t VL53L5CX::VL53L5CX_StopRanging()
{
	int8_t status = 0;

	status = vl53l5cx_stop_ranging(&l5_config); /* Disable VL53L5CX */
	ScheduleClear();
	return status;
}

/**
 * @brief This function returns the distance measured by the sensor in mm
 */
int8_t VL53L5CX::VL53L5CX_GetDistance(uint16_t *distance)
{
	uint8_t status = 0;
	uint16_t tmp = 0;

	status = l5_results.target_status[9];
	tmp = l5_results.distance_mm[9];
	*distance = tmp;

	return status;
}

uint8_t VL53L5CX::WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs)
{

	ScheduleDelayed(1000 * TimeMs);

	return PX4_OK;
}
/* ST low level functions for sensor reading/writing*/
uint8_t VL53L5CX::RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
			 uint8_t *p_value)
{
	int8_t ret;
	uint8_t value_local[2];
	/* Write the register address of the sensor */

	value_local[0] = (RegisterAddress >> 8) & 0xff;
	value_local[1] = RegisterAddress & 0xff;

	ret = transfer(&value_local[0], sizeof(value_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	ret = transfer(nullptr, 0, p_value, 1);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

uint8_t VL53L5CX::WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
			 uint8_t value)
{

	int8_t ret;
	uint8_t write_local[3];
	/* Write the register address of the sensor and send data byte*/

	write_local[0] = (RegisterAddress >> 8) & 0xff;
	write_local[1] = RegisterAddress & 0xff;
	write_local[2] = value;

	ret = transfer(&write_local[0], sizeof(write_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

uint8_t VL53L5CX::WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
			  uint8_t *p_values, uint32_t size)
{

	int8_t ret = 0;

	/* Add the register address bytes (2) to the array */
	uint32_t packet_buff_len = size + 2;

	uint8_t mwrite_local[packet_buff_len];

	/* Write the register address of the sensor and send data byte*/

	mwrite_local[0] = (RegisterAddress >> 8) & 0xff;
	mwrite_local[1] = RegisterAddress & 0xff;
	memcpy(&mwrite_local[2], p_values, size);

	const unsigned i2c_buffer_size = sizeof(mwrite_local);
	/* transfer all data bytes in the buffer to the sensor */
	ret = transfer(&mwrite_local[0], i2c_buffer_size, nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

uint8_t VL53L5CX::RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
			  uint8_t *p_values, uint32_t size)
{
	int8_t ret;

	uint8_t mread_local[2];

	/* Write the register address of the sensor*/

	mread_local[0] = (RegisterAddress >> 8) & 0xff;
	mread_local[1] = RegisterAddress & 0xff;

	ret = transfer(&mread_local[0], sizeof(mread_local), nullptr, 0);

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	/* Read the incoming buffer from the sensor */
	// for (uint32_t i = 0; i < size; ++i) {
	//
	ret = transfer(nullptr, 0, &p_values[0], size);
	//
	// }

	if (ret != PX4_OK) {
		perf_count(_comms_errors);
		return ret;
	}

	return PX4_OK;
}

uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform)
{
	// uint8_t status = 0;
//TODO
	/* (Optional) Need to be implemented by customer. This function returns 0 if
	 * OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	// WaitMs(p_platform, 100);

	/* Set pin LPN of to HIGH */
	/* Set pin AVDD of to HIGH */
	/* Set pin VDDIO of  to HIGH */
	// WaitMs(p_platform, 100);

	return PX4_OK;
}

void VL53L5CX::SwapBuffer(uint8_t *buffer, uint16_t size)
{
	uint32_t i, tmp;

//TODO
	/* Example of possible implementation using <string.h> */
	for (i = 0; i < size; i = i + 4) {
		tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) |
		      (buffer[i + 3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

/* ST basic functions for ranging */
uint8_t VL53L5CX::vl53l5cx_is_alive(VL53L5CX_Configuration *p_dev, uint8_t *p_is_alive)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint8_t device_id, revision_id = 0;

	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= RdByte(&(p_dev->platform), 0, &device_id);
	status |= RdByte(&(p_dev->platform), 1, &revision_id);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);

	if (revision_id) {
		if ((device_id == (uint8_t)0xF0) && (revision_id == (uint8_t)0x02)) {
			*p_is_alive = 1;

		} else {
			*p_is_alive = 0;
		}
	}

	return status;
}

uint8_t VL53L5CX::vl53l5cx_poll_for_answer(VL53L5CX_Configuration *p_dev,
		uint8_t size, uint8_t pos,
		uint16_t address, uint8_t mask,
		uint8_t expected_value)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint8_t timeout = 0;

	do {
		status |= RdMulti(&(p_dev->platform), address, p_dev->temp_buffer, size);

		status |= WaitMs(&(p_dev->platform), 10);
		/* Poll timeout for init fix */

		if (timeout >= (uint8_t)200) { /* 2s timeout */
			// status |= (uint8_t)VL53L5CX_STATUS_TIMEOUT_ERROR;
			status |= p_dev->temp_buffer[2];
			break;

		} else if ((size >= (uint8_t)4) &&
			   (p_dev->temp_buffer[2] >= (uint8_t)0x7f)) {

			timeout++;
			status |= VL53L5CX_MCU_ERROR;
			break;

		} else {
			timeout++;
		}
	} while ((p_dev->temp_buffer[pos] & mask) != expected_value);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_send_offset_data(VL53L5CX_Configuration *p_dev,
		uint8_t resolution)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint32_t signal_grid[64];
	int16_t range_grid[64];
	uint8_t dss_4x4[] = {0x0F, 0x04, 0x04, 0x00, 0x08, 0x10, 0x10, 0x07};
	uint8_t footer[] = {0x00, 0x00, 0x00, 0x0F, 0x03, 0x01, 0x01, 0xE4};
	int8_t i, j;
	uint16_t k;

	(void)memcpy(p_dev->temp_buffer, p_dev->offset_data,
		     VL53L5CX_OFFSET_BUFFER_SIZE);

	/* Data extrapolation is required for 4X4 offset */
	if (resolution == (uint8_t)VL53L5CX_RESOLUTION_4X4) {
		(void)memcpy(&(p_dev->temp_buffer[0x10]), dss_4x4, sizeof(dss_4x4));
		SwapBuffer(p_dev->temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);
		(void)memcpy(signal_grid, &(p_dev->temp_buffer[0x3C]), sizeof(signal_grid));
		(void)memcpy(range_grid, &(p_dev->temp_buffer[0x140]), sizeof(range_grid));

		for (j = 0; j < (int8_t)4; j++) {
			for (i = 0; i < (int8_t)4; i++) {
				signal_grid[i + (4 * j)] =
					(signal_grid[(2 * i) + (16 * j) + (int8_t)0] +
					 signal_grid[(2 * i) + (16 * j) + (int8_t)1] +
					 signal_grid[(2 * i) + (16 * j) + (int8_t)8] +
					 signal_grid[(2 * i) + (16 * j) + (int8_t)9]) /
					(uint32_t)4;
				range_grid[i + (4 * j)] = (range_grid[(2 * i) + (16 * j)] +
							   range_grid[(2 * i) + (16 * j) + 1] +
							   range_grid[(2 * i) + (16 * j) + 8] +
							   range_grid[(2 * i) + (16 * j) + 9]) /
							  (int16_t)4;
			}
		}

		(void)memset(&range_grid[0x10], 0, (uint16_t)96);
		(void)memset(&signal_grid[0x10], 0, (uint16_t)192);
		(void)memcpy(&(p_dev->temp_buffer[0x3C]), signal_grid, sizeof(signal_grid));
		(void)memcpy(&(p_dev->temp_buffer[0x140]), range_grid, sizeof(range_grid));
		SwapBuffer(p_dev->temp_buffer, VL53L5CX_OFFSET_BUFFER_SIZE);
	}

	for (k = 0; k < (VL53L5CX_OFFSET_BUFFER_SIZE - (uint16_t)4); k++) {
		p_dev->temp_buffer[k] = p_dev->temp_buffer[k + (uint16_t)8];
	}

	(void)memcpy(&(p_dev->temp_buffer[0x1E0]), footer, 8);
	status |= WrMulti(&(p_dev->platform), 0x2e18, p_dev->temp_buffer,
			  VL53L5CX_OFFSET_BUFFER_SIZE);
	status |= vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS, 0xff,
					   0x03);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_poll_for_mcu_boot(VL53L5CX_Configuration *p_dev)
{
	uint8_t go2_status0 = 0;
	uint8_t go2_status1 = 0;
	uint8_t status = VL53L5CX_STATUS_OK;
	uint16_t timeout = 0;

	do {
		status |= RdByte(&(p_dev->platform), 0x06, &go2_status0);

		if ((go2_status0 & (uint8_t)0x80) != (uint8_t)0) {
			status |= RdByte(&(p_dev->platform), 0x07, &go2_status1);
			status |= go2_status1;
			break;
		}

		(void)WaitMs(&(p_dev->platform), 1);
		timeout++;

		if ((go2_status0 & (uint8_t)0x1) != (uint8_t)0) {
			break;
		}

	} while (timeout < (uint16_t)500);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_send_xtalk_data(VL53L5CX_Configuration *p_dev,
		uint8_t resolution)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint8_t res4x4[] = {0x0F, 0x04, 0x04, 0x17, 0x08, 0x10, 0x10, 0x07};
	uint8_t dss_4x4[] = {0x00, 0x78, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
	uint8_t profile_4x4[] = {0xA0, 0xFC, 0x01, 0x00};
	uint32_t signal_grid[64];
	int8_t i, j;

	(void)memcpy(p_dev->temp_buffer, &(p_dev->xtalk_data[0]),
		     VL53L5CX_XTALK_BUFFER_SIZE);

	/* Data extrapolation is required for 4X4 Xtalk */
	if (resolution == (uint8_t)VL53L5CX_RESOLUTION_4X4) {
		(void)memcpy(&(p_dev->temp_buffer[0x8]), res4x4, sizeof(res4x4));
		(void)memcpy(&(p_dev->temp_buffer[0x020]), dss_4x4, sizeof(dss_4x4));

		SwapBuffer(p_dev->temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE);
		(void)memcpy(signal_grid, &(p_dev->temp_buffer[0x34]), sizeof(signal_grid));

		for (j = 0; j < (int8_t)4; j++) {
			for (i = 0; i < (int8_t)4; i++) {
				signal_grid[i + (4 * j)] = (signal_grid[(2 * i) + (16 * j) + 0] +
							    signal_grid[(2 * i) + (16 * j) + 1] +
							    signal_grid[(2 * i) + (16 * j) + 8] +
							    signal_grid[(2 * i) + (16 * j) + 9]) /
							   (uint32_t)4;
			}
		}

		(void)memset(&signal_grid[0x10], 0, (uint32_t)192);
		(void)memcpy(&(p_dev->temp_buffer[0x34]), signal_grid, sizeof(signal_grid));
		SwapBuffer(p_dev->temp_buffer, VL53L5CX_XTALK_BUFFER_SIZE);
		(void)memcpy(&(p_dev->temp_buffer[0x134]), profile_4x4,
			     sizeof(profile_4x4));
		(void)memset(&(p_dev->temp_buffer[0x078]), 0,
			     (uint32_t)4 * sizeof(uint8_t));
	}

	status |= WrMulti(&(p_dev->platform), 0x2cf8, p_dev->temp_buffer,
			  VL53L5CX_XTALK_BUFFER_SIZE);
	status |= vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS, 0xff,
					   0x03);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_dci_read_data(VL53L5CX_Configuration *p_dev, uint8_t *data,
		uint32_t index, uint16_t data_size)
{
	// uint16_t timeout = 0;
	int16_t i;
	uint8_t status = VL53L5CX_STATUS_OK;

	// Read size is the data size plus cmd bytes (12)
	uint32_t rd_size = (uint32_t)data_size + (uint32_t)12;
	uint8_t cmd[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			 0x00, 0x0f, 0x00, 0x02, 0x00, 0x08
			};

	/* Check if tmp buffer is large enough */
	if ((data_size + (uint16_t)12) > (uint16_t)VL53L5CX_TEMPORARY_BUFFER_SIZE) {
		status |= VL53L5CX_STATUS_ERROR;
		PX4_INFO("Data to read too large");

	} else {
		cmd[0] = (uint8_t)(index >> 8);
		cmd[1] = (uint8_t)(index & (uint32_t)0xff);
		cmd[2] = (uint8_t)((data_size & (uint16_t)0xff0) >> 4);
		cmd[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

		/* Request data reading from FW */
		status |= WrMulti(&(p_dev->platform), (VL53L5CX_UI_CMD_END - (uint16_t)11),
				  cmd, sizeof(cmd));
		status |= vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS,
						   0xff, 0x03);

		/* Read new data sent (4 bytes header + data_size + 8 bytes footer) */
		// do {

		status |= RdMulti(&(p_dev->platform), VL53L5CX_UI_CMD_START,
				  p_dev->temp_buffer, rd_size);


		SwapBuffer(p_dev->temp_buffer, data_size + (uint16_t)12);

		/* Copy data from FW into input structure (-4 bytes to remove header) */
		for (i = 0; i < (int16_t)data_size; i++) {
			data[i] = p_dev->temp_buffer[i + 4];
		}
	}

	return status;
}

uint8_t VL53L5CX::vl53l5cx_dci_write_data(VL53L5CX_Configuration *p_dev, uint8_t *data,
		uint32_t index, uint16_t data_size)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	int16_t i;

	uint8_t headers[] = {0x00, 0x00, 0x00, 0x00};
	uint8_t footer[] = {0x00,
			    0x00,
			    0x00,
			    0x0f,
			    0x05,
			    0x01,
			    (uint8_t)((data_size + (uint16_t)8) >> 8),
			    (uint8_t)((data_size + (uint16_t)8) & (uint8_t)0xFF)
			   };

	uint16_t address =
		(uint16_t)VL53L5CX_UI_CMD_END - (data_size + (uint16_t)12) + (uint16_t)1;

	/* Check if cmd buffer is large enough */
	if ((data_size + (uint16_t)12) > (uint16_t)VL53L5CX_TEMPORARY_BUFFER_SIZE) {
		status |= VL53L5CX_STATUS_ERROR;

	} else {
		headers[0] = (uint8_t)(index >> 8);
		headers[1] = (uint8_t)(index & (uint32_t)0xff);
		headers[2] = (uint8_t)(((data_size & (uint16_t)0xff0) >> 4));
		headers[3] = (uint8_t)((data_size & (uint16_t)0xf) << 4);

		/* Copy data from structure to FW format (+4 bytes to add header) */
		SwapBuffer(data, data_size);

		for (i = (int16_t)data_size - (int16_t)1; i >= 0; i--) {
			p_dev->temp_buffer[i + 4] = data[i];
		}

		/* Add headers and footer */
		(void)memcpy(&p_dev->temp_buffer[0], headers, sizeof(headers));
		(void)memcpy(&p_dev->temp_buffer[data_size + (uint16_t)4], footer,
			     sizeof(footer));

		/* Send data to FW */
		status |= WrMulti(&(p_dev->platform), address, p_dev->temp_buffer,
				  (uint32_t)((uint32_t)data_size + (uint32_t)12));
		status |= vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS,
						   0xff, 0x03);

		SwapBuffer(data, data_size);
	}

	return status;
}

uint8_t VL53L5CX::vl53l5cx_dci_replace_data(VL53L5CX_Configuration *p_dev, uint8_t *data,
		uint32_t index, uint16_t data_size,
		uint8_t *new_data, uint16_t new_data_size,
		uint16_t new_data_pos)
{
	uint8_t status = VL53L5CX_STATUS_OK;

	status |= vl53l5cx_dci_read_data(p_dev, data, index, data_size);
	(void)memcpy(&(data[new_data_pos]), new_data, new_data_size);
	status |= vl53l5cx_dci_write_data(p_dev, data, index, data_size);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_get_resolution(VL53L5CX_Configuration *p_dev,
		uint8_t *p_resolution)
{
	uint8_t status = VL53L5CX_STATUS_OK;

	status |= vl53l5cx_dci_read_data(p_dev, p_dev->temp_buffer,
					 VL53L5CX_DCI_ZONE_CONFIG, 8);

	*p_resolution = p_dev->temp_buffer[0x00] * p_dev->temp_buffer[0x01];
	return status;
}

uint8_t VL53L5CX::vl53l5cx_set_resolution(VL53L5CX_Configuration *p_dev,
		uint8_t resolution)
{
	uint8_t status = VL53L5CX_STATUS_OK;

	switch (resolution) {
	case VL53L5CX_RESOLUTION_4X4:
		status |= vl53l5cx_dci_read_data(p_dev, p_dev->temp_buffer,
						 VL53L5CX_DCI_DSS_CONFIG, 16);
		p_dev->temp_buffer[0x04] = 64;
		p_dev->temp_buffer[0x06] = 64;
		p_dev->temp_buffer[0x09] = 4;
		status |= vl53l5cx_dci_write_data(p_dev, p_dev->temp_buffer,
						  VL53L5CX_DCI_DSS_CONFIG, 16);

		status |= vl53l5cx_dci_read_data(p_dev, p_dev->temp_buffer,
						 VL53L5CX_DCI_ZONE_CONFIG, 8);
		p_dev->temp_buffer[0x00] = 4;
		p_dev->temp_buffer[0x01] = 4;
		p_dev->temp_buffer[0x04] = 8;
		p_dev->temp_buffer[0x05] = 8;
		status |= vl53l5cx_dci_write_data(p_dev, p_dev->temp_buffer,
						  VL53L5CX_DCI_ZONE_CONFIG, 8);
		break;

	case VL53L5CX_RESOLUTION_8X8:
		status |= vl53l5cx_dci_read_data(p_dev, p_dev->temp_buffer,
						 VL53L5CX_DCI_DSS_CONFIG, 16);
		p_dev->temp_buffer[0x04] = 16;
		p_dev->temp_buffer[0x06] = 16;
		p_dev->temp_buffer[0x09] = 1;
		status |= vl53l5cx_dci_write_data(p_dev, p_dev->temp_buffer,
						  VL53L5CX_DCI_DSS_CONFIG, 16);

		status |= vl53l5cx_dci_read_data(p_dev, p_dev->temp_buffer,
						 VL53L5CX_DCI_ZONE_CONFIG, 8);
		p_dev->temp_buffer[0x00] = 8;
		p_dev->temp_buffer[0x01] = 8;
		p_dev->temp_buffer[0x04] = 4;
		p_dev->temp_buffer[0x05] = 4;
		status |= vl53l5cx_dci_write_data(p_dev, p_dev->temp_buffer,
						  VL53L5CX_DCI_ZONE_CONFIG, 8);

		break;

	default:
		status = VL53L5CX_STATUS_INVALID_PARAM;
		break;
	}

	status |= vl53l5cx_send_offset_data(p_dev, resolution);
	status |= vl53l5cx_send_xtalk_data(p_dev, resolution);
	return status;
}

uint8_t VL53L5CX::vl53l5cx_init(VL53L5CX_Configuration *p_dev)
{
	uint8_t tmp, status = VL53L5CX_STATUS_OK;
	uint8_t pipe_ctrl[] = {VL53L5CX_NB_TARGET_PER_ZONE, 0x00, 0x01, 0x00};
	uint32_t single_range = 0x01;

	p_dev->default_xtalk = (uint8_t *)VL53L5CX_DEFAULT_XTALK;
	p_dev->default_configuration = (uint8_t *)VL53L5CX_DEFAULT_CONFIGURATION;
	p_dev->is_auto_stop_enabled = (uint8_t)0x0;

	/* SW reboot sequence */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= WrByte(&(p_dev->platform), 0x0009, 0x04);
	status |= WrByte(&(p_dev->platform), 0x000F, 0x40);
	status |= WrByte(&(p_dev->platform), 0x000A, 0x03);
	status |= RdByte(&(p_dev->platform), 0x7FFF, &tmp);
	status |= WrByte(&(p_dev->platform), 0x000C, 0x01);


	status |= WrByte(&(p_dev->platform), 0x0101, 0x00);
	status |= WrByte(&(p_dev->platform), 0x0102, 0x00);
	status |= WrByte(&(p_dev->platform), 0x010A, 0x01);
	status |= WrByte(&(p_dev->platform), 0x4002, 0x01);
	status |= WrByte(&(p_dev->platform), 0x4002, 0x00);
	status |= WrByte(&(p_dev->platform), 0x010A, 0x03);
	status |= WrByte(&(p_dev->platform), 0x0103, 0x01);
	status |= WrByte(&(p_dev->platform), 0x000C, 0x00);
	status |= WrByte(&(p_dev->platform), 0x000F, 0x43);
	status |= WaitMs(&(p_dev->platform), 1);

	status |= WrByte(&(p_dev->platform), 0x000F, 0x40);
	status |= WrByte(&(p_dev->platform), 0x000A, 0x01);
	status |= WaitMs(&(p_dev->platform), 100);

	/* Wait for sensor booted (several ms required to get sensor ready ) */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= vl53l5cx_poll_for_answer(p_dev, 1, 0, 0x06, 0xff, 1);

	if (status != (uint8_t)0) {
		return PX4_ERROR;
	}

	status |= WrByte(&(p_dev->platform), 0x000E, 0x01);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);

	/* Enable FW access */
	status |= WrByte(&(p_dev->platform), 0x03, 0x0D);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x01);
	status |= vl53l5cx_poll_for_answer(p_dev, 1, 0, 0x21, 0x10, 0x10);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);

	/* Enable host access to GO1 */
	status |= RdByte(&(p_dev->platform), 0x7fff, &tmp);
	status |= WrByte(&(p_dev->platform), 0x0C, 0x01);

	/* Power ON status */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= WrByte(&(p_dev->platform), 0x101, 0x00);
	status |= WrByte(&(p_dev->platform), 0x102, 0x00);
	status |= WrByte(&(p_dev->platform), 0x010A, 0x01);
	status |= WrByte(&(p_dev->platform), 0x4002, 0x01);
	status |= WrByte(&(p_dev->platform), 0x4002, 0x00);
	status |= WrByte(&(p_dev->platform), 0x010A, 0x03);
	status |= WrByte(&(p_dev->platform), 0x103, 0x01);
	status |= WrByte(&(p_dev->platform), 0x400F, 0x00);
	status |= WrByte(&(p_dev->platform), 0x21A, 0x43);
	status |= WrByte(&(p_dev->platform), 0x21A, 0x03);
	status |= WrByte(&(p_dev->platform), 0x21A, 0x01);
	status |= WrByte(&(p_dev->platform), 0x21A, 0x00);
	status |= WrByte(&(p_dev->platform), 0x219, 0x00);
	status |= WrByte(&(p_dev->platform), 0x21B, 0x00);

	/* Wake up MCU */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= RdByte(&(p_dev->platform), 0x7fff, &tmp);
	status |= WrByte(&(p_dev->platform), 0x0C, 0x00);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x01);
	status |= WrByte(&(p_dev->platform), 0x20, 0x07);
	status |= WrByte(&(p_dev->platform), 0x20, 0x06);

	/* Download FW into VL53L5 */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x09);

	// WaitMs(&(p_dev->platform), 1);

	uint32_t fw_addr = 0x0;

	// Transfer initial 32k bytes
	for (uint8_t i = 0; i < 32; ++i) {

		status |= WrMulti(&(p_dev->platform), 0, (uint8_t *)&VL53L5CX_FIRMWARE[fw_addr], 0x400);
		// WaitMs(&(p_dev->platform), 5);
		fw_addr += 0x400;

	}

	status |= WrByte(&(p_dev->platform), 0x7fff, 0x0a);

	// Transfer next 32k bytes
	for (uint8_t i = 0; i < 32; ++i) {

		status |= WrMulti(&(p_dev->platform), 0, (uint8_t *)&VL53L5CX_FIRMWARE[fw_addr], 0x400);
		// WaitMs(&(p_dev->platform), 5);
		fw_addr += 0x400;

	}

	status |= WrByte(&(p_dev->platform), 0x7fff, 0x0b);

	for (uint8_t i = 0; i < 20; ++i) {

		status |= WrMulti(&(p_dev->platform), 0, (uint8_t *)&VL53L5CX_FIRMWARE[fw_addr], 0x400);
		// WaitMs(&(p_dev->platform), 5);
		fw_addr += 0x400;

	}

	// WaitMs(&(p_dev->platform), 10);
	// PX4_INFO("last transfer: %lu", fw_addr);

	status |= WrByte(&(p_dev->platform), 0x7fff, 0x01);
	/* Check if FW correctly downloaded */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);
	status |= WrByte(&(p_dev->platform), 0x03, 0x0D);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x01);
	status |= vl53l5cx_poll_for_answer(p_dev, 1, 0, 0x21, 0x10, 0x10);

	if (status != (uint8_t)0) {

		PX4_INFO("FW load fail");
		return PX4_ERROR;
	}

	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= RdByte(&(p_dev->platform), 0x7fff, &tmp);
	status |= WrByte(&(p_dev->platform), 0x0C, 0x01);

	/* Reset MCU and wait boot */
	status |= WrByte(&(p_dev->platform), 0x7FFF, 0x00);
	status |= WrByte(&(p_dev->platform), 0x114, 0x00);
	status |= WrByte(&(p_dev->platform), 0x115, 0x00);
	status |= WrByte(&(p_dev->platform), 0x116, 0x42);
	status |= WrByte(&(p_dev->platform), 0x117, 0x00);
	status |= WrByte(&(p_dev->platform), 0x0B, 0x00);
	status |= RdByte(&(p_dev->platform), 0x7fff, &tmp);
	status |= WrByte(&(p_dev->platform), 0x0C, 0x00);
	status |= WrByte(&(p_dev->platform), 0x0B, 0x01);
	status |= vl53l5cx_poll_for_mcu_boot(p_dev);

	// WaitMs(&(p_dev->platform), 20);
	if (status != (uint8_t)0) {

		PX4_INFO("reset MCU fail");
		return PX4_ERROR;
	}

	status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);

	/* Get offset NVM data and store them into the offset buffer */
	status |= WrMulti(&(p_dev->platform), 0x2fd8, (uint8_t *)VL53L5CX_GET_NVM_CMD,
			  sizeof(VL53L5CX_GET_NVM_CMD));
// Changed to 20, was 10
	// WaitMs(&(p_dev->platform), 20);


	/* poll fails and subsequent polls also fail, see fix in function */

	status |= vl53l5cx_poll_for_answer(p_dev, 4, 0, VL53L5CX_UI_CMD_STATUS, 0xff, 2);

	status |= RdMulti(&(p_dev->platform), VL53L5CX_UI_CMD_START,
			  p_dev->temp_buffer, VL53L5CX_NVM_DATA_SIZE);

	(void)memcpy(p_dev->offset_data, p_dev->temp_buffer,
		     VL53L5CX_OFFSET_BUFFER_SIZE);
	status |= vl53l5cx_send_offset_data(p_dev, VL53L5CX_RESOLUTION_4X4);

	/* Set default Xtalk shape. Send Xtalk to sensor */
	(void)memcpy(p_dev->xtalk_data, (uint8_t *)VL53L5CX_DEFAULT_XTALK,
		     VL53L5CX_XTALK_BUFFER_SIZE);
	status |= vl53l5cx_send_xtalk_data(p_dev, VL53L5CX_RESOLUTION_4X4);

	/* Send default configuration to VL53L5CX firmware */
	status |= WrMulti(&(p_dev->platform), 0x2c34, p_dev->default_configuration,
			  sizeof(VL53L5CX_DEFAULT_CONFIGURATION));
	status |= vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS, 0xff,
					   0x03);
	status |= vl53l5cx_dci_write_data(p_dev, (uint8_t *)&pipe_ctrl,
					  VL53L5CX_DCI_PIPE_CONTROL,
					  (uint16_t)sizeof(pipe_ctrl));
#if VL53L5CX_NB_TARGET_PER_ZONE != 1
	tmp = VL53L5CX_NB_TARGET_PER_ZONE;
	status |= vl53l5cx_dci_replace_data(p_dev, p_dev->temp_buffer,
					    VL53L5CX_DCI_FW_NB_TARGET, 16,
					    (uint8_t *)&tmp, 1, 0x0C);
#endif

	status |= vl53l5cx_dci_write_data(p_dev, (uint8_t *)&single_range,
					  VL53L5CX_DCI_SINGLE_RANGE,
					  (uint16_t)sizeof(single_range));

	tmp = (uint8_t)1;
	status |= vl53l5cx_dci_replace_data(p_dev, p_dev->temp_buffer,
					    VL53L5CX_GLARE_FILTER, 40,
					    (uint8_t *)&tmp, 1, 0x26);
	status |= vl53l5cx_dci_replace_data(p_dev, p_dev->temp_buffer,
					    VL53L5CX_GLARE_FILTER, 40,
					    (uint8_t *)&tmp, 1, 0x25);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_start_ranging(VL53L5CX_Configuration *p_dev)
{
	uint8_t resolution, status = VL53L5CX_STATUS_OK;
	uint16_t tmp;
	uint32_t i;
	uint32_t header_config[2] = {0, 0};
	// uint16_t timeout = 0;
	union Block_header *bh_ptr;
	uint8_t cmd[] = {0x00, 0x03, 0x00, 0x00};

	status |= vl53l5cx_get_resolution(&l5_config, &resolution);

	// status |= WaitMs(&(p_dev->platform), 10);
	PX4_INFO("Resolution set as: %d, status: %d", resolution, status);

	p_dev->data_read_size = 0;
	p_dev->streamcount = 255;
	/* Enable mandatory output (meta and common data) */
	uint32_t output_bh_enable[] = {0x00000007U, 0x00000000U, 0x00000000U,
				       0xC0000000U
				      };

	/* Send addresses of possible output */
	uint32_t output[] = {
		VL53L5CX_START_BH,         VL53L5CX_METADATA_BH,
		VL53L5CX_COMMONDATA_BH,    VL53L5CX_AMBIENT_RATE_BH,
		VL53L5CX_SPAD_COUNT_BH,    VL53L5CX_NB_TARGET_DETECTED_BH,
		VL53L5CX_SIGNAL_RATE_BH,   VL53L5CX_RANGE_SIGMA_MM_BH,
		VL53L5CX_DISTANCE_BH,      VL53L5CX_REFLECTANCE_BH,
		VL53L5CX_TARGET_STATUS_BH, VL53L5CX_MOTION_DETECT_BH
	};

	/* Enable selected outputs in the 'platform.h' file */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
	output_bh_enable[0] += (uint32_t)8;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
	output_bh_enable[0] += (uint32_t)16;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
	output_bh_enable[0] += (uint32_t)32;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
	output_bh_enable[0] += (uint32_t)64;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
	output_bh_enable[0] += (uint32_t)128;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
	output_bh_enable[0] += (uint32_t)256;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
	output_bh_enable[0] += (uint32_t)512;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
	output_bh_enable[0] += (uint32_t)1024;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
	output_bh_enable[0] += (uint32_t)2048;
#endif

	/* Update data size */
	for (i = 0; i < (uint32_t)(sizeof(output) / sizeof(uint32_t)); i++) {
		if ((output[i] == (uint8_t)0) ||
		    ((output_bh_enable[i / (uint32_t)32] &
		      ((uint32_t)1 << (i % (uint32_t)32))) == (uint32_t)0)) {
			continue;
		}

		bh_ptr = (union Block_header *) & (output[i]);

		if (((uint8_t)bh_ptr->type >= (uint8_t)0x1) &&
		    ((uint8_t)bh_ptr->type < (uint8_t)0x0d)) {
			if ((bh_ptr->idx >= (uint16_t)0x54d0) &&
			    (bh_ptr->idx < (uint16_t)(0x54d0 + 960))) {
				bh_ptr->size = resolution;

			} else {
				bh_ptr->size = (uint16_t)((uint16_t)resolution *
							  (uint16_t)VL53L5CX_NB_TARGET_PER_ZONE);
			}

			p_dev->data_read_size += bh_ptr->type * bh_ptr->size;

		} else {
			p_dev->data_read_size += bh_ptr->size;
		}

		p_dev->data_read_size += (uint32_t)4;
	}

	p_dev->data_read_size += (uint32_t)24;

	status |= vl53l5cx_dci_write_data(p_dev, (uint8_t *) & (output),
					  VL53L5CX_DCI_OUTPUT_LIST,
					  (uint16_t)sizeof(output));
	header_config[0] = p_dev->data_read_size;
	header_config[1] = i + (uint32_t)1;

	status |= vl53l5cx_dci_write_data(p_dev, (uint8_t *) & (header_config),
					  VL53L5CX_DCI_OUTPUT_CONFIG,
					  (uint16_t)sizeof(header_config));

	status |= vl53l5cx_dci_write_data(p_dev, (uint8_t *) & (output_bh_enable),
					  VL53L5CX_DCI_OUTPUT_ENABLES,
					  (uint16_t)sizeof(output_bh_enable));

	/* Start xshut bypass (interrupt mode) */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= WrByte(&(p_dev->platform), 0x09, 0x05);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);

	/* Start ranging session */
	status |= WrMulti(&(p_dev->platform), VL53L5CX_UI_CMD_END - (uint16_t)(4 - 1),
			  (uint8_t *)cmd, sizeof(cmd));
	status |= vl53l5cx_poll_for_answer(p_dev, 4, 1, VL53L5CX_UI_CMD_STATUS, 0xff,
					   0x03);
	/* Read ui range data content and compare if data size is the correct one */
	// do {

	status |=
		vl53l5cx_dci_read_data(p_dev, (uint8_t *)p_dev->temp_buffer, 0x5440, 12);


	if (sizeof(p_dev->temp_buffer) <= 0) {

		PX4_INFO("NO data received");
		// timeout++;
	}

	else if (sizeof(p_dev->temp_buffer) < 24) {

		PX4_INFO("Some ranging data received");
		// timeout++;
	}

	else {
		PX4_INFO("All ranging data received");
		// break;
	}

	// 	timeout ++;
	// }
	// while(timeout < (uint16_t)200);

	// PX4_INFO("Temp buf data byte 1: %d", p_dev->temp_buffer[0x8]);
	(void)memcpy(&tmp, &(p_dev->temp_buffer[0x8]), sizeof(tmp));
	// size_t data_rd_sz = sizeof(p_dev->data_read_size);

	if (tmp != p_dev->data_read_size) {
		PX4_INFO("Read data: %lu", p_dev->data_read_size);
		PX4_INFO("Tmp data: %d", tmp);
		status |= VL53L5CX_STATUS_ERROR;
	}

	PX4_INFO("Read ui range content status: %d", status);
	return status;
}

uint8_t VL53L5CX::vl53l5cx_stop_ranging(VL53L5CX_Configuration *p_dev)
{
	uint8_t tmp = 0, status = VL53L5CX_STATUS_OK;
	uint16_t timeout = 0;
	uint32_t auto_stop_flag = 0;

	status |= RdMulti(&(p_dev->platform), 0x2FFC, (uint8_t *)&auto_stop_flag, 4);

	if ((auto_stop_flag != (uint32_t)0x4FF) &&
	    (p_dev->is_auto_stop_enabled == (uint8_t)1)) {
		status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);

		/* Provoke MCU stop */
		status |= WrByte(&(p_dev->platform), 0x15, 0x16);
		status |= WrByte(&(p_dev->platform), 0x14, 0x01);

		/* Poll for G02 status 0 MCU stop */
		while (((tmp & (uint8_t)0x80) >> 7) == (uint8_t)0x00) {
			status |= RdByte(&(p_dev->platform), 0x6, &tmp);
			status |= WaitMs(&(p_dev->platform), 10);
			timeout++; /* Timeout reached after 5 seconds */

			if (timeout > (uint16_t)500) {
				status |= tmp;
				break;
			}
		}
	}

	/* Check GO2 status 1 if status is still OK */
	status |= RdByte(&(p_dev->platform), 0x6, &tmp);

	if ((tmp & (uint8_t)0x80) != (uint8_t)0) {
		status |= RdByte(&(p_dev->platform), 0x7, &tmp);

		if ((tmp != (uint8_t)0x84) && (tmp != (uint8_t)0x85)) {
			status |= tmp;
		}
	}

	/* Undo MCU stop */
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x00);
	status |= WrByte(&(p_dev->platform), 0x14, 0x00);
	status |= WrByte(&(p_dev->platform), 0x15, 0x00);

	/* Stop xshut bypass */
	status |= WrByte(&(p_dev->platform), 0x09, 0x04);
	status |= WrByte(&(p_dev->platform), 0x7fff, 0x02);

	return status;
}

uint8_t VL53L5CX::vl53l5cx_check_data_ready(VL53L5CX_Configuration *p_dev,
		uint8_t *p_isReady)
{
	uint8_t status = VL53L5CX_STATUS_OK;

	status |= RdMulti(&(p_dev->platform), 0x0, p_dev->temp_buffer, 4);

	if ((p_dev->temp_buffer[0] != p_dev->streamcount) &&
	    (p_dev->temp_buffer[0] != (uint8_t)255) &&
	    (p_dev->temp_buffer[1] == (uint8_t)0x5) &&
	    ((p_dev->temp_buffer[2] & (uint8_t)0x5) == (uint8_t)0x5) &&
	    ((p_dev->temp_buffer[3] & (uint8_t)0x10) == (uint8_t)0x10)) {
		*p_isReady = (uint8_t)1;
		p_dev->streamcount = p_dev->temp_buffer[0];

	} else {
		if ((p_dev->temp_buffer[3] & (uint8_t)0x80) != (uint8_t)0) {
			status |= p_dev->temp_buffer[2]; /* Return GO2 error status */
		}

		*p_isReady = 0;
	}

	return status;
}

uint8_t VL53L5CX::vl53l5cx_get_ranging_data(VL53L5CX_Configuration *p_dev,
		VL53L5CX_ResultsData *p_results)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	union Block_header temp;
	uint16_t header_id, footer_id;
	uint32_t i, msize;
	status |= RdMulti(&(p_dev->platform), 0x0, p_dev->temp_buffer,
			  p_dev->data_read_size);
	p_dev->streamcount = p_dev->temp_buffer[0];
	SwapBuffer(p_dev->temp_buffer, (uint16_t)p_dev->data_read_size);

	/* Start conversion at position 16 to avoid headers */
	for (i = (uint32_t)16; i < (uint32_t)p_dev->data_read_size;
	     i += (uint32_t)4) {

		void *mptr = &p_dev->temp_buffer[i];
		//TODO (check) avoid alignment compiler warning
		memcpy(&temp, mptr, sizeof(union Block_header));
		// bh_ptr = (union Block_header *) & (p_dev->temp_buffer[i]);

		if ((temp.type > (uint32_t)0x1) && (temp.type < (uint32_t)0xd)) {
			msize = temp.type * temp.size;

		} else {
			msize = temp.size;
		}

		switch (temp.idx) {
		case VL53L5CX_METADATA_IDX:
			p_results->silicon_temp_degc =
				(int8_t)p_dev->temp_buffer[i + (uint32_t)12];
			break;

#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD

		case VL53L5CX_AMBIENT_RATE_IDX:
			(void)memcpy(p_results->ambient_per_spad,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED

		case VL53L5CX_SPAD_COUNT_IDX:
			(void)memcpy(p_results->nb_spads_enabled,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED

		case VL53L5CX_NB_TARGET_DETECTED_IDX:
			(void)memcpy(p_results->nb_target_detected,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD

		case VL53L5CX_SIGNAL_RATE_IDX:
			(void)memcpy(p_results->signal_per_spad,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM

		case VL53L5CX_RANGE_SIGMA_MM_IDX:
			(void)memcpy(p_results->range_sigma_mm,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_DISTANCE_MM

		case VL53L5CX_DISTANCE_IDX:
			(void)memcpy(p_results->distance_mm,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT

		case VL53L5CX_REFLECTANCE_EST_PC_IDX:
			(void)memcpy(p_results->reflectance,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_TARGET_STATUS

		case VL53L5CX_TARGET_STATUS_IDX:
			(void)memcpy(p_results->target_status,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR

		case VL53L5CX_MOTION_DETEC_IDX:
			(void)memcpy(&p_results->motion_indicator,
				     &(p_dev->temp_buffer[i + (uint32_t)4]), msize);
			break;
#endif

		default:
			break;
		}

		i += msize;
	}

#ifndef VL53L5CX_USE_RAW_FORMAT

	/* Convert data into their real format */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD

	for (i = 0; i < (uint32_t)VL53L5CX_RESOLUTION_8X8; i++) {
		p_results->ambient_per_spad[i] /= (uint32_t)2048;
	}

#endif

	for (i = 0;
	     i < (uint32_t)(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE);
	     i++) {
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
		p_results->distance_mm[i] /= 4;

		if (p_results->distance_mm[i] < 0) {
			p_results->distance_mm[i] = 0;
		}

#endif
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
		p_results->reflectance[i] /= (uint8_t)2;
#endif
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
		p_results->range_sigma_mm[i] /= (uint16_t)128;
#endif
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
		p_results->signal_per_spad[i] /= (uint32_t)2048;
#endif
	}

	/* Set target status to 255 if no target is detected for this zone */
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED

	for (i = 0; i < (uint32_t)VL53L5CX_RESOLUTION_8X8; i++) {
		if (p_results->nb_target_detected[i] == (uint8_t)0) {
			for (uint32_t j = 0; j < (uint32_t)VL53L5CX_NB_TARGET_PER_ZONE; j++) {
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
				p_results->target_status[((uint32_t)VL53L5CX_NB_TARGET_PER_ZONE *
												 (uint32_t)i) +
												j] = (uint8_t)255;
#endif
			}
		}
	}

#endif

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR

	for (i = 0; i < (uint32_t)32; i++) {
		p_results->motion_indicator.motion[i] /= (uint32_t)65535;
	}

#endif

#endif

	/* Check if footer id and header id are matching. This allows to detect
	 * corrupted frames */
	header_id = ((uint16_t)(p_dev->temp_buffer[0x8]) << 8) & 0xFF00U;
	header_id |= ((uint16_t)(p_dev->temp_buffer[0x9])) & 0x00FFU;

	footer_id =
		((uint16_t)(p_dev->temp_buffer[p_dev->data_read_size - (uint32_t)4])
		 << 8) &
		0xFF00U;
	footer_id |=
		((uint16_t)(p_dev->temp_buffer[p_dev->data_read_size - (uint32_t)3])) &
		0xFFU;

	if (header_id != footer_id) {
		status |= VL53L5CX_STATUS_CORRUPTED_FRAME;
	}

	return status;
}

uint8_t VL53L5CX::vl53l5cx_set_ranging_mode(VL53L5CX_Configuration *p_dev,
		uint8_t ranging_mode)
{
	uint8_t status = VL53L5CX_STATUS_OK;
	uint32_t single_range = 0x00;

	status |= vl53l5cx_dci_read_data(p_dev, p_dev->temp_buffer,
					 VL53L5CX_DCI_RANGING_MODE, 8);

	switch (ranging_mode) {
	case VL53L5CX_RANGING_MODE_CONTINUOUS:
		p_dev->temp_buffer[0x01] = 0x1;
		p_dev->temp_buffer[0x03] = 0x3;
		single_range = 0x00;
		break;

	case VL53L5CX_RANGING_MODE_AUTONOMOUS:
		p_dev->temp_buffer[0x01] = 0x3;
		p_dev->temp_buffer[0x03] = 0x2;
		single_range = 0x01;
		break;

	default:
		status = VL53L5CX_STATUS_INVALID_PARAM;
		break;
	}

	status |= vl53l5cx_dci_write_data(p_dev, p_dev->temp_buffer,
					  VL53L5CX_DCI_RANGING_MODE, (uint16_t)8);

	status |= vl53l5cx_dci_write_data(p_dev, (uint8_t *)&single_range,
					  VL53L5CX_DCI_SINGLE_RANGE,
					  (uint16_t)sizeof(single_range));

	return status;
}

extern "C" __EXPORT int vl53l5cx_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = VL53L5CX;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;
	cli.rotation = (Rotation)distance_sensor_s::ROTATION_DOWNWARD_FACING;
	cli.i2c_address = VL53L5CX_UNSHIFTED_I2C_ADDR;

	while ((ch = cli.getOpt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (Rotation)atoi(cli.optArg());
			break;
		}
	}

	const char *verb = cli.optArg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_DIST_DEVTYPE_VL53L5CX);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}
