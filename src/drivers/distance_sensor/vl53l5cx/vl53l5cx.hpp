
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

/**
 * @file VL53L5CX.hpp
 *
 * Driver for the ST VL53L5CX ToF Sensor connected via I2C.
 */

#pragma once

#include <px4_log.h>
#include <drivers/device/i2c.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>


/* ST API Preprocessor*/
#define VL53L5CX_RANGING_MODE_CONTINUOUS ((uint8_t)1U)
#define VL53L5CX_RANGING_MODE_AUTONOMOUS ((uint8_t)3U)
#define VL53L5CX_POWER_MODE_SLEEP ((uint8_t)0U)
#define VL53L5CX_POWER_MODE_WAKEUP ((uint8_t)1U)
#define VL53L5CX_STATUS_OK ((uint8_t)0U)
#define VL53L5CX_STATUS_TIMEOUT_ERROR ((uint8_t)1U)
#define VL53L5CX_STATUS_CORRUPTED_FRAME ((uint8_t)2U)
#define VL53L5CX_STATUS_CRC_CSUM_FAILED ((uint8_t)3U)
#define VL53L5CX_MCU_ERROR ((uint8_t)66U)
#define VL53L5CX_STATUS_INVALID_PARAM ((uint8_t)127U)
#define VL53L5CX_STATUS_ERROR ((uint8_t)255U)

#define VL53L5CX_API_REVISION "VL53L5CX_1.3.9"
#define VL53L5CX_DEFAULT_I2C_ADDRESS ((uint16_t)0x52)
#define VL53L5CX_RESOLUTION_4X4 ((uint8_t)16U)
#define VL53L5CX_RESOLUTION_8X8 ((uint8_t)64U)
#define VL53L5CX_NB_TARGET_PER_ZONE 1U

#define VL53L5CX_START_BH ((uint32_t)0x0000000DU)
#define VL53L5CX_METADATA_BH ((uint32_t)0x54B400C0U)
#define VL53L5CX_COMMONDATA_BH ((uint32_t)0x54C00040U)
#define VL53L5CX_AMBIENT_RATE_BH ((uint32_t)0x54D00104U)
#define VL53L5CX_NB_TARGET_DETECTED_BH ((uint32_t)0x57D00401U)
#define VL53L5CX_SPAD_COUNT_BH ((uint32_t)0x55D00404U)
#define VL53L5CX_SIGNAL_RATE_BH ((uint32_t)0x58900404U)
#define VL53L5CX_RANGE_SIGMA_MM_BH ((uint32_t)0x64900402U)
#define VL53L5CX_DISTANCE_BH ((uint32_t)0x66900402U)
#define VL53L5CX_REFLECTANCE_BH ((uint32_t)0x6A900401U)
#define VL53L5CX_TARGET_STATUS_BH ((uint32_t)0x6B900401U)
#define VL53L5CX_MOTION_DETECT_BH ((uint32_t)0xCC5008C0U)

#define VL53L5CX_METADATA_IDX ((uint16_t)0x54B4U)
#define VL53L5CX_SPAD_COUNT_IDX ((uint16_t)0x55D0U)
#define VL53L5CX_AMBIENT_RATE_IDX ((uint16_t)0x54D0U)
#define VL53L5CX_NB_TARGET_DETECTED_IDX ((uint16_t)0x57D0U)
#define VL53L5CX_SIGNAL_RATE_IDX ((uint16_t)0x5890U)
#define VL53L5CX_RANGE_SIGMA_MM_IDX ((uint16_t)0x6490U)
#define VL53L5CX_DISTANCE_IDX ((uint16_t)0x6690U)
#define VL53L5CX_REFLECTANCE_EST_PC_IDX ((uint16_t)0x6A90U)
#define VL53L5CX_TARGET_STATUS_IDX ((uint16_t)0x6B90U)
#define VL53L5CX_MOTION_DETEC_IDX ((uint16_t)0xCC50U)

/* ST API Inner macro */
#define VL53L5CX_NVM_DATA_SIZE ((uint16_t)492U)
#define VL53L5CX_CONFIGURATION_SIZE ((uint16_t)972U)
#define VL53L5CX_OFFSET_BUFFER_SIZE ((uint16_t)488U)
#define VL53L5CX_XTALK_BUFFER_SIZE ((uint16_t)776U)

#define VL53L5CX_DCI_ZONE_CONFIG ((uint16_t)0x5450U)
#define VL53L5CX_DCI_FREQ_HZ ((uint16_t)0x5458U)
#define VL53L5CX_DCI_INT_TIME ((uint16_t)0x545CU)
#define VL53L5CX_DCI_FW_NB_TARGET ((uint16_t)0x5478)
#define VL53L5CX_DCI_RANGING_MODE ((uint16_t)0xAD30U)
#define VL53L5CX_DCI_DSS_CONFIG ((uint16_t)0xAD38U)
#define VL53L5CX_DCI_TARGET_ORDER ((uint16_t)0xAE64U)
#define VL53L5CX_DCI_SHARPENER ((uint16_t)0xAED8U)
#define VL53L5CX_DCI_INTERNAL_CP ((uint16_t)0xB39CU)
#define VL53L5CX_DCI_SYNC_PIN ((uint16_t)0xB5F0U)
#define VL53L5CX_DCI_MOTION_DETECTOR_CFG ((uint16_t)0xBFACU)
#define VL53L5CX_DCI_SINGLE_RANGE ((uint16_t)0xD964U)
#define VL53L5CX_DCI_OUTPUT_CONFIG ((uint16_t)0xD968U)
#define VL53L5CX_DCI_OUTPUT_ENABLES ((uint16_t)0xD970U)
#define VL53L5CX_DCI_OUTPUT_LIST ((uint16_t)0xD980U)
#define VL53L5CX_DCI_PIPE_CONTROL ((uint16_t)0xDB80U)
#define VL53L5CX_GLARE_FILTER ((uint16_t)0xE108U)

#define VL53L5CX_UI_CMD_STATUS ((uint16_t)0x2C00U)
#define VL53L5CX_UI_CMD_START ((uint16_t)0x2C04U)
#define VL53L5CX_UI_CMD_END ((uint16_t)0x2FFFU)

/* All disabled except distance in mm and target status */

#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L5CX_DISABLE_MOTION_INDICATOR


#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define L5CX_AMB_SIZE 260U
#else
#define L5CX_AMB_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define L5CX_SPAD_SIZE 260U
#else
#define L5CX_SPAD_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
#define L5CX_NTAR_SIZE 68U
#else
#define L5CX_NTAR_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define L5CX_SPS_SIZE ((256U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_SPS_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define L5CX_SIGR_SIZE ((128U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_SIGR_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_DISTANCE_MM
#define L5CX_DIST_SIZE ((128U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_DIST_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define L5CX_RFLEST_SIZE ((64U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_RFLEST_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_TARGET_STATUS
#define L5CX_STA_SIZE ((64U * VL53L5CX_NB_TARGET_PER_ZONE) + 4U)
#else
#define L5CX_STA_SIZE 0U
#endif

#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
#define L5CX_MOT_SIZE 144U
#else
#define L5CX_MOT_SIZE 0U
#endif

/**
 * @brief Macro VL53L5CX_MAX_RESULTS_SIZE indicates the maximum size used by
 * output through I2C. Value 40 corresponds to headers + meta-data + common-data
 * and 20 corresponds to the footer.
 */

#define VL53L5CX_MAX_RESULTS_SIZE                                              \
	(40U + L5CX_AMB_SIZE + L5CX_SPAD_SIZE + L5CX_NTAR_SIZE + L5CX_SPS_SIZE +     \
	 L5CX_SIGR_SIZE + L5CX_DIST_SIZE + L5CX_RFLEST_SIZE + L5CX_STA_SIZE +        \
	 L5CX_MOT_SIZE + 20U)

/**
 * @brief Macro VL53L5CX_TEMPORARY_BUFFER_SIZE can be used to know the size of
 * the temporary buffer. The minimum size is 1024, and the maximum depends of
 * the output configuration.
 */

#if VL53L5CX_MAX_RESULTS_SIZE < 1024U
#define VL53L5CX_TEMPORARY_BUFFER_SIZE ((uint32_t)1024U)
#else
#define VL53L5CX_TEMPORARY_BUFFER_SIZE ((uint32_t)VL53L5CX_MAX_RESULTS_SIZE)
#endif

union Block_header {
	uint32_t bytes;
	struct {
		uint32_t type : 4;
		uint32_t size : 12;
		uint32_t idx : 16;
	};
};
/* ST Platform */
typedef struct {

	uint16_t address;

} VL53L5CX_Platform;

/* ST API Configuration*/
//TODO: fill out configuration struct
typedef struct {
	/* Platform, filled by customer into the 'platform.h' file */
	VL53L5CX_Platform platform;
	/* Results streamcount, value auto-incremented at each range */
	uint8_t streamcount;
	/* Size of data read though I2C */
	uint32_t data_read_size;
	/* Address of default configuration buffer */
	uint8_t *default_configuration;
	/* Address of default Xtalk buffer */
	uint8_t *default_xtalk;
	/* Offset buffer */
	uint8_t offset_data[VL53L5CX_OFFSET_BUFFER_SIZE];
	/* Xtalk buffer */
	uint8_t xtalk_data[VL53L5CX_XTALK_BUFFER_SIZE];
	/* Temporary buffer used for internal driver processing */
	uint8_t temp_buffer[VL53L5CX_TEMPORARY_BUFFER_SIZE];
	/* Auto-stop flag for stopping the sensor */
	uint8_t is_auto_stop_enabled;
} VL53L5CX_Configuration;

/* ST API Results */
typedef struct {
	/* Internal sensor silicon temperature */
	int8_t silicon_temp_degc;

	/* Ambient noise in kcps/spads */
#ifndef VL53L5CX_DISABLE_AMBIENT_PER_SPAD
	uint32_t ambient_per_spad[VL53L5CX_RESOLUTION_8X8];
#endif

	/* Number of valid target detected for 1 zone */
#ifndef VL53L5CX_DISABLE_NB_TARGET_DETECTED
	uint8_t nb_target_detected[VL53L5CX_RESOLUTION_8X8];
#endif

	/* Number of spads enabled for this ranging */
#ifndef VL53L5CX_DISABLE_NB_SPADS_ENABLED
	uint32_t nb_spads_enabled[VL53L5CX_RESOLUTION_8X8];
#endif

	/* Signal returned to the sensor in kcps/spads */
#ifndef VL53L5CX_DISABLE_SIGNAL_PER_SPAD
	uint32_t
	signal_per_spad[(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE)];
#endif

	/* Sigma of the current distance in mm */
#ifndef VL53L5CX_DISABLE_RANGE_SIGMA_MM
	uint16_t
	range_sigma_mm[(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE)];
#endif

	/* Measured distance in mm */
#ifndef VL53L5CX_DISABLE_DISTANCE_MM
	int16_t distance_mm[(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE)];
#endif

	/* Estimated reflectance in percent */
#ifndef VL53L5CX_DISABLE_REFLECTANCE_PERCENT
	uint8_t reflectance[(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE)];
#endif

	/* Status indicating the measurement validity (5 & 9 means ranging OK)*/
#ifndef VL53L5CX_DISABLE_TARGET_STATUS
	uint8_t
	target_status[(VL53L5CX_RESOLUTION_8X8 * VL53L5CX_NB_TARGET_PER_ZONE)];
#endif

	/* Motion detector results */
#ifndef VL53L5CX_DISABLE_MOTION_INDICATOR
	struct {
		uint32_t global_indicator_1;
		uint32_t global_indicator_2;
		uint8_t status;
		uint8_t nb_of_detected_aggregates;
		uint8_t nb_of_aggregates;
		uint8_t spare;
		uint32_t motion[32];
	} motion_indicator;
#endif

} VL53L5CX_ResultsData;

class VL53L5CX : public device::I2C, public I2CSPIDriver<VL53L5CX>
{
public:
	VL53L5CX(const I2CSPIDriverConfig &config);

	~VL53L5CX() override;

	static void print_usage();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void print_status() override;

	virtual int init() override;

	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 */
	void RunImpl();


	VL53L5CX_Configuration l5_config;
	VL53L5CX_ResultsData l5_results;

private:
	int probe() override;

	/**
	 * Collects the most recent sensor measurement data from the i2c bus.
	 */
	int collect();

	/* ST function prototypes*/
	uint8_t vl53l5cx_is_alive(VL53L5CX_Configuration *p_dev, uint8_t *p_is_alive);

	uint8_t vl53l5cx_init(VL53L5CX_Configuration *p_dev);

	uint8_t vl53l5cx_set_i2c_address(VL53L5CX_Configuration *p_dev,
					 uint16_t i2c_address);

	uint8_t vl53l5cx_start_ranging(VL53L5CX_Configuration *p_dev);

	uint8_t vl53l5cx_stop_ranging(VL53L5CX_Configuration *p_dev);

	uint8_t vl53l5cx_check_data_ready(VL53L5CX_Configuration *p_dev,
					  uint8_t *p_isReady);

	uint8_t vl53l5cx_get_ranging_data(VL53L5CX_Configuration *p_dev,
					  VL53L5CX_ResultsData *p_results);

	uint8_t vl53l5cx_get_resolution(VL53L5CX_Configuration *p_dev,
					uint8_t *p_resolution);

	uint8_t vl53l5cx_set_resolution(VL53L5CX_Configuration *p_dev,
					uint8_t resolution);

	uint8_t vl53l5cx_get_ranging_frequency_hz(VL53L5CX_Configuration *p_dev,
			uint8_t *p_frequency_hz);


	uint8_t vl53l5cx_set_ranging_frequency_hz(VL53L5CX_Configuration *p_dev,
			uint8_t frequency_hz);

	uint8_t vl53l5cx_set_ranging_mode(VL53L5CX_Configuration *p_dev,
					  uint8_t ranging_mode);

	uint8_t vl53l5cx_dci_read_data(VL53L5CX_Configuration *p_dev, uint8_t *data,
				       uint32_t index, uint16_t data_size);

	uint8_t vl53l5cx_dci_write_data(VL53L5CX_Configuration *p_dev, uint8_t *data,
					uint32_t index, uint16_t data_size);

	uint8_t vl53l5cx_dci_replace_data(VL53L5CX_Configuration *p_dev,
					  uint8_t *data, uint32_t index,
					  uint16_t data_size, uint8_t *new_data,
					  uint16_t new_data_size,
					  uint16_t new_data_pos);

	uint8_t vl53l5cx_poll_for_answer(VL53L5CX_Configuration *p_dev,
					 uint8_t size, uint8_t pos,
					 uint16_t address, uint8_t mask,
					 uint8_t expected_value);
	uint8_t vl53l5cx_send_xtalk_data(VL53L5CX_Configuration *p_dev,
					 uint8_t resolution);
	uint8_t vl53l5cx_send_offset_data(VL53L5CX_Configuration *p_dev,
					  uint8_t resolution);
	uint8_t vl53l5cx_poll_for_mcu_boot(VL53L5CX_Configuration *p_dev);

	/* ST PLATFORM DECLARATION */
	uint8_t RdByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
		       uint8_t *p_value);
	uint8_t WrByte(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
		       uint8_t value);
	uint8_t RdMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
			uint8_t *p_values, uint32_t size);

	uint8_t WrMulti(VL53L5CX_Platform *p_platform, uint16_t RegisterAddress,
			uint8_t *p_values, uint32_t size);
	uint8_t Reset_Sensor(VL53L5CX_Platform *p_platform);
	void SwapBuffer(uint8_t *buffer, uint16_t size);
	uint8_t WaitMs(VL53L5CX_Platform *p_platform, uint32_t TimeMs);
	/* PX4 */
	int8_t VL53L5CX_SensorInit();
	int8_t VL53L5CX_StartRanging();
	int8_t VL53L5CX_CheckForDataReady(uint8_t *dataready);
	int8_t VL53L5CX_StopRanging();
	int8_t VL53L5CX_GetDistance(uint16_t *distance);

	PX4Rangefinder _px4_rangefinder;

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME ": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME ": read")};
};
