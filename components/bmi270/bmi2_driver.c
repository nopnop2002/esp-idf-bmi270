/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "driver/i2c_master.h"
#include "esp_rom_sys.h"
#include "esp_log.h"

#include "bmi2_driver.h"
#include "bmi2_defs.h"

#define I2C_NUM I2C_NUM_0
#define I2C_TICKS_TO_WAIT 100 // Maximum ticks to wait before issuing a timeout.
#define __DEBUG__ 0

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;

/******************************************************************************/
/*!				   User interface functions									  */

/*!
 * I2C read function map to ESP-IDF platform
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_read(uint8_t reg_addr, uint8_t* reg_data, uint32_t length, void *intf_ptr) {
	uint8_t out_buf[1];
	out_buf[0] = reg_addr;
	ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, out_buf, sizeof(out_buf), reg_data, length, I2C_TICKS_TO_WAIT));

#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "reg_addr=0x%x length=%u", reg_addr, length);
	for (int i=0;i<length;i++) {
		ESP_LOGI(__FUNCTION__, "reg_data[%d]=0x%x", i, reg_data[i]);
	}
#endif

	return 0;
}

/*!
 * I2C write function map to ESP-IDF platform
 */
BMI2_INTF_RETURN_TYPE bmi2_i2c_write(uint8_t reg_addr, const uint8_t* reg_data, uint32_t length, void *intf_ptr) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "reg_addr=0x%x length=%u", reg_addr, length);
	for (int i=0;i<length;i++) {
		ESP_LOGI(__FUNCTION__, "reg_data[%d]=0x%x", i, reg_data[i]);
	}
#endif

	uint8_t *out_buf;
	out_buf = malloc(length+1);
	if (out_buf == NULL) {
		ESP_LOGE(__FUNCTION__, "malloc fail");
		return 0;
	}
	out_buf[0] = reg_addr;
	for(int i=0;i<length;i++) out_buf[i+1] = reg_data[i];
	ESP_LOGD(__FUNCTION__, "dev_handle=%d", dev_handle);
	ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, out_buf, length+1, I2C_TICKS_TO_WAIT));
	free(out_buf);

	return 0;
}

/*!
 * Delay function map to ESP-IDF platform
 */
void bmi2_delay_us(uint32_t period, void *intf_ptr) {
#if __DEBUG__
	ESP_LOGI(__FUNCTION__, "period=%" PRIu32, period);
#endif
	esp_rom_delay_us(period);
}

/*!
 *	@brief Function to select the interface between SPI and I2C.
 */
void bmi2_interface_init(struct bmi2_dev *dev, uint8_t intf) {
	i2c_master_bus_config_t i2c_mst_config = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.i2c_port = I2C_NUM,
		.scl_io_num = CONFIG_GPIO_SCL,
		.sda_io_num = CONFIG_GPIO_SDA,
		.flags.enable_internal_pullup = true,
	};

	//i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
	ESP_LOGI(__FUNCTION__, "i2c_new_master_bus");

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = CONFIG_I2C_ADDR,
		.scl_speed_hz = 400000,
	};

	//i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
	ESP_LOGI(__FUNCTION__, "i2c_master_bus_add_device dev_handle=%d", dev_handle);

	//I2C address is 0x68 (if SDO-pin is gnd) or 0x69 (if SDO-pin is vddio).
	//dev_addr = CONFIG_I2C_ADDR;
	dev->intf = BMI2_I2C_INTF;
	dev->read = bmi2_i2c_read;
	dev->write = bmi2_i2c_write;
	dev->delay_us = bmi2_delay_us;
}

/*!
 *	@brief Prints the execution status of the APIs.
 */
void bmi2_error_codes_print_result(int8_t rslt)
{
	switch (rslt)
	{
		case BMI2_OK:

			/* Do nothing */
			break;

		case BMI2_W_FIFO_EMPTY:
			printf("Warning [%d] : FIFO empty\r\n", rslt);
			break;
		case BMI2_W_PARTIAL_READ:
			printf("Warning [%d] : FIFO partial read\r\n", rslt);
			break;
		case BMI2_E_NULL_PTR:
			printf(
				"Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
				rslt);
			break;

		case BMI2_E_COM_FAIL:
			printf(
				"Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
				rslt);
			break;

		case BMI2_E_DEV_NOT_FOUND:
			printf("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
				   rslt);
			break;

		case BMI2_E_INVALID_SENSOR:
			printf(
				"Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
				rslt);
			break;

		case BMI2_E_SELF_TEST_FAIL:
			printf(
				"Error [%d] : Self-test failed error. It occurs when the validation of accel self-test data is " "not satisfied\r\n",
				rslt);
			break;

		case BMI2_E_INVALID_INT_PIN:
			printf(
				"Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
				rslt);
			break;

		case BMI2_E_OUT_OF_RANGE:
			printf(
				"Error [%d] : Out of range error. It occurs when the data exceeds from filtered or unfiltered data from " "fifo and also when the range exceeds the maximum range for accel and gyro while performing FOC\r\n",
				rslt);
			break;

		case BMI2_E_ACC_INVALID_CFG:
			printf(
				"Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x40\r\n",
				rslt);
			break;

		case BMI2_E_GYRO_INVALID_CFG:
			printf(
				"Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x42\r\n",
				rslt);
			break;

		case BMI2_E_ACC_GYR_INVALID_CFG:
			printf(
				"Error [%d] : Invalid Accel-Gyro configuration error. It occurs when there is a error in accel and gyro" " configuration registers which could be one among range, BW or filter performance in reg address 0x40 " "and 0x42\r\n",
				rslt);
			break;

		case BMI2_E_CONFIG_LOAD:
			printf(
				"Error [%d] : Configuration load error. It occurs when failure observed while loading the configuration " "into the sensor\r\n",
				rslt);
			break;

		case BMI2_E_INVALID_PAGE:
			printf(
				"Error [%d] : Invalid page error. It occurs due to failure in writing the correct feature configuration " "from selected page\r\n",
				rslt);
			break;

		case BMI2_E_SET_APS_FAIL:
			printf(
				"Error [%d] : APS failure error. It occurs due to failure in write of advance power mode configuration " "register\r\n",
				rslt);
			break;

		case BMI2_E_AUX_INVALID_CFG:
			printf(
				"Error [%d] : Invalid AUX configuration error. It occurs when the auxiliary interface settings are not " "enabled properly\r\n",
				rslt);
			break;

		case BMI2_E_AUX_BUSY:
			printf(
				"Error [%d] : AUX busy error. It occurs when the auxiliary interface buses are engaged while configuring" " the AUX\r\n",
				rslt);
			break;

		case BMI2_E_REMAP_ERROR:
			printf(
				"Error [%d] : Remap error. It occurs due to failure in assigning the remap axes data for all the axes " "after change in axis position\r\n",
				rslt);
			break;

		case BMI2_E_GYR_USER_GAIN_UPD_FAIL:
			printf(
				"Error [%d] : Gyro user gain update fail error. It occurs when the reading of user gain update status " "fails\r\n",
				rslt);
			break;

		case BMI2_E_SELF_TEST_NOT_DONE:
			printf(
				"Error [%d] : Self-test not done error. It occurs when the self-test process is ongoing or not " "completed\r\n",
				rslt);
			break;

		case BMI2_E_INVALID_INPUT:
			printf("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
			break;

		case BMI2_E_INVALID_STATUS:
			printf("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
			break;

		case BMI2_E_CRT_ERROR:
			printf("Error [%d] : CRT error. It occurs when the CRT test has failed\r\n", rslt);
			break;

		case BMI2_E_ST_ALREADY_RUNNING:
			printf(
				"Error [%d] : Self-test already running error. It occurs when the self-test is already running and " "another has been initiated\r\n",
				rslt);
			break;

		case BMI2_E_CRT_READY_FOR_DL_FAIL_ABORT:
			printf(
				"Error [%d] : CRT ready for download fail abort error. It occurs when download in CRT fails due to wrong " "address location\r\n",
				rslt);
			break;

		case BMI2_E_DL_ERROR:
			printf(
				"Error [%d] : Download error. It occurs when write length exceeds that of the maximum burst length\r\n",
				rslt);
			break;

		case BMI2_E_PRECON_ERROR:
			printf(
				"Error [%d] : Pre-conditional error. It occurs when precondition to start the feature was not " "completed\r\n",
				rslt);
			break;

		case BMI2_E_ABORT_ERROR:
			printf("Error [%d] : Abort error. It occurs when the device was shaken during CRT test\r\n", rslt);
			break;

		case BMI2_E_WRITE_CYCLE_ONGOING:
			printf(
				"Error [%d] : Write cycle ongoing error. It occurs when the write cycle is already running and another " "has been initiated\r\n",
				rslt);
			break;

		case BMI2_E_ST_NOT_RUNING:
			printf(
				"Error [%d] : Self-test is not running error. It occurs when self-test running is disabled while it's " "running\r\n",
				rslt);
			break;

		case BMI2_E_DATA_RDY_INT_FAILED:
			printf(
				"Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
				rslt);
			break;

		case BMI2_E_INVALID_FOC_POSITION:
			printf(
				"Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
				rslt);
			break;

		default:
			printf("Error [%d] : Unknown error code\r\n", rslt);
			break;
	}
}

