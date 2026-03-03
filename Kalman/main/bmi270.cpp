/* The example of ESP-IDF
 *
 * This sample code is in the public domain.
 */

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/message_buffer.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "cJSON.h"

#include "parameter.h"

extern QueueHandle_t xQueueTrans;
extern MessageBufferHandle_t xMessageBufferToClient;

static const char *TAG = "IMU";

// bmi270 stuff
#include "bmi270_dsd.h"
#include "bmi2_driver.h"

// Source: https://github.com/TKJElectronics/KalmanFilter
#include "Kalman.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead
#define RAD_TO_DEG (180.0/M_PI)
#define DEG_TO_RAD 0.0174533

// Arduino macro
#define micros() (unsigned long) (esp_timer_get_time())
#define delay(ms) esp_rom_delay_us(ms*1000)

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
struct bmi2_dev bmi;

/* Macros to select the sensors */
#define ACCEL UINT8_C(0x00)
#define GYRO  UINT8_C(0x01)

/* Enum to string converter*/
#ifndef enum_to_string
#define enum_to_string(a)  #a
#endif

/*!
 *	@brief This internal API is used to set configurations for accel.
 *
 *	@param[in] bmi : Structure instance of bmi2_dev.
 *
 *	@return Status of execution.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi);

/*!
 *	@brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 *	range 2G, 4G, 8G or 16G.
 *
 *	@param[in] val : LSB from each axis.
 *	@param[in] g_range : Gravity range.
 *	@param[in] bit_width : Resolution for accel.
 *
 *	@return Accel values in meter per second squared.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width);

/*!
 *	@brief This function converts lsb to degree per second for 16 bit gyro at
 *	range 125, 250, 500, 1000 or 2000dps.
 *
 *	@param[in] val : LSB from each axis.
 *	@param[in] dps : Degree per second.
 *	@param[in] bit_width : Resolution for gyro.
 *
 *	@return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

// Get scaled value
void getMotion6(double *acc_x, double *acc_y, double *acc_z, double *gyr_x, double *gyr_y, double *gyr_z) {
	struct bmi2_sens_data sensor_data;
	memset(&sensor_data, 0, sizeof(bmi2_sens_data));

	while(1) {
		int8_t rslt = bmi2_get_sensor_data(&sensor_data, &bmi);
		bmi2_error_codes_print_result(rslt);
		if ((rslt == BMI2_OK) && (sensor_data.status & BMI2_DRDY_ACC) &&
			(sensor_data.status & BMI2_DRDY_GYR)) break;
		vTaskDelay(10);
	}

	/* Converting lsb to meter per second squared for 16 bit accelerometer at 2G range. */
	*acc_x = lsb_to_mps2(sensor_data.acc.x, BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G), bmi.resolution);
	*acc_y = lsb_to_mps2(sensor_data.acc.y, BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G), bmi.resolution);
	*acc_z = lsb_to_mps2(sensor_data.acc.z, BMI2_GET_RANGE_VAL(BMI2_ACC_RANGE_2G), bmi.resolution);

	/* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
	*gyr_x = lsb_to_dps(sensor_data.gyr.x, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi.resolution);
	*gyr_y = lsb_to_dps(sensor_data.gyr.y, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi.resolution);
	*gyr_z = lsb_to_dps(sensor_data.gyr.z, BMI2_GET_DPS_VAL(BMI2_GYR_RANGE_2000), bmi.resolution);
}

void getRollPitch(double accX, double accY, double accZ, double *roll, double *pitch) {
	// atan2 outputs the value of -πto π(radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	*roll = atan2(accY, accZ) * RAD_TO_DEG;
	*pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	*roll = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	*pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif
}

void bmi270(void *pvParameters)
{
	/* Status of api are returned to this variable. */
	int8_t rslt;

	/* Assign accel and gyro sensor to variable. */
	uint8_t sensor_list[2] = { BMI2_ACCEL, BMI2_GYRO };

	/* Initialize interface */
	bmi2_interface_init(&bmi, BMI2_I2C_INTF);

	/* Initialize bmi270_dsd. */
	rslt = bmi270_dsd_init(&bmi);
	bmi2_error_codes_print_result(rslt);
	if (rslt != BMI2_OK) {
		vTaskDelete(NULL);
	}
	ESP_LOGI(TAG, "Chip ID :0x%x", bmi.chip_id);
	if (bmi.chip_id != 0x24) {
		ESP_LOGE(TAG, "BMI270 not found");
		vTaskDelete(NULL);
	}

	/* Accel and gyro configuration settings. */
	rslt = set_accel_gyro_config(&bmi);
	bmi2_error_codes_print_result(rslt);
	if (rslt != BMI2_OK) {
		vTaskDelete(NULL);
	}

	/* NOTE:
	 * Accel and Gyro enable must be done after setting configurations
	 */
	rslt = bmi2_sensor_enable(sensor_list, 2, &bmi);
	bmi2_error_codes_print_result(rslt);
	if (rslt != BMI2_OK) {
		vTaskDelete(NULL);
	}

	/* Get the accel configurations. */
	struct bmi2_sens_config config;
	config.type = BMI2_ACCEL;
	rslt = bmi2_get_sensor_config(&config, 1, &bmi);
	bmi2_error_codes_print_result(rslt);

	// Set Kalman and gyro starting angle
	double ax, ay, az;
	double gx, gy, gz;
	double roll, pitch; // Roll and pitch are calculated using the accelerometer
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

	getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	getRollPitch(ax, ay, az, &roll, &pitch);
	kalAngleX = roll;
	kalAngleY = pitch;
	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	uint32_t timer = micros();

	int elasped = 0;
	bool initialized = false;
	double initial_kalAngleX = 0.0;
	double initial_kalAngleY = 0.0;

	while(1) {
		getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		//printf("%f %f %f - %f %f %f\n", ax, ay, az, gx, gy, gz);
		getRollPitch(ax, ay, az, &roll, &pitch);

		double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
		timer = micros();

		/* Roll and pitch estimation */
		double gyroXrate = gx;
		double gyroYrate = gy;

#ifdef RESTRICT_PITCH
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
			kalmanX.setAngle(roll);
			kalAngleX = roll;
		} else
			kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
	

		if (abs(kalAngleX) > 90)
			gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
		// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
		if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
			kalmanY.setAngle(pitch);
			kalAngleY = pitch;
		} else
			kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

		if (abs(kalAngleY) > 90)
			gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

		/* Print Data every 10 times */
		if (elasped > 10) {
			// Set the first data
			if (!initialized) {
				initial_kalAngleX = roll;
				initial_kalAngleY = pitch;
				initialized = true;
			}

#if 0
			printf("roll:%f", roll); printf(" ");
			printf("kalAngleX:%f", kalAngleX); printf(" ");
			printf("initial_kalAngleX:%f", initial_kalAngleX); printf(" ");
			printf("kalAngleX-initial_kalAngleX:%f", kalAngleX-initial_kalAngleX); printf(" ");
			printf("\n");

			printf("pitch: %f", pitch); printf(" ");
			printf("kalAngleY:%f", kalAngleY); printf("  ");
			printf("initial_kalAngleY: %f", initial_kalAngleY); printf(" ");
			printf("kalAngleY-initial_kalAngleY: %f", kalAngleY-initial_kalAngleY); printf(" ");
			printf("\n");
#endif

			// Send UDP packet
			float _roll = kalAngleX-initial_kalAngleX;
			float _pitch = kalAngleY-initial_kalAngleY;
			ESP_LOGI(TAG, "roll:%f pitch=%f", _roll, _pitch);

			POSE_t pose;
			pose.roll = _roll;
			pose.pitch = _pitch;
			pose.yaw = 0.0;
			if (xQueueSend(xQueueTrans, &pose, 100) != pdPASS ) {
				ESP_LOGE(pcTaskGetName(NULL), "xQueueSend fail");
			}

			// Send WEB request
			cJSON *request;
			request = cJSON_CreateObject();
			cJSON_AddStringToObject(request, "id", "data-request");
			cJSON_AddNumberToObject(request, "roll", _roll);
			cJSON_AddNumberToObject(request, "pitch", _pitch);
			cJSON_AddNumberToObject(request, "yaw", 0.0);
			char *my_json_string = cJSON_Print(request);
			ESP_LOGD(TAG, "my_json_string\n%s",my_json_string);
			size_t xBytesSent = xMessageBufferSend(xMessageBufferToClient, my_json_string, strlen(my_json_string), 100);
			if (xBytesSent != strlen(my_json_string)) {
				ESP_LOGE(TAG, "xMessageBufferSend fail");
			}
			cJSON_Delete(request);
			cJSON_free(my_json_string);

			vTaskDelay(1);
			elasped = 0;
		}
	
		elasped++;
		vTaskDelay(1);
	} // end while

	// Never reach here
	vTaskDelete( NULL );
}

/*!
 * @brief This internal API is used to set configurations for accel and gyro.
 */
static int8_t set_accel_gyro_config(struct bmi2_dev *bmi)
{
	/* Status of api are returned to this variable. */
	int8_t rslt;

	/* Structure to define accelerometer and gyro configuration. */
	struct bmi2_sens_config config[2];

	/* Configure the type of feature. */
	config[ACCEL].type = BMI2_ACCEL;
	config[GYRO].type = BMI2_GYRO;

	/* Get default configurations for the type of feature selected. */
	rslt = bmi2_get_sensor_config(config, 2, bmi);
	bmi2_error_codes_print_result(rslt);

	/* Map data ready interrupt to interrupt pin. */
	rslt = bmi2_map_data_int(BMI2_DRDY_INT, BMI2_INT1, bmi);
	bmi2_error_codes_print_result(rslt);

	if (rslt == BMI2_OK)
	{
		/* NOTE: The user can change the following configuration parameters according to their requirement. */
		/* Set Output Data Rate */
		config[ACCEL].cfg.acc.odr = BMI2_ACC_ODR_200HZ;

		/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
		config[ACCEL].cfg.acc.range = BMI2_ACC_RANGE_2G;

		/* The bandwidth parameter is used to configure the number of sensor samples that are averaged
		 * if it is set to 2, then 2^(bandwidth parameter) samples
		 * are averaged, resulting in 4 averaged samples.
		 * Note1 : For more information, refer the datasheet.
		 * Note2 : A higher number of averaged samples will result in a lower noise level of the signal, but
		 * this has an adverse effect on the power consumed.
		 */
		config[ACCEL].cfg.acc.bwp = BMI2_ACC_NORMAL_AVG4;

		/* Enable the filter performance mode where averaging of samples
		 * will be done based on above set bandwidth and ODR.
		 * There are two modes
		 *	0 -> Ultra low power mode
		 *	1 -> High performance mode(Default)
		 * For more info refer datasheet.
		 */
		config[ACCEL].cfg.acc.filter_perf = BMI2_PERF_OPT_MODE;

		/* The user can change the following configuration parameters according to their requirement. */
		/* Set Output Data Rate */
		config[GYRO].cfg.gyr.odr = BMI2_GYR_ODR_200HZ;

		/* Gyroscope Angular Rate Measurement Range.By default the range is 2000dps. */
		config[GYRO].cfg.gyr.range = BMI2_GYR_RANGE_2000;

		/* Gyroscope bandwidth parameters. By default the gyro bandwidth is in normal mode. */
		config[GYRO].cfg.gyr.bwp = BMI2_GYR_NORMAL_MODE;

		/* Enable/Disable the noise performance mode for precision yaw rate sensing
		 * There are two modes
		 *	0 -> Ultra low power mode(Default)
		 *	1 -> High performance mode
		 */
		config[GYRO].cfg.gyr.noise_perf = BMI2_POWER_OPT_MODE;

		/* Enable/Disable the filter performance mode where averaging of samples
		 * will be done based on above set bandwidth and ODR.
		 * There are two modes
		 *	0 -> Ultra low power mode
		 *	1 -> High performance mode(Default)
		 */
		config[GYRO].cfg.gyr.filter_perf = BMI2_PERF_OPT_MODE;

		/* Set the accel and gyro configurations. */
		rslt = bmi2_set_sensor_config(config, 2, bmi);
		bmi2_error_codes_print_result(rslt);

#if 0
		printf("*************************************\n");
		printf("Accel Configurations\n");
		printf("ODR:\t %s\n", enum_to_string(BMI2_ACC_ODR_200HZ));
		printf("Range:\t %s\n", enum_to_string(BMI2_ACC_RANGE_2G));
		printf("Bandwidth:\t %s\n", enum_to_string(BMI2_ACC_NORMAL_AVG4));
		printf("Filter performance:\t %s\n", enum_to_string(BMI2_PERF_OPT_MODE));
		printf("Resolution:%u\n", bmi->resolution);

		printf("*************************************\n");
		printf("Gyro Configurations\n");
		printf("ODR:\t %s\n", enum_to_string(BMI2_GYR_ODR_200HZ));
		printf("Range:\t %s\n", enum_to_string(BMI2_GYR_RANGE_2000));
		printf("Bandwidth:\t %s\n", enum_to_string(BMI2_GYR_NORMAL_MODE));
		printf("Noise performance:\t %s\n", enum_to_string(BMI2_POWER_OPT_MODE));
		printf("Filter performance:\t %s\n", enum_to_string(BMI2_PERF_OPT_MODE));
		printf("Resolution:%u\n", bmi->resolution);
#endif
	}

	return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
	double power = 2;

	float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

	return (val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
	double power = 2;

	float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

	return (val * dps)	/ half_scale;
}

