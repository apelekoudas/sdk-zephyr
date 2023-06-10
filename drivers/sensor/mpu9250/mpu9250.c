/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT invensense_mpu9250

#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/devicetree.h>
#include <stdlib.h>	// due to abs()

#include "mpu9250.h"

#ifdef CONFIG_MPU9250_MAGN_EN
#include "ak8963.h"
#endif

LOG_MODULE_REGISTER(MPU9250, CONFIG_SENSOR_LOG_LEVEL);

static int setupLPAccelWakeup(const struct device *dev, bool reset);

#define MPU9250_REG_CHIP_ID		0x75
#define MPU9250_CHIP_ID			0x71

#define MPU9250_REG_XG_OFFSET_H		0x13
#define MPU9250_REG_XG_OFFSET_L		0x14
#define MPU9250_REG_YG_OFFSET_H		0x15
#define MPU9250_REG_YG_OFFSET_L		0x16
#define MPU9250_REG_ZG_OFFSET_H		0x17
#define MPU9250_REG_ZG_OFFSET_L		0x18

#define MPU9250_REG_SR_DIV		0x19

#define MPU9250_REG_CONFIG		0x1A
#define MPU9250_GYRO_DLPF_MAX		7

#define MPU9250_REG_GYRO_CFG		0x1B
#define MPU9250_GYRO_FS_SHIFT		3
#define MPU9250_GYRO_FS_MAX		3

#define MPU9250_REG_ACCEL_CFG		0x1C
#define MPU9250_ACCEL_FS_SHIFT		3
#define MPU9250_ACCEL_FS_MAX		3

#define MPU9250_REG_ACCEL_CFG2		0x1D
#define MPU9250_ACCEL_DLPF_MAX		7

#define MPU9250_REG_LP_ACCEL_ODR	0x1E
#define MPU9250_ACCEL_LP_ODR_MAX	11

#define MPU9250_REG_WOM_THR			0x1F

#define MPU9250_REG_FIFO_EN			0x23

#define MPU9259_REG_INT_PIN_CFG		0x37
#define MPU9250_LATCH_INT_EN_BIT    BIT(5)

#define MPU9250_REG_INT_EN			0x38
#define MPU9250_WOM_EN_BIT			BIT(6)

#define MPU9250_REG_DATA_START		0x3B

#define MPU0259_TEMP_SENSITIVITY	334
#define MPU9250_TEMP_OFFSET		21

#define MPU9250_REG_MOT_DET_CTRL	0x69
#define MPU9250_ACCEL_INTEL_EN_BIT	 BIT(7)
#define MPU9250_ACCEL_INTEL_MODE_BIT BIT(6)

#define MPU9250_REG_USR_CTRL		0x6A

#define MPU9250_REG_PWR_MGMT1		0x6B
#define MPU9250_RESET_BIT		BIT(7)
#define MPU9250_SLEEP_EN		BIT(6)
#define MPU9250_CYCLE_BIT		BIT(5)
#define MPU9250_STANDBY_BIT		BIT(4)

#define MPU9250_REG_PWR_MGMT2		0x6C
#define MPU9250_DIS_XA_BIT		BIT(5)
#define MPU9250_DIS_YA_BIT		BIT(4)
#define MPU9250_DIS_ZA_BIT		BIT(3)
#define MPU9250_DIS_XG_BIT		BIT(2)
#define MPU9250_DIS_YG_BIT		BIT(1)
#define MPU9250_DIS_ZG_BIT		BIT(0)

#define MPU9250_REG_FIFO_COUNTH 	0x72
#define MPU9250_REG_FIFO_R_W		0x74

#define MPU9250_REG_XA_OFFSET_H 	0x77
#define MPU9250_REG_XA_OFFSET_L 	0x78
#define MPU9250_REG_YA_OFFSET_H 	0x7A
#define MPU9250_REG_YA_OFFSET_L 	0x7B
#define MPU9250_REG_ZA_OFFSET_H 	0x7D
#define MPU9250_REG_ZA_OFFSET_L 	0x7E

#ifdef CONFIG_MPU9250_MAGN_EN
#define MPU9250_READ_BUF_SIZE 11
#else
#define MPU9250_READ_BUF_SIZE 7
#endif

static uint8_t m_gyro_cal[8] __attribute__ ((aligned (4)));
static uint8_t m_accel_cal[8] __attribute__ ((aligned (4)));

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU9250(const struct device *dev) {
  const struct mpu9250_config *cfg = dev->config;	
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};
 
  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, 0x01);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT2, 0x00);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_SR_DIV, 0x13);
  k_msleep(200);

  // Configure device for bias calculation
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_INT_EN, 0x00);   // Disable all interrupts
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_FIFO_EN, 0x00);      // Disable FIFO
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, 0x00);   // Turn on internal clock source
  k_msleep(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_CONFIG, 0x01);       // Set low-pass filter to 188 Hz
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_SR_DIV, 0x00);   // Set sample rate to 1 kHz
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_GYRO_CFG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ACCEL_CFG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t accelsensitivity = 16384; // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_USR_CTRL, 0x40); // Enable FIFO
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_FIFO_EN, 0x78);   // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  k_msleep(40);                            // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_FIFO_EN, 0x00);            // Disable gyro and accelerometer sensors for FIFO
  i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_FIFO_COUNTH, &data[0], 2); // read FIFO sample count

  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_FIFO_R_W, &data[0], 12);           // read data for averaging
    accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
    accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
    gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
    gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
    gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

    accel_bias[0] += (int32_t)accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t)accel_temp[1];
    accel_bias[2] += (int32_t)accel_temp[2];
    gyro_bias[0] += (int32_t)gyro_temp[0];
    gyro_bias[1] += (int32_t)gyro_temp[1];
    gyro_bias[2] += (int32_t)gyro_temp[2];
  }
  accel_bias[0] /= (int32_t)packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t)packet_count;
  accel_bias[2] /= (int32_t)packet_count;
  gyro_bias[0] /= (int32_t)packet_count;
  gyro_bias[1] /= (int32_t)packet_count;
  gyro_bias[2] /= (int32_t)packet_count;

  if (accel_bias[2] > 0L) {
    accel_bias[2] -= (int32_t)accelsensitivity;
  } // Remove gravity from the z-axis accelerometer bias calculation
  else {
    accel_bias[2] += (int32_t)accelsensitivity;
  }

  LOG_DBG("accel_bias[0]=%d", accel_bias[0]);
  LOG_DBG("accel_bias[1]=%d", accel_bias[1]);
  LOG_DBG("accel_bias[2]=%d", accel_bias[2]);
  LOG_DBG("gyro_bias[0]=%d", gyro_bias[0]);
  LOG_DBG("gyro_bias[1]=%d", gyro_bias[1]);
  LOG_DBG("gyro_bias[2]=%d", gyro_bias[2]);

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0] / 4) & 0xFF;      // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
  data[3] = (-gyro_bias[1] / 4) & 0xFF;
  data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
  data[5] = (-gyro_bias[2] / 4) & 0xFF;

  memcpy(m_gyro_cal, data, 6);  //use a buffer 
  m_gyro_cal[6] = 0x55; // a signature this is initialized;

  // Push gyro biases to hardware registers
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XG_OFFSET_H, data[0]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XG_OFFSET_L, data[1]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YG_OFFSET_H, data[2]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YG_OFFSET_L, data[3]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZG_OFFSET_H, data[4]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZG_OFFSET_L, data[5]);

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0};                // A place to hold the factory accelerometer trim biases
  i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t)(((int16_t)data[0] << 8) | (data[1]));
  i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_YA_OFFSET_H, &data[0], 2);
  accel_bias_reg[1] = (int32_t)(((int16_t)data[0] << 8) | (data[1]));
  i2c_burst_read_dt(&cfg->i2c, MPU9250_REG_ZA_OFFSET_H, &data[0], 2);
  accel_bias_reg[2] = (int32_t)(((int16_t)data[0] << 8) | (data[1]));

  LOG_DBG("11accel_bias_reg[0]=%u", accel_bias_reg[0]);
  LOG_DBG("11accel_bias_reg[1]=%u", accel_bias_reg[1]);
  LOG_DBG("11accel_bias_reg[2]=%u", accel_bias_reg[2]);

  int sign;
  uint32_t mask = 1uL;             // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis
  for (uint8_t ii = 0; ii < 3; ii++) {
    if ((accel_bias_reg[ii] & mask))
      mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit

    if (accel_bias[ii] >= 0)
      sign = +1; // determine sign of accel_bias
    else
      sign = -1;
    // Shift right abs(accel_bias) four positions and shift left 1 postion; result is division by 8
    // with least significant bit cleared. Then restore sign and subtract from factory value
    accel_bias_reg[ii] -= sign * (((uint16_t)abs(accel_bias[ii]) >> 4) << 1);
    LOG_DBG("22accel_bias_reg[%d]=%u", ii, accel_bias_reg[ii]);
  }

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0]) & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1]) & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2]) & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  LOG_DBG("data[0]=%u", data[0]);
  LOG_DBG("data[1]=%u", data[2]);
  LOG_DBG("data[2]=%u", data[4]);

  memcpy(m_accel_cal, data, 6);
  m_accel_cal[6] = 0x55;  // a signature it is initialized

  // Push accelerometer biases to hardware registers
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_H, data[0]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_L, data[1]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YA_OFFSET_H, data[2]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YA_OFFSET_L, data[3]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZA_OFFSET_H, data[4]);
  i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZA_OFFSET_L, data[5]);
}

/*
*/
int loadAccelGyroCalibration(const struct device *dev, uint8_t *accel_cal, uint8_t *gyro_cal) {
  const struct mpu9250_config *cfg = dev->config;	

  // Wakeup the device
  int ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, 0);

  // Push gyro biases to hardware registers
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XG_OFFSET_H, gyro_cal[0]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XG_OFFSET_L, gyro_cal[1]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YG_OFFSET_H, gyro_cal[2]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YG_OFFSET_L, gyro_cal[3]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZG_OFFSET_H, gyro_cal[4]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZG_OFFSET_L, gyro_cal[5]);

  // Push accelerometer biases to hardware registers
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_H, accel_cal[0]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_XA_OFFSET_L, accel_cal[1]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YA_OFFSET_H, accel_cal[2]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_YA_OFFSET_L, accel_cal[3]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZA_OFFSET_H, accel_cal[4]);
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ZA_OFFSET_L, accel_cal[5]);

  // Enable Cycle Mode (Accel Low Power Mode)
  ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, MPU9250_CYCLE_BIT);

  return ret;
}

/* see "Accelerometer Measurements" section from register map description */
static void mpu9250_convert_accel(struct sensor_value *val, int16_t raw_val,
				  uint16_t sensitivity_shift)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_G) >> sensitivity_shift;
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Gyroscope Measurements" section from register map description */
static void mpu9250_convert_gyro(struct sensor_value *val, int16_t raw_val,
				 uint16_t sensitivity_x10)
{
	int64_t conv_val;

	conv_val = ((int64_t)raw_val * SENSOR_PI * 10) /
		   (sensitivity_x10 * 180U);
	val->val1 = conv_val / 1000000;
	val->val2 = conv_val % 1000000;
}

/* see "Temperature Measurement" section from register map description */
static inline void mpu9250_convert_temp(struct sensor_value *val,
					int16_t raw_val)
{
	/* Temp[*C] = (raw / sensitivity) + offset */
	val->val1 = (raw_val / MPU0259_TEMP_SENSITIVITY) + MPU9250_TEMP_OFFSET;
	val->val2 = (((int64_t)(raw_val % MPU0259_TEMP_SENSITIVITY) * 1000000)
				/ MPU0259_TEMP_SENSITIVITY);

	if (val->val2 < 0) {
		val->val1--;
		val->val2 += 1000000;
	} else if (val->val2 >= 1000000) {
		val->val1++;
		val->val2 -= 1000000;
	}
}

static int mpu9250_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mpu9250_data *drv_data = dev->data;
#ifdef CONFIG_MPU9250_MAGN_EN
	int ret;
#endif

	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		mpu9250_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		mpu9250_convert_accel(val + 1, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		mpu9250_convert_accel(val + 2, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_X:
		mpu9250_convert_accel(val, drv_data->accel_x,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		mpu9250_convert_accel(val, drv_data->accel_y,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		mpu9250_convert_accel(val, drv_data->accel_z,
				      drv_data->accel_sensitivity_shift);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		mpu9250_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		mpu9250_convert_gyro(val + 1, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		mpu9250_convert_gyro(val + 2, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_X:
		mpu9250_convert_gyro(val, drv_data->gyro_x,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Y:
		mpu9250_convert_gyro(val, drv_data->gyro_y,
				     drv_data->gyro_sensitivity_x10);
		break;
	case SENSOR_CHAN_GYRO_Z:
		mpu9250_convert_gyro(val, drv_data->gyro_z,
				     drv_data->gyro_sensitivity_x10);
		break;
#ifdef CONFIG_MPU9250_MAGN_EN
	case SENSOR_CHAN_MAGN_XYZ:
		ret = ak8963_convert_magn(val, drv_data->magn_x,
					  drv_data->magn_scale_x,
					  drv_data->magn_st2);
		if (ret < 0) {
			return ret;
		}
		ret = ak8963_convert_magn(val + 1, drv_data->magn_y,
					  drv_data->magn_scale_y,
					  drv_data->magn_st2);
		if (ret < 0) {
			return ret;
		}
		ret = ak8963_convert_magn(val + 2, drv_data->magn_z,
					  drv_data->magn_scale_z,
					  drv_data->magn_st2);
		return ret;
	case SENSOR_CHAN_MAGN_X:
		return ak8963_convert_magn(val, drv_data->magn_x,
				    drv_data->magn_scale_x,
				    drv_data->magn_st2);
	case SENSOR_CHAN_MAGN_Y:
		return ak8963_convert_magn(val, drv_data->magn_y,
				    drv_data->magn_scale_y,
				    drv_data->magn_st2);
	case SENSOR_CHAN_MAGN_Z:
		return ak8963_convert_magn(val, drv_data->magn_z,
				    drv_data->magn_scale_z,
				    drv_data->magn_st2);
	case SENSOR_CHAN_DIE_TEMP:
		mpu9250_convert_temp(val, drv_data->temp);
		break;
#endif
	default:
		return -ENOTSUP;
	}

	return 0;
}

static int mpu9250_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	int16_t buf[MPU9250_READ_BUF_SIZE];
	int ret;

	ret = i2c_burst_read_dt(&cfg->i2c,
				MPU9250_REG_DATA_START, (uint8_t *)buf,
				sizeof(buf));
	if (ret < 0) {
		LOG_ERR("Failed to read data sample.");
		return ret;
	}

	drv_data->accel_x = sys_be16_to_cpu(buf[0]);
	drv_data->accel_y = sys_be16_to_cpu(buf[1]);
	drv_data->accel_z = sys_be16_to_cpu(buf[2]);
	drv_data->temp = sys_be16_to_cpu(buf[3]);
	drv_data->gyro_x = sys_be16_to_cpu(buf[4]);
	drv_data->gyro_y = sys_be16_to_cpu(buf[5]);
	drv_data->gyro_z = sys_be16_to_cpu(buf[6]);
#ifdef CONFIG_MPU9250_MAGN_EN
	drv_data->magn_x = sys_le16_to_cpu(buf[7]);
	drv_data->magn_y = sys_le16_to_cpu(buf[8]);
	drv_data->magn_z = sys_le16_to_cpu(buf[9]);
	drv_data->magn_st2 = ((uint8_t *)buf)[20];
	LOG_DBG("magn_st2: %u", drv_data->magn_st2);
#endif

	return 0;
}

/*
*/
static int mpu9250_attr_get(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 struct sensor_value *val) {
	
  if (attr == SENSOR_ATTR_CALIBRATION) {		// load calibration values
	val->val1 = (uint32_t)m_accel_cal;
	val->val2 = (uint32_t)m_gyro_cal;

	return 0;
  }

  return -EINVAL;
}

/*
*/
static int mpu9250_attr_set(const struct device *dev,
				 enum sensor_channel chan,
				 enum sensor_attribute attr,
				 const struct sensor_value *val) {
	
  struct mpu9250_data *drv_data = dev->data;
  struct mpu9250_config *cfg = (struct mpu9250_config *)dev->config;

  if(attr == SENSOR_ATTR_UPPER_THRESH) {
	drv_data->wom_threshold = val->val1;
	int ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, 0);					// Wakeup the device
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_WOM_THR, drv_data->wom_threshold);  // Set Motion Threshold	
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, MPU9250_CYCLE_BIT);		// Enable Cycle Mode (Accel Low Power Mode)
	return ret;
  } else if(attr == SENSOR_ATTR_HYSTERESIS) {
	drv_data->wom_hysterisis = (uint16_t)val->val1;
	return 0;
  } else if (attr == SENSOR_ATTR_CALIBRATION) {		// load calibration values
  	return loadAccelGyroCalibration(dev, (uint8_t *)val->val1, (uint8_t *)val->val2);
  } else if (attr == SENSOR_ATTR_CALIB_TARGET) {	// perform calibration
	calibrateMPU9250(dev);
	return setupLPAccelWakeup(dev, false);	// restore LP accell wakeup mode
  } else {
	return -EINVAL;
  }
}

static const struct sensor_driver_api mpu9250_driver_api = {
#if CONFIG_MPU9250_TRIGGER
	.trigger_set = mpu9250_trigger_set,
#endif
	.sample_fetch = mpu9250_sample_fetch,
	.channel_get = mpu9250_channel_get,
	.attr_set = mpu9250_attr_set,
	.attr_get = mpu9250_attr_get
};

/* measured in degrees/sec x10 to avoid floating point */
static const uint16_t mpu9250_gyro_sensitivity_x10[] = {
	1310, 655, 328, 164
};

/*
*/
static int setupLPAccelWakeup(const struct device *dev, bool reset) {
	struct mpu9250_data *drv_data = dev->data;	
	const struct mpu9250_config *cfg = dev->config;
	uint8_t id;
	int ret = 0;

	if (!device_is_ready(cfg->i2c.bus)) {
		LOG_ERR("I2C dev %s not ready", cfg->i2c.bus->name);
		return -ENODEV;
	}

	if(reset) {
		ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, MPU9250_RESET_BIT);	// Reset
		// k_msleep(100);
		k_busy_wait(100000);
	}

#if 0
	// As found in Nena nRF5 SDK
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6B, 0x00);	//PWR_MGMT_1, WAKEUP
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1D, 0x48);	//ACCEL_CONFIG_2				?????
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6B, 0x01);	//PWR_MGMT_1, CLKSEL[2:0]
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6C, 0x00);	//PWR_MGMT_2, enable all sensors
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6A, 0x00);	//USER_CTRL
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1B, 0x00);	//GYRO_CONFIG
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1C, 0x00);	//ACCEL_CONFIG
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1A, 0x03);	//CONFIG, LPF_CFG[2:0]				!!!
	i2c_reg_write_byte_dt(&cfg->i2c, 0x19, 0x13);	//SMPLRT_DIV, SMPLRT_DIV[7:0]
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1A, 0x04);	//CONFIG, LPF_CFG[2:0]				!!!
	i2c_reg_write_byte_dt(&cfg->i2c, 0x38, 0x00);	//INT_ENABLE, disable all interrupts
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6A, 0x00);	//USER_CTRL
	i2c_reg_write_byte_dt(&cfg->i2c, 0x37, 0xB2);	//INT_PIN_CFG, ACTL | LATCH_INT_EN | INT_ANYRD_2CLEAR | BYPASS_EN
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6B, 0x01);	//PWR_MGMT_1, CLKSEL[2:0]
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6C, 0x00);	//PWR_MGMT_2, enable all sensors
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6A, 0x20);	//USER_CTRL, I2C_MST_EN

	i2c_reg_write_byte_dt(&cfg->i2c, 0x38, 0x01);	//INT_ENABLE, enable data_rdy
	i2c_reg_write_byte_dt(&cfg->i2c, 0x37, 0x32);	//INT_PIN_CFG, ACTL | LATCH_INT_EN | INT_ANYRD_2CLEAR | BYPASS_EN

	i2c_reg_write_byte_dt(&cfg->i2c, 0x38, 0x00);	//INT_ENABLE, disable all interrupts
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6A, 0x00);	//USER_CTRL
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1F, 0x0A);	//WOM_THR
	i2c_reg_write_byte_dt(&cfg->i2c, 0x1E, 0x05);	//LP_ACCEL_ODR, 15.63Hz
	i2c_reg_write_byte_dt(&cfg->i2c, 0x69, 0xC0);	//MOT_DETECT_CTRL
	i2c_reg_write_byte_dt(&cfg->i2c, 0x6B, 0x20);	//PWR_MGMT_1, CYCLE !
	i2c_reg_write_byte_dt(&cfg->i2c, 0x38, 0x40);	//INT_ENABLE, WOM_EN
	drv_data->accel_sensitivity_shift = 14 - cfg->accel_fs;
#else
	/* check chip ID */
	ret |= i2c_reg_read_byte_dt(&cfg->i2c, MPU9250_REG_CHIP_ID, &id);
	if (ret < 0) {
		LOG_ERR("Failed to read chip ID.");
		return ret;
	}

	if (id != MPU9250_CHIP_ID) {
		LOG_ERR("Invalid chip ID.");
		return -ENOTSUP;
	}

	/* wake up chip */
	ret = i2c_reg_update_byte_dt(&cfg->i2c,
	 			     MPU9250_REG_PWR_MGMT1,
	 			     MPU9250_SLEEP_EN, 0);	
	if (ret < 0) {
		LOG_ERR("Failed to wake up chip.");
		return ret;
	}

	if (cfg->accel_fs > MPU9250_ACCEL_FS_MAX) {
		LOG_ERR("Accel FS is too big: %d", cfg->accel_fs);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ACCEL_CFG,
				    cfg->accel_fs << MPU9250_ACCEL_FS_SHIFT);
	if (ret < 0) {
		LOG_ERR("Failed to write accel full-scale range.");
		return ret;
	}
	drv_data->accel_sensitivity_shift = 14 - cfg->accel_fs;

	if (cfg->gyro_fs > MPU9250_GYRO_FS_MAX) {
		LOG_ERR("Gyro FS is too big: %d", cfg->gyro_fs);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_GYRO_CFG,
				    cfg->gyro_fs << MPU9250_GYRO_FS_SHIFT);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro full-scale range.");
		return ret;
	}

	if (cfg->gyro_dlpf > MPU9250_GYRO_DLPF_MAX) {
		LOG_ERR("Gyro DLPF is too big: %d", cfg->gyro_dlpf);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_CONFIG,
				    cfg->gyro_dlpf);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro digital LPF settings.");
		return ret;
	}

	if (cfg->accel_dlpf > MPU9250_ACCEL_DLPF_MAX) {
		LOG_ERR("Accel DLPF is too big: %d", cfg->accel_dlpf);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ACCEL_CFG2,
				    cfg->gyro_dlpf);	// This is wrong! Overriden later
	if (ret < 0) {
		LOG_ERR("Failed to write accel digital LPF settings.");
		return ret;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_SR_DIV,
				    cfg->gyro_sr_div);
	if (ret < 0) {
		LOG_ERR("Failed to write gyro ODR divider.");
		return ret;
	}

	drv_data->gyro_sensitivity_x10 =
				mpu9250_gyro_sensitivity_x10[cfg->gyro_fs];

#ifdef CONFIG_MPU9250_MAGN_EN
	ret = ak8963_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize AK8963.");
		return ret;
	}
#endif

	if (cfg->accel_lp_odr > MPU9250_ACCEL_LP_ODR_MAX) {
		LOG_ERR("Accel LP ODR is too big: %d", cfg->accel_lp_odr);
		return -EINVAL;
	}

	ret = i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT2, MPU9250_DIS_XG_BIT | MPU9250_DIS_YG_BIT | MPU9250_DIS_ZG_BIT);
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_ACCEL_CFG2, 0x09);						// Set Accel LPF setting to 184 Hz Bandwidt
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_INT_EN, MPU9250_WOM_EN_BIT);			// Enable Motion Interrupt
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_MOT_DET_CTRL, MPU9250_ACCEL_INTEL_EN_BIT | MPU9250_ACCEL_INTEL_MODE_BIT);	// Enable Accel Hardware Intelligence
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_WOM_THR, drv_data->wom_threshold);		// Set Motion Threshold
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_LP_ACCEL_ODR, cfg->accel_lp_odr);		// Set Frequency of Wake‐up
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9250_REG_PWR_MGMT1, MPU9250_CYCLE_BIT);			// Enable Cycle Mode (Accel Low Power Mode)
	ret |= i2c_reg_write_byte_dt(&cfg->i2c, MPU9259_REG_INT_PIN_CFG, MPU9250_LATCH_INT_EN_BIT);	// Latch IRQs to make sure we do not miss any

#endif	
	return ret;
}

static int mpu9250_init(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	int ret;

	drv_data->wom_hysterisis = 4;	// initial values
	drv_data->wom_threshold = 100;

	ret = setupLPAccelWakeup(dev, true);
	if (ret < 0) {
		LOG_ERR("Failed to write LP settings.");
		return ret;
	}

#ifdef CONFIG_MPU9250_TRIGGER
	ret = mpu9250_init_interrupt(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize interrupts.");
		return ret;
	}
#endif

	return 0;
}


#define INIT_MPU9250_INST(inst)						\
	static struct mpu9250_data mpu9250_data_##inst;			\
	static const struct mpu9250_config mpu9250_cfg_##inst = {	\
	.i2c = I2C_DT_SPEC_INST_GET(inst),				\
	.gyro_sr_div = DT_INST_PROP(inst, gyro_sr_div),			\
	.gyro_dlpf = DT_INST_ENUM_IDX(inst, gyro_dlpf),			\
	.gyro_fs = DT_INST_ENUM_IDX(inst, gyro_fs),			\
	.accel_fs = DT_INST_ENUM_IDX(inst, accel_fs),			\
	.accel_dlpf = DT_INST_ENUM_IDX(inst, accel_dlpf),		\
	.accel_lp_odr = DT_INST_ENUM_IDX(inst, accel_lp_odr),		\
	IF_ENABLED(CONFIG_MPU9250_TRIGGER,				\
		  (.int_pin = GPIO_DT_SPEC_INST_GET(inst, irq_gpios)))	\
	};								\
									\
	DEVICE_DT_INST_DEFINE(inst, mpu9250_init, NULL,			\
			      &mpu9250_data_##inst, &mpu9250_cfg_##inst,\
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,	\
			      &mpu9250_driver_api);

DT_INST_FOREACH_STATUS_OKAY(INIT_MPU9250_INST)
