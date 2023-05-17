/*
 * Copyright (c) 2021, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>

#include "mpu9250.h"

LOG_MODULE_DECLARE(MPU9250, CONFIG_SENSOR_LOG_LEVEL);


#define MPU9250_REG_INT_EN	0x38
#define MPU9250_DRDY_BIT	BIT(0)
#define MPU9250_WOM_BIT		BIT(6)

#define MPU9250_REG_INT_STATUS	0x3A

static uint16_t TailPulseCntr = 0;

int mpu9250_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	int ret;
	
	LOG_DBG("mpu9250_trigger_set %d %d", trig->type, (int)handler);
	if (trig->type == SENSOR_TRIG_DATA_READY) {
		drv_data->data_rdy_handler = handler;
	  	return i2c_reg_update_byte_dt(&cfg->i2c, MPU9250_REG_INT_EN, MPU9250_DRDY_BIT, (handler == NULL ? 0 : 1));	
	} else if (trig->type == SENSOR_TRIG_STATIONARY) {
		drv_data->stationary_handler = handler;
		return 0;
	} else if (trig->type == SENSOR_TRIG_MOTION) {

		ret = gpio_pin_interrupt_configure_dt(&cfg->int_pin, GPIO_INT_DISABLE);
		if (ret < 0) {
			LOG_ERR("Failed to disable gpio interrupt.");
			return ret;
		}

		drv_data->motion_handler = handler;
		if (handler == NULL) {
			return 0;
		}

		ret = gpio_pin_interrupt_configure_dt(&cfg->int_pin,
							GPIO_INT_LEVEL_ACTIVE);			// TP changed from EDGE to LEVEL
		if (ret < 0) {
			LOG_ERR("Failed to enable gpio interrupt.");
			return ret;
		}

		return 0;
	} else {
		return -EINVAL;
	}
}

static void mpu9250_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct mpu9250_data *drv_data =
		CONTAINER_OF(cb, struct mpu9250_data, gpio_cb);
	const struct mpu9250_config *cfg = drv_data->dev->config;
	int ret;

	ARG_UNUSED(pins);

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_pin, GPIO_INT_DISABLE);
	if (ret < 0) {
		LOG_ERR("Disabling gpio interrupt failed with err: %d", ret);
		return;
	}

#if defined(CONFIG_MPU9250_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void mpu9250_thread_cb(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	int ret;

	uint8_t irqSource;
	ret = i2c_reg_read_byte_dt(&cfg->i2c, MPU9250_REG_INT_STATUS, &irqSource);	// read irq source
	LOG_DBG("mpu9250_thread_cb %02X", irqSource);
	if(irqSource & MPU9250_WOM_BIT) {	
		if ((drv_data->motion_handler != NULL) && (TailPulseCntr == 0)) {	// do not send another motion irq, just restart the counter
			drv_data->trigger.type = SENSOR_TRIG_MOTION;
			drv_data->motion_handler(dev, &drv_data->trigger);			
		}		
		TailPulseCntr = drv_data->wom_hysterisis;	// restart counter;
	}

	if(irqSource & MPU9250_DRDY_BIT) {
		if ((drv_data->data_rdy_handler == NULL) || (TailPulseCntr == 0)) {
			if(drv_data->stationary_handler != NULL) {
				drv_data->trigger.type = SENSOR_TRIG_STATIONARY;
				drv_data->stationary_handler(dev, &drv_data->trigger);
			}
		} else {
			drv_data->trigger.type = SENSOR_TRIG_DATA_READY;
			drv_data->data_rdy_handler(dev, &drv_data->trigger);
			TailPulseCntr--;
		}
	}

	ret = gpio_pin_interrupt_configure_dt(&cfg->int_pin,
					      GPIO_INT_LEVEL_ACTIVE);		// TP changed from EDGE to LEVEL
	if (ret < 0) {
		LOG_ERR("Enabling gpio interrupt failed with err: %d", ret);
	}

}

#ifdef CONFIG_MPU9250_TRIGGER_OWN_THREAD
static void mpu9250_thread(struct mpu9250_data *drv_data)
{
	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		mpu9250_thread_cb(drv_data->dev);
	}
}
#endif

#ifdef CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD
static void mpu9250_work_cb(struct k_work *work)
{
	struct mpu9250_data *drv_data =
		CONTAINER_OF(work, struct mpu9250_data, work);

	mpu9250_thread_cb(drv_data->dev);
}
#endif

int mpu9250_init_interrupt(const struct device *dev)
{
	struct mpu9250_data *drv_data = dev->data;
	const struct mpu9250_config *cfg = dev->config;
	int ret;

	/* setup data ready gpio interrupt */
	if (!device_is_ready(cfg->int_pin.port)) {
		LOG_ERR("Interrupt pin is not ready.");
		return -EIO;
	}

	drv_data->dev = dev;

	ret = gpio_pin_configure_dt(&cfg->int_pin, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Failed to configure interrupt pin.");
		return ret;
	}

	gpio_init_callback(&drv_data->gpio_cb,
			   mpu9250_gpio_callback,
			   BIT(cfg->int_pin.pin));

	ret = gpio_add_callback(cfg->int_pin.port, &drv_data->gpio_cb);
	if (ret < 0) {
		LOG_ERR("Failed to set gpio callback.");
		return ret;
	}

#if defined(CONFIG_MPU9250_TRIGGER_OWN_THREAD)
	ret = k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);
	if (ret < 0) {
		LOG_ERR("Failed to enable semaphore");
		return ret;
	}

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_MPU9250_THREAD_STACK_SIZE,
			(k_thread_entry_t)mpu9250_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_MPU9250_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_MPU9250_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = mpu9250_work_cb;
#endif

	if (ret < 0) {
		LOG_ERR("Failed to enable interrupt");
		return ret;
	}

	return 0;
}
