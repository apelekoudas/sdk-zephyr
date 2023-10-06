/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 * Copyright (c) 2020 Grinn
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/lora.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "sx12xx_common.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(lr1110, CONFIG_LORA_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(semtech_lr1110)

#define DT_DRV_COMPAT semtech_lr1110

#else
#error No LR1110 instance in device tree.
#endif

struct lr1110_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec reset;
#if DT_INST_NODE_HAS_PROP(0, antenna_enable_gpios)
	struct gpio_dt_spec antenna_enable;
#endif
#if DT_INST_NODE_HAS_PROP(0, tcxo_power_gpios)
	struct gpio_dt_spec tcxo_power;
#endif
};

static const struct lr1110_config dev_config = {
	.bus = SPI_DT_SPEC_INST_GET(0, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),
	.reset = GPIO_DT_SPEC_INST_GET(0, reset_gpios),
#if DT_INST_NODE_HAS_PROP(0, antenna_enable_gpios)
	.antenna_enable = GPIO_DT_SPEC_INST_GET(0, antenna_enable_gpios),
#endif
};

static int lr1110_antenna_configure(void)
{
	LOG_DBG("Entry");
	
	int ret;

	ret = sx12xx_configure_pin(antenna_enable, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	ret = sx12xx_configure_pin(rfi_enable, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	ret = sx12xx_configure_pin(rfo_enable, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	ret = sx12xx_configure_pin(pa_boost_enable, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	return 0;
}

static int lr1110_lora_init(const struct device *dev)
{
	return -1;
	LOG_DBG("Entry");

	int ret;
	// uint8_t regval;

	if (!spi_is_ready_dt(&dev_config.bus)) {
		LOG_ERR("SPI device not ready");
		return -ENODEV;
	}

	ret = sx12xx_configure_pin(tcxo_power, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	/* Setup Reset gpio and perform soft reset */
	ret = sx12xx_configure_pin(reset, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	k_sleep(K_MSEC(100));
	gpio_pin_set_dt(&dev_config.reset, 0);
	k_sleep(K_MSEC(100));

	// ret = sx127x_read(REG_VERSION, &regval, 1);
	// if (ret < 0) {
	// 	LOG_ERR("Unable to read version info");
	// 	return -EIO;
	// }

	// LOG_INF("SX127x version 0x%02x found", regval);

	ret = lr1110_antenna_configure();
	if (ret < 0) {
		LOG_ERR("Unable to configure antenna");
		return -EIO;
	}

	ret = sx12xx_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize SX12xx common");
		return ret;
	}

	return 0;
}

static const struct lora_driver_api lr1110_lora_api = {
	.config = sx12xx_lora_config,
	.send = sx12xx_lora_send,
	.send_async = sx12xx_lora_send_async,
	.recv = sx12xx_lora_recv,
	.recv_async = sx12xx_lora_recv_async,
	.test_cw = sx12xx_lora_test_cw,
};

DEVICE_DT_INST_DEFINE(0, &lr1110_lora_init, NULL, NULL,
		      NULL, POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,
		      &lr1110_lora_api);
