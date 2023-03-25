/*
 * Zephyr GPIO Example
 * Demonstrates basic general purpose input-output (GPIO) support.
 * Device Tree settings can be found in app.overlay file. 
 * 
 * Copyright (c) 2023 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec gpio_pwr_enable = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), pwr_enable_gpios);
static const struct gpio_dt_spec gpio_reset = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), reset_gpios);

void main(void)
{
	int ret;

	printk("Zephyr GPIO Example \nBoard: %s\n", CONFIG_BOARD);

	if (!device_is_ready(gpio_pwr_enable.port)) {
		return;
	}

	// Configures GPIO pin. Options include:
	//	GPIO_INPUT
	//	GPIO_OUTPUT
	//	GPIO_DISCONNECTED
	//	GPIO_OUTPUT_LOW
	//	GPIO_OUTPUT_HIGH
	//	GPIO_OUTPUT_INACTIVE
	//	GPIO_OUTPUT_ACTIVE
	ret = gpio_pin_configure_dt(&gpio_pwr_enable, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}

	while (1) {
		printk("Toggle pwr_enable:\n");

		// Toggle pin:
		gpio_pin_toggle_dt(&gpio_pwr_enable);
		// Set pin:
		//gpio_pin_set_dt(&gpio_pwr_enable, 1);
		// Clear pin:
		//gpio_pin_set_dt(&gpio_pwr_enable, 0);
		
		k_msleep(1000);
	}
}
