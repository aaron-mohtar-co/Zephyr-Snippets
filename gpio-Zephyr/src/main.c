/*
 * Zephyr GPIO Example
 * Demonstrates basic general purpose input-output (GPIO) support.
 * Device Tree settings can be found in app.overlay file. 
 * 
 * Zephyr GPIO API documentation can be found at 
 * https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html
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

static const struct gpio_dt_spec gpio_reg_en = GPIO_DT_SPEC_GET(DT_ALIAS(reg_en), gpios);

void main(void)
{
	int ret;

	printk("Zephyr GPIO Example \nBoard: %s\n", CONFIG_BOARD);

	if (!device_is_ready(gpio_pwr_enable.port)) {
		return;
	}

	// Configures GPIO pin. Options include:
	//	GPIO_INPUT - Enables pin as input.
	//	GPIO_OUTPUT - Enables pin as output, no change to the output state.
	//	GPIO_DISCONNECTED - Disables pin for both input and output.
	//	GPIO_OUTPUT_LOW - Configures GPIO pin as output and initializes it to a low state.
	//	GPIO_OUTPUT_HIGH - Configures GPIO pin as output and initializes it to a high state.
	//	GPIO_OUTPUT_INACTIVE - Configures GPIO pin as output and initializes it to a logic 0.
	//	GPIO_OUTPUT_ACTIVE - Configures GPIO pin as output and initializes it to a logic 1.
	//	GPIO_ACTIVE_LOW - GPIO pin is active (has logical value ‘1’) in low state.
	//	GPIO_ACTIVE_HIGH - GPIO pin is active (has logical value ‘1’) in high state.
	//	GPIO_OPEN_DRAIN - Configures GPIO output in open drain mode (wired AND).
	//	GPIO_OPEN_SOURCE - Configures GPIO output in open source mode (wired OR).
	//	GPIO_PULL_UP - Enables GPIO pin pull-up.
	//	GPIO_PULL_DOWN - Enable GPIO pin pull-down.
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
