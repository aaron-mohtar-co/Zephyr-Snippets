/*
 * Zephyr Hello World Boiler Plate Example
 * Demonstrates a basic Zephyr project with console support
 * 
 * Copyright (c) 2023 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

void main(void)
{
	printk("Zephyr Hello World! Example \nBoard: %s\n", CONFIG_BOARD);

	while (1) {
		printk("Hello World\n");
		k_msleep(1000);
	}
}
