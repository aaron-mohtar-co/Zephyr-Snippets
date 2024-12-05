/*
 * Zephyr callback Example
 * Demonstrates the definition and use of custom callback functions.
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>

#include <zephyr/logging/log.h>

#include "function.h"

LOG_MODULE_REGISTER(main);

void function_callback(struct CALLBACK_DATA *data)
{
        LOG_INF("Callback triggered with data1 %d", data->data1);
}


int main(void)
{
        printk("Zephyr callback example \nBoard: %s\n", CONFIG_BOARD);

        register_callback(function_callback);
        function_init();

        while(1)
        {
                k_msleep(1000);
        }
        return 0;
}
