/*
 * function.h
 * Zephyr callback Example
 * Demonstrates the definition and use of custom callback functions.
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

// Define callback data structure.
struct CALLBACK_DATA {
        uint8_t data1;
        uint16_t data2;
};

typedef void (*callback_function)(struct CALLBACK_DATA *cb_data); // callback funtion type def

int register_callback(callback_function *callback_function);

int function_init();
