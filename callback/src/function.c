/*
 * function.c
 * Zephyr callback Example
 * Demonstrates the definition and use of custom callback functions.
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include "function.h"
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(function);

callback_function callback = NULL;
struct CALLBACK_DATA callback_data = {.data1 = 0, .data2 = 0};

static struct k_work_delayable dwork;
static void work_function(struct k_work *work);

/**
 * @brief Reigster the callback function.
 */
int register_callback(callback_function *callback_function)
{
    LOG_INF("Register callback");
    callback = (void *)callback_function;
    return 0;
}

/**
 * @brief Initialises the work function that gets called every 100ms.
 */
int function_init()
{
    LOG_INF("Initialising function");
    k_work_init_delayable(&dwork,work_function);
    k_work_schedule(&dwork, K_NO_WAIT);
    return 0;
}


/** 
 * @brief Work function that gets called evey 100msec. Calls the callback every 10sec.
*/
static void work_function(struct k_work *work)
{
    static uint16_t delay_ms = 100;
    static uint16_t counter = 0;
    counter++;
  
    if(counter >= 100)
    {
        counter =0;
        // If registered, call the callback function and pass on data.
        if(callback != NULL)
        {
            LOG_INF("Callback called");
            callback_data.data1++;
            callback(&callback_data);
        }
        else
        {
            LOG_INF("No registered callback");
        }
    }

    //k_work_schedule(&work, K_MSEC(100));
    k_work_schedule(&dwork, K_MSEC(delay_ms));
}



