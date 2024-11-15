/*
 * Zephyr work function Example
 * Demonstrates the use of delayable and non-delayble work functions.
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

static struct k_work_delayable delayable_work;
static struct k_work non_delayable_work;

static void delayable_work_function(struct k_work *work);
static void non_delayable_work_function(struct k_work *work);


// LEDs for nRF52840 DK
static struct gpio_dt_spec led[] = {    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0}),
				        GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0}),
				        GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0})};

static void delayable_work_function(struct k_work *work)
{
        LOG_INF("Delayable work called");
        gpio_pin_set_dt(&led[0],0);     // Turn off LED 0

        // Adding this will cause the start of other work function to be delayed.
        //k_msleep(10);   // Check the effect of delays within a work function on other works
}

static void non_delayable_work_function(struct k_work *work)
{
        LOG_INF("Non-delayable work called");
        gpio_pin_set_dt(&led[1],0);     // Turn off LED 1
}


int main(void)
{       
        LOG_INF("Zephyr work function example on board: %s", CONFIG_BOARD);
        //uint16_t delay_ms = 10;       // 10ms

        LOG_INF("Initialise LEDs");
        for(uint8_t i = 0; i<ARRAY_SIZE(led); i++)
        {
                gpio_pin_configure_dt(&led[i],GPIO_OUTPUT_HIGH);
        }
       
        LOG_INF("Initialise work");
        k_work_init_delayable(&delayable_work,delayable_work_function);
        k_work_init(&non_delayable_work, non_delayable_work_function);
        
        while(1)
        {
                LOG_INF("Schedule work");
                gpio_pin_set_dt(&led[0],1);     // Turn on LED 0
                //k_work_schedule(&delayable_work, K_MSEC(delay_ms));
                k_work_schedule(&delayable_work, K_NO_WAIT);    // about a 30us delay

                gpio_pin_set_dt(&led[1],1);     // Turn on LED 1
                k_work_submit(&non_delayable_work);     // about a 40us delay
                k_msleep(2000); // cycle every 2 seconds.
        }
       
        return 0;
}
