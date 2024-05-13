/*
 * Zephyr GPI Interrupt Example
 * Demonstrates basic use of a general purpose input with a high speed 
 * interrupt service routine
 * LED1 toggles in the while loop to indicate operation
 * LED2 turns on when button1 is depressed (with a delay)
 * LED3 flashs on when button1 is depressed 
 * 
 * Zephyr GPIO API documentation can be found at 
 * https://docs.zephyrproject.org/latest/hardware/peripherals/gpio.html
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include <zephyr/irq.h>
#include <nrfx_gpiote.h>

LOG_MODULE_REGISTER(main);

#define led2_on()       gpio_pin_set_dt(&gpio_led2,1)
#define led2_off()      gpio_pin_set_dt(&gpio_led2,0)
#define led2_toggle()   gpio_pin_toggle_dt(&gpio_led2);

#define INT_OPTION      1

#define INT_IN_PIN NRF_GPIO_PIN_MAP(0, button_1.pin)

static const struct gpio_dt_spec button_1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);

static const struct gpio_dt_spec gpio_led0 = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec gpio_led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec gpio_led2 = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

struct gpio_callback cb_data;

void check_error(nrfx_err_t error, uint32_t line);

void int_callback_handler(const struct device *dev,struct gpio_callback *cb, uint32_t pins)
{
        NRF_GPIOTE->EVENTS_IN[GPIOTE_IRQn] = 0;
        led2_toggle();
        LOG_INF("GPIO interrupt!");
}

ISR_DIRECT_DECLARE(my_isr)
{
        NRF_GPIOTE->EVENTS_IN[GPIOTE_IRQn] = 0;
        led2_toggle();
        
        LOG_INF("ISR DIRECT GPIO interrupt!");
        ISR_DIRECT_PM(); /* PM done after servicing interrupt for best latency */
        return 1;
}

int main(void)
{
        LOG_INF("GPIO interrupt example");

        #if (INT_OPTION == 1)
        {
                // INT OPTION 1
                // Zephyr interrupt takes ~20us to service ISR
                gpio_pin_configure_dt(&button_1,GPIO_INPUT);
                gpio_pin_interrupt_configure_dt(&button_1,GPIO_INT_EDGE_TO_ACTIVE);
                gpio_init_callback(&cb_data,int_callback_handler,BIT(button_1.pin));
                gpio_add_callback(button_1.port, &cb_data);
        }
        #elif (INT_OPTION == 2)
        {
                // INT OPTION 2
                // Interrupt takes ~10us to service ISR, however ISR continues to trigger.
                int err_code;
                err_code = nrfx_gpiote_init(0);
                check_error(err_code, __LINE__);

                nrfx_gpiote_in_config_t in_config = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
                in_config.pull = NRF_GPIO_PIN_PULLUP;

                err_code = nrfx_gpiote_in_init(INT_IN_PIN, &in_config, my_isr);
                IRQ_DIRECT_CONNECT(GPIOTE_IRQn, 0, my_isr,IRQ_ZERO_LATENCY);

                //err_code = nrfx_gpiote_in_init(INT_IN_PIN, &in_config, int_callback_handler);
                //IRQ_DIRECT_CONNECT(GPIOTE_IRQn, 0, int_callback_handler,IRQ_ZERO_LATENCY);

                check_error(err_code, __LINE__);

                irq_enable(GPIOTE_IRQn);
                nrfx_gpiote_trigger_enable(INT_IN_PIN, true);
        }
        #endif

        // confgure LEDs
        gpio_pin_configure_dt(&gpio_led0,GPIO_OUTPUT);
        gpio_pin_configure_dt(&gpio_led1,GPIO_OUTPUT);
        gpio_pin_configure_dt(&gpio_led2,GPIO_OUTPUT);

        led2_off();

        while(1)
        {
                gpio_pin_toggle_dt(&gpio_led0);
                k_msleep(500);

                gpio_pin_set_dt(&gpio_led1,gpio_pin_get_dt(&button_1));             
        }

        return 0;
}

void check_error(nrfx_err_t error, uint32_t line)
{
	if (error != NRFX_SUCCESS)
		printk("Error on line %d: %d\n", line, error - NRFX_SUCCESS);
}