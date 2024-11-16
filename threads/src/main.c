/*
 * Zephyr threads example
 * Demonstrates the use of threads and a FIFO queue for data sharing.
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(main);

char *fw_version_str = "V0.10";


// Thread definitions
#define THREAD1_STACKSIZE       1024
#define THREAD2_STACKSIZE       512
#define THREAD1_PRIORITY        7
#define THREAD2_PRIORITY        4

// Function prototypes
void thread1();
void thread2();


K_THREAD_DEFINE(thread1_id, THREAD1_STACKSIZE, thread1, NULL, NULL, NULL,THREAD1_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread2_id, THREAD2_STACKSIZE, thread2, NULL, NULL, NULL,THREAD2_PRIORITY, 0, 0);


struct shared_data_t {
	void *fifo_reserved; /* 1st word reserved for use by fifo */
	bool changed;
	uint8_t value;
};

// Use a FIFO queue to manage data sharing.
K_FIFO_DEFINE(shared_data_fifo);

// LEDs for nRF52840 DK
static struct gpio_dt_spec led[] = {    GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0}),
				        GPIO_DT_SPEC_GET_OR(DT_ALIAS(led1), gpios, {0}),
				        GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios, {0})};

static const struct gpio_dt_spec btn0 = GPIO_DT_SPEC_GET(DT_ALIAS(sw0), gpios);
static const struct gpio_dt_spec btn1 = GPIO_DT_SPEC_GET(DT_ALIAS(sw1), gpios);

void thread1()
{
        LOG_INF("Thread1: initialise LED 1");
        gpio_pin_configure_dt(&led[1],GPIO_OUTPUT_HIGH);

        while(1)
        {
                gpio_pin_set_dt(&led[1],1);     // Turn on LED 1
                //k_msleep(100);

                struct shared_data_t *shared_data1 = k_fifo_get(&shared_data_fifo,K_MSEC(100));
                gpio_pin_set_dt(&led[1],0);     // Turn off LED 1
                if(shared_data1 != NULL) // Check if data is available in FIFO queue
                {
                        // Data is available.
                        if(shared_data1->changed)
                        {
                                LOG_INF("Shared data available: %d", shared_data1->value);
                                k_msleep(100);
                        }
                }
                else
                {
                        k_msleep(900);
                }
                k_free(shared_data1);
        }
}

void thread2()
{
        LOG_INF("Thread2: initialise LED 2");
        gpio_pin_configure_dt(&led[2],GPIO_OUTPUT_HIGH);

        while(1)
        {
                gpio_pin_set_dt(&led[2],1);     // Turn on LED 2
                k_msleep(50);
                gpio_pin_set_dt(&led[2],0);     // Turn off LED 2
                k_msleep(950);
        }
}

int main(void)
{
        LOG_INF("Zephyr thread example");
	LOG_INF("Board: %s", CONFIG_BOARD);
	LOG_INF("Firmware version: %s", fw_version_str);
        
        LOG_INF("Main: initialise LED 0");
        gpio_pin_configure_dt(&led[0],GPIO_OUTPUT_HIGH);

        gpio_pin_configure_dt(&btn0, GPIO_INPUT);
        gpio_pin_configure_dt(&btn1, GPIO_INPUT);

        // Definitions for FIFO
        size_t size = sizeof(struct shared_data_t);
        struct shared_data_t shared_data0;

        LOG_INF("Entering main loop");
        uint8_t shared_data_counter = 0;

        while(1)
        {
                gpio_pin_set_dt(&led[0],1);     // Turn on LED 0
                k_msleep(100);
                gpio_pin_set_dt(&led[0],0);     // Turn off LED 0
                k_msleep(100);

                if(gpio_pin_get_dt(&btn0) == 1) // Check if btn0 depressed
                {       
                        // Put shared data onto FIFO queue
                        shared_data0.changed = true;
                        shared_data0.value = shared_data_counter++;
		        char *mem_ptr = k_malloc(size);
		        __ASSERT_NO_MSG(mem_ptr != 0);

                        memcpy(mem_ptr, &shared_data0, size);

                        k_fifo_put(&shared_data_fifo, mem_ptr);
                }
        }

        return 0;
}
