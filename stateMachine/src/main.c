/*
 * Zephyr State machine example
 * Provides a template for firmware that utilises a state machine
 * 
 * Copyright (c) 2024 Aaron Mohtar & Co Pty Ltd
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

// define the different states
#define STATE_INITIALISING	0
#define STATE_IDLE			1
#define STATE_DOSOMETHING	2
#define STATE_POWERDOWN		3

#define STATE_MACHINE_DELAY_NORMAL_MS	50	// default state machine is 50ms.

char *fw_version_str = "V0.10";

// Define all the strings that correspond to the states. These are used in debug messages.
char *device_state_strings[] = {"Initialising", "Idle", "Do something", "Power down"};

uint8_t device_state = STATE_INITIALISING;
uint8_t state_changed_flag = true;	// Used to flag when the state has just changed.
uint32_t state_machine_delay = STATE_MACHINE_DELAY_NORMAL_MS;

static struct k_work_delayable state_machine_work;

static const struct gpio_dt_spec led1 = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);

/**
 * @brief Main state machine. Function keeps calling itself after preset delay using k_work_schedule()
 */
static void process_state_machine(struct k_work *work)
{
	// Reset state machine delay to default. This can be changed in the switch statement.
	state_machine_delay = STATE_MACHINE_DELAY_NORMAL_MS;

	switch (device_state)
	{
		case STATE_INITIALISING:
		{
			// Just changed to this state?
			if(state_changed_flag == true)
			{
				state_changed_flag = false;
				printk("Entered new state %s\n", device_state_strings[device_state]);

				// Initialise LED output pin
				gpio_pin_configure_dt(&led1,GPIO_OUTPUT);
				gpio_pin_set_dt(&led1, 0);
			}
			else
			{
				// Do something/check something?
				device_state = STATE_IDLE;
				state_changed_flag = true;
			}
			break;
		}
		case STATE_IDLE:
		{
			// Just changed to this state?
			if(state_changed_flag == true)
			{
				state_changed_flag = false;
				printk("Entered new state %s\n", device_state_strings[device_state]);

				// Do stuff the first time it enters this state
			}
			else
			{
				// Do something/check something?
				//device_state = STATE_XXXX;
				//state_changed_flag = true;
			}
			break;
		}
		case STATE_DOSOMETHING:
		{
			// Just changed to this state?
			if(state_changed_flag == true)
			{
				state_changed_flag = false;
				printk("Entered new state %s\n", device_state_strings[device_state]);

				// Do stuff the first time it enters this state

			}
			else
			{
				// Do something/check something?
				//device_state = STATE_XXXX;
				//state_changed_flag = true;
			}
			break;
		}
		case STATE_POWERDOWN:
		{
			// Just changed to this state?
			if(state_changed_flag == true)
			{
				state_changed_flag = false;
				printk("Entered new state %s\n", device_state_strings[device_state]);

				// Do stuff the first time it enters this state

			}
			else
			{
				// Do something/check something?
				//device_state = STATE_XXXX;
				//state_changed_flag = true;
			}
			break;
		}
		default:
		{
			printk("Reached an unimplmented state %d\n", device_state);
			state_machine_delay = 1000;	// Delay by 1000ms 
			break;
		}
	}

	// Schedule next state_machine iteration
	k_work_schedule(&state_machine_work, K_MSEC(state_machine_delay));
}


int main(void)
{
	printk("State machine example \n");
	printk("Board: %s\n", CONFIG_BOARD);
	printk("Firmware version: %s\n", fw_version_str);

	printk("Start state machine\n");
	k_work_init_delayable(&state_machine_work, process_state_machine);
	k_work_schedule(&state_machine_work, K_NO_WAIT);

	printk("Entering main loop\n");

	while (1) {	
		k_msleep(1000);
		gpio_pin_toggle_dt(&led1);
	}

	return 0;
}
