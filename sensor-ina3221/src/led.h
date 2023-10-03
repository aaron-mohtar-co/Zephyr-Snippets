
/*
 * led.h
 *
 * Copyright 2023 Aaron Mothar & Co Pty Ltd
 * Copyright (c) Aaron Mohtar & Craig Peacock
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>


enum device_mode{
	NORMAL,
	TEST,
};

enum led_mode{
	OFF,
	ON,
	HEARTBEAT,
	PULSING,
};

enum led_color
{
	RED = 0,
	GREEN,
	BLUE,
};

enum led_heartrateDirection
{
	UP,
	DOWN,
};

struct rgb_led_data{
	uint8_t redLevelMax;
	uint8_t greenLevelMax;
	uint8_t blueLevelMax;
	float redLevel;
	float greenLevel;
	float blueLevel;
	float redPulseStep;
	float greenPulseStep;
	float bluePulseStep;
	uint8_t heartrateDirection;
	enum led_mode	mode;
	uint16_t periodMsX10;
	uint8_t onPeriodMsX10;
};

static void led_indicator_work_function(struct k_work *work);

/** @brief Initialise the LED and set the default settings 
 * @param red sets the intensity of the red LED (0x00-0xFF)
 * @param green sets the intensity of the green LED (0x00-0xFF)
 * @param blue sets the intensity of the blue LED (0x00-0xFF)
 * @param newMode sets the LED mode: OFF = 0, ON = 1, HEARTRATE = 2, PUSLING = 3
 * @param periodX10 set the period of the LED in 10's or ms. This only applies when mode = PULSING. E.g. 100 -> 1000ms = 1s
 * @param onPeriodX10 set the ON period of the LED in 10's of ms. This only applies when mode = PULSING. 1 -> 10ms
*/
void led_initialise(uint8_t red,uint8_t green, uint8_t blue, enum led_mode newMode, uint16_t periodX10,uint8_t onPeriodX10);

/** @brief Changes ONLY the color of LED
 * @param red sets the intensity of the red LED (0x00-0xFF)
 * @param green sets the intensity of the green LED (0x00-0xFF)
 * @param blue sets the intensity of the blue LED (0x00-0xFF)
*/
void led_changeColor(uint8_t red,uint8_t green, uint8_t blue);

/** @brief Changes the LED mode 
 * @param newMode LED modes: OFF = 0, ON = 1, HEARTRATE = 2, PUSLING = 3
*/
void led_changeMode(enum led_mode newMode);

int led_configPwm(uint8_t intensity,enum led_color color);