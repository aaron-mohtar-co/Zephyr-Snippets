
/*
 * led.c
 *
 * Copyright 2023 Aaron Mothar & Co Pty Ltd
 * Copyright (c) Aaron Mohtar & Craig Peacock
 * SPDX-License-Identifier: Apache-2.0
 *
 * This works with the LemonIot boards & requires three defined PWM pins:
 * - red_pwm_led
 * - green_pwm_led
 * - blue_pwm_led
 */

#include "led.h"

static struct rgb_led_data rgbData;
static enum led_mode mode = OFF;

static uint32_t pulse_width = 0U;

static struct k_work_delayable led_indicator_work;
static void led_indicator_work_function(struct k_work *work);

static const struct pwm_dt_spec pwm_led[] = {PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led)),
											 PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led)),
											 PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led))};

static void led_indicator_work_function(struct k_work *work)
{
	// In test mode, don't do anything with LEDs.
	// if(mode == TEST)
	//{
	//	return;
	//}

	switch (rgbData.mode)
	{
	case ON:
	{
		rgbData.redLevel = rgbData.redLevelMax;
		rgbData.greenLevel = rgbData.greenLevelMax;
		rgbData.blueLevel = rgbData.blueLevelMax;
		break;
	}
	case OFF:
	{
		rgbData.redLevel = 0;
		rgbData.greenLevel = 0;
		rgbData.blueLevel = 0;
		break;
	}
	case HEARTBEAT:
	{

		if (rgbData.heartrateDirection == UP)
		{
			rgbData.redLevel += rgbData.redPulseStep;
			rgbData.greenLevel += rgbData.greenPulseStep;
			rgbData.blueLevel += rgbData.bluePulseStep;

			if ((rgbData.redLevel > rgbData.redLevelMax) || (rgbData.greenLevel > rgbData.greenLevelMax) || (rgbData.blueLevel > rgbData.blueLevelMax))
			{
				rgbData.redLevel = rgbData.redLevelMax;
				rgbData.greenLevel = rgbData.greenLevelMax;
				rgbData.blueLevel = rgbData.blueLevelMax;
				rgbData.heartrateDirection = DOWN;
			}
		}
		else // direction is DOWN
		{
			if ((rgbData.redLevel < rgbData.redPulseStep) || (rgbData.greenLevel < rgbData.greenPulseStep) || (rgbData.blueLevel < rgbData.bluePulseStep))
			{
				rgbData.redLevel = 0;
				rgbData.greenLevel = 0;
				rgbData.blueLevel = 0;
				rgbData.heartrateDirection = UP;
			}
			else
			{
				rgbData.redLevel -= rgbData.redPulseStep;
				rgbData.greenLevel -= rgbData.greenPulseStep;
				rgbData.blueLevel -= rgbData.bluePulseStep;
			}
		}
		break;
	}
	case PULSING:
	{

		break;
	}
	default:
	{
	}
	}

	led_configPwm(rgbData.redLevel, RED);
	led_configPwm(rgbData.greenLevel, GREEN);
	led_configPwm(rgbData.blueLevel, BLUE);

	if (rgbData.mode == HEARTBEAT)
	{
		k_work_schedule(&led_indicator_work, K_MSEC(25));
	}
	else
	{
		k_work_schedule(&led_indicator_work, K_MSEC(10));
	}

	return;

	// if (ret)
	//{
	//	printk("Error %d: failed to set pulse width\n", ret);
	//	return;
	// }
}
/*

 */
void led_initialise(uint8_t red, uint8_t green, uint8_t blue, enum led_mode newMode, uint16_t periodX10, uint8_t onPeriodX10)
{
	if (!device_is_ready(pwm_led[0].dev))
	{
		printk("Error: PWM device %s is not ready\n", pwm_led[0].dev->name);
		return;
	}

	led_changeColor(red, green, blue);
	rgbData.mode = newMode;
	rgbData.periodMsX10 = periodX10;
	rgbData.onPeriodMsX10 = onPeriodX10;

	k_work_init_delayable(&led_indicator_work, led_indicator_work_function);
	k_work_schedule(&led_indicator_work, K_MSEC(10));
}

void led_changeColor(uint8_t red, uint8_t green, uint8_t blue)
{
	rgbData.redLevelMax = red;
	rgbData.greenLevelMax = green;
	rgbData.blueLevelMax = blue;

	rgbData.redPulseStep = (float)rgbData.redLevelMax / 40;
	rgbData.greenPulseStep = (float)rgbData.greenLevelMax / 40;
	rgbData.bluePulseStep = (float)rgbData.blueLevelMax / 40;

	rgbData.redLevel = 0;
	rgbData.greenLevel = 0;
	rgbData.blueLevel = 0;
	rgbData.heartrateDirection = UP;
}

void led_changeMode(enum led_mode newMode)
{

}

int led_configPwm(uint8_t intensity, enum led_color color)
{
	pulse_width = ((float)intensity * pwm_led[color].period) / 256;
	return pwm_set_pulse_dt(&pwm_led[color], pulse_width);
}