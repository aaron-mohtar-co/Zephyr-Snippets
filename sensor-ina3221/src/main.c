/* 
 * INA3221 example
 * https://www.ti.com/lit/gpn/INA3221
 * 
 * Copyright 2023 Aaron Mothar & Co Pty Ltd
 * Copyright (c) Aaron Mohtar & Craig Peacock
 * SPDX-License-Identifier: Apache-2.0
 * 
 * If this example does not work due to sensor driver issue, check the following:
 * - The ina3221 folder is under zephyr/drivers/sensor. The folder can be downloaded from: https://github.com/zephyrproject-rtos/zephyr/tree/d44e96e486ae442d523c3b88ba1ec39b6d5891bd/drivers/sensor/ina3221
 * - This line is added to CMakeLists.txt (found in zephyr/drivers/sensor): add_subdirectory_ifdef(CONFIG_INA3221 ina3221)
 * - This line is added to Kconfig (found in zephyr/drivers/sensor): source "drivers/sensor/ina3221/Kconfig"
 * - This file is in the zephyr\dts\bindings\sensor folder: ti,ina3221.yaml. This can be found here https://github.com/zephyrproject-rtos/zephyr/blob/d44e96e486ae442d523c3b88ba1ec39b6d5891bd/dts/bindings/sensor/ti%2Cina3221.yaml
 *
 * Other potential errors: a float function in ina3221.c. The function can be found online in sensor.c (?)
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/shell/shell.h>

/*
 * The following is defined in zephyr/drivers/sensor/ti/ina3221/ina3221.h
 * It should have a specific header file in zephyr/include/zephyr/drivers/sensor/ that we can include, but doesn't.
 */
#define SENSOR_ATTR_INA3221_SELECTED_CHANNEL (SENSOR_ATTR_PRIV_START+1)

const struct device *dev_ina3221 = DEVICE_DT_GET_ANY(ti_ina3221);

static int ina3221_cmd_configure_handler(const struct shell *sh, size_t argc, char **argv);
static int ina3221_cmd_measure_handler(const struct shell *sh, size_t argc, char **argv);

// Shell configuration
SHELL_STATIC_SUBCMD_SET_CREATE
(sub_ina3221,
	SHELL_CMD(configure, NULL, "not yet implemented", ina3221_cmd_configure_handler),
	SHELL_CMD(measure, NULL, "measure <channel number>", ina3221_cmd_measure_handler),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(ina3221, &sub_ina3221, "INA3221 based measurement system", NULL);

int main(void)
{
	printk("INA3221 Sensor Example with Shell\n");

	if (!dev_ina3221) {
		printk("I2C: Device driver for ina3221 not found\n");
	}

	while(1) {
		k_msleep(2000);
	}

	return 0;
}

static int ina3221_cmd_configure_handler(const struct shell *sh, size_t argc, char **argv)
{
	shell_print(sh,"INA3221 configure command received - not implemented yet\n");
	return 0;
}

static int ina3221_cmd_measure_handler(const struct shell *sh, size_t argc, char **argv)
{
	double voltage;
	double current;
	double power;

	int channel = atoi(argv[1]); 

	struct sensor_value value;
	value.val1 = channel;
	sensor_attr_set(dev_ina3221, SENSOR_CHAN_VOLTAGE, SENSOR_ATTR_INA3221_SELECTED_CHANNEL, &value);

	sensor_sample_fetch(dev_ina3221);

	sensor_channel_get(dev_ina3221, SENSOR_CHAN_VOLTAGE, &value);
	voltage = sensor_value_to_double(&value);

	sensor_channel_get(dev_ina3221, SENSOR_CHAN_CURRENT, &value);
	current = sensor_value_to_double(&value);

	sensor_channel_get(dev_ina3221, SENSOR_CHAN_POWER, &value);
	power = sensor_value_to_double(&value);

	shell_print(sh,"INA3221 CH%d %2.2fV %2.3fA %2.2fW", channel, voltage, current, power);
	return 0;
}