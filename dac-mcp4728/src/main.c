/* 
 * mcp4728 example
 * https://www.microchip.com/en-us/product/mcp4728
 * 
 * Copyright 2024 Aaron Mothar & Co Pty Ltd
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
#include <string.h>
#include <hal/nrf_gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/dac.h>

#include <zephyr/settings/settings.h>

#include <zephyr/shell/shell.h>

//#define DAC_SPEED_TEST

struct shell *shellPtr;
const struct device *dev_mcp4728 = DEVICE_DT_GET(DT_NODELABEL(dac0));

const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(uart0));

static int mcp4728_cmd_configure_handler(const struct shell *sh, size_t argc, char **argv);
static int mcp4728_cmd_channel_set_handler(const struct shell *sh, size_t argc, char **argv);

// Shell configuration
SHELL_STATIC_SUBCMD_SET_CREATE
(sub_mcp4728,
	SHELL_CMD(configure, NULL, "NOT IMPLEMENTED YET", mcp4728_cmd_configure_handler),
        SHELL_CMD(set, NULL, "set <channel number 0-4> <value 0-4095>", mcp4728_cmd_channel_set_handler),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(mcp4728, &sub_mcp4728, "MCP4728 4-channel DAC output", NULL);


int main(void)
{
        k_msleep(100);
        printk("MCP4728 DAC test with SHELL\r\n");
	
	if (!device_is_ready(uart)){
		printk("UART device not ready\r\n");
		return 1 ;
	}

        if (!dev_mcp4728) {
		printk("I2C: Device driver for mcp4728 not found.\n");
	} 
        else
        {
             /*   const struct dac_channel_cfg *	channel_cfg;
                channel_cfg.channel_id = 0;
                channel_cfg.resolution= 12;
                dac_channel_setup(&dev_mcp4728,channel_cfg);
                */
        }
        uint16_t counter = 0;
        while(1)
        {
                //


        #ifdef DAC_SPEED_TEST
                // I2C speed set to 100KHz, 490us
                // I2C speed set to 400KHz, 162us
                // I2C speed set at 1000KHz, not supported in DTS
                dac_write_value(dev_mcp4728,0,counter); // Sends a DAC output to channel 0. Check above for speed of execution

                counter = counter + 1000;

                if(counter >= 4096)     // 12 bit resolution
                        counter = 0;
        #else
                k_msleep(100);
        #endif

        }
        return 0;
}

static int mcp4728_cmd_configure_handler(const struct shell *sh, size_t argc, char **argv)
{
        shell_print(sh,"mcp4728 configure cmd received - COMMAND NOT IMPLEMENTED YET\n");
        return 0;
}

static int mcp4728_cmd_channel_set_handler(const struct shell *sh, size_t argc, char **argv)
{
        int channel = atoi(argv[1]); 
        int dac_value = atoi(argv[2]); 
        int err = 0;

        if((channel >= 4) || (channel < 0))
        {
                shell_print(sh,"Incorrect channel number. Accepted values 0 - 4.");
                return -1;
        }
        
        if(dac_value > 4095)
        {
                dac_value = 4095;
                shell_print(sh,"Value is greater than max. Value set to 4095.");
        }
        
        err = dac_write_value(dev_mcp4728,channel,dac_value);
        shell_print(sh,"Write to DAC channel %d with %d and response: %d", channel, dac_value,err);
        return 0;
}

