/*
 * Copyright (c) 2023 Aaron Mohtar & Co Pty Ltd
 * Copyright (c) 2020 Libre Solar Technologies GmbH
 *
 * Reads two ADC Channels specified in the Device Tree overlay 
 * (See boards directory). ADC0 is set to P0.05 and takes a
 * single sample. ADC1 is measuring the VDD voltage oversampled 
 * 8 times.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/kernel.h>

#if !DT_NODE_EXISTS(DT_PATH(zephyr_user)) || \
	!DT_NODE_HAS_PROP(DT_PATH(zephyr_user), io_channels)
	#error "No suitable devicetree overlay specified"
#endif

#define DT_SPEC_AND_COMMA(node_id, prop, idx) \
	ADC_DT_SPEC_GET_BY_IDX(node_id, idx),

// Data of ADC io-channels specified in DeviceTree.
static const struct adc_dt_spec adc_channels[] = {
	DT_FOREACH_PROP_ELEM(DT_PATH(zephyr_user), io_channels,
				DT_SPEC_AND_COMMA)
};

void main(void)
{
	int err;
	int16_t buf;

	printk("ADC Zephyr Example\nBoard: %s\n", CONFIG_BOARD);

	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf), // buffer size in bytes, not number of samples
		//channels, resolution, oversampling populated by adc_sequence_init_dt() from DeviceTree
	};

	// Configure ADC channels individually prior to sampling
	for (size_t i = 0U; i < ARRAY_SIZE(adc_channels); i++) {
		// Print out configuration (for validating DeviceTree)
		printk("Channel %d:\n", adc_channels[i].channel_id);
		printk(" Resolution %d bits\n", adc_channels[i].resolution);
		printk(" Oversampling %d times\n", adc_channels[i].oversampling);
		printk(" Reference %dmV\n", adc_channels[i].vref_mv);
		printk(" Gain %d\n", adc_channels[i].channel_cfg.gain);

		if (!device_is_ready(adc_channels[i].dev)) {
			printk("ADC controller device not ready\n");
			return;
		}

		err = adc_channel_setup_dt(&adc_channels[i]);
		if (err < 0) {
			printk("Could not setup channel #%d (%d)\n", i, err);
			return;
		}
	}

	k_sleep(K_MSEC(1000));

	while (1) {

		printk("ADC reading:\n");

		for (size_t i = 0; i < ARRAY_SIZE(adc_channels); i++) {

			printk("Channel %d: ", adc_channels[i].channel_id);

			// Populate channel, resolution and oversampling from DeviceTree
			(void)adc_sequence_init_dt(&adc_channels[i], &sequence);

			// Request a conversion
			err = adc_read(adc_channels[i].dev, &sequence);
			if (err < 0) {
				printk("Could not read (%d)\n", err);
				continue;
			} else {
				printk("Raw: %d, ", buf);
			}

			// Convert to scaled voltage (mV)
			int32_t val_mv = buf;
			err = adc_raw_to_millivolts_dt(&adc_channels[i], &val_mv);
			if (err < 0) {
				printk("Scaled value not avaliable\n");
			} else {
				printk("Scaled: %d mV\n", val_mv);
			}
		}

		k_sleep(K_MSEC(1000));
	}
}
