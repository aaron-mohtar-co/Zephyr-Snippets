/*
 * Copyright (c) 2022, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* 
 * ADC Example for Nordic Semiconductor SoCs
 * Uses vendor specific nrf_saadc_* API instead of common Zephyr API.
 * 
 * Reads an analog voltage from two ADC inputs (P0.30 & P0.31) and prints 
 * result on console.
 * 
 * This operates in NON-BLOCKING mode. P0.25 toggles each time the 
 * NRF_SAADC_EVENT_RESULTDONE is triggered.
 * 
 * Nordic has SAADC examples located in your nRF Connect SDK folder at
 * <nRF Connect SDK root>\modules\hal\nordic\samples\src\nrfx_saadc
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>

#include <helpers/nrfx_gppi.h>
#include <nrfx_saadc.h>
#include <nrfx_timer.h>
#include <nrfx_gpiote.h>


#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(ADC);

#define ADC_NUM_OF_CHANNELS		2

#define ADC_INPUT_0_CHANNEL_INDEX	0
#define ADC_INPUT_1_CHANNEL_INDEX	1

#define ADC_INPUT_0_PIN		NRF_SAADC_INPUT_AIN6	// AIN6 = P0.30
#define ADC_INPUT_1_PIN		NRF_SAADC_INPUT_AIN7	// AIN7 = P0.31

#define ADC_BUFF_SIZE 64

/** @brief Symbol specifying timer instance to be used. */
#define TIMER_INST_IDX 0

/** @brief Symbol specifying GPIO pin used to test the functionality of SAADC. */
#define OUT_GPIO_PIN 25	// P0.25

/** @brief Acquisition time [us] */
#define ACQ_TIME 40UL

/** @brief Acquisition time [us] used in channel configuration */
#define ADC_CHANNEL_ACQ_TIME	NRF_SAADC_ACQTIME_40US

/** @brief Conversion time [us] (see SAADC electrical specification). */
#define CONV_TIME 2UL

/**
 * @brief Symbol specifying the number of sample buffers ( @ref adc_rawDataBuffer ).
 *        Two buffers are required for performing double-buffered conversions.
 */
#define BUFFER_COUNT 2UL

/** @brief Symbol specifying the size of singular sample buffer ( @ref adc_rawDataBuffer ). */
#define BUFFER_SIZE 2UL

/** @brief Symbol specifying the number of SAADC samplings to trigger. */
#define SAMPLING_ITERATIONS 3UL

/** @brief Symbol specifying maximal possible SAADC sample rate (see SAADC electrical specification). */
#define MAX_SAADC_SAMPLE_FREQUENCY 200000UL

/** @brief Symbol specifying SAADC sample frequency for the continuous sampling. */
#define SAADC_SAMPLE_FREQUENCY MAX_SAADC_SAMPLE_FREQUENCY

/** @brief Symbol specifying time in microseconds to wait for timer handler execution. */
#define TIME_TO_WAIT_US (uint32_t)(1000000UL / SAADC_SAMPLE_FREQUENCY)


/** @brief Samples buffer to store values from a SAADC channel. */
static nrf_saadc_value_t adc_rawDataBuffer[BUFFER_COUNT][BUFFER_SIZE];

/** @brief Array of the GPPI channels. */
static uint8_t m_gppi_channels[2];

/** @brief Enum with intended uses of GPPI channels defined as @ref m_gppi_channels. */
typedef enum
{
    SAADC_SAMPLING,     ///< Triggers SAADC sampling task on external timer event.
    SAADC_START_ON_END, ///< Triggers SAADC start task on SAADC end event.
} gppi_channels_purpose_t;


// Function prototypes 
uint8_t adc_initialise();

/** TAKEN FROM saadc_examples_common.h
 * @brief Function for setting up a pin to be toggled once specified event is triggered.
 *
 * @param pin The pin to toggle.
 * @param eep Address of the event register. This event will trigger the @p pin to toggle.
 */
void pin_on_event_toggle_setup(nrfx_gpiote_pin_t pin,
                               uint32_t          eep);

/**
 * @brief Function for handling TIMER driver events.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of the timer.
 *                       This parameter can be used to pass additional information to the handler
 *                       function for example the timer ID.
 */
static void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    (void)event_type;
    (void)p_context;
}

static void adc_eventHandler(nrfx_saadc_evt_t const * p_event)
{

	nrfx_err_t status;
    (void)status;

    static uint16_t buffer_index = 1;
    static uint16_t buf_req_evt_counter;
    uint16_t samples_number;

	switch (p_event->type) {
		case NRFX_SAADC_EVT_DONE:
			LOG_INF("RFX_SAADC_EVT_DONE");
			LOG_INF("ADC result %d %d", p_event->data.done.p_buffer[0], p_event->data.done.p_buffer[1]);
			break;
			
		case NRFX_SAADC_EVT_LIMIT:
			LOG_INF("NRFX_SAADC_EVT_LIMIT");
			break;

		case NRFX_SAADC_EVT_CALIBRATEDONE:
			LOG_INF("NRFX_SAADC_EVT_CALIBRATEDONE\n");

            status = nrfx_saadc_mode_trigger();
            //NRFX_ASSERT(status == NRFX_SUCCESS);
			break;

		case NRFX_SAADC_EVT_BUF_REQ:
			LOG_INF("NRFX_SAADC_EVT_BUF_REQ\n");
			if (++buf_req_evt_counter < SAMPLING_ITERATIONS)
            {
                /* Next available buffer must be set on the NRFX_SAADC_EVT_BUF_REQ event to achieve the continuous conversion. */
                status = nrfx_saadc_buffer_set(adc_rawDataBuffer[buffer_index++], BUFFER_SIZE);
                NRFX_ASSERT(status == NRFX_SUCCESS);
                buffer_index = buffer_index % BUFFER_COUNT;
            }
            else
            {
                nrfx_gppi_channels_disable(NRFX_BIT(m_gppi_channels[SAADC_START_ON_END]));
            }
			break;
		
		case NRFX_SAADC_EVT_READY:
			LOG_INF("NRFX_SAADC_EVT_READY\n");
			nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));
			break;

		case NRFX_SAADC_EVT_FINISHED:
			LOG_INF("NRFX_SAADC_EVT_FINISHED\n");
			nrfx_gppi_channels_disable(NRFX_BIT(m_gppi_channels[SAADC_SAMPLING]));
			break;

		default:
			LOG_INF("Unknown ADC Event %d\n", p_event->type);
			break;
	}
}

uint8_t adc_initialise()
{
	nrfx_err_t status;
    (void)status;

	nrfx_err_t err;

	// Initialise SAADC driver
	err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
	if(err != NRFX_SUCCESS)
		LOG_INF("Error %d in nrfx_saadc_init()\n", err);

	IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0);

	// Initialise Timer
	nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(TIMER_INST_IDX);
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG;
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
    timer_config.p_context = &timer_inst;

    status = nrfx_timer_init(&timer_inst, &timer_config, timer_handler);
    //NRFX_ASSERT(status == NRFX_SUCCESS);

	nrfx_timer_clear(&timer_inst);

    /* Creating variable desired_ticks to store the output of nrfx_timer_us_to_ticks function. */
    uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US);


	nrfx_saadc_channel_t adc_input1Config = 
	{
		.channel_config ={
			.reference = NRF_SAADC_REFERENCE_VDD4,		// Reference = VDD / 4
			.acq_time = ADC_CHANNEL_ACQ_TIME,			// Acquisition time 40uS
			.mode = NRF_SAADC_MODE_SINGLE_ENDED,		// Single Ended
			.burst = NRF_SAADC_BURST_DISABLED, 			// Normal Operation (Burst disabled)
			.gain = NRF_SAADC_GAIN1_4,
		},
		.channel_index = ADC_INPUT_0_CHANNEL_INDEX,		// ADC0
		.pin_p = ADC_INPUT_0_PIN						// Input 0 = AIN6/P0.30
	};

	nrfx_saadc_channel_t adc_input2Config = 
	{
		.channel_config ={
			.reference = NRF_SAADC_REFERENCE_VDD4,		// Reference = VDD / 4
			.acq_time = ADC_CHANNEL_ACQ_TIME,			// Acquisition time 40uS
			.mode = NRF_SAADC_MODE_SINGLE_ENDED,		// Single Ended
			.burst = NRF_SAADC_BURST_DISABLED,			// Normal Operation (Burst disabled)
			.gain = NRF_SAADC_GAIN1_4,
		},
		.channel_index = ADC_INPUT_1_CHANNEL_INDEX,		// ADC1
		.pin_p = ADC_INPUT_1_PIN						// Input 1 = AIN7/P0.31
	};

	// Configuration includes the two inputs described above
	nrfx_saadc_channel_t adc_configArray[ADC_NUM_OF_CHANNELS] = {
		adc_input1Config, 
		adc_input2Config
	};

	// Apply configuration above
	err = nrfx_saadc_channels_config(adc_configArray, ADC_NUM_OF_CHANNELS);
	if(err != NRFX_SUCCESS)
		LOG_INF("Error %d in nrfx_saadc_channels_config()\n", err);

    /*
     * Setting the timer channel NRF_TIMER_CC_CHANNEL0 in the extended compare mode to clear
     * the timer and to not trigger an interrupt if the internal counter register is equal to
     * desired_ticks.
     */
    nrfx_timer_extended_compare(&timer_inst,
                                NRF_TIMER_CC_CHANNEL0,
                                desired_ticks,
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                false);

	/* ADC advanced configuration */
	nrfx_saadc_adv_config_t adc_advancedConfig =
	{
		.start_on_end = false,
		.burst = NRF_SAADC_BURST_DISABLED,
		.internal_timer_cc = 0,
	};

	// For non-blocking provide adc event handler function
    err = nrfx_saadc_advanced_mode_set(
		BIT(ADC_INPUT_0_CHANNEL_INDEX)|BIT(ADC_INPUT_1_CHANNEL_INDEX),
        NRF_SAADC_RESOLUTION_12BIT,
        &adc_advancedConfig,
        adc_eventHandler);
    if(err != NRFX_SUCCESS)
		LOG_INF("Error %d in nrfx_saadc_simple_mode_set()\n", err);

	// Set-up buffer
    status = nrfx_saadc_buffer_set(adc_rawDataBuffer[0], BUFFER_SIZE);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    /*
     * Allocate a dedicated channel and configure endpoints of that channel so that the timer compare event
     * is connected with the SAADC sample task. This means that each time the timer interrupt occurs,
     * the SAADC sampling will be triggered.
     */
    status = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_SAMPLING]);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_SAMPLING],
        nrfx_timer_compare_event_address_get(&timer_inst, NRF_TIMER_CC_CHANNEL0),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE));

    /*
     * Allocate a dedicated channel and configure endpoints of that so that the SAADC event end is connected
     * with the SAADC task start. This means that each time the SAADC fills up the result buffer,
     * the SAADC will be restarted and the result buffer will be prepared in RAM.
     */
    status = nrfx_gppi_channel_alloc(&m_gppi_channels[SAADC_START_ON_END]);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gppi_channel_endpoints_setup(m_gppi_channels[SAADC_START_ON_END],
        nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
        nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START));

    pin_on_event_toggle_setup(OUT_GPIO_PIN,
                       nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_RESULTDONE));

    nrfx_timer_enable(&timer_inst);

    nrfx_gppi_channels_enable(NRFX_BIT(m_gppi_channels[SAADC_START_ON_END]));

    status = nrfx_saadc_offset_calibrate(adc_eventHandler);

	return 0;
}

void main(void)
{
	int ret;

	printk("ADC Max Performance\nBoard: %s\n", CONFIG_BOARD);
	
	if(adc_initialise() == 0)
	{
		LOG_INF("ADC initialised\n");
	}

	while (1) {

		k_sleep(K_MSEC(1000));
	}
}


// FUNCTION FROM SAADC_EXAMPLED_COMMON.C
void gpiote_pin_toggle_task_setup(nrfx_gpiote_pin_t pin)
{
    nrfx_err_t status;
    (void)status;

    uint8_t gpiote_channel;
    status = nrfx_gpiote_channel_alloc(&gpiote_channel);
    //NRFX_ASSERT(status == NRFX_SUCCESS);

    static const nrfx_gpiote_output_config_t output_config =
    {
        .drive = NRF_GPIO_PIN_H0H1,
        .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
        .pull = NRF_GPIO_PIN_NOPULL,
    };

    const nrfx_gpiote_task_config_t task_config =
    {
        .task_ch = gpiote_channel,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = NRF_GPIOTE_INITIAL_VALUE_LOW,
    };

    status = nrfx_gpiote_output_configure(pin, &output_config, &task_config);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gpiote_out_task_enable(pin);
}

// FUNCTION TAKEN FROM SAADC_EXAMPLED_COMMON.C
void pin_on_event_toggle_setup(nrfx_gpiote_pin_t pin,
                               uint32_t eep)
{
    nrfx_err_t status;
    (void)status;

    uint8_t gppi_channel;

    gpiote_pin_toggle_task_setup(pin);

    status = nrfx_gppi_channel_alloc(&gppi_channel);
    NRFX_ASSERT(status == NRFX_SUCCESS);

    nrfx_gppi_channel_endpoints_setup(gppi_channel, eep, nrfx_gpiote_out_task_addr_get(pin));

    nrfx_gppi_channels_enable(NRFX_BIT(gppi_channel));
}
