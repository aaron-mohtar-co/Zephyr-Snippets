/* 
 * ADC Example for Nordic Semiconductor SoCs
 * Uses vendor specific nrf_saadc_* API instead of common Zephyr API.
 * 
 * Reads an analog voltage from two ADC inputs (P0.30 & P0.31) and prints 
 * result on console.
 * 
 * Un-comment or comment out BLOCKING and NONBLOCKING defines to switch 
 * between blocking and non-blocking behaviour.
 * 
 * Nordic has SAADC examples located in your nRF Connect SDK folder at
 * <nRF Connect SDK root>\modules\hal\nordic\samples\src\nrfx_saadc
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <nrfx_saadc.h>

// Select between blocking or non-blocking behaviour
//#define BLOCKING
#define NONBLOCKING

#define ADC_NUM_OF_CHANNELS		2

#define ADC_INPUT_0_CHANNEL_INDEX	0
#define ADC_INPUT_1_CHANNEL_INDEX	1

#define ADC_INPUT_0_PIN		NRF_SAADC_INPUT_AIN6	// AIN6 = P0.30
#define ADC_INPUT_1_PIN		NRF_SAADC_INPUT_AIN7	// AIN7 = P0.31

#define ADC_BUFF_SIZE 64

static nrf_saadc_value_t adc_rawDataBuffer[ADC_NUM_OF_CHANNELS];

// Function prototypes 
uint8_t adc_initialise();

static void adc_eventHandler(nrfx_saadc_evt_t const * p_event)
{
	switch (p_event->type) {
		case NRFX_SAADC_EVT_DONE:
			printk("RFX_SAADC_EVT_DONE\n");
			printk("ADC result %d %d\n", p_event->data.done.p_buffer[0], p_event->data.done.p_buffer[1]);
			break;
			
		case NRFX_SAADC_EVT_LIMIT:
			printk("NRFX_SAADC_EVT_LIMIT\n");
			break;

		case NRFX_SAADC_EVT_CALIBRATEDONE:
			printk("NRFX_SAADC_EVT_CALIBRATEDONE\n");
			break;

		case NRFX_SAADC_EVT_BUF_REQ:
			printk("NRFX_SAADC_EVT_BUF_REQ\n");
			break;
		
		case NRFX_SAADC_EVT_READY:
			printk("NRFX_SAADC_EVT_READY\n");
			break;

		case NRFX_SAADC_EVT_FINISHED:
			printk("NRFX_SAADC_EVT_FINISHED\n");
			break;

		default:
			printk("Unknown ADC Event %d\n", p_event->type);
			break;
	}
}

uint8_t adc_initialise()
{
	nrfx_err_t err;

	// Initialise SAADC driver
	err = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
	if(err != NRFX_SUCCESS)
		printk("Error %d in nrfx_saadc_init()\n", err);

	IRQ_DIRECT_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0);

	nrfx_saadc_channel_t adc_input1Config = 
	{
		.channel_config ={
			.reference = NRF_SAADC_REFERENCE_VDD4,		// Reference = VDD / 4
			.acq_time = NRF_SAADC_ACQTIME_40US,			// Acquisition time 40uS
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
			.acq_time = NRF_SAADC_ACQTIME_40US,			// Acquisition time 40uS
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
		printk("Error %d in nrfx_saadc_channels_config()\n", err);

#ifdef BLOCKING 
	// For blocking, set event_handler to NULL
	err = nrfx_saadc_simple_mode_set(
		BIT(ADC_INPUT_0_CHANNEL_INDEX) | BIT(ADC_INPUT_1_CHANNEL_INDEX),
		NRF_SAADC_RESOLUTION_12BIT,
		NRF_SAADC_OVERSAMPLE_DISABLED,
		NULL);	// Blocking
	if(err != NRFX_SUCCESS)
		printk("Error %d in nrfx_saadc_simple_mode_set()\n", err);
#endif

#ifdef NONBLOCKING
	// For non-blocking provide adc event handler function 
	err = nrfx_saadc_simple_mode_set(
		BIT(ADC_INPUT_0_CHANNEL_INDEX)|BIT(ADC_INPUT_1_CHANNEL_INDEX),
		NRF_SAADC_RESOLUTION_12BIT,
		NRF_SAADC_OVERSAMPLE_DISABLED,
		adc_eventHandler);
	if(err != NRFX_SUCCESS)
		printk("Error %d in nrfx_saadc_simple_mode_set()\n", err);
#endif

	// Set-up buffer
	err = nrfx_saadc_buffer_set(&adc_rawDataBuffer[0], ADC_NUM_OF_CHANNELS);
	if(err != NRFX_SUCCESS)
		printk("Error %d in nrfx_saadc_buffer_set()\n", err);

	return 0;
}

void main(void)
{
	int ret;

	printk("ADC Single Shot Example\nBoard: %s\n", CONFIG_BOARD);
	
	if(adc_initialise() == 0)
	{
		printk("ADC initialised\n");
	}

	while (1) {

#ifdef NONBLOCKING
		// This triggers a reading (non-blocking)
		ret = nrfx_saadc_mode_trigger();
		if (ret != NRFX_SUCCESS) {
			printk("Error in nrfx_saadc_mode_trigger()\n");
		}
#endif

#ifdef BLOCKING	
		printk("Triggering ADC\n");
		nrfx_saadc_mode_trigger();
		printk("ADC Conversion Complete\n");
		printk("ADC0 = %d\n", adc_rawDataBuffer[0]);
		printk("ADC1 = %d\n", adc_rawDataBuffer[1]);
#endif

		k_sleep(K_MSEC(1000));
	}
}

