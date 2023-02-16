
#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>


#include <nrfx_saadc.h>


static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led));
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led));
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led));

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});

#define NUM_STEPS	50U
#define SLEEP_MSEC	25U

#define ADC_NUM_OF_CHANNELS		2

#define ADC_INPUT_0_CHANNEL_INDEX	0
#define ADC_INPUT_1_CHANNEL_INDEX	1

#define ADC_INPUT_0_PIN		NRF_SAADC_INPUT_AIN6	// AIN6 = P0.30
#define ADC_INPUT_1_PIN		NRF_SAADC_INPUT_AIN7	// AIN7 = P0.31

K_SEM_DEFINE(pb_pushed, 0, 1);

bool led_enabled;

static nrf_saadc_value_t adc_rawDataBuffer[ADC_NUM_OF_CHANNELS];

#define ADC_BUFF_SIZE 64
static uint8_t adc_bufferCounter = 0;
static uint16_t adc_buffer[ADC_BUFF_SIZE];

static struct gpio_callback button_cb_data;

static struct k_work led_indicator_work;

/* Function prototypes */
static void led_indicator_work_function(struct k_work *work);
void fade_pwm_led(const struct pwm_dt_spec *spec);
uint8_t adc_initialise();
uint16_t adc_nrfx_triggerReading();

/* Function to print out error line for ADC errors */
void printNrfxErrorLine(uint32_t errCode, uint16_t line)
{
	if(errCode != NRFX_SUCCESS)
	{
		uint32_t ret = errCode - NRFX_SUCCESS;
		printk("Error in %s on line %d: %d \n", __FILE__, line,ret);
	}
}

void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&pb_pushed);
}

static void adc_eventHandler(nrfx_saadc_evt_t const * p_event)
{

	NRFX_IRQ_PENDING_CLEAR(SAADC_IRQn);
	//NRFX_IRQ_DISABLE(SAADC_IRQn);

	adc_buffer[adc_bufferCounter++] = adc_rawDataBuffer[0];

	if(adc_bufferCounter >= ADC_BUFF_SIZE)
	{
		adc_bufferCounter = 0;
		
		nrfx_saadc_abort();

		printk("ADC ISR, buffer size: %d\n",p_event->data.done.size);
		printk("ADC result %d %d\n", adc_rawDataBuffer[0],adc_rawDataBuffer[1]);
		printk("ADC result %d %d\n", p_event->data.done.p_buffer[0],p_event->data.done.p_buffer[1]);

		printk("ADC results: ");
		for(int i = 0;i<ADC_BUFF_SIZE;i++)
		{
			printk("%d - ", adc_buffer[i]);
		}
		printk("\n");
	}

}

void main(void)
{
	int ret;
	led_enabled = true;

	printk("PWM-based RGB LED fade\n");

	if (!device_is_ready(pwm_led0.dev)) {
		printk("Error: PWM device %s is not ready\n", pwm_led0.dev->name);
		return;
	}

	if (!device_is_ready(button.port)) {
		printk("Error: button device %s is not ready\n", button.port->name);
		return;
	}

	ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
	if (ret != 0) {
		printk("Error %d: failed to configure %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}

	ret = gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	if (ret != 0) {
		printk("Error %d: failed to configure interrupt on %s pin %d\n", ret, button.port->name, button.pin);
		return;
	}

	gpio_init_callback(&button_cb_data, button_callback, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	k_work_init(&led_indicator_work, led_indicator_work_function);
	k_work_submit(&led_indicator_work);

	
	if(adc_initialise() == 0)
	{
		printk("ADC initialised\n");
	}

	/* THIS TRIGGERS AND CONTINEOUS READING */
	printk("ADC triggering: %d\n", adc_nrfx_triggerReading());

	while (1) {

		if (k_sem_take(&pb_pushed, K_MSEC(5000)) != 0) {
			//printk("Error\n");
			//printk("ADC triggering: %d\n", adc_nrfx_triggerReading());
		} else {
			printk("Push button pressed\n");
			do {
				k_sleep(K_MSEC(100));
			} while(gpio_pin_get_dt(&button));
			printk("Push button released\n");
			led_enabled = !led_enabled;
			if (led_enabled) printk("LED is enabled\n");
			else 			 printk("LED is disabled\n");
		}

	}
}

static void led_indicator_work_function(struct k_work *work)
{
	// Turn off LEDs
	pwm_set_pulse_dt(&pwm_led0, 0);
	pwm_set_pulse_dt(&pwm_led1, 0);
	pwm_set_pulse_dt(&pwm_led2, 0);

	while(1) {
			fade_pwm_led(&pwm_led0);
			fade_pwm_led(&pwm_led1);
			fade_pwm_led(&pwm_led2);
	}
}

void fade_pwm_led(const struct pwm_dt_spec *spec)
{
	uint32_t pulse_width = 0U;
	uint32_t step = pwm_led0.period / NUM_STEPS;
	uint8_t dir = 1U;
	int ret;

	while (1) {
		ret = pwm_set_pulse_dt(spec, pulse_width);
		if (ret) {
			printk("Error %d: failed to set pulse width\n", ret);
			return;
		}

		if (dir) {
			pulse_width += step;
			if (pulse_width >= pwm_led0.period) {
				pulse_width = pwm_led0.period - step;
				dir = 0U;
			}
		} else {
			if (pulse_width >= step) {
				pulse_width -= step;
			} else {
				pulse_width = step;
				dir = 1U;
				break;
			}
		}

		// break loop if button pressed
		//if (!led_enabled) break;	

		k_sleep(K_MSEC(SLEEP_MSEC));
	}

	// Disable LED
	pwm_set_pulse_dt(spec, 0);
} 

uint8_t adc_initialise()
{
	nrfx_err_t errCode;
	uint8_t ret;

	nrfx_saadc_adv_config_t saadc_adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
    saadc_adv_config.internal_timer_cc = 0;
    saadc_adv_config.start_on_end = false;


	errCode = nrfx_saadc_init(3);
	if(errCode !=NRFX_SUCCESS)
	printNrfxErrorLine(errCode,__LINE__);

	nrfx_saadc_channel_t adc_input1Config = 
    {
        .channel_config ={
			.reference = NRF_SAADC_REFERENCE_VDD4,
			.acq_time = NRF_SAADC_ACQTIME_40US,
			.mode = NRF_SAADC_MODE_SINGLE_ENDED,
			.burst = NRF_SAADC_BURST_DISABLED // SAADC_CH_CONFIG_BURST_Disabled,
		
        },
        .channel_index = ADC_INPUT_0_CHANNEL_INDEX,
        .pin_p = ADC_INPUT_0_PIN
    };

    nrfx_saadc_channel_t adc_input2Config = 
    {
        .channel_config ={
            .reference = NRF_SAADC_REFERENCE_VDD4,
            .acq_time = NRF_SAADC_ACQTIME_40US,
            .mode = NRF_SAADC_MODE_SINGLE_ENDED,
            .burst = NRF_SAADC_BURST_DISABLED,
        },
        .channel_index = ADC_INPUT_1_CHANNEL_INDEX,
        .pin_p = ADC_INPUT_1_PIN,
    };

    nrfx_saadc_channel_t adc_configArray[ADC_NUM_OF_CHANNELS] = {
        adc_input1Config, 
        adc_input2Config
    };

	errCode = nrfx_saadc_channels_config(adc_configArray,ADC_NUM_OF_CHANNELS);
	printNrfxErrorLine(errCode,__LINE__);

	errCode = nrfx_saadc_simple_mode_set(
		BIT(ADC_INPUT_0_CHANNEL_INDEX)|BIT(ADC_INPUT_1_CHANNEL_INDEX),
        NRF_SAADC_RESOLUTION_12BIT,
        NRF_SAADC_OVERSAMPLE_DISABLED,
        adc_eventHandler);
	printNrfxErrorLine(errCode,__LINE__);

	
/*
	nrfx_saadc_adv_config_t adc_advConfig =
    {
        .oversampling = NRF_SAADC_OVERSAMPLE_DISABLED,
        .burst = NRF_SAADC_BURST_DISABLED,
        .start_on_end   = false
    };

	errCode = nrfx_saadc_advanced_mode_set(
        BIT(ADC_INPUT_0_CHANNEL_INDEX)|BIT(ADC_INPUT_1_CHANNEL_INDEX),
        NRF_SAADC_RESOLUTION_12BIT,
        &adc_advConfig,
        adc_eventHandler
    );
	printNrfxErrorLine(errCode,__LINE__);
	*/

	errCode = nrfx_saadc_buffer_set(&adc_rawDataBuffer[0], ADC_NUM_OF_CHANNELS);
	if(errCode !=NRFX_SUCCESS)
	printNrfxErrorLine(errCode,__LINE__);


	IRQ_CONNECT(SAADC_IRQn,3,adc_eventHandler,0,0);
	irq_enable(SAADC_IRQn);
	

	return 0;
}

uint16_t adc_nrfx_triggerReading()
{
	//printk("Got to line %d\n", __LINE__);
	//printk("ADC trigger return: %d\n",nrfx_saadc_mode_trigger());
	NRFX_IRQ_ENABLE(SAADC_IRQn);
    return (nrfx_saadc_mode_trigger()-NRFX_SUCCESS);
}
