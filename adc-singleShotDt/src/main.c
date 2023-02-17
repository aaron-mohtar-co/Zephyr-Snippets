
#include <zephyr/zephyr.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

static const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(red_pwm_led));
static const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(green_pwm_led));
static const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(blue_pwm_led));

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(DT_ALIAS(sw0), gpios, {0});

static const struct device *adcDev;

#define NUM_STEPS	50U
#define SLEEP_MSEC	25U

#define ADC_NUM_OF_CHANNELS		2

#define ADC_INPUT_0_CHANNEL_INDEX	0
#define ADC_INPUT_1_CHANNEL_INDEX	1

#define ADC_INPUT_0_PIN SAADC_CH_PSELP_PSELP_AnalogInput6	// AIN6 = P0.30
#define ADC_INPUT_1_PIN	SAADC_CH_PSELP_PSELP_AnalogInput7	// AIN7 = P0.31

#define ADC_INTERVAL_US    500000  // 500ms

#define ADC_BUFF_SIZE 64
//static uint8_t adc_bufferCounter = 0;
//static uint16_t adc_buffer[ADC_BUFF_SIZE];

#define ADC_RESOLUTION  12
#define ADC_RAW_BUFFER_SIZE 16

uint16_t adc_rawDataBuffer[ADC_RAW_BUFFER_SIZE];
uint16_t adc_userData;
uint8_t adc_stopReadingFlag = false;

K_SEM_DEFINE(pb_pushed, 0, 1);

bool led_enabled;

static struct gpio_callback button_cb_data;

static struct k_work led_indicator_work;

/* Function prototypes */
static void led_indicator_work_function(struct k_work *work);
void fade_pwm_led(const struct pwm_dt_spec *spec);
int8_t adc_initialise();
int8_t adc_startReading();

void button_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	k_sem_give(&pb_pushed);
}

uint8_t adc_callback(const struct device *dev, const struct adc_sequence *sequence,uint16_t sampling_index)
{
    int16_t adc_input0Result = adc_rawDataBuffer[0];
    int16_t adc_input1Result = adc_rawDataBuffer[1];

    printk("ADC:%d %d\n",adc_input0Result, adc_input1Result);
   // if(adc_stopReadingFlag == true)
        return ADC_ACTION_CONTINUE;
   // else
   //     return ADC_ACTION_REPEAT;
    
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

	adc_initialise();

	while (1) {

		if (k_sem_take(&pb_pushed, K_MSEC(2000)) != 0) {
			//printk("Error\n");
			adc_startReading();
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

int8_t adc_initialise()
{

	adcDev = DEVICE_DT_GET(DT_ALIAS(adc));
    if(adcDev == NULL) 
	{
		return -1;
	} 

	struct adc_channel_cfg adc_input0Configuration =
	{
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_VDD_1_4,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = ADC_INPUT_0_CHANNEL_INDEX,
		.input_positive = ADC_INPUT_0_PIN
	};

	struct adc_channel_cfg adc_input1Configuration =
	{
		.gain = ADC_GAIN_1_4,
		.reference = ADC_REF_VDD_1_4,
		.acquisition_time = ADC_ACQ_TIME(ADC_ACQ_TIME_MICROSECONDS, 40),
		.channel_id = ADC_INPUT_1_CHANNEL_INDEX,
		.input_positive = ADC_INPUT_1_PIN
	};
   
    adc_channel_setup(adcDev,&adc_input0Configuration);
    adc_channel_setup(adcDev,&adc_input1Configuration);

}

int8_t adc_startReading()
{    
	struct adc_sequence_options adc_options = 
	{
		.interval_us = ADC_INTERVAL_US,
		.callback = adc_callback,
		.user_data = &adc_userData,
		.extra_samplings = 0,
	};

	struct adc_sequence adc_sequence = 
	{
		.channels = BIT(ADC_INPUT_0_CHANNEL_INDEX)|BIT(ADC_INPUT_1_CHANNEL_INDEX),
		.buffer = adc_rawDataBuffer,
		.buffer_size = ADC_RAW_BUFFER_SIZE,
		.options = &adc_options,
		.oversampling = 0,
		.resolution = ADC_RESOLUTION,
		.calibrate = 0,
	};

    //printk("ADC read sequence started\n");
    adc_stopReadingFlag = false;
    adc_read(adcDev,&adc_sequence);
    return 0;
}