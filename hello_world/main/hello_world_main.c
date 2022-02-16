#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define V_REF 1100

#include <sys/time.h>



// Steps if idf.py not found
// run ". export.sh" (Mac)
// then go to project and build


// Interrupt - https://www.youtube.com/watch?v=VkCvKtRsunU
// ADC - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html


#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (13) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_DUTY_ZERO		  (0) // zero duty cycle
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 5 kHz

static void example_ledc_init(void)
{
    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

TaskHandle_t ISR = NULL;

void IRAM_ATTR button_isr_handler(void* arg)
{
  xTaskResumeFromISR(ISR);
}

void button_task(void* arg)
{
  int duty;
  int count = 0;
  while(1)
  {
    vTaskSuspend(NULL); // runs interrupt once
    count++;
    printf("Alert!\n");
    gpio_set_level(GPIO_NUM_26, count % 2);
    
    example_ledc_init();
    
    if(count %2)
    {
        duty = LEDC_DUTY;
    }
    else
    {
        duty = LEDC_DUTY_ZERO;
    }

    // Set duty to 50% or 0
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
    // Update duty to apply the new value
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
  }
}

void blink1Sec(void* arg)
{
  bool status = false;
  while(1)
  {
    vTaskDelay(1000/portTICK_RATE_MS);
    gpio_set_level(GPIO_NUM_25, status);
    status = !status;
  }
}

void readPulse(void* arg)
{
  //time
  struct timeval current_time;  
  esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
  uint32_t reading, voltage;
  while(1)
  {
    vTaskDelay(100/portTICK_RATE_MS); // read delay
    reading = adc1_get_raw(ADC1_CHANNEL_6);
    voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
	gettimeofday(&current_time, NULL);
    printf("%lu.%lums %dmV\n",current_time.tv_sec, (current_time.tv_usec /10000), voltage);
  }
}

void app_main(void)
{
    // GPIO
    // initialize Pin 26 for output and turn on
    gpio_pad_select_gpio(GPIO_NUM_26);
    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_26, 0);

    // initialize Pin 25 for output and turn on
    gpio_pad_select_gpio(GPIO_NUM_25);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    xTaskCreate(blink1Sec, "blink LED", 1024, NULL, 1, NULL); // 1024=size, NULL=param, 1=lowest priority, NULL=handler?

    // Interrupt (contains buzzer via PWM)
    // Setting up interrupt and GPIO4 for input
	
    gpio_pad_select_gpio(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_NUM_4, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_4, button_isr_handler, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, &ISR);

    // ADC
    xTaskCreate(readPulse, "read pulse values", 2048, NULL, 10, NULL);

    // Set the LEDC peripheral configuration
}
