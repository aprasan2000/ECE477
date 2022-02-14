#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define V_REF 1100

#include <sys/time.h>

// Steps if idf.py not found
// run ". export.sh" (Mac)
// then go to project and build


// Interrupt - https://www.youtube.com/watch?v=VkCvKtRsunU
// ADC - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html


TaskHandle_t ISR = NULL;

void IRAM_ATTR button_isr_handler(void* arg)
{
  xTaskResumeFromISR(ISR);
}

void button_task(void* arg)
{
  int count = 0;
  while(1)
  {
    vTaskSuspend(NULL); // runs interrupt once
    count++;
    printf("Alert!\n");
    gpio_set_level(GPIO_NUM_26, count % 2);
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
    vTaskDelay(200/portTICK_RATE_MS); // read delay
    reading = adc1_get_raw(ADC1_CHANNEL_6);
    voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
	gettimeofday(&current_time, NULL);
    printf("%ld ms %d mVolts\n", current_time.tv_usec, voltage);
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

    // Interrupt
    // Setting up interrupt and GPIO4 for input
    gpio_pad_select_gpio(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_NUM_4, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_4, button_isr_handler, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, &ISR);

    // ADC
    xTaskCreate(readPulse, "read pulse values", 2048, NULL, 5, NULL);
}
