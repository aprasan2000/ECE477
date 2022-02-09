/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "driver/gpio.h"

#define GPIO_OUTPUT_IO_0 GPIO_NUM_26
#define GPIO_OUTPUT_IO_1 GPIO_NUM_25
#define GPIO_OUTPUT_PIN_SEL ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))
#define GPIO_INPUT_IO_0 GPIO_NUM_4
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

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
    gpio_set_level(GPIO_OUTPUT_IO_0, count % 2);
  }
}

void app_main(void)
{
    // initialize Pin 26 for output and turn on
    esp_rom_gpio_pad_select_gpio(GPIO_OUTPUT_IO_0);
    gpio_set_direction(GPIO_OUTPUT_IO_0, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_OUTPUT_IO_0, 0);

    // initialize Pin 25 for output and turn on
    esp_rom_gpio_pad_select_gpio(GPIO_OUTPUT_IO_1);
    gpio_set_direction(GPIO_OUTPUT_IO_1, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_OUTPUT_IO_1, 1);

    // Setting up interrupt and GPIO4 for input
    esp_rom_gpio_pad_select_gpio(GPIO_INPUT_IO_0);
    gpio_set_direction(GPIO_INPUT_IO_0, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_INPUT_IO_0, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_INPUT_IO_0, button_isr_handler, NULL);

    xTaskCreate(button_task, "button_task", 4096, NULL, 10, &ISR);
}
