#include <stdio.h>
#include <sys/time.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/i2c.h"
#include "driver/mcpwm.h"

#define ESP_INTR_FLAG_DEFAULT 0
#define V_REF 1100

// Steps if idf.py not found
// run ". export.sh" (Mac)
// then go to project and build


// Interrupt - https://www.youtube.com/watch?v=VkCvKtRsunU
// ADC - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
// UART - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html

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
  esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
  uint32_t reading, voltage;
  while(1)
  {
    vTaskDelay(200/portTICK_RATE_MS); // read delay
    reading = adc1_get_raw(ADC1_CHANNEL_6);
    voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
    printf("%d mV\n", voltage);
  }
}

void app_main(void)
{
    // GPIO
    // initialize Pin 26 for output and turn on
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_26);
    gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_26, 0);

    // initialize Pin 25 for output and turn on
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_25);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    xTaskCreate(blink1Sec, "blink LED", 1024, NULL, 1, NULL); // 1024=size, NULL=param, 1=lowest priority, NULL=handler?

    mcpwm_gpio_init(0, MCPWM0A, GPIO_NUM_13);

    // Interrupt
    // Setting up interrupt and GPIO4 for input
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_4);
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(GPIO_NUM_4, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(GPIO_NUM_4, button_isr_handler, NULL);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, &ISR);

    // ADC
    xTaskCreate(readPulse, "read pulse values", 2048, NULL, 5, NULL);

    // UART Example
    // const uart_port_t uart_num = UART_NUM_2; // Change UART number
    // uart_config_t uart_config = {
    //   .baud_rate = 115200,
    //   .data_bits = UART_DATA_8_BITS,
    //   .parity = UART_PARITY_DISABLE,
    //   .stop_bits = UART_STOP_BITS_1,
    //   .flow_ctrl = UART_HW_FLOWCTRL_CTS_RTS,
    //   .rx_flow_ctrl_thresh = 122,
    // };
    // // Configure UART parameters
    // ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config)); // ESP_ERROR_CHECK acts like "assert"
    // // Set UART pins(TX: IO4, RX: IO5, RTS: IO18, CTS: IO19)
    // ESP_ERROR_CHECK(uart_set_pin(UART_NUM_2, 4, 5, 18, 19));
    // // Setup UART buffered IO with event queue
    // const int uart_buffer_size = (1024 * 2);
    // QueueHandle_t uart_queue;
    // // Install UART driver using an event queue here
    // ESP_ERROR_CHECK(uart_driver_install(UART_NUM_2, uart_buffer_size, uart_buffer_size, 10, &uart_queue, 0));
    //
    // // UART Transmitting
    // // Write data to UART.
    // char* test_str = "This is a test string.\n";
    // uart_write_bytes(uart_num, (const char*)test_str, strlen(test_str));
    // // Write data to UART, end with a break signal.
    // uart_write_bytes_with_break(uart_num, "test break\n",strlen("test break\n"), 100);
    // // Wait for packet to be sent
    // const uart_port_t uart_num = UART_NUM_2;
    // ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 100)); // wait timeout is 100 RTOS ticks (TickType_t)
    //
    // // UART Receiving
    // // Read data from UART.
    // const uart_port_t uart_num = UART_NUM_2;
    // uint8_t data[128];
    // int length = 0;
    // ESP_ERROR_CHECK(uart_get_buffered_data_len(uart_num, (size_t*)&length));
    // length = uart_read_bytes(uart_num, data, length, 100);
}
