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
#define LOW_VOLTAGE 3500 // mV
#define ADC_CORRECTION 0.977
#define L_TO_H_COEFF 1.515

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (9)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

// Steps if idf.py not found
// run ". export.sh" (Mac)
// then go to project and build


// Interrupt - https://www.youtube.com/watch?v=VkCvKtRsunU
// ADC - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc.html
// UART - https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html

// TaskHandle_t ISR = NULL;

// void IRAM_ATTR button_isr_handler(void* arg)
// {
//   xTaskResumeFromISR(ISR);
// }

// void button_task(void* arg)
// {
//   int count = 0;
//   while(1)
//   {
//     vTaskSuspend(NULL); // runs interrupt once
//     count++;
//     printf("Alert!\n");
//     gpio_set_level(GPIO_NUM_26, count % 2);
//   }
// }

void blink1Sec(void* arg)
{
  bool status = false;
  while(1)
  {
    vTaskDelay(1000/portTICK_RATE_MS);
    gpio_set_level(GPIO_NUM_27, status);
    status = !status;
  }
}

// void readPulse(void* arg)
// {
//   esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//   esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
//   uint32_t reading, voltage;
//   while(1)
//   {
//     vTaskDelay(200/portTICK_RATE_MS); // read delay
//     reading = adc1_get_raw(ADC1_CHANNEL_6);
//     voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
//     printf("%d mV\n", voltage);
//   }
// }

// void beepInit(void* arg)
// {
//     cprintf("initializing pwm");
//     ledc_timer_config_t ledc_timer = {
//         .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
//         .freq_hz = 5000,                      // frequency of PWM signal
//         .speed_mode = LEDC_LS_MODE,           // timer mode
//         .timer_num = LEDC_LS_TIMER,            // timer index
//         .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
//     };
//     // Set configuration of timer0 for high speed channels
//     ledc_timer_config(&ledc_timer);
    
//     ledc_channel_config_t ledc_channel[LEDC_TEST_CH_NUM] = {
//         .channel    = LEDC_HS_CH0_CHANNEL,
//         .duty       = 0,
//         .gpio_num   = LEDC_HS_CH0_GPIO,
//         .speed_mode = LEDC_HS_MODE,
//         .hpoint     = 0,
//         .timer_sel  = LEDC_HS_TIMER,
//         .flags.output_invert = 0
//     };
    
//     int ch;
//     for (ch = 0; ch < LEDC_TEST_CH_NUM; ch++) {
//         ledc_channel_config(&ledc_channel[ch]);
//     }    
    
    
//     vTaskDelete(NULL);
// }

// void beep(void* arg)
// {
//     printf("LOW Battery\n");
//     ledc_timer_config_t ledc_timer = {
//         .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
//         .freq_hz = 5000,                      // frequency of PWM signal
//         .speed_mode = LEDC_HS_MODE,           // timer mode
//         .timer_num = LEDC_HS_TIMER,            // timer index
//         .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
//     };
//     // Set configuration of timer0 for high speed channels
//     ledc_timer_config(&ledc_timer);

//     vTaskDelete(NULL);
// }

// void readBatt(void* arg)
// {
//     // printf("reading battery...\n");
//     esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//     esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
//     esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
//     uint32_t reading, voltage, real_voltage, x;
//     x = 0;
//     while(1)
//     {
//         vTaskDelay(200/portTICK_RATE_MS); // read delay
//         reading = adc1_get_raw(ADC1_CHANNEL_6);
//         // printf("reading: %d\n", reading);
//         voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
//         real_voltage = (uint32_t)(voltage * 2 * ADC_CORRECTION);
//         // printf("%d mV\n", real_voltage);
//         if(real_voltage < LOW_VOLTAGE){
//             xTaskCreate(beep, "beep", 2048, NULL, 5, NULL);
//             for(int i = 0; i < 10; i++){
//                 x = !x;
//                 gpio_set_level(GPIO_NUM_27, x);
//                 vTaskDelay(100/portTICK_RATE_MS);
//             }
//         }
//     }
// }

void gasSensorPrelim(void* arg)
{
    printf("gas sensor code running...\n");
    // Init GPIO25
    gpio_pad_select_gpio(GPIO_NUM_25);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_25, 0);
    // Init ADC
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
    esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
    uint32_t reading, voltage, real_voltage, i;
    // Heat up gas sensor
    gpio_set_level(GPIO_NUM_25, 1);
    for(int i = 0; i < 20; i++){
        vTaskDelay(5000/portTICK_RATE_MS);
        reading = adc1_get_raw(ADC1_CHANNEL_5);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        printf("...raw voltage: %d mV\n", voltage);
        printf("...corrected v: %f mV\n", voltage * L_TO_H_COEFF);
    }
    while(1){
        // Turn on MOSFET (HIGH VOLTAGE)
        gpio_set_level(GPIO_NUM_25, 1);
        printf("...MOSFET on\n");
        // Wait 60 seconds
        vTaskDelay(6000/portTICK_RATE_MS);

        // Turn off MOSFET (LOW VOLTAGE )
        gpio_set_level(GPIO_NUM_25, 0);
        printf("...MOSFET off\n");
        // Wait 90 seconds
        vTaskDelay(9000/portTICK_RATE_MS);
        // Read ADC
        reading = adc1_get_raw(ADC1_CHANNEL_5);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        printf("...raw voltage: %d mV\n", voltage);
        printf("...corrected v: %f mV\n", voltage * L_TO_H_COEFF);
    }

    vTaskDelete(NULL);
}

void readButton(void* arg)
{
    gpio_pad_select_gpio(GPIO_NUM_5);
    gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT);
    while(1){
        printf("...GPIO5: %d\n", gpio_get_level(GPIO_NUM_5));
        vTaskDelay(250/portTICK_RATE_MS);
    }
}

void app_main(void)
{
    printf("running...\n");
    // GPIO 27 for output and turn on
    gpio_pad_select_gpio(GPIO_NUM_27);
    gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_27, 1);

    // xTaskCreate(beepInit, "init pwm for buzzer", 2048, NULL, 5, NULL);
    xTaskCreate(gasSensorPrelim, "prelim code for gas sensor", 2048, NULL, 5, NULL);
    xTaskCreate(readButton, "prelim code for button", 2048, NULL, 5, NULL);
    xTaskCreate(blink1Sec, "prelim code for blinking", 2048, NULL, 5, NULL);
    
    // int x = 0;
    // while(1){
    //     x = !x;
    //     gpio_set_level(GPIO_NUM_27, x);
    //     vTaskDelay(1000/portTICK_RATE_MS);
    // }

    // esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    // esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
    // uint32_t reading, voltage;
    // while(1)
    // {
    //     vTaskDelay(200/portTICK_RATE_MS); // read delay
    //     reading = adc1_get_raw(ADC1_CHANNEL_6);
    //     voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
    //     printf("%d mV\n", voltage);
    // }

    // // initialize Pin 25 for output and turn on
    // esp_rom_gpio_pad_select_gpio(GPIO_NUM_25);
    // gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    // xTaskCreate(blink1Sec, "blink LED", 1024, NULL, 1, NULL); // 1024=size, NULL=param, 1=lowest priority, NULL=handler?

    // mcpwm_gpio_init(0, MCPWM0A, GPIO_NUM_13);

    // // Interrupt
    // // Setting up interrupt and GPIO4 for input
    // esp_rom_gpio_pad_select_gpio(GPIO_NUM_4);
    // gpio_set_direction(GPIO_NUM_4, GPIO_MODE_INPUT);
    // gpio_set_pull_mode(GPIO_NUM_4, GPIO_PULLUP_ONLY);
    // gpio_set_intr_type(GPIO_NUM_4, GPIO_INTR_NEGEDGE);
    // gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    // gpio_isr_handler_add(GPIO_NUM_4, button_isr_handler, NULL);
    // xTaskCreate(button_task, "button_task", 4096, NULL, 10, &ISR);

    // // ADC
    // xTaskCreate(readPulse, "read pulse values", 2048, NULL, 5, NULL);
    // xTaskCreate(readBatt, "battery voltage", 2048, NULL, 5, NULL);

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
