#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/param.h>
#include <sys/time.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_attr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/lsm6dsl.h"
#include "sdkconfig.h"

// ADC
#define ESP_INTR_FLAG_DEFAULT 0
#define V_REF 1100
#define LOW_VOLTAGE 3500 // mV
#define ADC_CORRECTION 0.977

// I2C
#define I2C_MASTER_SCL_IO 22
#define I2C_MASTER_SDA_IO 21
#define I2C_MASTER_PORT I2C_NUM_0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_MASTER_FREQ_HZ I2C_APB_CLK_FREQ
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT  I2C_MASTER_READ
#define ACK_CHECK_EN true
#define ACK_CHECK_DIS false
#define ACK_VAL 0x00
#define NACK_VAL 0x1
#define SLAVE_ADDR LSM6DSL_ACC_GYRO_WHO_AM_I

// void readPulse(void* arg)
// {
//   esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//   esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
//   esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
//   if(err != ESP_OK)
//   {
//     printf("Pulse invert error\n");
//     return;
//   }
//   uint32_t reading, voltage;
//   int count = 9;
//   bool counted = false;
//   time_t startSec;
//   while(1)
//   {
//     // vTaskDelay(200/portTICK_RATE_MS); // read delay
//     time(&startSec);
//     while(time(NULL) < startSec+10)
//     {
//       reading = adc1_get_raw(ADC1_CHANNEL_4);
//       voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
//       // vTaskDelay(1000/TICK_1);
//       printf("Pulse Voltage: %d mV\n", voltage);
//       if(voltage > 550 && counted==false)
//       {
//         count++;
//         // vTaskDelay(pdMS_TO_TICKS(50));
//         // printf("count: %d\n", count);
//         counted = true;
//       }
//       else if(voltage < 550)
//       {
//         counted = false;
//       }
//       else
//       {
//         continue;
//       }
//     }
//     vTaskDelay(pdMS_TO_TICKS(50));
//     int bpm = count * 6;
//     count = 0;
//     printf("BPM: %d\n", bpm);
//   }
// }

// void readPulse(void* arg)
// {
//   esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//   esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
//   esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
//   if(err != ESP_OK)
//   {
//     printf("Pulse invert error\n");
//     return;
//   }
//
//   uint32_t reading, voltage;
//   int upperThreshold = 3300;
//   int lowerThreshold = 2310;
//   int pulseInterval = 0;
//   bool ignoreReading = false;
//   bool firstPulseDetected = false;
//   struct timeval time1, time2;
//   while(1)
//   {
//     reading = adc1_get_raw(ADC1_CHANNEL_4);
//     voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
//     printf("Pulse Voltage: %d mV\n", voltage);
//     if(voltage > upperThreshold && !ignoreReading)
//     {
//       if(!firstPulseDetected)
//       {
//         gettimeofday(&time1, NULL);
//         firstPulseDetected = true;
//       }
//       else
//       {
//         gettimeofday(&time2, NULL);
//         pulseInterval = (time2.tv_sec - time1.tv_sec) * 1000000 + time2.tv_usec - time1.tv_usec;
//         time1 = time2;
//       }
//       ignoreReading = true;
//     }
//     if(reading < lowerThreshold)
//     {
//       ignoreReading = false;
//     }
//     float bpm = (1.0/pulseInterval) * 60.0 * 1000;
//     printf("BPM: %f\n", bpm);
//     vTaskDelay(pdMS_TO_TICKS(100));
//   }
// }

void readPulse(void* arg)
{
  esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
  // esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
  // if(err != ESP_OK)
  // {
  //   printf("Pulse invert error\n");
  //   return;
  // }

  uint32_t reading, voltage;
  int threshold = 2000;
  int count = 0;
  bool counted = false;
  // struct timeval time1, time2;
  time_t start;
  while(1)
  {
    time(&start);
    while(time(NULL) < start+10)
    {
      reading = adc1_get_raw(ADC1_CHANNEL_4);
      voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
      printf("Pulse Voltage: %d mV\n", voltage);
      if(voltage > threshold && !counted)
      {
        gpio_set_level(GPIO_NUM_27, 1);
        count++;
        counted = true;
      }
      else
      {
        gpio_set_level(GPIO_NUM_27, 0);
        counted = false;
      }
      vTaskDelay(pdMS_TO_TICKS(20));
    }
    int bpm = count * 6;
    printf("BPM: %d\n", bpm);
    vTaskDelay(pdMS_TO_TICKS(50));
    counted = false;
    count = 0;
  }
}

void readBatt(void* arg)
{
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
    esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
    if(err != ESP_OK)
    {
      printf("Battery invert error\n");
      return;
    }
    uint32_t reading, voltage, real_voltage;
    while(1)
    {
        vTaskDelay(pdMS_TO_TICKS(50)); // read delay
        reading = adc1_get_raw(ADC1_CHANNEL_6);
        // printf("reading: %d\n", reading);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        real_voltage = (uint32_t)(voltage * 2 * ADC_CORRECTION);
        // printf("%d mV\n", real_voltage);
        if(real_voltage < LOW_VOLTAGE){
            printf("LOW Battery\n");
            for(int i = 0; i < 10; i++){
                gpio_set_level(GPIO_NUM_27, i%2);
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }
}

static int i2c_master_init()
{
  int err;
  int i2c_master_port = I2C_MASTER_PORT;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 10000;
  conf.clk_flags = 0;
  err = i2c_param_config(i2c_master_port, &conf);
  if (err == ESP_OK)
      printf("Parameters checked\n");
  else
  {
      printf("Parameters invalid\n");
      return 0;
  }

  err = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK)
      printf("Initialization Complete\n");
  else
  {
      printf("Initialization failed\n");
      return 0;
  }

  return 1;
}

static int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

static int i2c_scanner() {
  int32_t scanTimeout = 1000;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "Device found at address: 0x%X%s", i, "\n");
      return i;
    }
  }
  printf("No I2C devices found" "\n");
  return 0;
}

static esp_err_t write_data(uint8_t reg, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// static esp_err_t readReg(uint8_t i2c_reg, uint8_t* data_rd, size_t size)
// {
//     esp_err_t err;
//     if (size == 0) {
//         return ESP_OK;
//     }
//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ), ACK_CHECK_EN);
//     i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
//     i2c_master_stop(cmd);
//     err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     if(err != ESP_OK)
//     {
//       return err;
//     }
//     cmd = i2c_cmd_link_create();
//     i2c_master_start(cmd);
//     i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | 1, ACK_CHECK_EN);
//     i2c_master_read(cmd, data_rd, size, NACK_VAL);
//
//     i2c_master_stop(cmd);
//     err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
//     i2c_cmd_link_delete(cmd);
//     return err;
// }

static void freeFallHandler()
{
  while(1)
  {
    // printf("%d\n", gpio_get_level(GPIO_NUM_18));
    if(gpio_get_level(GPIO_NUM_18) == 1)
    {
      printf("I've fallen and can't get up\n");
      gpio_set_level(GPIO_NUM_27, 1);
      vTaskDelay(pdMS_TO_TICKS(4000));
      gpio_set_level(GPIO_NUM_27, 0);
    }
    // uint8_t tap;
    // esp_err_t err = readReg(LSM6DSL_ACC_GYRO_TAP_SRC, &tap, 8);
    // if(err != ESP_OK)
    // {
    //   printf("Tap read fail\n");
    //   return;
    // }
    // printf("%x\n", tap);
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void app_main(void)
{
  esp_rom_gpio_pad_select_gpio(GPIO_NUM_23); // accel pull down
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLDOWN_ONLY);
  gpio_set_level(GPIO_NUM_23, 0);

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_18);
  gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLDOWN_ONLY);

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_27);
  gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_27, 0);

  int master_cal = i2c_master_init();
  if(!master_cal) {return;}
  int dev_add = i2c_scanner();
  if(dev_add != SLAVE_ADDR)
  {
    printf("LSM6DSL not found\n");
    return;
  }

  // Tap instructions -- Does not work
  // uint8_t instructions[] = {LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60,
  //                           LSM6DSL_ACC_GYRO_TAP_CFG1, 0x8e,
  //                           LSM6DSL_ACC_GYRO_TAP_THS_6D, 0x89,
  //                           LSM6DSL_ACC_GYRO_INT_DUR2, 0x06,
  //                           LSM6DSL_ACC_GYRO_WAKE_UP_THS, 0x00,
  //                           LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 0x00, // not in guide
  //                           LSM6DSL_ACC_GYRO_MD1_CFG, 0x40};

  // Free Fall instructions -- WORKS
  // Threshold of 312 mg
  // FF Duration of ~15 ms
  uint8_t instructions[] = {LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60,
                            LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80,
                            LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 0x00,
                            LSM6DSL_ACC_GYRO_FREE_FALL, 0x33,
                            LSM6DSL_ACC_GYRO_MD1_CFG, 0x10};

  esp_err_t err;
  for(int i=0; i<sizeof(instructions)/sizeof(instructions[0]); i+=2)
  {
    err = write_data(instructions[i], instructions[i+1]);
    if(err != ESP_OK)
    {
      printf("Write Error (index = %d)\n", i);
      return;
    }
  }
  printf("Write Succeded\n");

  xTaskCreate(readBatt, "battery voltage", 2048, NULL, 10, NULL);
  xTaskCreate(readPulse, "read pulse values", 2048, NULL, 6, NULL);
  xTaskCreate(freeFallHandler,"freeFallHandler_task", 4096, NULL, 5, NULL);
  return;
}
