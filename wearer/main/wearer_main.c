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
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "driver/ledc.h"

// ADC
#define ESP_INTR_FLAG_DEFAULT 0
#define V_REF 1100
#define LOW_VOLTAGE 3500 // mV
#define ADC_CORRECTION 0.977
#define WINDOW_SIZE 4
#define G_CUTOFF 2500
#define L_TO_H_COEFF 1.515

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

// SPI - LoRa
#define CONFIG_CS_GPIO     15//4
#define CONFIG_RST_GPIO    16//3
#define CONFIG_MISO_GPIO   12//19
#define CONFIG_MOSI_GPIO   13//18
#define CONFIG_SCK_GPIO    14//5
#define LCD1602_GPIO_SDA	21
#define LCD1602_GPIO_SCL	22

#ifdef CONFIG_IDF_TARGET_ESP32
#define SPI_HOST_ID HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define SPI_HOST_ID SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define SPI_HOST_ID SPI2_HOST
#endif


#define I2C_MASTER_NUM                 I2C_NUM_1
#define LCD1602_I2C_ADDR (0x27)
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_DIO_MAPPING_2              0x41
#define REG_VERSION                    0x42
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06
#define PA_BOOST                       0x80
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40
#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1
#define TIMEOUT_RESET                  100
#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#define LCD_DELAY_MS 5

// Buzzer
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (26) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_DUTY_ZERO		  (0) // zero duty cycle
#define LEDC_FREQUENCY          (1000) // Frequency in Hertz. Set frequency at 5 kHz


static spi_device_handle_t __spi;
static int __implicit;
static long __frequency;

char hazard = 's';
int sensing = 1;

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

void readPulse(void *arg)
{
   esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
   esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
   esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
   uint32_t reading, voltage;
   time_t start;
   int bigWindow = 10 * WINDOW_SIZE;
   int arr[bigWindow], bpmArr[WINDOW_SIZE];
   int count = 0;
   int sum = 0;
   float avg = 0;
   int pulse = 0;
   int prevPulse = 0;
   int bpmCount = 0;
   int bpmSum = 0;
   float bpmAvg = 0;
   float prevBPMAvg = 0;

   if(err != ESP_OK)
   {
      printf("Pulse invert error\n");
      return;
   }
   while(1)
   {
      time(&start);
      while(time(NULL) < start + (60 / WINDOW_SIZE))
      {
       reading = adc1_get_raw(ADC1_CHANNEL_4);
       voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
       // printf("Voltage: %d mV\n", voltage);
       arr[count % bigWindow] = voltage;

       // calculate average every big window size
       if(count % bigWindow == 0){
          sum = 0;
          for(int i = 0; i < bigWindow; i++)
          {
             sum += arr[i];
          }
          avg = (float) sum / bigWindow;
       }
       // printf("Avg: %f mV\n", avg);

       // check if pulse
       if(voltage > (avg + 400) && !prevPulse){
         if(!prevPulse){
           printf("Pulse\n");
           gpio_set_level(GPIO_NUM_27, 1);
           // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
           // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
           pulse++;
           prevPulse = 1;
         }
       }
       else
       {
         prevPulse = 0;
       }

       count++;
       vTaskDelay(pdMS_TO_TICKS(50));
       gpio_set_level(GPIO_NUM_27, 0);
       // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ZERO));
       // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
      }
      int bpm = pulse * 2;
      bpmArr[bpmCount % WINDOW_SIZE] = bpm;
      bpmSum = 0;
      for(int i = 0; i < WINDOW_SIZE; i++)
      {
        bpmSum += bpmArr[i];
      }
      bpmAvg = (float) bpmSum / WINDOW_SIZE; // takes 1 min to fully populate and read correctly
      printf("BPM: %d\n", bpm);
      printf("BPM Avg: %f\n", bpmAvg);
      // if(bpmAvg > 0 && prevBPMAvg > 0 && bpmAvg > prevBPMAvg * 2)
      // {
      //   hazard = 'H';
      //   sensing = 0;
      // }
      // if(bpmCount % WINDOW_SIZE == 0)
      // {
      //     prevBPMAvg = bpmAvg;
      // }
      pulse = 0;
      bpmCount++;
   }
}

// void readPulse(void* arg)
// {
//   int arr[WINDOW_SIZE], bpmArr[WINDOW_SIZE];
//   int bpmCount = 0;
//   int count = 0;
//   int sum = 0;
//   int bpmSum = 0;
//   int pulse = 0;
//   float avg, bpmAvg;
//   float prevAvg = 0;
//   float prevBPMAvg = 0;
//   bool trend;
//   bool prevTrend = 0;
//   esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
//   esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
//   esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
//   uint32_t reading, voltage;
//   time_t start;
//   if(err != ESP_OK)
//   {
//     printf("Pulse invert error\n");
//     return;
//   }
//   while(1)
//   {
//     time(&start);
//     while(time(NULL) < start + 15)
//     {
//       reading = adc1_get_raw(ADC1_CHANNEL_4);
//       voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
//       // printf("Voltage: %d mV\n", voltage);
//       arr[count % WINDOW_SIZE] = voltage;
//       sum = 0;
//       for(int i = 0; i < WINDOW_SIZE; i++)
//       {
//         sum += arr[i];
//       }
//       avg = (float) sum / WINDOW_SIZE;
//       trend = avg > prevAvg ? 1 : 0;
//       if(trend && !prevTrend)
//       {
//         printf("Pulse\n");
//         gpio_set_level(GPIO_NUM_27, 1);
//         // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY));
//         // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//         pulse++;
//       }
//       prevTrend = trend;
//       prevAvg = avg;
//       count++;
//       vTaskDelay(pdMS_TO_TICKS(50));
//       gpio_set_level(GPIO_NUM_27, 0);
//       // ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, LEDC_DUTY_ZERO));
//       // ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
//     }
//     int bpm = pulse * 4;
//     bpmArr[bpmCount % WINDOW_SIZE] = bpm;
//     bpmSum = 0;
//     for(int i = 0; i < WINDOW_SIZE; i++)
//     {
//       bpmSum += bpmArr[i];
//     }
//     bpmAvg = (float) bpmSum / WINDOW_SIZE;
//     // printf("BPM: %d\n", bpm);
//     // printf("BPM Avg: %f\n", bpmAvg);
//     if(prevBPMAvg != 0 && bpmAvg > prevBPMAvg * 2)
//     {
//       hazard = 'H';
//       sensing = 0;
//     }
//     if(bpmCount % WINDOW_SIZE == 0)
//     {
//         prevBPMAvg = bpmAvg;
//     }
//     pulse = 0;
//     bpmCount++;
//   }
// }

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
        vTaskDelay(pdMS_TO_TICKS(10000)); // read delay
        reading = adc1_get_raw(ADC1_CHANNEL_6);
        // printf("reading: %d\n", reading);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        real_voltage = (uint32_t)(voltage * 2 * ADC_CORRECTION);
        printf("%d mV\n", real_voltage);
        if(real_voltage < LOW_VOLTAGE){
            printf("LOW Battery\n");
            for(int i = 0; i < 10; i++){
                // gpio_set_level(GPIO_NUM_27, i%2);
                int duty = i % 2 ? LEDC_DUTY_ZERO : LEDC_DUTY;
                // Set duty to 50% or 0
                ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
                // Update duty to apply the new value
                ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
                vTaskDelay(pdMS_TO_TICKS(250));
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

static esp_err_t readReg(uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    esp_err_t err;
    if (size == 0) {
        return ESP_OK;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    if(err != ESP_OK)
    {
      return err;
    }
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SLAVE_ADDR << 1) | 1, ACK_CHECK_EN);
    i2c_master_read(cmd, data_rd, size, NACK_VAL);

    i2c_master_stop(cmd);
    err = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static void freeFallHandler()
{
  while(1)
  {
    // printf("%d\n", gpio_get_level(GPIO_NUM_18));
    if(gpio_get_level(GPIO_NUM_18) == 1)
    {
      hazard = 'F';
      sensing = 0;
      printf("I've fallen and can't get up\n");
      for(int i = 0; i < 8; i++){
          gpio_set_level(GPIO_NUM_27, i%2);
          int duty = i % 2 ? LEDC_DUTY_ZERO : LEDC_DUTY;
          // Set duty to 50% or 0
          ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty));
          // Update duty to apply the new value
          ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
          vTaskDelay(pdMS_TO_TICKS(100));
        }
        gpio_set_level(GPIO_NUM_27, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void
lora_write_reg(int reg, int val)
{
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   //gpio_set_level(CONFIG_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   //gpio_set_level(CONFIG_CS_GPIO, 1);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int
lora_read_reg(int reg)
{
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   //gpio_set_level(CONFIG_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   //gpio_set_level(CONFIG_CS_GPIO, 1);
   return in[1];
}

/**
 * Perform physical reset on the Lora chip
 */
void
lora_reset(void)
{
   gpio_set_level(CONFIG_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void
lora_explicit_header_mode(void)
{
   __implicit = 0;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void
lora_implicit_header_mode(int size)
{
   __implicit = 1;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void
lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void
lora_sleep(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void
lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void
lora_set_tx_power(int level)
{
   // RF9x module uses PA_BOOST pin
   if (level < 2) level = 2;
   else if (level > 17) level = 17;
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void
lora_set_frequency(long frequency)
{
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void
lora_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Get spreading factor.
 */
int
lora_get_spreading_factor(void)
{
   return (lora_read_reg(REG_MODEM_CONFIG_2) >> 4);
}

/**
 * Set Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 * @param mode mode of DIO(0 to 3)
 */
void
lora_set_dio_mapping(int dio, int mode)
{
   if (dio < 4) {
      int _mode = lora_read_reg(REG_DIO_MAPPING_1);
      if (dio == 0) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 1) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      } else if (dio == 2) {
         _mode = _mode & 0xF3;
         _mode = _mode | (mode << 2);
      } else if (dio == 3) {
         _mode = _mode & 0xFC;
         _mode = _mode | mode;
      }
      lora_write_reg(REG_DIO_MAPPING_1, _mode);
   } else if (dio < 6) {
      int _mode = lora_read_reg(REG_DIO_MAPPING_2);
      if (dio == 4) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 5) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      lora_write_reg(REG_DIO_MAPPING_2, _mode);
   }
}

/**
 * Get Mapping of pins DIO0 to DIO5
 * @param dio Number of DIO(0 to 5)
 */
int
lora_get_dio_mapping(int dio)
{
   if (dio < 4) {
      int _mode = lora_read_reg(REG_DIO_MAPPING_1);
      if (dio == 0) {
         return ((_mode >> 6) & 0x03);
      } else if (dio == 1) {
         return ((_mode >> 4) & 0x03);
      } else if (dio == 2) {
         return ((_mode >> 2) & 0x03);
      } else if (dio == 3) {
         return (_mode & 0x03);
      }
   } else if (dio < 6) {
      int _mode = lora_read_reg(REG_DIO_MAPPING_2);
      if (dio == 4) {
         return ((_mode >> 6) & 0x03);
      } else if (dio == 5) {
         return ((_mode >> 4) & 0x03);
      }
   }
   return 0;
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
void
lora_set_bandwidth(int sbw)
{
   if (sbw < 10) {
      lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (sbw << 4));
   }
}

/**
 * Get bandwidth (bit rate)
 * @param sbw Signal bandwidth(0 to 9)
 */
int
lora_get_bandwidth(void)
{
   return ((lora_read_reg(REG_MODEM_CONFIG_1) & 0xf0) >> 4);
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
void
lora_set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Get coding rate
 */
int
lora_get_coding_rate(void)
{
   return ((lora_read_reg(REG_MODEM_CONFIG_1) & 0x0E) >> 1);
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void
lora_set_preamble_length(long length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Get the size of preamble.
 */
long
lora_get_preamble_length(void)
{
   long preamble;
   preamble = lora_read_reg(REG_PREAMBLE_MSB) << 8;
   preamble = preamble + lora_read_reg(REG_PREAMBLE_LSB);
   return preamble;
}

void
lora_set_sync_word(int sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void
lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void
lora_disable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int
lora_init(void)
{
   esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   gpio_reset_pin(CONFIG_RST_GPIO);
   gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
   gpio_reset_pin(CONFIG_CS_GPIO);
   gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);
   gpio_set_level(CONFIG_CS_GPIO, 1);

   spi_bus_config_t bus = {
      .miso_io_num = CONFIG_MISO_GPIO,
      .mosi_io_num = CONFIG_MOSI_GPIO,
      .sclk_io_num = CONFIG_SCK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0
   };

   //ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
   ret = spi_bus_initialize(SPI_HOST_ID, &bus, SPI_DMA_CH_AUTO);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
      .clock_speed_hz = 9000000,
      .mode = 0,
      .spics_io_num = CONFIG_CS_GPIO,
      .queue_size = 7,
      .flags = 0,
      .pre_cb = NULL
   };
   //ret = spi_bus_add_device(VSPI_HOST, &dev, &__spi);
   ret = spi_bus_add_device(SPI_HOST_ID, &dev, &__spi);
   assert(ret == ESP_OK);

   /*
    * Perform hardware reset.
    */
   lora_reset();

   /*
    * Check version.
    */
   uint8_t version;
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
      version = lora_read_reg(REG_VERSION);
      if(version == 0x12) break;
      vTaskDelay(2);
   }
   if (i == TIMEOUT_RESET + 1) return 0; // Illegal version
   //assert(i < TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   lora_sleep();
   lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
   lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   lora_set_tx_power(17);

   lora_idle();
   return 1;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void
lora_send_packet(uint8_t *buf, int size)
{
   /*
    * Transfer data to radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);

   for(int i=0; i<size; i++)
      lora_write_reg(REG_FIFO, *buf++);

   lora_write_reg(REG_PAYLOAD_LENGTH, size);

   /*
    * Start transmission and wait for conclusion.
    */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2);

   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int
lora_receive_packet(uint8_t *buf, int size)
{
   int len = 0;

   /*
    * Check interrupts.
    */
   int irq = lora_read_reg(REG_IRQ_FLAGS);
   lora_write_reg(REG_IRQ_FLAGS, irq);
   if((irq & IRQ_RX_DONE_MASK) == 0) return 0;
   if(irq & IRQ_PAYLOAD_CRC_ERROR_MASK) return 0;

   /*
    * Find packet size.
    */
   if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
   else len = lora_read_reg(REG_RX_NB_BYTES);

   /*
    * Transfer data from radio.
    */
   lora_idle();
   lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));
   if(len > size) len = size;
   for(int i=0; i<len; i++)
      *buf++ = lora_read_reg(REG_FIFO);

   return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int
lora_received(void)
{
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

/**
 * Returns RegIrqFlags.
 */
int
lora_get_irq(void)
{
   return (lora_read_reg(REG_IRQ_FLAGS));
}


/**
 * Return last packet's RSSI.
 */
int
lora_packet_rssi(void)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float
lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void
lora_close(void)
{
   lora_sleep();
}

void
lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", lora_read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}

void task_tx(void *pvParameters)
{
	ESP_LOGI(pcTaskGetName(NULL), "Start");
	uint8_t buf[256];
  vTaskDelay(pdMS_TO_TICKS(5000));

	while(1)
  {
    if(!sensing)
    {
      int send_len = sprintf((char *)buf,"ID 1919,CODE: %c", hazard);
  		lora_send_packet(buf, send_len);
  		ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
    }
    // else
    // {
    //   int send_len = sprintf((char *)buf,"%s", "Sensing");
  	// 	lora_send_packet(buf, send_len);
  	// 	ESP_LOGI(pcTaskGetName(NULL), "%d byte packet sent...", send_len);
    // }
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

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
    hazard = 'B';
    sensing = 0;

  }
}

void readGas(void* arg)
{
    // printf("gas sensor code running...\n");
    // Init GPIO25
    esp_rom_gpio_pad_select_gpio(GPIO_NUM_25);
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_25, 0);
    // Init ADC
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, V_REF, adc_chars);
    esp_err_t err = adc_set_data_inv(ADC_UNIT_1, 1);
    uint32_t reading, voltage, real_voltage, i;
    // Heat up gas sensor
    gpio_set_level(GPIO_NUM_25, 1);
    vTaskDelay(pdMS_TO_TICKS(1*60*1000));
    while(1){
        // Turn on MOSFET (HIGH VOLTAGE)
        gpio_set_level(GPIO_NUM_25, 1);
        // printf("...gas MOSFET on\n");
        // Wait 60 seconds
        vTaskDelay(pdMS_TO_TICKS(60*1000));

        // Turn off MOSFET (LOW VOLTAGE )
        gpio_set_level(GPIO_NUM_25, 0);
        // printf("...gas MOSFET off\n");
        // Wait 90 seconds
        vTaskDelay(pdMS_TO_TICKS(90*1000));
        // Read ADC
        reading = adc1_get_raw(ADC1_CHANNEL_5);
        voltage = esp_adc_cal_raw_to_voltage(reading, adc_chars);
        // printf("...raw voltage: %d mV\n", voltage);
        // printf("...corrected v: %f mV\n", voltage * L_TO_H_COEFF);
        if(voltage > G_CUTOFF)
        {
          hazard = 'g';
          sensing = 0;
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
  esp_rom_gpio_pad_select_gpio(GPIO_NUM_27);
  gpio_set_direction(GPIO_NUM_27, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_27, 0);

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_23); // accel pull down
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GPIO_NUM_23, GPIO_PULLDOWN_ONLY);
  gpio_set_level(GPIO_NUM_23, 0);

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_18);
  gpio_set_direction(GPIO_NUM_18, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_18, GPIO_PULLDOWN_ONLY);

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_5);
  gpio_set_direction(GPIO_NUM_5, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_5, GPIO_PULLUP_ONLY);
  gpio_set_intr_type(GPIO_NUM_5, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_NUM_5, button_isr_handler, NULL);

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_26);
  gpio_set_direction(GPIO_NUM_26, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_26, 0);

  int master_cal = i2c_master_init();
  if(!master_cal) {return;}
  int dev_add = i2c_scanner();
  if(dev_add != SLAVE_ADDR)
  {
    printf("LSM6DSL not found\n");
    return;
  }

  // Free Fall instructions -- WORKS
  // Threshold of 156 mg
  // FF Duration of ~75 ms
  uint8_t instructions[] = {LSM6DSL_ACC_GYRO_CTRL1_XL, 0x60,
                            LSM6DSL_ACC_GYRO_TAP_CFG1, 0x80,
                            LSM6DSL_ACC_GYRO_WAKE_UP_DUR, 0x00,
                            LSM6DSL_ACC_GYRO_FREE_FALL, 0xf8,
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

  esp_rom_gpio_pad_select_gpio(GPIO_NUM_2);
  gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_2, 1);

  if (lora_init() == 0) {
		ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
	lora_set_frequency(915e6); // 915MHz
	lora_enable_crc();

  example_ledc_init();

	xTaskCreate(&task_tx, "task_tx", 1024*2, NULL, 10, NULL);
  xTaskCreate(button_task, "button_task", 4096, NULL, 9, &ISR);
  xTaskCreate(readBatt,"readBatt_task", 4096, NULL, 8, NULL);
  xTaskCreate(readGas,"readGas_task", 4096, NULL, 7, NULL);
  xTaskCreate(readPulse,"readPulse_task", 4096, NULL, 6, NULL);
  xTaskCreate(freeFallHandler,"freeFallHandler_task", 4096, NULL, 5, NULL);



  return;
}
