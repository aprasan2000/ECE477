#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include <string.h>
#include <stdio.h>
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <string.h>
#include <ctype.h>

#define CONFIG_CS_GPIO     4//15//4
#define CONFIG_RST_GPIO    3//16//3
#define CONFIG_MISO_GPIO   19//12//19
#define CONFIG_MOSI_GPIO   18//13//18
#define CONFIG_SCK_GPIO    5//14//5
#define LCD1602_GPIO_SDA	21
#define LCD1602_GPIO_SCL	22

#define I2C_MASTER_NUM             I2C_NUM_1        
#define I2C_MASTER_TX_BUF_DISABLE  0                
#define I2C_MASTER_RX_BUF_DISABLE  0                
#define I2C_MASTER_FREQ_HZ         100000            
#define LCD1602_I2C_ADDR (0x27)

#define WRITE_BIT                          I2C_MASTER_WRITE 
#define READ_BIT                           I2C_MASTER_READ  
#define ACK_CHECK_EN                       0x1              
#define ACK_CHECK_DIS                      0x0             
#define ACK_VAL                            0x0            
#define NACK_VAL                           0x1         

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
#ifdef CONFIG_IDF_TARGET_ESP32
#define SPI_HOST_ID HSPI_HOST
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define SPI_HOST_ID SPI2_HOST
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define SPI_HOST_ID SPI2_HOST
#endif

#define TAG "LORA"

static spi_device_handle_t __spi;

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

#define LCD_DELAY_MS 5

static int __implicit;
static long __frequency;

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
   spi_device_transmit(__spi, &t);
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
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
   } else if (dio < 6) {
      int _mode = lora_read_reg(REG_DIO_MAPPING_2);
      if (dio == 4) {
         _mode = _mode & 0x3F;
         _mode = _mode | (mode << 6);
      } else if (dio == 5) {
         _mode = _mode & 0xCF;
         _mode = _mode | (mode << 4);
      }
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
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
      ESP_LOGD(TAG, "REG_DIO_MAPPING_1=0x%02x", _mode);
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
      ESP_LOGD(TAG, "REG_DIO_MAPPING_2=0x%02x", _mode);
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


int 
lora_get_bandwidth(void)
{
   return ((lora_read_reg(REG_MODEM_CONFIG_1) & 0xf0) >> 4);
}

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

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
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
   //gpio_pad_select_gpio(CONFIG_RST_GPIO);
   gpio_reset_pin(CONFIG_RST_GPIO);
   gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
   //gpio_pad_select_gpio(CONFIG_CS_GPIO);
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
      ESP_LOGD(TAG, "version=0x%02x", version);
      if(version == 0x12) break;
      vTaskDelay(2);
   }
   ESP_LOGD(TAG, "i=%d, TIMEOUT_RESET=%d", i, TIMEOUT_RESET);
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
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) {       
      return 1;
   }   
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
//   close(__spi);  FIXME: end hardware features after lora_close
//   close(__cs);
//   close(__rst);
//   __spi = -1;
//   __cs = -1;
//   __rst = -1;
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

int lcd1602_write_cmd(uint8_t addr, uint8_t cmd)
{
    i2c_cmd_handle_t hCmd = i2c_cmd_link_create();
    i2c_master_start(hCmd);
    i2c_master_write_byte(hCmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(hCmd, cmd, ACK_CHECK_EN);
    i2c_master_stop(hCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, hCmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(hCmd);

    return ret;
}

int lcd1602_write_data(uint8_t addr, uint8_t data[], int len)
{
    i2c_cmd_handle_t hCmd = i2c_cmd_link_create();
    i2c_master_start(hCmd);
    i2c_master_write_byte(hCmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(hCmd, data, len, ACK_CHECK_EN);
    i2c_master_stop(hCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, hCmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(hCmd);

    return ret;
}

int lcd1602_write_cmdNdata(uint8_t addr, uint8_t cmd, uint8_t data)
{
    i2c_cmd_handle_t hCmd = i2c_cmd_link_create();
    i2c_master_start(hCmd);
    i2c_master_write_byte(hCmd, ( addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(hCmd, cmd, ACK_CHECK_EN);
    i2c_master_write_byte(hCmd, data, ACK_CHECK_EN);
    i2c_master_stop(hCmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, hCmd, 10 / portTICK_RATE_MS);
    i2c_cmd_link_delete(hCmd);

    return ret;
}

int lcd1602_SendInternal(uint8_t data, uint8_t flags)
{
	int res;
	uint8_t up = data & 0xF0;
	uint8_t lo = (data << 4) & 0xF0;

	uint8_t data_arr[4];
	data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
	data_arr[1] = up|flags|BACKLIGHT;
	data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
	data_arr[3] = lo|flags|BACKLIGHT;

    res = lcd1602_write_data(LCD1602_I2C_ADDR, data_arr, 4);

    vTaskDelay(LCD_DELAY_MS / portTICK_RATE_MS);
	return res;
}

void lcd1602_SendCommand(uint8_t cmd)
{
	lcd1602_SendInternal(cmd, 0);
}

void lcd1602_SendData(uint8_t data)
{
	lcd1602_SendInternal(data, PIN_RS);
}

void lcd1602_SendString(char *str)
{
	while(*str)
	{
		lcd1602_SendData((uint8_t)(*str));
		str++;
	}
}

void lcd1602_Init(void)
{
	lcd1602_SendCommand(0x30);
	lcd1602_SendCommand(0x02);
	lcd1602_SendCommand(0x0C);
	lcd1602_SendCommand(0x01);
}

void i2c_scan(void)
{
    int res;

	for(uint16_t i = 0; i < 128; i++)
	{
		res = lcd1602_write_cmd(i, 0x00);
		if(res == ESP_OK)
		{
    		char msg[64];

    		snprintf(msg, sizeof(msg), "0x%02X", i);
    		printf("%s", msg);
		}
        else
		{
			printf("%s", ".");
		}
	}

	printf("%s", "\r\n");
}

void lcd1602_i2c_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = LCD1602_GPIO_SDA;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = LCD1602_GPIO_SCL;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2c_master_port, &conf);
    i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_MASTER_RX_BUF_DISABLE,
                       I2C_MASTER_TX_BUF_DISABLE, 0);
}

//int cnt = 20;

static void lcd1602_task(void* arg)
{
    lcd1602_i2c_init();
	i2c_scan();
	lcd1602_Init();

	// set address to 0x00
	lcd1602_SendCommand(0x80);
    char* top_char2 = {"Hello"};
    char top_char1[16] = {" "};
    strcat(top_char1,top_char2);
	lcd1602_SendString(top_char1);
    char bot_char1[16] = {"World"};
    //char* c = cnt +'0';
	// set address to 0x40
	lcd1602_SendCommand(0xC0);
	lcd1602_SendString(bot_char1);
    while(1)
    {
        printf(".\n");
        vTaskDelay(1000 / portTICK_RATE_MS);
    }
}

void task_rx(void *p)
{

   lcd1602_i2c_init();
	i2c_scan();
	lcd1602_Init();

	// set address to 0x00

   uint8_t buf[32];   
   int x;
   for(;;) {
      lora_receive();    
      while(lora_received()) {
         x = lora_receive_packet(buf, sizeof(buf));
         buf[x] = 0;
		   ESP_LOGI(pcTaskGetName(NULL), "Received: %s", buf);         
         lora_receive();
	      lcd1602_SendCommand(0x80);
         char* top_char = {"Received:"};
	      lcd1602_SendString(top_char);
	      lcd1602_SendCommand(0xC0);
	      lcd1602_SendString(&buf);         
      }
      vTaskDelay(1);
   }
}


void app_main()
{
/*
	if (lora_init() == 0) {
		ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the module");
		while(1) {
			vTaskDelay(1);
		}
	}

	ESP_LOGI(pcTaskGetName(NULL), "Frequency is 915MHz");
	lora_set_frequency(915e6); // 915MHz

	lora_enable_crc();
*/

   xTaskCreate(&lcd1602_task, "lcd1602_task", 2048, (void* ) 0, 10, NULL);
	//xTaskCreate(&task_rx, "task_rx", 1024*2, NULL, 5, NULL);
}
