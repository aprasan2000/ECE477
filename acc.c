#include <stdio.h>
#include <math.h>
#include <string.h>
#include <sys/param.h>
#include "esp_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_attr.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "./ADXL343.h"

#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "freertos/event_groups.h"

#include <ctype.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "sdkconfig.h"

#define TIMER_DIVIDER 16                            
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER)
#define TIMER_INTERVAL_SEC (1)                       
#define TEST_WITH_RELOAD 1                           
#define BLINK_GPIO 13

#define I2C_EXAMPLE_MASTER_SCL_IO 22   
#define I2C_EXAMPLE_MASTER_SDA_IO 23   
#define I2C_EXAMPLE_MASTER_NUM I2C_NUM_0  
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE 0    
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE 0    
#define I2C_EXAMPLE_MASTER_FREQ_HZ 100000     
#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT  I2C_MASTER_READ 
#define ACK_CHECK_EN true 
#define ACK_CHECK_DIS false
#define ACK_VAL 0x00 
#define NACK_VAL 0xFF
#define SLAVE_ADDR ADXL343_ADDRESS 

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR "192.168.1.9"
#endif

#define PORT 9001

int flag = 0;

static const char *TAG = "Noggin";


static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    int ledOn = 0;

    while (1) {

        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
       /* if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);
*/
        while (1) {

            struct sockaddr_in source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                if (rx_buffer == 0) {
                    break;
                }
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                printf(rx_buffer);
                printf("-----------------");
                if(strncmp(rx_buffer, "Ok!", 3) == 0) {
                    // ledOn = 1;
                    printf("TOGGLED--------------");
                    ESP_LOGI(TAG, "Toggle LED");
                    if (ledOn==0){
                        gpio_set_level(BLINK_GPIO, 1);
                        ledOn=1;
                    }
                    else if (ledOn==1){
                        gpio_set_level(BLINK_GPIO, 0);
                        ledOn=0;
                    }
                    //break;
                
                
                } else if(strncmp(rx_buffer, "Nk!", 3) == 0){
                    printf("STAYING--------------");
                    ESP_LOGI(TAG, "Hold LED");
                    
                    //break;
                }
            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define EXAMPLE_ESP_WIFI_SSID      "무"
#define EXAMPLE_ESP_WIFI_PASS      "199806288"
#define EXAMPLE_ESP_MAXIMUM_RETRY  4

static EventGroupHandle_t s_wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static int s_retry_num = 0;

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            /*ESP_LOGI(TAG, "retry to connect to the AP");*/
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        /*ESP_LOGI(TAG,"connect to the AP fail");*/
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );
/*
    ESP_LOGI(TAG, "wifi_init_sta finished.");
*/
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);
/*
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        //ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
*/
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
}

static void i2c_master_init(){
  int err;

  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                            
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;             
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;             
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       
  err = i2c_param_config(i2c_master_port, &conf); 

  if (err == ESP_OK) 
      printf("Parameters checked\n");

  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);

  if (err == ESP_OK) 
      printf("Initialization Complete\n");

  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) 
      printf("No I2C devices found" "\n");
}

int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

void writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, data, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}

uint8_t readRegister(uint8_t reg) {
    uint8_t value;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &value, ACK_CHECK_DIS);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return value;
}

int16_t read16(uint8_t reg) {
    uint8_t val1;
    uint8_t val2;
    val1 = readRegister(reg);
    if (reg == 41) 
        val2 = 0;
    else 
        val2 = readRegister(reg+1);

    return (((int16_t)val2 << 8) | val1);
}

void setRange(range_t range) {
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);
  format &= ~0x0F;
  format |= range;
  format |= 0x08;
  writeRegister(ADXL343_REG_DATA_FORMAT, format);
}

range_t getRange(void) {
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}


float AccelMinX = 0;
float AccelMaxX = 0;
float AccelMinY = 0;
float AccelMaxY = 0;
float AccelMinZ = 0;
float AccelMaxZ = 0;

#define GPIO_DETECTED 14
#define GPIO_NOT 15


int detect_flag = 0;

bool detect(float * xp, float *yp, float *zp){
    if(*xp >= AccelMaxX || *xp <= AccelMinX || *yp >= AccelMaxY || *yp <= AccelMinY || *zp >= AccelMaxZ || *zp <= AccelMinZ){
        return true;
    }

    return false;
}

void getAccel(float * xp, float *yp, float *zp) {
    *xp = (read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD);
    *yp = (read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD);
    *zp = (read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD);

    /*if(detect(xp, yp, zp))
        detect_flag = 1;
    else
        detect_flag = 0;
*/
    printf("\nX: %.3f\n", *xp);
    printf("Y: %.3f\n", *yp);
    printf("Z: %.3f\n\n", *zp);
}

void Toggle_Led(int detect_flag){
    if(detect_flag){
        gpio_set_level(GPIO_DETECTED, 1);
        gpio_set_level(GPIO_NOT, 0);
    }
    else{
        gpio_set_level(GPIO_DETECTED, 0);
        gpio_set_level(GPIO_NOT, 1);
    }

}

static void accelHandler() {
  while (1) {
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    Toggle_Led(detect_flag);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

}

void app_main(void){

    i2c_master_init();
    i2c_scanner();

    uint8_t deviceID;
    getDeviceID(&deviceID);
    if (deviceID != 0xE5) 
        printf("\n>> Cannot Find ADAXL343\n");

    writeRegister(ADXL343_REG_INT_ENABLE, 0);
    setRange(ADXL343_RANGE_16_G);
    writeRegister(ADXL343_REG_POWER_CTL, 0x08);
    xTaskCreate(accelHandler,"accelHandler_task", 4096, NULL, 5, NULL);
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    
    ESP_ERROR_CHECK(ret);
    /*ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");*/
    wifi_init_sta();

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
           
}
