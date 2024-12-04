#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <driver/adc.h>
#include "esp_adc_cal.h"

#include "lcd.h"

#define POWER_BUTTON_PIN    GPIO_NUM_4
#define MODE_BUTTON_PIN     GPIO_NUM_5
#define BRIGHTNESS_KNOB_PIN ADC1_CHANNEL_5
#define BRIGHTNESS_LDR_PIN  ADC2_CHANNEL_2
#define PIR_SENSOR_PIN      GPIO_NUM_19

#define WIFI_SSID "YOUR_WIFI_NETWORK"
#define WIFI_PASS "YOUR_WIFI_PASSWORD"
#define SERVER_IP "192.168.1.101"
#define SERVER_PORT 38899

QueueHandle_t udpQueue;
QueueHandle_t lcdQueue;
TaskHandle_t  pirTaskHandle;

enum powerModes { OFF, ON, AUTO };
char powerModesText[3][5] = {"OFF", "ON", "AUTO"};
uint8_t currentPowerMode = 0;

enum lightModes { WARM_WHITE = 11, DAYLIGHT, COOL_WHITE, NIGHT_LIGHT };
char lightModesText[4][15] = {"WARM WHITE", "DAYLIGHT", "COOL WHITE", "NIGHT LIGHT"};
uint8_t currentLightMode = 1;

uint8_t dimming = 75;


void wifi_init() {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_connect());
}

void pir_task(void *pvParameters) {
    char *udpMessage  = malloc(256);
    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << GPIO_NUM_19,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);

    int motion_detected = gpio_get_level(PIR_SENSOR_PIN);
    while (1) {
        if(motion_detected != gpio_get_level(PIR_SENSOR_PIN)) {
            motion_detected = !motion_detected;
            if(motion_detected) {
                snprintf(udpMessage, 256,
                    "{\"method\":\"setPilot\",\"params\":{\"orig\":\"andr\",\"sceneid\":%d,\"dimming\":%d}}",
                    11+currentLightMode, dimming); 
                xQueueSend(udpQueue, &udpMessage, portMAX_DELAY);
            } else {
                snprintf(udpMessage, 256, "{\"method\":\"setPilot\",\"params\":{\"state\":\"False\"}}");
                xQueueSend(udpQueue, &udpMessage, portMAX_DELAY);
            }
            vTaskDelay(pdMS_TO_TICKS(300));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void update_status() {
    char *lcdMessage1 = malloc(19);
    char *lcdMessage2 = malloc(19);
    char *udpMessage  = malloc(256);

    snprintf(lcdMessage1,18, "0%-12s%3d%%",powerModesText[currentPowerMode], dimming);
    xQueueSend(lcdQueue, &lcdMessage1, portMAX_DELAY);
        
    snprintf(lcdMessage2,17, "1%-15s",lightModesText[currentLightMode]);
    xQueueSend(lcdQueue, &lcdMessage2, portMAX_DELAY);

    switch(currentPowerMode) {
        case OFF:
            if(pirTaskHandle != NULL) {
                vTaskDelete(pirTaskHandle);
                pirTaskHandle = NULL;
            } 
            snprintf(udpMessage, 256, "{\"method\":\"setPilot\",\"params\":{\"state\":\"False\"}}");
            xQueueSend(udpQueue, &udpMessage, portMAX_DELAY);
        break;
        case ON:
            snprintf(udpMessage, 256,
             "{\"method\":\"setPilot\",\"params\":{\"orig\":\"andr\",\"sceneid\":%d,\"dimming\":%d}}",
             11+currentLightMode, dimming); 
            xQueueSend(udpQueue, &udpMessage, portMAX_DELAY);
        break;
        case AUTO:
            xTaskCreate(pir_task, "pir_task", 4096, NULL, 5, &pirTaskHandle);
        break;
    }
}

void brightness_task(void *pvParameters) {
    uint16_t adc_reading = dimming;
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BRIGHTNESS_KNOB_PIN, ADC_ATTEN_DB_12);
    
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BRIGHTNESS_LDR_PIN, ADC_ATTEN_DB_12);

    while (1) {
        if(currentPowerMode == AUTO) {
            adc_reading = adc1_get_raw(BRIGHTNESS_LDR_PIN);
            adc_reading = 100 - ((adc_reading * 100) / 4095);
        } else {
            adc_reading = adc1_get_raw(BRIGHTNESS_KNOB_PIN);
            adc_reading = (adc_reading * 100) / 4095;
        }
    
        if(abs(dimming - adc_reading) > 2) {
            dimming = adc_reading;
            update_status();
        } 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void udp_client_task(void* pvParameters) {
    char *receivedMessage;
    struct sockaddr_in destAddr;
    destAddr.sin_addr.s_addr = inet_addr(SERVER_IP);
    destAddr.sin_family = AF_INET;
    destAddr.sin_port = htons(SERVER_PORT);
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    
    if (sock < 0) {
        ESP_LOGE("UDP Client", "Socket creation failed: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    while(1) {
         if (xQueueReceive(udpQueue, &receivedMessage, portMAX_DELAY)) {
            int err = sendto(sock, receivedMessage, strlen(receivedMessage), 0, (struct sockaddr*)&destAddr, sizeof(destAddr));
            if (err < 0) {
                ESP_LOGE("UDP Client", "Error sending message: errno %d", errno);
            } else {
                ESP_LOGE("UDP Client", "Sent: %s", receivedMessage);
            }
        }
    }

    shutdown(sock, 0);
    close(sock);
    vTaskDelete(NULL);
}

void lcd_task(void *pvParameters) {
    char *receivedMessage;
    lcd_init();     // Initialize LCD
    lcd_clear();    // Clear LCD

    while(1) {
        if (xQueueReceive(lcdQueue, &receivedMessage, portMAX_DELAY)) {
            lcd_put_cur(receivedMessage[0] - '0', 0);
            lcd_send_string(receivedMessage+1);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }

    vTaskDelete(NULL);
}

void buttons_task(void *pvParameters) {
    // Configuring power button
    gpio_config_t pwr_btn_config = {
        .pin_bit_mask = (1ULL << POWER_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pwr_btn_config);

    // Configuring mode button
    gpio_config_t mode_btn_config = {
        .pin_bit_mask = (1ULL << MODE_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&mode_btn_config);

     while (1) {
        int power_state = gpio_get_level(POWER_BUTTON_PIN);
        int mode_state = gpio_get_level(MODE_BUTTON_PIN);

        if(!power_state) {
            currentPowerMode = (currentPowerMode + 1) % 3;
            update_status();
            vTaskDelay(pdMS_TO_TICKS(300)); // Debounce
        }
            
        if(!mode_state) {
            currentLightMode = (currentLightMode + 1) % 4;
            update_status();
            vTaskDelay(pdMS_TO_TICKS(300)); // Debounce
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void app_main() {
    wifi_init();    // Initialize WIFI    
    i2c_init();     // Initialize I2C 

    lcdQueue = xQueueCreate(10, sizeof(char *));
    if(lcdQueue != NULL) {
        xTaskCreate(lcd_task, "lcd_task", 4096, NULL, 5, NULL);
    } else {    
        ESP_LOGI("LCD Queue", "Initialization Failed");
    }

    udpQueue = xQueueCreate(10, sizeof(char *));
    if(udpQueue != NULL) {
        xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGI("UDP Queue", "Initialization Failed");
    }

    xTaskCreate(buttons_task, "buttons_task", 4096, NULL, 5, NULL);
    xTaskCreate(brightness_task, "brightness_task", 4096, NULL, 5, NULL);
    update_status();
}
