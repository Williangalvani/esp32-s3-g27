#include "esp_log.h"
#include "driver/usb_serial_jtag.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "usb_cdc";

void app_main() {
    /* Initialize USB CDC */
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    
    ESP_LOGI(TAG, "Initializing USB CDC");
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_config));
    
    /* Test message to verify CDC is working */
    ESP_LOGI(TAG, "USB CDC initialized");
    
    /* Example of periodically sending a message over USB CDC */
    int counter = 0;
    while (1) {
        char message[64];
        sprintf(message, "Hello from ESP32-S3, count: %d\n", counter++);
        usb_serial_jtag_write_bytes((const uint8_t*)message, strlen(message), portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Send message every second
    }
}