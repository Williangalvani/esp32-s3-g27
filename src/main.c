#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include <string.h>
#include <stdio.h>

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUD_ENABLED
#error CFG_TUD_ENABLED must be defined
#endif

#if CONFIG_TINYUSB_HID_COUNT == 0
#error CONFIG_TINYUSB_HID_COUNT must be more than zero
#endif

static const char *TAG = "tinyusb_gamepad";

// Gamepad report structure
typedef struct __attribute__((packed)) {
    uint8_t x;
    uint8_t y;
    uint8_t buttons;
} gamepad_report_t;

static gamepad_report_t gamepad_report;

// HID report descriptor for gamepad
const uint8_t gamepad_report_descriptor[] = {
    0x05, 0x01,        // USAGE_PAGE (Generic Desktop)
    0x09, 0x05,        // USAGE (Game Pad)
    0xa1, 0x01,        // COLLECTION (Application)
    0x09, 0x01,        //   USAGE (Pointer)
    0xa1, 0x00,        //   COLLECTION (Physical)
    0x09, 0x30,        //     USAGE (X)
    0x09, 0x31,        //     USAGE (Y)
    0x15, 0x00,        //     LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,  //     LOGICAL_MAXIMUM (255)
    0x75, 0x08,        //     REPORT_SIZE (8)
    0x95, 0x02,        //     REPORT_COUNT (2)
    0x81, 0x02,        //     INPUT (Data,Var,Abs)
    0xc0,              //   END_COLLECTION
    0x05, 0x09,        //   USAGE_PAGE (Button)
    0x19, 0x01,        //   USAGE_MINIMUM (Button 1)
    0x29, 0x08,        //   USAGE_MAXIMUM (Button 8)
    0x15, 0x00,        //   LOGICAL_MINIMUM (0)
    0x25, 0x01,        //   LOGICAL_MAXIMUM (1)
    0x75, 0x01,        //   REPORT_SIZE (1)
    0x95, 0x08,        //   REPORT_COUNT (8)
    0x81, 0x02,        //   INPUT (Data,Var,Abs)
    0xc0               // END_COLLECTION
};

// Add these definitions for USB descriptor configuration
#define EPNUM_HID   0x81

enum {
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

// Create a configuration descriptor
static const uint8_t desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN, 0, 100),

    // Interface number, string index, protocol, report descriptor len, EP In address, size & polling interval
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_NONE, sizeof(gamepad_report_descriptor), EPNUM_HID, 16, 10)
};

// Fix string descriptor array - language ID must be first
static const char *string_desc_arr[] = {
    (char[]){0x09, 0x04},       // 0: Language ID (English)
    "TinyUSB",                  // 1: Manufacturer
    "ESP32-S3 Gamepad",         // 2: Product
    "123456"                    // 3: Serial number - this was missing!
};

// USB Device Descriptor
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,          // USB 2.0 specification
    .bDeviceClass = 0x00,      // Class defined at interface level
    .bDeviceSubClass = 0x00,   // Subclass defined at interface level
    .bDeviceProtocol = 0x00,   // Protocol defined at interface level
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A,        // ESP32 vendor ID
    .idProduct = 0x4002,       // A unique product ID
    .bcdDevice = 0x0100,       // Device version 1.0
    .iManufacturer = 1,        // Index of manufacturer string
    .iProduct = 2,             // Index of product string
    .iSerialNumber = 3,        // Index of serial number string
    .bNumConfigurations = 1    // We only have one configuration
};

// Task to generate random gamepad values
void hid_gamepad_task(void *pvParameters)
{
    while (1) {
        // Generate random values for the gamepad
        gamepad_report.x = esp_random() % 256;
        gamepad_report.y = esp_random() % 256;
        gamepad_report.buttons = esp_random() % 256;
        
        // Send the report
        if (tud_hid_ready()) {
            tud_hid_report(0, &gamepad_report, sizeof(gamepad_report));
            
            // Debug via log
            ESP_LOGI(TAG, "Gamepad: X=%d, Y=%d, Buttons=0x%02x", 
                     gamepad_report.x, gamepad_report.y, gamepad_report.buttons);
            
            // Debug via Serial JTAG
            printf("Gamepad: X=%d, Y=%d, Buttons=0x%02x\r\n", 
                   gamepad_report.x, gamepad_report.y, gamepad_report.buttons);
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Update at 10 Hz
    }

}

// HID callbacks required by TinyUSB
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, 
                              hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                          hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize)
{
    // Nothing to do
}

// Invoked when received GET HID REPORT DESCRIPTOR request
// Application returns pointer to descriptor
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance; // Ignore parameter
    return gamepad_report_descriptor;
}

void app_main(void)
{
    ESP_LOGI(TAG, "USB HID Gamepad Example");
    
    // Initialize TinyUSB with the required descriptor
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,  // Set our custom device descriptor
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration
    };
    
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    
    // Create task for gamepad updates
    xTaskCreate(hid_gamepad_task, "hid_task", 4096, NULL, 5, NULL);
}