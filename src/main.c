#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_random.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "esp_timer.h"

// Define M_PI if it's not already defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef CFG_TUSB_MCU
#error CFG_TUSB_MCU must be defined
#endif

#ifndef CFG_TUD_ENABLED
#error CFG_TUD_ENABLED must be defined
#endif

#if CONFIG_TINYUSB_HID_COUNT == 0
#error CONFIG_TINYUSB_HID_COUNT must be more than zero
#endif

static const char *TAG = "g27_wheel";

// Device constants
#define DEV_VID (0x046d)
#define DEV_PID (0xc29b)
#define DEV_PRODUCT_NAME "G27 Racing Wheel"
#define DEV_MANUFACTURER_NAME "Logitech, Inc."
#define DEV_REPORT_ID (0)
#define DEV_REPORT_SIZE (11)    // Size of our HID report (wheel_report structure size)
#define DEV_FFB_REQUEST_SIZE (7) // Size of force feedback request from host

// Force feedback data structure
typedef struct __attribute__((packed)) {
    uint8_t cmd;                // Command byte
    uint8_t params[6];          // Command parameters
} g27_ffb_command_t;

// FFB command identifiers
#define FFB_CMD_AUTOCENTER    0x01
#define FFB_CMD_FRICTION      0x02
#define FFB_CMD_DAMPER        0x03
#define FFB_CMD_SPRING        0x04
#define FFB_CMD_CONSTANT      0x05
#define FFB_CMD_PERIODIC      0x06

// G27 Wheel report structure
typedef struct __attribute__((packed)) {
    uint8_t buttons_0;              // 8 buttons  
    uint8_t buttons_1;              // 8 buttons
    uint8_t buttons_2;              // 8 buttons
    uint8_t axis_wheel_lsb6_and_btns2; // 6 bits lower wheel position + 2 buttons
    uint8_t axis_wheel_msb;         // 8 bits upper wheel position
    int8_t axis_throttle;           // 8 bits (throttle)
    int8_t axis_brake;              // 8 bits (brake)
    int8_t axis_clutch;             // 8 bits (clutch)
    uint8_t shifter_x;              // Shifter X position
    uint8_t shifter_y;              // Shifter Y position
    uint8_t misc;                   // Miscellaneous data
} g27_wheel_report_t;

static g27_wheel_report_t wheel_report;

// HID descriptor for G27 Racing Wheel
static const uint8_t hid_report_descriptor[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x04,        // Usage (Joystick)
    0xA1, 0x01,        // Collection (Application)
    0x15, 0x00,        //   Logical minimum (0)
    0x25, 0x07,        //   Logical maximum (7)
    0x35, 0x00,        //   Physical minimum (0)
    0x46, 0x3B, 0x01,  //   Physical maximum (315)
    0x65, 0x14,        //   Unit (20)
    0x09, 0x39,        //   Usage (Hat switch)
    0x75, 0x04,        //   Report size (4)
    0x95, 0x01,        //   Report count (1)
    0x81, 0x42,        //   Item
    0x65, 0x00,        //   Unit (0)
    0x25, 0x01,        //   Logical maximum (1)
    0x45, 0x01,        //   Physical maximum (1)
    0x05, 0x09,        //   Usage Page (Button)
    0x19, 0x01,        //   Usage Minimum (Button 1)
    0x29, 0x16,        //   Usage Maximum (Button 22)
    0x75, 0x01,        //   Report size (1)
    0x95, 0x16,        //   Report count (22)
    0x81, 0x02,        //   Item
    0x26, 0xFF, 0x3F,  //   Logical maximum (16383)
    0x46, 0xFF, 0x3F,  //   Physical maximum (16383)
    0x75, 0x0E,        //   Report size (14)
    0x95, 0x01,        //   Report count (1)
    0x05, 0x01,        //   Usage Page (Generic Desktop)
    0x09, 0x30,        //   Usage (X)
    0x81, 0x02,        //   Item
    0x26, 0xFF, 0x00,  //   Logical maximum (255)
    0x46, 0xFF, 0x00,  //   Physical maximum (255)
    0x75, 0x08,        //   Report size (8)
    0x95, 0x03,        //   Report count (3)
    0x09, 0x32,        //   Usage (Z)
    0x09, 0x35,        //   Usage (Rz)
    0x09, 0x31,        //   Usage (Y)
    0x81, 0x02,        //   Item
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
    0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
    0x95, 0x02,        //   Report count (2)
    0x81, 0x02,        //   Item
    0x95, 0x01,        //   Report count (1)
    0x75, 0x01,        //   Report size (1)
    0x25, 0x01,        //   Logical maximum (1)
    0x45, 0x01,        //   Physical maximum (1)
    0x05, 0x09,        //   Usage Page (Button)
    0x09, 0x17,        //   Usage (Button 23)
    0x81, 0x02,        //   Item
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
    0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
    0x95, 0x07,        //   Report count (7)
    0x81, 0x02,        //   Item
    0x26, 0xFF, 0x00,  //   Logical maximum (255)
    0x46, 0xFF, 0x00,  //   Physical maximum (255)
    0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
    0x09, 0x02,        //   Usage (Vendor-defined {ff00:2))
    0x95, 0x07,        //   Report count (7)
    0x75, 0x08,        //   Report size (8)
    0x91, 0x02,        //   Item
    0x95, 0x90,        //   Report count (144)
    0x09, 0x03,        //   Usage (Vendor-defined {ff00:3))
    0xB1, 0x02,        //   Item
    0xC0,              // End Collection
};

// USB descriptor configuration
#define EPNUM_HID_IN    0x81  // Endpoint for HID IN (device to host)
#define EPNUM_HID_OUT   0x01  // Endpoint for HID OUT (host to device)

enum {
    ITF_NUM_HID,
    ITF_NUM_TOTAL
};

// Configuration descriptor - modified to match captured USB traffic
static const uint8_t desc_configuration[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN, 0x80, 98), // Self-powered (0x80) with max power of 98mA

    // Custom HID Interface descriptor with proper bcdHID value
    // Interface descriptor
    9, TUSB_DESC_INTERFACE, ITF_NUM_HID, 0, 2, TUSB_CLASS_HID, 0, HID_ITF_PROTOCOL_NONE, 0,
    // HID descriptor with bcdHID = 0x0111 (version 1.11)
    9, HID_DESC_TYPE_HID, 0x11, 0x01, 0, 1, HID_DESC_TYPE_REPORT, U16_TO_U8S_LE(sizeof(hid_report_descriptor)),
    // Endpoint In
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_IN, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(16), 2,
    // Endpoint Out
    7, TUSB_DESC_ENDPOINT, EPNUM_HID_OUT, TUSB_XFER_INTERRUPT, U16_TO_U8S_LE(16), 2
};

// String descriptors
static const char *string_desc_arr[] = {
    (char[]){0x09, 0x04},        // 0: Language ID (English)
    DEV_MANUFACTURER_NAME,       // 1: Manufacturer
    DEV_PRODUCT_NAME,            // 2: Product
    "G27-ESP32S3"                // 3: Serial number
};

// USB Device Descriptor
static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0130,         // USB 1.3.0
    .bDeviceClass = 0x00,     // Class defined at interface level
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = DEV_VID,      // Logitech VID
    .idProduct = DEV_PID,     // G27 PID
    .bcdDevice = 0x0100,      // Device release v1.0
    .iManufacturer = 1,
    .iProduct = 2,
    .iSerialNumber = 3,
    .bNumConfigurations = 1
};

// Callback function for force feedback commands
void process_ffb_command(const uint8_t *buffer, uint16_t bufsize)
{
    // Make sure we have at least one byte for the command
    if (bufsize < 1) {
        ESP_LOGW(TAG, "FFB command too short: %d bytes", bufsize);
        return;
    }
    
    uint8_t cmd = buffer[0];
    
    // Log the FFB command in a readable format
    ESP_LOGI(TAG, "FFB Command 0x%02X received, %d bytes: [%02X %02X %02X %02X %02X %02X %02X]", 
           cmd, bufsize,
           bufsize > 0 ? buffer[0] : 0,
           bufsize > 1 ? buffer[1] : 0,
           bufsize > 2 ? buffer[2] : 0,
           bufsize > 3 ? buffer[3] : 0,
           bufsize > 4 ? buffer[4] : 0,
           bufsize > 5 ? buffer[5] : 0,
           bufsize > 6 ? buffer[6] : 0);
    
    // Process different FFB commands
    switch (cmd) {
        case FFB_CMD_AUTOCENTER:
            if (bufsize >= 3) {
                uint8_t enable = buffer[1];
                uint8_t strength = buffer[2];
                ESP_LOGI(TAG, "FFB: Autocenter %s, strength %d", 
                         enable ? "ON" : "OFF", strength);
                // TODO: Implement autocenter effect
            }
            break;
            
        case FFB_CMD_CONSTANT:
            if (bufsize >= 3) {
                int8_t force = (int8_t)buffer[1]; // Force direction and magnitude
                uint8_t duration = buffer[2];     // Duration in 10ms units
                ESP_LOGI(TAG, "FFB: Constant force %d, duration %d0ms", force, duration);
                // TODO: Implement constant force effect
            }
            break;
            
        case FFB_CMD_SPRING:
        case FFB_CMD_DAMPER:
        case FFB_CMD_FRICTION:
        case FFB_CMD_PERIODIC:
            ESP_LOGI(TAG, "FFB: Command 0x%02X not yet implemented", cmd);
            // TODO: Implement other effects
            break;
            
        default:
            ESP_LOGW(TAG, "FFB: Unknown command 0x%02X", cmd);
            break;
    }
}

// Task to generate wheel values based on time
void g27_wheel_task(void *pvParameters)
{
    const float period_ms = 5000.0f;  // Full rotation period in milliseconds (5 seconds)
    
    // Initialize the report structure
    memset(&wheel_report, 0, sizeof(wheel_report));
    
    while (1) {
        // Calculate angle based on time
        int64_t time_us = esp_timer_get_time();
        float time_ms = time_us / 1000.0f;
        float angle = (2.0f * M_PI * fmodf(time_ms, period_ms)) / period_ms;
        
        // Calculate wheel position (14 bits: 6 in LSB, 8 in MSB)
        uint16_t wheel_pos = 0x1FFF + (int16_t)(sinf(angle) * 0x1FFE);
        wheel_report.axis_wheel_msb = (wheel_pos >> 6) & 0xFF;
        wheel_report.axis_wheel_lsb6_and_btns2 = wheel_pos & 0x3F;
        
        // Other axes: center at 0x7F, amplitude of 0x60
        wheel_report.axis_throttle = 0x7F + (int8_t)(sinf(angle) * 0x7E);
        wheel_report.axis_brake = 0x7F + (int8_t)(sinf(angle + M_PI/3) * 0x7E);
        wheel_report.axis_clutch = 0x7F + (int8_t)(sinf(angle + 2*M_PI/3) * 0x7E);
        
        // Send the report if USB is ready
        if (tud_hid_ready()) {
            tud_hid_report(DEV_REPORT_ID, &wheel_report, sizeof(wheel_report));
        }
        
        // Run at 10ms intervals (100Hz report rate)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// HID callbacks required by TinyUSB
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, 
                              hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    // If we get a GET_REPORT request, send the current wheel state
    if (report_type == HID_REPORT_TYPE_INPUT) {
        ESP_LOGI(TAG, "GET_REPORT request received for report ID %d, type %d", report_id, report_type);
        memcpy(buffer, &wheel_report, sizeof(wheel_report));
        return sizeof(wheel_report);
    }
    
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                          hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize)
{
    // This handles force feedback data from the host
    if (report_type == 0) {
        // Process Force Feedback commands
        ESP_LOGI(TAG, "Force Feedback data received, %d bytes: [%02X %02X %02X %02X %02X %02X %02X]", 
               bufsize,
               bufsize > 0 ? buffer[0] : 0,
               bufsize > 1 ? buffer[1] : 0,
               bufsize > 2 ? buffer[2] : 0,
               bufsize > 3 ? buffer[3] : 0,
               bufsize > 4 ? buffer[4] : 0,
               bufsize > 5 ? buffer[5] : 0,
               bufsize > 6 ? buffer[6] : 0);
        
        // Process the force feedback commands
        process_ffb_command(buffer, bufsize);
    } else if (report_type == HID_REPORT_TYPE_FEATURE) {
        ESP_LOGI(TAG, "HID Feature report received, ID %d, %d bytes", report_id, bufsize);
        // You could handle feature reports here if needed
    } else {
        ESP_LOGI(TAG, "Unsupported SET_REPORT: ID %d, type %d, size %d", 
                 report_id, report_type, bufsize);
    }
}

// Return the HID report descriptor
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    return hid_report_descriptor;
}

void app_main(void)
{
    ESP_LOGI(TAG, "G27 Racing Wheel Emulation Example");
    
    // Initialize TinyUSB
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
    };
    
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    
    // Create task for wheel updates
    xTaskCreate(g27_wheel_task, "g27_task", 4096, NULL, 5, NULL);
}

