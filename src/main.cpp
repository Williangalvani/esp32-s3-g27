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
#include <stdint.h>
#include <stdlib.h>  // For atoi function
#include "wheel_controller.h"  // Include the wheel controller header
#include "ffbTypes.h"
#include "ffbController.h"
#include "nvs_flash.h"
#include "leds.h"  // Include our new LED controller header
#include "buttons.h" // Include our Button controller header
#include "shared_pins.h" // Include shared pins header
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

// First create the FFB controller
FfbController ffb_controller;

// Then create the wheel controller with a reference to the FFB controller
WheelController wheel_controller(&ffb_controller);

// Global instance of our LED controller
LedController led_controller;

ButtonController button_controller;

// Task to generate wheel values based on actual encoder position
void g27_wheel_task(void *pvParameters)
{
    // Initialize the report structure
    memset(&wheel_report, 0, sizeof(wheel_report));
    
    while (1) {
        // Get the actual wheel position from the encoder
        float position = wheel_controller.get_position_zero_centered();
        
        // Calculate wheel position for report (14 bits: 6 in LSB, 8 in MSB)
        // Scale position from -1.0...1.0 to full wheel range
        uint16_t wheel_pos = 0x1FFF + (int16_t)(position * 0x1FFE);
        wheel_report.axis_wheel_msb = (wheel_pos >> 6) & 0xFF;
        wheel_report.axis_wheel_lsb6_and_btns2 = wheel_pos & 0b11111100;
        
        // Read pedal values from analog inputs or other source
        // For now, just using simulated values
        int64_t time_us = esp_timer_get_time();
        float time_ms = time_us / 1000.0f;
        float period_ms = 5000.0f;
        float angle = (2.0f * M_PI * fmodf(time_ms, period_ms)) / period_ms;
        
        wheel_report.axis_throttle = 0x7F + (int8_t)(sinf(angle) * 0x7E);
        wheel_report.axis_brake = 0x7F + (int8_t)(sinf(angle + M_PI/3) * 0x7E);
        wheel_report.axis_clutch = 0x7F + (int8_t)(sinf(angle + 2*M_PI/3) * 0x7E);
        
        // Read button states from our button controller
        uint16_t button_state = button_controller.get_all_buttons();
        
        // Map button states to the report structure
        // first 4 buttons are hat switch, so lets start from 5th button
        
        bool left_paddle = button_state & 1;
        bool right_paddle = button_state & 2;
        bool right_button_top = button_state & 4;
        bool right_button_center = button_state & 8;
        bool right_button_bottom = button_state & 16;
        bool left_button_top = button_state & 32;
        bool left_button_center = button_state & 64;
        bool left_button_bottom = button_state & 128;
        
        wheel_report.buttons_0 = (0 << 0) // (hat?) 
                               | (0 << 1)  // (hat?)
                               | (0 << 2) // (hat?)
                               | (0 << 3) // nothing
                               | (0 << 4) // button 17 (shifter)
                               | (0 << 5) //  button 18 (shifter)
                               | (0 << 6) // button 19 shifter
                               | (0 << 7); // button 16 (shifter)

        wheel_report.buttons_1 = right_paddle
                               | (left_paddle << 1)
                               | (right_button_top << 2) // Button 7 (wheel)
                               | (left_button_top << 3) // Button 8 (wheel)
                               | (0 << 4) // Button 2 (shifter)
                               | (0 << 5) // Button 3 (shifter)
                               | (0 << 6) // Button 4 (shifter)
                               | (0 << 7); // Button 1 (shifter)
        wheel_report.buttons_2 = 0
                               | (0 << 1) // nothing
                               | (0 << 2) // nothing
                               | (0 << 3) // nothing
                               | (0 << 4) // nothing
                               | (0 << 5) // nothing
                               | (right_button_center << 6) // Button 20 (wheel)
                               | (right_button_bottom << 7); // Button 22 (wheel) 
         
         wheel_report.axis_wheel_lsb6_and_btns2 = wheel_report.axis_wheel_lsb6_and_btns2 | (left_button_center << 0)
          | (left_button_bottom << 1);
        

        // Log button state periodically (every ~1 second)
        static int log_counter = 0;
        if (++log_counter >= 100) {
            ESP_LOGI(TAG, "Button state: 0x%04x", button_state);
            log_counter = 0;
        }
        
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

void process_ffb_command(const uint8_t *buffer, uint16_t bufsize) {
    uint8_t cmd = buffer[0];
    uint8_t param0 = buffer[1];
    uint8_t param1 = buffer[2];
    uint8_t param2 = buffer[3];
    uint8_t param3 = buffer[4];
    cmd = cmd & 0x0F;

  switch ((EnumFfbCmd)cmd) {
    case EnumFfbCmd::SET_DEFAULT_SPRING:
      ESP_LOGI(TAG, "SET_DEFAULT_SPRING: %d %d %d", param0, param1, param2);
      ffb_controller.set_default_spring(param0, param1, param2);
      break;
    case EnumFfbCmd::DEFAULT_SPRING_ON:
      ESP_LOGI(TAG, "DEFAULT_SPRING_ON");
      ffb_controller.set_default_spring_enabled(true);
      break;
    case EnumFfbCmd::DEFAULT_SPRING_OFF:
      ESP_LOGI(TAG, "DEFAULT_SPRING_OFF");
      ffb_controller.set_default_spring_enabled(false);
      break;
    case EnumFfbCmd::DOWNLOAD_FORCE:
      ESP_LOGI(TAG, "DOWNLOAD_FORCE: %d", param0);
      break;
    case EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE:
      ESP_LOGI(TAG, "DOWNLOAD_AND_PLAY_FORCE: %d", param0);
      break;
    case EnumFfbCmd::PLAY_FORCE:
      ESP_LOGI(TAG, "PLAY_FORCE: %d", param0);
      break;
    case EnumFfbCmd::STOP_FORCE:
      ESP_LOGI(TAG, "STOP_FORCE: %d", param0);
      break;
    case EnumFfbCmd::REFRESH_FORCE:
      ESP_LOGI(TAG, "REFRESH_FORCE: %d", param0);
      break;
    case EnumFfbCmd::FIXED_TIME_LOOP:
      ESP_LOGI(TAG, "FIXED_TIME_LOOP: %d", param0);
      break;
    case EnumFfbCmd::SET_DEAD_BAND:
      ESP_LOGI(TAG, "SET_DEAD_BAND: %d", param0);
      break;
    case EnumFfbCmd::EXTENDED_COMMAND:
      ESP_LOGI(TAG, "EXTENDED_COMMAND: %d", param0);
      break;
  }
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                          hid_report_type_t report_type, const uint8_t *buffer, uint16_t bufsize)
{
    // This handles force feedback data from the host
    if (report_type == 0) {
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

// C++ implementation of app_main
extern "C" void cpp_app_main(void)
{
    ESP_LOGI(TAG, "G27 Racing Wheel Emulation Example");
    
    // // Initialize NVS
    // esp_err_t ret = nvs_flash_init();
    // if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    //     ESP_ERROR_CHECK(nvs_flash_erase());
    //     ret = nvs_flash_init();
    // }
    // ESP_ERROR_CHECK(ret);

    // Initialize shared pins first
    shared_pins_init();
    
    // Initialize the wheel controller
    wheel_controller.init();
    
    // Perform homing procedure to calibrate wheel range
    ESP_LOGI(TAG, "Starting wheel homing procedure");
    wheel_controller.home();
    
    // Start position monitoring
    wheel_controller.start_monitoring();
    
    // Initialize controllers
    led_controller.init();
    button_controller.init();
    
    // // Register button event callback
    // button_controller.register_callback(button_event_handler);
    
    // Start controller tasks
    led_controller.start_led_task();
    button_controller.start_button_task();
    
    // Initialize TinyUSB
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = &desc_device,
        .string_descriptor = string_desc_arr,
        .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
        .external_phy = false,
        .configuration_descriptor = desc_configuration,
    };
    
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "TinyUSB initialized");
    
    // Create task for wheel updates
    xTaskCreate(g27_wheel_task, "g27_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Wheel task started");
}

