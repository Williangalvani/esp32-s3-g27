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
#include <algorithm> // For std::min, std::max
#include "wheel_controller.h"  // Include the wheel controller header
#include "ffbTypes.h"
#include "ffbController.h"
#include "nvs_flash.h"
#include "leds.h"  // Include our new LED controller header
#include "buttons.h" // Include our Button controller header
#include "shared_pins.h" // Include shared pins header
#include "driver/adc.h" // Include ADC driver
#include "esp_adc_cal.h" // Include ADC calibration
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

#define constrain(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
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
    uint8_t axis_throttle;           // 8 bits (throttle)
    uint8_t axis_brake;              // 8 bits (brake)
    uint8_t axis_clutch;             // 8 bits (clutch)
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

// Button controllers for main buttons and shifter buttons
ButtonController button_controller;
// Use SHIFTER_BUTTONS_CLK_PIN as a clock (true for last parameter)
ButtonController shifter_button_controller(SHIFTER_BUTTONS_PL_PIN, SHIFTER_BUTTONS_CLK_PIN, SHIFTER_BUTTONS_Q7_PIN, SHIFTER_NUM_BUTTONS, true);

// Pedal ADC pins
#define PEDAL_THROTTLE_PIN ADC1_CHANNEL_3 // GPIO4 = ADC1 channel 3
#define PEDAL_BRAKE_PIN    ADC1_CHANNEL_4 // GPIO5 = ADC1 channel 4
#define PEDAL_CLUTCH_PIN   ADC1_CHANNEL_5 // GPIO6 = ADC1 channel 5
#define SHIFTER_X_PIN      ADC1_CHANNEL_8 // GPIO9 = ADC1 channel 8
#define SHIFTER_Y_PIN      ADC1_CHANNEL_9 // GPIO10 = ADC1 channel 9

// ADC attenuation
#define ADC_ATTEN ADC_ATTEN_DB_11 // 0-3.3V

// Pedal smoothing filter
#define PEDAL_FILTER_SAMPLES 4
static int throttle_values[PEDAL_FILTER_SAMPLES] = {0};
static int brake_values[PEDAL_FILTER_SAMPLES] = {0};
static int clutch_values[PEDAL_FILTER_SAMPLES] = {0};
static int pedal_sample_index = 0;

// Simple moving average filter for pedal values
static int pedal_filter(int new_value, int* filter_array) {
    filter_array[pedal_sample_index] = new_value;
    
    // Calculate average
    int sum = 0;
    for (int i = 0; i < PEDAL_FILTER_SAMPLES; i++) {
        sum += filter_array[i];
    }
    return sum / PEDAL_FILTER_SAMPLES;
}

// Task to generate wheel values based on actual encoder position
void g27_wheel_task(void *pvParameters)
{
    // Initialize the report structure
    memset(&wheel_report, 0, sizeof(wheel_report));
    
    while (1) {
        // Get the actual wheel position from the encoder
        float position = wheel_controller.get_position_zero_centered() * (900/wheel_controller.wheel_range);
        position = std::clamp(position, -1.0f, 1.0f);
        // Calculate wheel position for report (14 bits: 6 in LSB, 8 in MSB)
        // Scale position from -1.0...1.0 to full wheel range
        uint16_t wheel_pos = 0x1FFF + (int16_t)(position * 0x1FFE);
        wheel_report.axis_wheel_msb = (wheel_pos >> 6) & 0xFF;
        wheel_report.axis_wheel_lsb6_and_btns2 = wheel_pos & 0b11111100;
        
        // Read pedal values from ADC inputs
        int throttle_adc = adc1_get_raw(PEDAL_THROTTLE_PIN);
        int brake_adc = adc1_get_raw(PEDAL_BRAKE_PIN);
        int clutch_adc = adc1_get_raw(PEDAL_CLUTCH_PIN);
        int shifter_x_adc = adc1_get_raw(SHIFTER_X_PIN);
        int shifter_y_adc = adc1_get_raw(SHIFTER_Y_PIN);
        // Apply smoothing filter
        throttle_adc = pedal_filter(throttle_adc, throttle_values);
        brake_adc = pedal_filter(brake_adc, brake_values);
        clutch_adc = pedal_filter(clutch_adc, clutch_values);
        
        // Update filter index for next sample
        pedal_sample_index = (pedal_sample_index + 1) % PEDAL_FILTER_SAMPLES;
        
        // Convert to voltage if calibration is available
        uint32_t throttle_mv = 0, brake_mv = 0, clutch_mv = 0;
        // Map ADC values (0-4095) to pedal range (0-255)
        float throttle_val = -0.1 + 1.2 * (float)(throttle_adc * 255 / 4095);
        float brake_val = 1.2* (float)(brake_adc * 255 / 4095);
        float clutch_val = 1.2* (float)(clutch_adc * 255 / 4095);
        
        wheel_report.axis_throttle = constrain(throttle_val, 0, 255);
        wheel_report.axis_brake = constrain(brake_val, 0, 255);
        wheel_report.axis_clutch = constrain(clutch_val, 0, 255);

        
        // Read button states from our button controllers
        uint16_t button_state = button_controller.get_all_buttons();
        uint16_t shifter_button_state = shifter_button_controller.get_all_buttons();
        
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
        
        // Shifter buttons
        bool pov_up = shifter_button_state & 1;
        bool pov_down = shifter_button_state & 2;
        bool pov_left = shifter_button_state & 4;
        bool pov_right = shifter_button_state & 8;

        bool button_down = shifter_button_state & 16;
        bool button_left = shifter_button_state & 32;
        bool button_right = shifter_button_state & 64;
        bool button_up = shifter_button_state & 128;

        bool first_red_button = shifter_button_state & 256;
        bool second_red_button = shifter_button_state & 1024;
        bool third_red_button = shifter_button_state & 2048; 
        bool fourth_red_button = shifter_button_state & 512;
        
        bool shifter_down = shifter_button_state & 16384;

        // D-Pad/Hat switch encoding
        uint8_t hat_switch;
        if (!pov_up && !pov_down && !pov_left && !pov_right) {
            hat_switch = 8; // Released/centered
        } else if (pov_up && !pov_down && !pov_left && !pov_right) {
            hat_switch = 0; // North
        } else if (pov_up && !pov_down && !pov_left && pov_right) {
            hat_switch = 1; // Northeast
        } else if (!pov_up && !pov_down && !pov_left && pov_right) {
            hat_switch = 2; // East
        } else if (!pov_up && pov_down && !pov_left && pov_right) {
            hat_switch = 3; // Southeast
        } else if (!pov_up && pov_down && !pov_left && !pov_right) {
            hat_switch = 4; // South
        } else if (!pov_up && pov_down && pov_left && !pov_right) {
            hat_switch = 5; // Southwest
        } else if (!pov_up && !pov_down && pov_left && !pov_right) {
            hat_switch = 6; // West
        } else if (pov_up && !pov_down && pov_left && !pov_right) {
            hat_switch = 7; // Northwest
        } else {
            hat_switch = 8; // Default to released for any other combination
        }
        

        int neutral_x = 2700;
        int neutral_y = 2700;

        int shifter_x = shifter_x_adc - neutral_x;
        int shifter_y = shifter_y_adc - neutral_y;

        shifter_x = std::clamp(shifter_x, -1000, 1000);

        shifter_y = std::clamp(shifter_y, -1000, 1000);

        // Define thresholds for more precise detection
        const int X_THRESHOLD = 400;  // Threshold for left/right detection
        const int Y_THRESHOLD = 400;  // Threshold for up/down detection
        const int NEUTRAL_ZONE = 200; // Smaller threshold for detecting neutral position
        
        // Shifter layout for standard H-pattern:
        //
        //    Y-
        //    ^
        //    |
        // 2     1
        // |     |
        // 4-----3---> X-
        // |     |
        // 6/R   5
        // |
        // R (with push down)
        //
        // Check if shifter is in neutral zone
        bool in_neutral_x = abs(shifter_x) < NEUTRAL_ZONE;
        bool in_neutral_y = abs(shifter_y) < NEUTRAL_ZONE;
        bool in_neutral = in_neutral_x && in_neutral_y;
        
        // Top row (gears 1-2)
        bool gear1 = !in_neutral && shifter_y < -Y_THRESHOLD && shifter_x > X_THRESHOLD;   // Top-right
        bool gear2 = !in_neutral && shifter_y < -Y_THRESHOLD && shifter_x < -X_THRESHOLD;  // Top-left
        
        // Middle row (gears 3-4)
        bool gear3 = !in_neutral && abs(shifter_y) <= Y_THRESHOLD && shifter_x > X_THRESHOLD;   // Middle-right
        bool gear4 = !in_neutral && abs(shifter_y) <= Y_THRESHOLD && shifter_x < -X_THRESHOLD;  // Middle-left
        
        // Bottom row (gears 5-6)
        bool gear5 = !in_neutral && shifter_y > Y_THRESHOLD && shifter_x > X_THRESHOLD;   // Bottom-right without push down
        bool gear6 = !in_neutral && shifter_y > Y_THRESHOLD && shifter_x < -X_THRESHOLD && !shifter_down;  // Bottom-left without push down
        
        // Reverse (bottom-left with shifter pushed down)
        bool reverse = !in_neutral && shifter_down && shifter_y > Y_THRESHOLD && shifter_x < -X_THRESHOLD;

        // No need to check for conflicts since gear6 and reverse are in the same position
        // but explicitly differentiated by the shifter_down state
        
        // Debugging output for shifter positions
        static int counter = 0;
        if (counter % 100 == 0) {
            ESP_LOGI(TAG, "Shifter: X=%d, Y=%d, Down=%d | Neutral=%d | Gears: 1=%d, 2=%d, 3=%d, 4=%d, 5=%d, 6=%d, R=%d, Buttons: %d", 
                     shifter_x, shifter_y, shifter_down, in_neutral,
                     gear1, gear2, gear3, gear4, gear5, gear6, reverse, shifter_button_state);
        }
        
        // nice reference: https://gimx.fr/wiki/index.php?title=G27_PS3
        wheel_report.buttons_0 = (hat_switch & 0x0F)  // Hat switch value in bits 0-3
                               | (button_down << 4)   // button 18 (shifter)
                               | (button_left << 5)   // button 19 shifter
                               | (button_right << 6)  // button 16 (shifter)
                               | (button_up << 7);    // button 17 (shifter)

        wheel_report.buttons_1 = right_paddle
                               | (left_paddle << 1)
                               | (right_button_top << 2) // Button 7 (wheel)
                               | (left_button_top << 3) // Button 8 (wheel)
                               | (second_red_button << 4) // Button 3 (shifter)
                               | (first_red_button << 7) // Button 2 (shifter)
                               | (third_red_button << 5) // Button 4 (shifter)
                               | (fourth_red_button << 6); // Button 1 (shifter)
        wheel_report.buttons_2 = 0
                               | (gear1 << 0) // nothing
                               | (gear2 << 1) // nothing
                               | (gear3 << 2) // nothing
                               | (gear4 << 3) // nothing
                               | (gear5 << 4) // nothing
                               | (gear6 << 5) // nothing
                               | (right_button_center << 6) // Button 20 (wheel)
                               | (right_button_bottom << 7); // Button 22 (wheel) 
         
        wheel_report.axis_wheel_lsb6_and_btns2 = wheel_report.axis_wheel_lsb6_and_btns2 | (left_button_center << 0)
          | (left_button_bottom << 1);
        

        wheel_report.misc = 0b10011100
                         | (shifter_down << 0)
                         | (reverse << 6);

        // Send the report if USB is ready
        if (tud_hid_ready()) {
            tud_hid_report(DEV_REPORT_ID, &wheel_report, sizeof(wheel_report));
        }
        
        // Run at 10ms intervals (100Hz report rate)
        vTaskDelay(pdMS_TO_TICKS(10));
        counter++;
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
    uint8_t param4 = buffer[5];
    cmd = cmd & 0x0F;
    uint8_t force_mask = buffer[0] >> 4;

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
    {
      uint8_t force_type = param0;
      uint8_t force_mask = buffer[0] >> 4;
      ffb_controller.apply_forces(force_mask, (EnumForceType)force_type, buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
      // ESP_LOGI(TAG, "DOWNLOAD_AND_PLAY_FORCE: %d %d %d %d %d %d", buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5], buffer[6]);
      ffb_controller.play_force(force_mask);
      // ESP_LOGI(TAG, "DOWNLOAD_AND_PLAY_FORCE: %d", param0);
      break;
    }
    case EnumFfbCmd::PLAY_FORCE:
    {
      uint8_t force_mask = buffer[0] >> 4;
      ffb_controller.play_force(force_mask);
      ESP_LOGI(TAG, "PLAY_FORCE: %d", force_mask);
      break;
    }
    case EnumFfbCmd::STOP_FORCE:
    {
      uint8_t force_mask = buffer[0] >> 4;
      ffb_controller.stop_force(force_mask);
      ESP_LOGI(TAG, "STOP_FORCE: %d", force_mask);
      break;
    }
    case EnumFfbCmd::REFRESH_FORCE:
      ESP_LOGI(TAG, "REFRESH_FORCE: %d", force_mask);
      break;
    case EnumFfbCmd::FIXED_TIME_LOOP:
      ESP_LOGI(TAG, "FIXED_TIME_LOOP: %d", force_mask);
      break;
    case EnumFfbCmd::SET_DEAD_BAND:
      ESP_LOGI(TAG, "SET_DEAD_BAND");
      break;
    case EnumFfbCmd::EXTENDED_COMMAND:
    {
      uint8_t extended_cmd = buffer[1];
      switch ((EnumExtendedCommand)extended_cmd) {
        case EnumExtendedCommand::CHANGE_MODE_TO_DRIVING_FORCE_PRO:
          ESP_LOGI(TAG, "CHANGE_MODE_TO_DRIVING_FORCE_PRO");
          break;
        case EnumExtendedCommand::CHANGE_WHEEL_RANGE_TO_200_DEGREES:
          ESP_LOGI(TAG, "CHANGE_WHEEL_RANGE_TO_200_DEGREES");
          break;
        case EnumExtendedCommand::CHANGE_WHEEL_RANGE_TO_900_DEGREES:
          ESP_LOGI(TAG, "CHANGE_WHEEL_RANGE_TO_900_DEGREES");
          break;
        case EnumExtendedCommand::CHANGE_DEVICE_MODE:
          ESP_LOGI(TAG, "CHANGE_DEVICE_MODE");
          break;
        case EnumExtendedCommand::REVERT_IDENTITY:
          ESP_LOGI(TAG, "REVERT_IDENTITY");
          break;
        case EnumExtendedCommand::SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH:
          ESP_LOGI(TAG, "SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH");
          break;
        case EnumExtendedCommand::SET_RPM_LEDS:
          // ESP_LOGI(TAG, "SET_RPM_LEDS");
          break;
        case EnumExtendedCommand::WHEEL_RANGE_CHANGE:
          {
            ESP_LOGI(TAG, "WHEEL_RANGE_CHANGE");
            uint16_t wheel_range = (param1 | param2 << 8);
            wheel_controller.set_wheel_range(wheel_range);
          }
          break;
        default:
          ESP_LOGI(TAG, "Unknown extended command: %d", extended_cmd);
          break;
      }
    }
    break;
    default:
      ESP_LOGI(TAG, "Unknown command: %d", cmd);
      // log as hex
      ESP_LOGI(TAG, "param0: %d", param0);
      ESP_LOGI(TAG, "param1: %d", param1);
      ESP_LOGI(TAG, "param2: %d", param2);
      ESP_LOGI(TAG, "param3: %d", param3);
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
    shifter_button_controller.init();
    
    // Initialize ADC for pedal inputs
    ESP_LOGI(TAG, "Initializing ADC for pedal inputs");
    
    // Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(PEDAL_THROTTLE_PIN, ADC_ATTEN);
    adc1_config_channel_atten(PEDAL_BRAKE_PIN, ADC_ATTEN);
    adc1_config_channel_atten(PEDAL_CLUTCH_PIN, ADC_ATTEN);
    adc1_config_channel_atten(SHIFTER_X_PIN, ADC_ATTEN);
    adc1_config_channel_atten(SHIFTER_Y_PIN, ADC_ATTEN);
    
    // // Register button event callback
    // button_controller.register_callback(button_event_handler);
    
    // Start controller tasks
    led_controller.start_led_task();
    button_controller.start_button_task();
    
    // Add a small delay before starting the second button task
    vTaskDelay(pdMS_TO_TICKS(100));
    
    shifter_button_controller.start_button_task();
    
    // Log GPIO pin mappings for verification
    ESP_LOGI(TAG, "Main buttons: PL=%d, CLK_INH=%d, Q7=%d", DEFAULT_BUTTONS_PL_PIN, DEFAULT_BUTTONS_CLK_INH_PIN, DEFAULT_BUTTONS_Q7_PIN);
    ESP_LOGI(TAG, "Shifter buttons: PL=%d, CLK_INH=%d, Q7=%d", SHIFTER_BUTTONS_PL_PIN, SHIFTER_BUTTONS_CLK_PIN, SHIFTER_BUTTONS_Q7_PIN);
    
    // GPIO pin check for ESP32-S3 compatibility
    if (SHIFTER_BUTTONS_PL_PIN > 48 || SHIFTER_BUTTONS_CLK_PIN > 48 || SHIFTER_BUTTONS_Q7_PIN > 48) {
        ESP_LOGW(TAG, "WARNING: ESP32-S3 has GPIO pins 0-48. Check your shifter pin assignments!");
    }
    
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

