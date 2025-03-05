#pragma once

#include <stdint.h>

/*
  ## VID/PID
    046d  Logitech, Inc.
    c298  Driving Force Pro
    c299  G25 Racing Wheel
    c29b  G27 Racing Wheel
    c24f  G29 Driving Force Racing Wheel [PS3]
    c260  G29 Driving Force Racing Wheel [PS4]
*/

// Device constants
#define DEV_VID (0x046d)
#define DEV_PID (0xc29b)
#define DEV_PRODUCT_NAME "G27 Racing Wheel"
#define DEV_MANUFACTURER_NAME "Logitech, Inc."
#define DEV_REPORT_ID (0)
#define DEV_REPORT_SIZE (11)
#define DEV_FFB_REQUEST_SIZE (7)

// Timing constants
#define DT_REPORT_MS 10  // HID report time period (increased from 2ms to 10ms)
#define DT_MEASURE_US 800  // based on loop() iteration time measurements

// Motor pins
#define MOT_A 11
#define MOT_B 12

// HID descriptor compatible with G27
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