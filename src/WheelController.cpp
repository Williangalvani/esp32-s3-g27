#include "WheelController.h"
#include <Arduino.h>
#include "FfbTypes.h"
#include "USB.h"
#include "class/hid/hid_device.h"
#include "tusb.h"  // Direct TinyUSB access

// TinyUSB endpoint definitions - THIS IS CRITICAL
#define EPNUM_HID_IN    0x81  // HID IN endpoint address - Device TO Host (0x80 | 1)
#define EPNUM_HID_OUT   0x01  // HID OUT endpoint address - Host TO Device

WheelController::WheelController()
    : status{},
      ffb_controller(motor_controller)
{
  motor_controller.init();
  
  // Add HID device with our custom descriptor
  if (!hid.addDevice(this, sizeof(hid_report_descriptor))) {
    Serial.println("Failed to add HID device");
  } else {
    Serial.println("HID device added successfully");
  }
  
  status.status.buttons_0 = 0x08;
}

void WheelController::init()
{
  initialized = false;
  
  // Set up HID and begin
  hid.begin();
  Serial.println("HID initialized");
  status.status.init();
  initialized = true;
  
  // USB debugging
  Serial.println("USB Endpoints configured with custom descriptor:");
  Serial.print("HID IN Endpoint (Device->Host): 0x");
  Serial.println(EPNUM_HID_IN, HEX);
  Serial.print("HID OUT Endpoint (Host->Device): 0x");
  Serial.println(EPNUM_HID_OUT, HEX);
  
  // Send a test report on the IN endpoint to properly initialize it
  if (tud_hid_ready()) {
    // Pre-emptively send an initial report to ensure the IN endpoint is registered first
    tud_hid_report(DEV_REPORT_ID, status.bytes, sizeof(status.bytes));
    Serial.println("Sent initial HID report on IN endpoint (0x81)");
  }
}

void WheelController::update_ffb()
{
  // Update the force feedback controller with the current wheel position
  ffb_controller.update(motor_controller.get_position_zero_centered());
  // Get the current force value for debugging
  float f = this->ffb_controller.get_force();
  motor_controller.move(f);
}

void WheelController::update_axes()
{
  update_axis_wheel();
}

void WheelController::update_axis_wheel()
{
  status.status.axis_clutch = 0x80 + 100 * sin(millis() / 1000.0);
  status.status.axis_brake = 0x80 + 100 * sin(millis() / 1000.0);
  status.status.axis_throttle = 0x80 + 100 * sin(millis() / 1000.0);
  status.status.buttons_0 = (millis() / 1000) % 2 ? 0x08 : 0x01;
  status.status.set_axis_wheel_float(motor_controller.get_position_zero_centered_normalized(), true);
}

void WheelController::sendState() {
  // This function sends device state TO the host
  // Must use the IN endpoint (device->host)
  if (hid.ready()) {
    // Use direct TinyUSB API to send data on the IN endpoint
    if (tud_hid_ready()) {
      // Send data from device to host on IN endpoint (0x81)
      if (!tud_hid_report(DEV_REPORT_ID, status.bytes, sizeof(status.bytes))) {
        Serial.println("Failed to send state to host on IN endpoint");
      }
    }
  }
}

uint16_t WheelController::_onGetDescriptor(uint8_t *buffer)
{
  Serial.println("HOST->DEVICE: Get Descriptor request");
  memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
  return sizeof(hid_report_descriptor);
}

uint16_t WheelController::_onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len)
{
  Serial.println("HOST->DEVICE: Get Feature request");
  
  // Fill buffer with current status to respond to feature requests
  if (len >= sizeof(status.bytes)) {
    memcpy(buffer, status.bytes, sizeof(status.bytes));
    return sizeof(status.bytes);
  } else if (len > 0) {
    // At least return something if buffer is too small
    buffer[0] = report_id;
    return 1;
  }
  
  return 0;
}

void WheelController::_onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len)
{
  Serial.println("HOST->DEVICE: Set Feature request");
  Serial.print("[Feature][");
  Serial.print(report_id);
  Serial.print("] ");
  for (uint16_t i = 0; i < len; ++i)
  {
    if (buffer[i] < 0x10)
      Serial.print(0);
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Must respond directly on IN endpoint (device->host)
  if (tud_hid_ready()) {
    tud_hid_report(DEV_REPORT_ID, status.bytes, sizeof(status.bytes));
  }
}

void printMask(uint8_t mask)
{
  for (int i = 0; i < 4; i++)
  {
    Serial.print((mask >> i) & 1);
    Serial.print(" ");
  }
  Serial.println();
}

void WheelController::_onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len)
{
  if (!initialized) {
    return;
  }
  
  // Store last command to avoid excessive logging of repetitive commands
  static uint8_t last_cmd = 0xFF;
  static uint32_t last_cmd_time = 0;
  static uint32_t repetitive_cmd_count = 0;
  
  bool is_repetitive = false;
  
  // Check if this is a repetitive command
  if (len >= 1 && buffer[0] == last_cmd) {
    repetitive_cmd_count++;
    is_repetitive = true;
    
    // Only log repetitive commands periodically to avoid overwhelming Serial
    if (millis() - last_cmd_time < 1000) {
      // Skip detailed logging for repetitive commands within 1 second
      // Just acknowledge with response on IN endpoint
      if (tud_hid_ready()) {
        // CRITICAL: Always respond on IN endpoint (0x81) for Windows compatibility
        tud_hid_report(DEV_REPORT_ID, status.bytes, sizeof(status.bytes));
      }
      
      // Still process the command but skip detailed logging
      goto process_command;
    } else {
      // Log summary of repetitive commands
      Serial.print("Repeated command (");
      Serial.print(repetitive_cmd_count);
      Serial.println(" times)");
      repetitive_cmd_count = 0;
    }
  } else if (len >= 1) {
    // New command, reset counter
    last_cmd = buffer[0];
    repetitive_cmd_count = 0;
  }
  
  // Update timestamp for command logging
  last_cmd_time = millis();
  
  // Standard logging for non-repetitive commands
  Serial.println("HOST->DEVICE: Received output on OUT endpoint");
  Serial.print("Report ID: ");
  Serial.print(report_id);
  Serial.print(", Data: ");
  for (int i = 0; i < min(len, (uint16_t)8); i++) {
    if (buffer[i] < 0x10) Serial.print("0");
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // CRITICAL: Always respond on IN endpoint (0x81) for Windows compatibility
  if (tud_hid_ready()) {
    tud_hid_report(DEV_REPORT_ID, status.bytes, sizeof(status.bytes));
  }
  
process_command:
  // Process force feedback commands
  if (report_id == 0 && len >= 1)
  {
    uint8_t cmd_byte = buffer[0];

    // Check for pylinuxwheel commands
    cmd_byte &= 0x0F;
    uint8_t forces_mask = buffer[0] & 0x10;
    
    // Only log non-repetitive commands or periodic summaries
    bool fully_parsed = false;
    if (!is_repetitive || millis() - last_cmd_time >= 1000) {
      if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_FORCE)
      { // FF_CONSTANT
        Serial.println("DOWNLOAD_FORCE");
      }
      else if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE)
      { // FF_CONSTANT
        Serial.println("DOWNLOAD_AND_PLAY_FORCE");
      }
      // Similar pattern for other commands...
      else if (buffer[0] == (uint8_t)EnumFfbCmd::EXTENDED_COMMAND && len >= 2)
      {
        uint8_t extended_cmd = buffer[1];
        if (extended_cmd == (uint8_t)EnumExtendedCommand::SET_RPM_LEDS)
        {
          Serial.println("SET_RPM_LEDS");
          fully_parsed = true;
        }
        // Other extended commands...
      }
    }
    
    // Always process the commands even if we don't log them
    if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE)
    { // FF_CONSTANT
      ffb_controller.apply_forces((EnumForceType)buffer[1], forces_mask, buffer[2], buffer[3], buffer[4], buffer[5]);
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_ON)
    { // 0xF4 - DEFAULT_SPRING_ON
      ffb_controller.ffb_default_spring_on = true;
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_OFF)
    { // 0xF5 - DEFAULT_SPRING_OFF
      ffb_controller.ffb_default_spring_on = false;
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::SET_DEFAULT_SPRING) {
      FfbRequest* request = (FfbRequest*)buffer;
      ffb_controller.set_default_spring(request->set_default_spring.k1, request->set_default_spring.k2, request->set_default_spring.clip);
    }
    else if (buffer[0] == (uint8_t)EnumFfbCmd::EXTENDED_COMMAND && len >= 2)
    {
      uint8_t extended_cmd = buffer[1];
      
      if (extended_cmd == (uint8_t)EnumExtendedCommand::WHEEL_RANGE_CHANGE && len >= 4)
      {
        uint8_t lsb = buffer[2];
        uint8_t msb = buffer[3];
        uint16_t wheel_range = (msb << 8) | lsb;
        
        if (!is_repetitive) {
          Serial.print("WHEEL_RANGE_CHANGE: ");
          Serial.println(wheel_range);
        }
        
        ffb_controller.wheel_range_normalized = (float)wheel_range / 900.0;
      }
    }
    
    // Apply the translated force
    ffb_controller.update(motor_controller.get_position_zero_centered());
    
    // Final response on IN endpoint after processing
    if (tud_hid_ready()) {
      tud_hid_report(DEV_REPORT_ID, status.bytes, sizeof(status.bytes));
    }
  }
}