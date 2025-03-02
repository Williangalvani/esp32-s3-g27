#include "WheelController.h"
#include <Arduino.h>

WheelController::WheelController()
  : status{},
    ffb_controller(0x1fff, 0x3fff) {
  hid.addDevice(this, sizeof(hid_report_descriptor));
  status.status.buttons_0 = 0x08;
}

void WheelController::init() {
  motor_controller.init();
  hid.begin();
}

void WheelController::update_ffb() {
    // Add debug message to show we're updating FFB
    static uint32_t last_update_debug = 0;
    if (millis() - last_update_debug >= 50) {  // Print update debug info every 50ms
        last_update_debug = millis();
        // Serial.println(">force: " + String(ffb_controller.get_force()));
    }
    
    ffb_controller.update(status.status.get_axis_wheel_14bit());
    int16_t f = this->ffb_controller.get_force();

    bool dir_ccw = f > 0x7f;
    uint8_t force_magnitude = abs(f - 0x7f) << 1;
    if (force_magnitude > 0xf0) force_magnitude = 0xf0;

    if (force_magnitude > 0) {
        motor_controller.move(dir_ccw);
    } else {
        motor_controller.stop();
    }

    // Debug output
    static uint32_t last_debug = 0;
    if (millis() - last_debug >= 1000) {  // Print debug info once per second
        last_debug = millis();
        Serial.print("FFB Force: ");
        Serial.print(f);
        Serial.print(", Magnitude: ");
        Serial.print(force_magnitude);
        Serial.print(", Direction: ");
        Serial.println(dir_ccw ? "CCW" : "CW");
    }
}

void WheelController::update_axes() {
  update_axis_wheel();
  
  // Randomly toggle button 0 every ~500ms to verify HID updates
  static uint32_t last_toggle = 0;
  if (millis() - last_toggle >= 500) {
    last_toggle = millis();
    // Randomly set or clear bit 0 of buttons_0
    if (status.status.buttons_0 == 0x08) {
      status.status.buttons_0 = 0x00;
    } else {
      status.status.buttons_0 = 0x08;
    }
    
    // Serial.println(">pos:" + String(motor_controller.get_encoder_count()));
  }
}

void WheelController::update_axis_wheel() {
  status.status.set_axis_wheel_float(motor_controller.get_position_zero_centered(), true);
}

void WheelController::sendState() {
  if (hid.ready()) {
    // Implement retry mechanism for SendReport
    const int MAX_RETRIES = 3;
    int retries = 0;
    bool success = false;
    
    while (!success && retries < MAX_RETRIES) {
      // Try to send the report
      success = hid.SendReport(DEV_REPORT_ID, status.bytes, sizeof(status.bytes), 20);
      
      if (!success) {
        retries++;
        // Only log the first retry to avoid spamming the console
        if (retries == 1) {
          Serial.print("SendReport failed, retrying (");
          Serial.print(retries);
          Serial.print("/");
          Serial.print(MAX_RETRIES);
          Serial.println(")");
        }
        // Small delay to allow USB stack to recover
        delay(5);
      }
    }
    
    // Log if all retries failed
    if (!success && retries == MAX_RETRIES) {
      static uint32_t last_error_log = 0;
      // Only log errors once per second to avoid spamming
      if (millis() - last_error_log >= 1000) {
        last_error_log = millis();
        Serial.println("SendReport failed after maximum retries");
      }
    }
  }
}

uint16_t WheelController::_onGetDescriptor(uint8_t *buffer) {
  Serial.println("onGetDescriptor");
  memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
  return sizeof(hid_report_descriptor);
}

uint16_t WheelController::_onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) {
  Serial.println("onGetFeature");
  // Not implemented
  return 0;
}

void WheelController::_onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
  Serial.println("onSetFeature");
  if (len != sizeof(FfbRequest)) return;
  ffb_controller.apply_force(*((const FfbRequest *)buffer));
  Serial.print("[IN]["); Serial.print(report_id); Serial.print("] ");
  for (uint16_t i = 0; i < len; ++i)
  {
    if (buffer[i] < 0x10) Serial.print(0);
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void WheelController::_onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
  // Serial.println("onOutput " + String(report_id) + " " + String(len));
  // print raw buffer
  // for (uint16_t i = 0; i < len; ++i) {
  //   if (buffer[i] < 0x10) Serial.print(0);
  //   Serial.print(buffer[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println();
  
  // Handle force feedback commands sent as output reports
  if (len == sizeof(FfbRequest)) {
    // Serial.println("Processing force feedback command from output report");
    
    // Create a copy of the buffer that we can modify for translation
    uint8_t translated_buffer[sizeof(FfbRequest)];
    memcpy(translated_buffer, buffer, sizeof(FfbRequest));
    
    // Check the command byte
    uint8_t cmd_byte = buffer[0];
    bool translated = false;
    
    // Special handling for pylinuxwheel's specific command format
    // The high nibble (0xF_) seems to be used for force slots, and the low nibble for the command
    if ((cmd_byte & 0xF0) == 0xF0) {
      // Extract the actual command from the low nibble
      uint8_t actual_cmd = cmd_byte & 0x0F;
      
      Serial.print("Detected pylinuxwheel command: 0x");
      Serial.print(cmd_byte, HEX);
      Serial.print(", actual command: 0x");
      Serial.println(actual_cmd, HEX);
      
      // Handle specific commands
      if (actual_cmd == 0x8) { // 0xF8 - Appears to be a constant force command
        Serial.println("Translating to DOWNLOAD_AND_PLAY_FORCE with CONSTANT");
        
        // Translate to proper Logitech protocol command
        translated_buffer[0] = 0x11; // DOWNLOAD_AND_PLAY_FORCE (0x01) with force slot 0 enabled (0x10)
        translated_buffer[1] = 0x00; // CONSTANT force type
        
        // The force value is in the second byte of the original command
        // We need to copy it to the appropriate location for a CONSTANT force
        translated_buffer[2] = buffer[1]; // Force value
        
        Serial.print("Force value: ");
        Serial.println((int8_t)buffer[1]);
        
        translated = true;
      }
      else if (actual_cmd == 0x5) { // 0xF5 - Appears to be DEFAULT_SPRING_OFF
        Serial.println("Translating to DEFAULT_SPRING_OFF");
        
        // This is already DEFAULT_SPRING_OFF (0x05), but we need to clear the high nibble
        translated_buffer[0] = 0x05;
        
        translated = true;
      }
      // Add more command translations as needed
    }
    // Special handling for the 0x14 command (DEFAULT_SPRING_ON)
    else if (cmd_byte == 0x14) {
      Serial.println("Detected DEFAULT_SPRING_ON command (0x14)");
      
      // This is already DEFAULT_SPRING_ON with force slot 0, but let's make sure
      // it's properly formatted for the Logitech protocol
      translated_buffer[0] = 0x04; // DEFAULT_SPRING_ON
      
      translated = true;
    }
    // Original evdev translations
    else if (cmd_byte == 96) { // FF_GAIN
      Serial.println("Detected FF_GAIN command");
      // Convert to a format the wheel understands
      // For now, we'll just pass it through
    } 
    else if (cmd_byte == 97) { // FF_AUTOCENTER
      Serial.println("Detected FF_AUTOCENTER command");
      if (buffer[1] > 0) {
        translated_buffer[0] = 0x04; // DEFAULT_SPRING_ON
      } else {
        translated_buffer[0] = 0x05; // DEFAULT_SPRING_OFF
      }
      translated = true;
    }
    else if (cmd_byte == 82) { // FF_CONSTANT
      Serial.println("Detected FF_CONSTANT command");
      translated_buffer[0] = 0x11; // DOWNLOAD_AND_PLAY_FORCE (0x01) with force slot 0 enabled (0x10)
      translated_buffer[1] = 0x00; // CONSTANT force type
      translated = true;
    }
    else if (cmd_byte == 83) { // FF_SPRING
      Serial.println("Detected FF_SPRING command");
      translated_buffer[0] = 0x11; // DOWNLOAD_AND_PLAY_FORCE (0x01) with force slot 0 enabled (0x10)
      translated_buffer[1] = 0x01; // SPRING force type
      translated = true;
    }
    else if (cmd_byte == 85) { // FF_DAMPER
      Serial.println("Detected FF_DAMPER command");
      translated_buffer[0] = 0x11; // DOWNLOAD_AND_PLAY_FORCE (0x01) with force slot 0 enabled (0x10)
      translated_buffer[1] = 0x02; // DAMPER force type
      translated = true;
    }
    
    // Print the translated buffer for debugging
    Serial.print("Translated buffer: ");
    for (uint16_t i = 0; i < len; ++i) {
      if (translated_buffer[i] < 0x10) Serial.print(0);
      Serial.print(translated_buffer[i], HEX);
      Serial.print(" ");
    }
    Serial.println();
    
    if (translated) {
      Serial.println("Using translated command");
    } else {
      Serial.println("Using original command (no translation needed)");
    }
    
    // Apply the translated force
    ffb_controller.apply_force(*((const FfbRequest *)translated_buffer));
  }
} 