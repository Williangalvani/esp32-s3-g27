#include "WheelController.h"
#include <Arduino.h>

WheelController::WheelController()
  : status{},
    ffb_controller(0x1fff, 0x3fff, motor_controller) {
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
    
    // Update the force feedback controller with the current wheel position
    ffb_controller.update(motor_controller.get_position_zero_centered());
    
    // Get the current force value for debugging
    float f = this->ffb_controller.get_force();

    // Debug output
    static uint32_t last_debug = 0;
    if (millis() - last_debug >= 100) {  // Print debug info once per second
        last_debug = millis();
        // Serial.println(">force: " + String(f));
        // Serial.println(">target: " + String(0.0));
        // Serial.println(">pos: " + String(motor_controller.get_position_zero_centered()));
    }
    motor_controller.move(f);
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
  // Add debug output
  Serial.print("Received output report: ID=");
  Serial.print(report_id);
  Serial.print(", Length=");
  Serial.println(len);
  
  // Print the entire buffer for debugging
  Serial.print("Buffer contents: ");
  for (uint16_t i = 0; i < len; ++i) {
    if (buffer[i] < 0x10) Serial.print(0);
    Serial.print(buffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  
  // Process force feedback commands
  if (report_id == 0 && len >= 1) {
    uint8_t cmd_byte = buffer[0];

    // Check for pylinuxwheel commands
    bool is_pylinuxwheel = false;
    if (cmd_byte >= 0xF0) {
      is_pylinuxwheel = true;
      Serial.print(cmd_byte, HEX);
      Serial.println(cmd_byte & 0x0F, HEX);
      cmd_byte &= 0x0F;
    }
    
    // Create a copy of the buffer for translation
    uint8_t translated_buffer[len];
    memcpy(translated_buffer, buffer, len);
    bool translated = false;
    
    // Translate Linux input subsystem commands to Logitech protocol commands
    if (cmd_byte == (uint8_t)EnumFfbCmd::PLAY_FORCE) { // FF_CONSTANT
      Serial.println("Detected FF_CONSTANT command");
      
      // Print the entire buffer for detailed debugging
      Serial.println("FF_CONSTANT full buffer:");
      for (uint16_t i = 0; i < len; ++i) {
        Serial.print("Byte ");
        Serial.print(i);
        Serial.print(": 0x");
        if (buffer[i] < 0x10) Serial.print("0");
        Serial.print(buffer[i], HEX);
        Serial.print(" (");
        Serial.print(buffer[i]);
        Serial.println(")");
      }
      
      // Extract and print direction
      uint16_t direction = buffer[1] | (buffer[2] << 8);
      Serial.print("Direction: 0x");
      Serial.print(direction, HEX);
      Serial.print(" (");
      Serial.print(direction);
      Serial.println(")");
      
      // Try to extract level/magnitude
      if (len >= 8) {
        // Try different possible locations for the level
        int16_t level1 = (int16_t)(buffer[3] | (buffer[4] << 8));
        Serial.print("Level (bytes 3-4): 0x");
        Serial.print(level1, HEX);
        Serial.print(" (");
        Serial.print(level1);
        Serial.println(")");
        
        int16_t level2 = (int16_t)(buffer[5] | (buffer[6] << 8));
        Serial.print("Level (bytes 5-6): 0x");
        Serial.print(level2, HEX);
        Serial.print(" (");
        Serial.print(level2);
        Serial.println(")");
        
        int16_t level3 = (int16_t)(buffer[7] | (buffer[8] << 8));
        Serial.print("Level (bytes 7-8): 0x");
        Serial.print(level3, HEX);
        Serial.print(" (");
        Serial.print(level3);
        Serial.println(")");
      }
      
      // Print the translated command we're going to send
      Serial.println("Translating to DOWNLOAD_AND_PLAY_FORCE with CONSTANT");
      
      translated_buffer[0] = 0x11; // DOWNLOAD_AND_PLAY_FORCE (0x01) with force slot 0 enabled (0x10)
      translated_buffer[1] = 0x00; // CONSTANT force type
      
      // Set force value based on direction
      uint8_t force_value = 0x7F; // Default to center (no force)
      
      // If direction indicates right (0x8000 or 0xC000), increase force
      if (direction >= 0x8000) {
        force_value = 0xBF; // Positive force (right)
        Serial.println("Direction indicates RIGHT");
      } 
      // If direction indicates left (0x0000 or 0x4000), decrease force
      else if (direction < 0x4000) {
        force_value = 0x3F; // Negative force (left)
        Serial.println("Direction indicates LEFT");
      }
      else {
        Serial.println("Direction indicates CENTER");
      }
      
      translated_buffer[2] = force_value;
      Serial.print("Setting force value to: 0x");
      Serial.println(force_value, HEX);
      
      translated = true;
    }
    // Handle pylinuxwheel specific commands
    else if (is_pylinuxwheel) {
      if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_OFF) { // 0xF5 - DEFAULT_SPRING_OFF
        Serial.println("Translating to DEFAULT_SPRING_OFF");
        
        // Print the entire buffer for detailed debugging
        Serial.println("DEFAULT_SPRING_OFF full buffer:");
        for (uint16_t i = 0; i < len; ++i) {
          Serial.print("Byte ");
          Serial.print(i);
          Serial.print(": 0x");
          if (buffer[i] < 0x10) Serial.print("0");
          Serial.print(buffer[i], HEX);
          Serial.print(" (");
          Serial.print(buffer[i]);
          Serial.println(")");
        }
        
        // Translate to proper Logitech protocol command
        // The command byte should be 0x05 (DEFAULT_SPRING_OFF)
        // No force slots should be enabled for this command
        translated_buffer[0] = 0x05; // DEFAULT_SPRING_OFF
        for (int i = 1; i < len; i++) {
          translated_buffer[i] = 0x00; // Clear the rest of the buffer
        }
        
        translated = true;
      }
      else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_ON) { // 0xF4 - DEFAULT_SPRING_ON
        Serial.println("Translating to DEFAULT_SPRING_ON");
        
        // Print the entire buffer for detailed debugging
        Serial.println("DEFAULT_SPRING_ON full buffer:");
        for (uint16_t i = 0; i < len; ++i) {
          Serial.print("Byte ");
          Serial.print(i);
          Serial.print(": 0x");
          if (buffer[i] < 0x10) Serial.print("0");
          Serial.print(buffer[i], HEX);
          Serial.print(" (");
          Serial.print(buffer[i]);
          Serial.println(")");
        }
        
        // Translate to proper Logitech protocol command
        // The command byte should be 0x04 (DEFAULT_SPRING_ON)
        // No force slots should be enabled for this command
        translated_buffer[0] = 0x04; // DEFAULT_SPRING_ON
        for (int i = 1; i < len; i++) {
          translated_buffer[i] = 0x00; // Clear the rest of the buffer
        }
        
        translated = true;
      }
      else if (cmd_byte == 0x8) { // 0xF8 - Constant force
        Serial.println("Translating pylinuxwheel constant force command");
        
        // Print the entire buffer for detailed debugging
        Serial.println("CONSTANT force full buffer:");
        for (uint16_t i = 0; i < len; ++i) {
          Serial.print("Byte ");
          Serial.print(i);
          Serial.print(": 0x");
          if (buffer[i] < 0x10) Serial.print("0");
          Serial.print(buffer[i], HEX);
          Serial.print(" (");
          Serial.print(buffer[i]);
          Serial.println(")");
        }
        
        // Translate to proper Logitech protocol command
        // The command byte should be 0x11 (DOWNLOAD_AND_PLAY_FORCE with force slot 0 enabled)
        // The force type byte should be 0x00 (CONSTANT)
        translated_buffer[0] = 0x11; // DOWNLOAD_AND_PLAY_FORCE (0x01) with force slot 0 enabled (0x10)
        translated_buffer[1] = 0x00; // CONSTANT force type
        
        // The force value is in the second byte of the original command
        if (len >= 2) {
          // For constant forces, the value is centered at 0x7F (127)
          // 0x7F = no force, <0x7F = left force, >0x7F = right force
          translated_buffer[2] = buffer[1]; // Force value
          Serial.print("Force value: 0x");
          Serial.print(buffer[1], HEX);
          Serial.print(" (");
          Serial.print((int8_t)(buffer[1] - 0x80)); // Print as signed value relative to center
          Serial.println(")");
        } else {
          translated_buffer[2] = 0x80; // Default to center (no force)
          Serial.println("No force value provided, defaulting to center");
        }
        
        // Clear the rest of the buffer
        for (int i = 3; i < len; i++) {
          translated_buffer[i] = 0x00;
        }
        
        translated = true;
      }
    }
    
    // Apply the translated force
    ffb_controller.apply_force(*((const FfbRequest *)translated_buffer));
  }
} 