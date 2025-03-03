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
    status.status.init();
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
  
  // Process force feedback commands
  if (report_id == 0 && len >= 1) {
    uint8_t cmd_byte = buffer[0];

    // Check for pylinuxwheel commands
    cmd_byte &= 0x0F;
    uint8_t forces_mask = buffer[0] & 0x10;
    // print force mask in binary, only the first 4 bits
    Serial.print("Force mask: ");
    for (int i = 0; i < 4; i++) {
      Serial.print((forces_mask >> i) & 1);
      Serial.print(" ");
    }
    Serial.println();
    bool fully_parsed = false;   // this means we parsed, not handled
    if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_FORCE) { // FF_CONSTANT
      Serial.println("Detected DOWNLOAD_FORCE command");
    } else if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE) { // FF_CONSTANT
      Serial.println("Detected DOWNLOAD_AND_PLAY_FORCE command");
    }
    // Translate Linux input subsystem commands to Logitech protocol commands
    else if (cmd_byte == (uint8_t)EnumFfbCmd::PLAY_FORCE) { // FF_CONSTANT
      Serial.println("Detected PLAY_FORCE command");

    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::STOP_FORCE ) { // 0xF5 - DEFAULT_SPRING_OFF
      Serial.println("Detected STOP_FORCE command");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_ON) { // 0xF4 - DEFAULT_SPRING_ON
      Serial.println("Detected DEFAULT_SPRING_ON command");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_OFF) { // 0xF5 - DEFAULT_SPRING_OFF
      Serial.println("Detected DEFAULT_SPRING_OFF command");
      fully_parsed = true;
       // we are ignoring the x;y selection, which should be fine as we only have one axis
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::REFRESH_FORCE) { // 0x0C - REFRESH_FORCE
      Serial.println("Detected REFRESH_FORCE command");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::FIXED_TIME_LOOP) { // 0x0D - FIXED_TIME_LOOP
      Serial.println("Detected FIXED_TIME_LOOP command");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::SET_DEFAULT_SPRING) { // 0x0E - SET_DEFAULT_SPRING
      Serial.println("Detected SET_DEFAULT_SPRING command");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::SET_DEAD_BAND) { // 0x0F - SET_DEAD_BAND
      Serial.println("Detected SET_DEAD_BAND command");
    }
    else if (buffer[0] == (uint8_t)EnumFfbCmd::EXTENDED_COMMAND) {
      uint8_t extended_cmd = buffer[1];
      if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_MODE_TO_DRIVING_FORCE_PRO) {
        Serial.println("Detected CHANGE_MODE_TO_DRIVING_FORCE_PRO command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_WHEEL_RANGE_TO_200_DEGREES) {
        Serial.println("Detected CHANGE_WHEEL_RANGE_TO_200_DEGREES command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_WHEEL_RANGE_TO_900_DEGREES) {
        Serial.println("Detected CHANGE_WHEEL_RANGE_TO_900_DEGREES command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_DEVICE_MODE) {
        Serial.println("Detected CHANGE_DEVICE_MODE command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::REVERT_IDENTITY) {
        Serial.println("Detected REVERT_IDENTITY command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH) {
        Serial.println("Detected SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::SWITCH_TO_G25_IDENTITY_WITHOUT_USB_DETACH) {
        Serial.println("Detected SWITCH_TO_G25_IDENTITY_WITHOUT_USB_DETACH command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::SET_RPM_LEDS) {
        Serial.println("Detected SET_RPM_LEDS command");
      } else if (extended_cmd == (uint8_t)EnumExtendedCommand::WHEEL_RANGE_CHANGE) {
        Serial.println("Detected WHEEL_RANGE_CHANGE command");
      }
    }
    else {
      Serial.print("Unknown command ");
      Serial.println(buffer[0], HEX);
    }
    if (!fully_parsed) {
        Serial.print("Buffer: ");
        for (uint16_t i = 0; i < len; ++i) {
            if (buffer[i] < 0x10) Serial.print(0);
            Serial.print(buffer[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        Serial.print("decimal: ");
        for (uint16_t i = 0; i < len; ++i) {
          Serial.print(buffer[i]);
          Serial.print(" ");
        }
        Serial.println();
    }
    // Apply the translated force
    ffb_controller.apply_force();
  }
} 