#include "WheelController.h"
#include <Arduino.h>
#include "FfbTypes.h"

WheelController::WheelController()
    : status{},
      ffb_controller(0x1fff, 0x3fff, motor_controller)
{
  hid.addDevice(this, sizeof(hid_report_descriptor));
  status.status.buttons_0 = 0x08;
}

void WheelController::init()
{

  initialized = false;
  motor_controller.init();
  hid.begin();
  status.status.init();
  initialized = true;
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

  // Randomly toggle button 0 every ~500ms to verify HID updates
  static uint32_t last_toggle = 0;
  if (millis() - last_toggle >= 500)
  {
    last_toggle = millis();
    // Randomly set or clear bit 0 of buttons_0
    if (status.status.buttons_0 == 0x08)
    {
      status.status.buttons_0 = 0x00;
    }
    else
    {
      status.status.buttons_0 = 0x08;
    }
  }
}

void WheelController::update_axis_wheel()
{
  status.status.set_axis_wheel_float(motor_controller.get_position_zero_centered(), true);
}

void WheelController::sendState()
{
  if (hid.ready())
  {
    // Implement retry mechanism for SendReport
    const int MAX_RETRIES = 3;
    int retries = 0;
    bool success = false;

    while (!success && retries < MAX_RETRIES)
    {
      // Try to send the report
      success = hid.SendReport(DEV_REPORT_ID, status.bytes, sizeof(status.bytes), 20);

      if (!success)
      {
        retries++;
        // Only log the first retry to avoid spamming the console
        if (retries == 1)
        {
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
    if (!success && retries == MAX_RETRIES)
    {
      static uint32_t last_error_log = 0;
      // Only log errors once per second to avoid spamming
      if (millis() - last_error_log >= 1000)
      {
        last_error_log = millis();
        Serial.println("SendReport failed after maximum retries");
      }
    }
  }
}

uint16_t WheelController::_onGetDescriptor(uint8_t *buffer)
{
  Serial.println("onGetDescriptor");
  memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
  return sizeof(hid_report_descriptor);
}

uint16_t WheelController::_onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len)
{
  Serial.println("onGetFeature");
  // Not implemented
  return 0;
}

void WheelController::_onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len)
{
  Serial.println("onSetFeature");
  Serial.print("[IN][");
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
  // Process force feedback commands
  if (report_id == 0 && len >= 1)
  {
    uint8_t cmd_byte = buffer[0];

    // Check for pylinuxwheel commands
    cmd_byte &= 0x0F;
    uint8_t forces_mask = buffer[0] & 0x10;
    // print force mask in binary, only the first 4 bits
    bool fully_parsed = false; // this means we parsed, not handled
    if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_FORCE)
    { // FF_CONSTANT
      Serial.println("DOWNLOAD_FORCE");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE)
    { // FF_CONSTANT
      Serial.println("DOWNLOAD_AND_PLAY_FORCE");
      ffb_controller.apply_forces((EnumForceType)buffer[1], forces_mask, buffer[2], buffer[3], buffer[4], buffer[5]);
    }
    // Translate Linux input subsystem commands to Logitech protocol commands
    else if (cmd_byte == (uint8_t)EnumFfbCmd::PLAY_FORCE)
    { // FF_CONSTANT
      Serial.println("PLAY_FORCE");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::STOP_FORCE)
    { // 0xF5 - DEFAULT_SPRING_OFF
      Serial.print("STOP_FORCE > ");
      printMask(forces_mask);
      fully_parsed = true;
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_ON)
    { // 0xF4 - DEFAULT_SPRING_ON
      Serial.println("DEFAULT_SPRING_ON");
      ffb_controller.ffb_default_spring_on = true;
      fully_parsed = true;
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::DEFAULT_SPRING_OFF)
    { // 0xF5 - DEFAULT_SPRING_OFF
      Serial.println("DEFAULT_SPRING_OFF");
      ffb_controller.ffb_default_spring_on = false;
      fully_parsed = true;
      // we are ignoring the x;y selection, which should be fine as we only have one axis
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::REFRESH_FORCE)
    { // 0x0C - REFRESH_FORCE
      Serial.println("REFRESH_FORCE");
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::FIXED_TIME_LOOP)
    { // 0x0D - FIXED_TIME_LOOP
      Serial.println("Ignoring FIXED_TIME_LOOP");
      fully_parsed = true;
    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::SET_DEFAULT_SPRING) {
      Serial.print("SET_DEFAULT_SPRING > ");
      Serial.print("k1: ");
      Serial.print(buffer[1]);
      Serial.print(" k2: ");
      Serial.print(buffer[2]);
      Serial.print(" clip: ");
      Serial.println(buffer[3]);
      FfbRequest* request = (FfbRequest*)buffer;
      ffb_controller.set_default_spring(request->set_default_spring.k1, request->set_default_spring.k2, request->set_default_spring.clip);
      fully_parsed = true;

    }
    else if (cmd_byte == (uint8_t)EnumFfbCmd::SET_DEAD_BAND)
    { // 0x0F - SET_DEAD_BAND
      Serial.println("SET_DEAD_BAND");
    }
    else if (buffer[0] == (uint8_t)EnumFfbCmd::EXTENDED_COMMAND)
    {
      uint8_t extended_cmd = buffer[1];
      if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_MODE_TO_DRIVING_FORCE_PRO)
      {
        Serial.println("CHANGE_MODE_TO_DRIVING_FORCE_PRO");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_WHEEL_RANGE_TO_200_DEGREES)
      {
        Serial.println("CHANGE_WHEEL_RANGE_TO_200_DEGREES");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_WHEEL_RANGE_TO_900_DEGREES)
      {
        Serial.println("CHANGE_WHEEL_RANGE_TO_900_DEGREES");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::CHANGE_DEVICE_MODE)
      {
        Serial.println("CHANGE_DEVICE_MODE");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::REVERT_IDENTITY)
      {
        Serial.println("REVERT_IDENTITY");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH)
      {
        Serial.println("SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::SWITCH_TO_G25_IDENTITY_WITHOUT_USB_DETACH)
      {
        Serial.println("SWITCH_TO_G25_IDENTITY_WITHOUT_USB_DETACH");
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::SET_RPM_LEDS)
      {
        Serial.println("SET_RPM_LEDS");
        fully_parsed = true;
      }
      else if (extended_cmd == (uint8_t)EnumExtendedCommand::WHEEL_RANGE_CHANGE)
      {
        uint8_t lsb = buffer[2];
        uint8_t msb = buffer[3];
        uint16_t wheel_range = (msb << 8) | lsb;
        Serial.println("WHEEL_RANGE_CHANGE");
        Serial.print("Wheel range: ");
        Serial.println(wheel_range);
        fully_parsed = true;
      }
    }
    else
    {
      Serial.print("Unknown command ");
      Serial.println(buffer[0], HEX);
    }
    if (!fully_parsed)
    {
      printMask(forces_mask);
      Serial.print("Buffer: ");
      for (uint16_t i = 0; i < len; ++i)
      {
        if (buffer[i] < 0x10)
          Serial.print(0);
        Serial.print(buffer[i], HEX);
        Serial.print(" ");
      }
      Serial.println();
      Serial.print("decimal: ");
      for (uint16_t i = 0; i < len; ++i)
      {
        Serial.print(buffer[i]);
        Serial.print(" ");
      }
      Serial.println();
    }
    // Apply the translated force
    ffb_controller.update(motor_controller.get_position_zero_centered());
  }
}