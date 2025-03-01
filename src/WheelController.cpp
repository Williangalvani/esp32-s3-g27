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

    ffb_controller.update(status.status.get_axis_wheel_14bit());
    int16_t f = this->ffb_controller.get_force();
    USBSerial.println("f: " + String(f));

    bool dir_ccw = f > 0x7f;

    f = abs(f - 0x7f) << 1;
    if (f > 0xf0) f = 0xf0;

    if (f > 0) {
        motor_controller.stop();
    } else {
        motor_controller.move(dir_ccw);
    }


}

void WheelController::update_axes() {
  update_axis_wheel();
  
  // Randomly toggle button 0 every ~500ms to verify HID updates
  static uint32_t last_toggle = 0;
  if (millis() - last_toggle >= 500) {
    last_toggle = millis();
    // Randomly set or clear bit 0 of buttons_0
    if (random(2) == 0) {
      status.status.buttons_0 |= 0x01;  // Set bit 0
    } else {
      status.status.buttons_0 &= ~0x01; // Clear bit 0
    }
    
    USBSerial.println(">pos:" + String(motor_controller.get_encoder_count()));
  }
}

void WheelController::update_axis_wheel() {
  status.status.set_axis_wheel_float(motor_controller.get_position_zero_centered(), true);
}

void WheelController::sendState() {
  if (hid.ready()) {
    hid.SendReport(DEV_REPORT_ID, status.bytes, sizeof(status.bytes), 0);
  }
}

uint16_t WheelController::_onGetDescriptor(uint8_t *buffer) {
  memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
  return sizeof(hid_report_descriptor);
}

uint16_t WheelController::_onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) {
  // Not implemented
  return 0;
}

void WheelController::_onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
  if (len != sizeof(FfbRequest)) return;
  ffb_controller.apply_force(*((const FfbRequest *)buffer));
  // USBSerial.print("[IN]["); USBSerial.print(report_id); USBSerial.print("] ");
  // for (uint16_t i = 0; i < len; ++i)
  // {
  //   if (buffer[i] < 0x10) USBSerial.print(0);
  //   USBSerial.print(buffer[i], HEX);
  //   USBSerial.print(" ");
  // }
  // USBSerial.println();
}

void WheelController::_onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) {
  // Not implemented
} 