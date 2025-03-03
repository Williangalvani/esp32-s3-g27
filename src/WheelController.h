#pragma once

#include "USB.h"
#include "USBHID.h"
#include "Constants.h"
#include "FfbController.h"
#include "WheelStatus.h"
#include "MotorController.h"

// main wheel class, glues everything together
class WheelController : public USBHIDDevice {
public:
  WheelController();
  
  void init();
  void update_ffb();
  void update_axes();
  void update_axis_wheel();
  void sendState();
  
  WheelStatus status;
  FfbController ffb_controller;
  MotorController motor_controller;
  USBHID hid;
  bool initialized;

protected:
  // we set hid report descriptor in this callback, writing it to buffer provided
  uint16_t _onGetDescriptor(uint8_t *buffer) override;
  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) override;
  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) override;
  void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) override;
}; 