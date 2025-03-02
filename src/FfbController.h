#pragma once

#include "FfbTypes.h"
#include <stdint.h>
#include "MotorController.h"
// computing forces to apply to the wheel ffb motor
class FfbController {
public:
  FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range, MotorController &motorControl);
  
  void apply_force(const FfbRequest &req);
  float update(float axis_wheel_value);
  float get_force();

  MotorController &motor;
  float force_current;

  FfbRequest ffb_request;
  FfbForceType ffb_forces[4];
  bool ffb_forces_en[4];
  bool ffb_default_spring_on;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  uint8_t ffb_default_spring_clip;
}; 