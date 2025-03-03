#pragma once

#include "FfbTypes.h"
#include <stdint.h>
#include "MotorController.h"
// computing forces to apply to the wheel ffb motor
class FfbController {
public:
  FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range, MotorController &motorControl);
  
  void apply_forces(EnumForceType force_type, uint8_t force_mask, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3);
  float update(float axis_wheel_value);
  void set_default_spring(uint8_t k1, uint8_t k2, uint8_t clip);
  float coeff_from_table(uint8_t offset);
  float get_force();
  void printForces();
  MotorController &motor;
  float force_current;

  FfbRequest ffb_request;
  FfbForceType ffb_forces[4];
  bool ffb_forces_en[4];
  bool ffb_default_spring_on;
  uint8_t dead_band_lower = 126;
  uint8_t dead_band_upper = 128;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  float ffb_default_spring_clip;
}; 