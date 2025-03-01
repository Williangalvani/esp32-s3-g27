#pragma once

#include "FfbTypes.h"
#include <stdint.h>

// computing forces to apply to the wheel ffb motor
class FfbController {
public:
  FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range);
  
  void apply_force(const FfbRequest &req);
  void update(uint16_t axis_wheel_value);
  uint8_t get_force();

  uint16_t axis_wheel_min;
  uint16_t axis_wheel_cnt;
  uint16_t axis_wheel_max;

  uint8_t force_current;

  FfbRequest ffb_request;
  FfbForceType ffb_forces[4];
  bool ffb_forces_en[4];
  bool ffb_default_spring_on;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  uint8_t ffb_default_spring_clip;
}; 