#pragma once

#include "ffbTypes.h"
#include "wheel_controller.h"

class FfbController {
public:
  FfbController();
  
  float calculate_damper_force(uint8_t k1, uint8_t k2, uint8_t s1, uint8_t s2, uint8_t position);
  void apply_forces(EnumForceType force_type, uint8_t force_mask, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3);
  float update(float axis_wheel_value);
  void set_default_spring(uint8_t k1, uint8_t k2, uint8_t clip);
  void set_default_spring_enabled(bool on) { ffb_default_spring_on = on; }
  void printForces();
  float get_force();

private:
  float apply_force(uint8_t force_index, uint8_t position);
  float coeff_from_table(uint8_t offset);

  int8_t force_current;
  FfbRequest ffb_forces[4];
  bool ffb_forces_en[4];
  bool ffb_default_spring_on;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  uint8_t ffb_default_spring_clip;
  const uint8_t dead_band_lower = 125;
  const uint8_t dead_band_upper = 131;
  float wheel_range_normalized;
}; 