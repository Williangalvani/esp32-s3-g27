#pragma once

#include "ffbTypes.h"
#include "wheel_controller.h"

class FfbController {
public:
  FfbController();
  
  float calculate_damper_force(uint8_t k1, uint8_t k2, uint8_t s1, uint8_t s2, uint8_t position);
  void apply_forces(uint8_t force_mask, EnumForceType force_type, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6);

  float update(float axis_wheel_value);
  void set_default_spring(uint8_t k1, uint8_t k2, uint8_t clip);
  void set_default_spring_enabled(bool on) { ffb_default_spring_on = on; }
  void set_wheel_range_normalized(float wheel_range_normalized) { this->wheel_range_normalized = wheel_range_normalized; }
  void printForces();
  float get_force();
  void play_force(uint8_t force_mask);
  void stop_force(uint8_t force_mask);
  float evaluate_force(uint8_t force_index, uint8_t position);
private:
  float coeff_from_table(uint8_t offset);

  float force_current;
  FfbRequest ffb_forces[4];
  uint8_t ffb_forces_enabled;
  bool ffb_default_spring_on;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  uint8_t ffb_default_spring_clip;
  const uint8_t dead_band_lower = 125;
  const uint8_t dead_band_upper = 131;
  float wheel_range_normalized;

  //RAMP
  uint8_t F0_loop_count;
  uint8_t F2_loop_count;

}; 