#include "ffbController.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm> // For min/max functions
#include <cstring>   // For memset

static const char *TAG = "ffb";

FfbController::FfbController()
  : force_current(0x80),
    ffb_forces_en{ false, false, false, false },
    ffb_default_spring_on(false),
    ffb_default_spring_k1(0),
    ffb_default_spring_k2(0),
    ffb_default_spring_clip(0),
    wheel_range_normalized(1.0) {
  // Initialize ffb_forces
  for (int i = 0; i < 4; i++) {
    memset(&ffb_forces[i], 0, sizeof(FfbRequest));
  }
}

void FfbController::set_default_spring(uint8_t k1, uint8_t k2, uint8_t clip) {
  ffb_default_spring_k1 = k1;
  ffb_default_spring_k2 = k2;
  ffb_default_spring_clip = clip;
}

void FfbController::printForces() {
    ESP_LOGI(TAG, "total forces: %d", force_current);
  }
  
//   static int64_t last_print = 0;
//   int64_t now = esp_timer_get_time() / 1000; // Convert to ms
  
//   if (now - last_print < 500) {
//     return;
//   }
  
//   last_print = now;
//   ESP_LOGI(TAG, "------ forces -----");
  
//   for (int i = 0; i < 4; i++) {
//     if (ffb_forces_en[i]) { 
//       // Access the force data through the parameters array
//       uint8_t force_type = ffb_forces[i].params[0]; // first param is force type
//       const char* type_str = "Unknown";
      
//       switch ((EnumForceType)force_type) {
//         case EnumForceType::CONSTANT: type_str = "Constant"; break;
//         case EnumForceType::SPRING: type_str = "Spring"; break;
//         case EnumForceType::DAMPER: type_str = "Damper"; break;
//         case EnumForceType::AUTO_CNT_SPRING: type_str = "AutoCntSpring"; break;
//         case EnumForceType::SAWTOOTH_UP: type_str = "SawtoothUp"; break;
//         case EnumForceType::SAWTOOTH_DN: type_str = "SawtoothDn"; break;
//         case EnumForceType::TRAPEZOID: type_str = "Trapezoid"; break;
//         case EnumForceType::RECTANGLE: type_str = "Rectangle"; break;
//         case EnumForceType::VARIABLE: type_str = "Variable"; break;
//         case EnumForceType::RAMP: type_str = "Ramp"; break;
//         case EnumForceType::SQUARE_WAVE: type_str = "SquareWave"; break;
//         case EnumForceType::HI_RES_SPRING: type_str = "HiResSpring"; break;
//         case EnumForceType::HI_RES_DAMPER: type_str = "HiResDamper"; break;
//         case EnumForceType::HI_RES_AUTO_CNT_SPRING: type_str = "HiResAutoCntSpring"; break;
//         case EnumForceType::FRICTION: type_str = "Friction"; break;
//       }
      
//       ESP_LOGI(TAG, "F %d: %s %d %d %d %d", 
//               i, type_str,
//               ffb_forces[i].params[1],  // param0
//               ffb_forces[i].params[2],  // param1
//               ffb_forces[i].params[3],  // param2
//               ffb_forces[i].params[4]); // param3
//     }
//   }
// }

void FfbController::apply_forces(EnumForceType force_type, uint8_t force_mask, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3) {
  ffb_forces[force_mask].cmd = 0; // This is a force definition, not a command
  ffb_forces[force_mask].params[0] = (uint8_t)force_type;
  ffb_forces[force_mask].params[1] = param0;
  ffb_forces[force_mask].params[2] = param1;
  ffb_forces[force_mask].params[3] = param2;
  ffb_forces[force_mask].params[4] = param3;
  ffb_forces_en[force_mask] = true;
}

float FfbController::calculate_damper_force(uint8_t k1, uint8_t k2, uint8_t s1, uint8_t s2, uint8_t position) {
  static uint8_t last_position = 0;
  int8_t delta = position - last_position;
  last_position = position;

  if (delta < 0) {
     // k2, s2
     if (s2) {
      return -k2 * delta;
     }
     else {
      return k2 * delta;
     }
  }
  else {
     // k1, s1
     if (s1) {
      return -k1 * delta;
     }
     else {
      return k1 * delta;
     }
  }
}

float FfbController::apply_force(uint8_t force_index, uint8_t position) {
  // return a value between -127 and 127
  uint8_t force_type = ffb_forces[force_index].params[0];
  uint8_t param0 = ffb_forces[force_index].params[1];
  uint8_t param1 = ffb_forces[force_index].params[2];
  uint8_t param2 = ffb_forces[force_index].params[3];
  uint8_t param3 = ffb_forces[force_index].params[4];

  return 0;
}

float FfbController::coeff_from_table(uint8_t offset) {
  // K Value Spring Coefficient
  // 0x00 1/4 of offset
  // 0x01 1/2 of offset
  // 0x02 3/4 of offsett
  // 0x03 Force = offset
  // 0x04 3/2 of offset
  // 0x05 2 times offset
  // 0x06 3 times offset
  // 0x07 4 times offset

  switch (offset) {
    case 0: return 0.25;
    case 1: return 0.5;
    case 2: return 0.75;
    case 3: return 1.0;
    case 4: return 1.5;
    case 5: return 2.0;
    case 6: return 3.0;
    case 7: return 4.0;
    default: return 1.0;
  }
}

float FfbController::update(float axis_wheel_value) {
  force_current = 0;
  // STartign with the spring
  // map position to 0-255 as in the logitech protocol

  // enforce range limits
  if (axis_wheel_value > wheel_range_normalized) {
    force_current += 1.0;
  }
  if (axis_wheel_value < -wheel_range_normalized) {
    force_current -= 1.0;
  }

  // Map from -1.0..1.0 to 0..255
  uint16_t position = (uint16_t)((axis_wheel_value + 1.0) * 127.5);

  if (ffb_default_spring_on) {
    bool lower = position < dead_band_lower;
    bool upper = position > dead_band_upper;
    if (lower) {
      float spring_coef = coeff_from_table(ffb_default_spring_k1);
      float offset = position - dead_band_lower;
      float spring_force = std::min(spring_coef * std::abs((float)(position - dead_band_lower)), (float)ffb_default_spring_clip);
      force_current -= spring_force;
    }
    else if (upper) {
      float spring_coef = coeff_from_table(ffb_default_spring_k2);
      float offset = position - dead_band_upper;
      float spring_force = std::min(spring_coef * std::abs((float)(position - dead_band_upper)), (float)ffb_default_spring_clip);
      force_current += spring_force;
    }

    for (int i = 0; i < 4; i++) {
      if (ffb_forces_en[i]) {
        force_current += apply_force(i, position);
      }
    }
  }
  static int print_counter = 0;
  if (print_counter % 100 == 0) {
    ESP_LOGI(TAG, "force_current: %d", force_current);
  }
  print_counter++;
  return force_current;
}

float FfbController::get_force() {
  printForces();
  return force_current;
} 