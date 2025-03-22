#include "ffbController.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <algorithm> // For min/max functions
#include <cstring>   // For memset

static const char *TAG = "ffb";

FfbController::FfbController()
  : force_current(0x80),
    ffb_forces_enabled(0),
    ffb_default_spring_on(false),
    ffb_default_spring_k1(0),
    ffb_default_spring_k2(0),
    ffb_default_spring_clip(0),
    wheel_range_normalized(1.0),
    F0_loop_count(0),
    F2_loop_count(0) {
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

void FfbController::apply_forces(uint8_t force_mask, EnumForceType force_type, uint8_t byte2, uint8_t byte3, uint8_t byte4, uint8_t byte5, uint8_t byte6) {
  // ESP_LOGI(TAG, "force type: %d, force_mask %d", force_type, force_mask);
  for (int i = 0; i < 4; i++) {
    if (force_mask & (1 << i)) {
      // ESP_LOGI(TAG, "force index: %d", i);
      ffb_forces[i].forcetype  = (uint8_t)force_type;
      ffb_forces[i].params[0] = force_mask;
      ffb_forces[i].params[1] = (uint8_t)force_type;
      ffb_forces[i].params[2] = byte2;
      ffb_forces[i].params[3] = byte3;
      ffb_forces[i].params[4] = byte4;
      ffb_forces[i].params[5] = byte5;
      ffb_forces[i].params[6] = byte6;
      if (force_type == EnumForceType::VARIABLE) {
          F0_loop_count = 0;
          F2_loop_count = 0;
      }
    }
  }
}

void FfbController::play_force(uint8_t force_mask) {
  ffb_forces_enabled |= force_mask;
}
void FfbController::stop_force(uint8_t force_mask) {
  ffb_forces_enabled &= ~force_mask;
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

float FfbController::evaluate_force(uint8_t force_index, uint8_t position) {
  EnumForceType force_type = (EnumForceType)ffb_forces[force_index].forcetype;
  uint8_t param0 = ffb_forces[force_index].params[0];
  uint8_t param1 = ffb_forces[force_index].params[1];
  uint8_t param2 = ffb_forces[force_index].params[2];
  uint8_t param3 = ffb_forces[force_index].params[3];
  uint8_t param4 = ffb_forces[force_index].params[4];
  uint8_t param5 = ffb_forces[force_index].params[5];
  uint8_t param6 = ffb_forces[force_index].params[6];
  switch (force_type) {
    case EnumForceType::CONSTANT:
    {
      ESP_LOGI(TAG, "Force %d: Constant force %d %d %d %d %d %d", force_index, param0, param1, param2, param3, param4, param5);
      float force = param2 - 127;
      return force/127.0f;
    }
    case EnumForceType::SPRING:
      ESP_LOGI(TAG, "Force %d: Spring force", force_index);
      return 0;
    case EnumForceType::DAMPER:
      ESP_LOGI(TAG, "Force %d: Damper force", force_index);
      return 0;
    case EnumForceType::AUTO_CNT_SPRING:
      ESP_LOGI(TAG, "Force %d: AutoCntSpring force", force_index);
      return 0;
    case EnumForceType::SAWTOOTH_UP:
      ESP_LOGI(TAG, "Force %d: SawtoothUp force", force_index);
      return 0;
    case EnumForceType::SAWTOOTH_DN:
      ESP_LOGI(TAG, "Force %d: SawtoothDn force", force_index);
      return 0;
    case EnumForceType::TRAPEZOID:
      ESP_LOGI(TAG, "Force %d: Trapezoid force", force_index);
      return 0;
    case EnumForceType::RECTANGLE:
      ESP_LOGI(TAG, "Force %d: Rectangle force", force_index);
      return 0;
    case EnumForceType::VARIABLE:
      {
      uint8_t L1 = param2;        // Initial level for Force 0
      uint8_t L2 = param3;        // Initial level for Force 2
      uint8_t T1 = (param4 & 0xF0) >> 4;  // Force 0 Step duration (in main loops)
      uint8_t S1 = param4 & 0x0F;         // Force 0 Step size
      uint8_t T2 = (param5 & 0xF0) >> 4;  // Force 2 Step duration (in main loops)
      uint8_t S2 = param5 & 0x0F;         // Force 2 Step size
      uint8_t D1 = param6 & 0x01;         // Force 0 Direction (0=increasing, 1=decreasing)
      uint8_t D2 = (param6 >> 4) & 0x01;  // Force 2 Direction (0=increasing, 1=decreasing)
      
      if (force_index == 0) {
        // Apply F0
        int direction = D1 ? 1 : -1;
        int step_size = S1 * direction;
        int step_duration = std::clamp(T1, (uint8_t)1, (uint8_t)255);
        int current_level = L1;
        int steps = 0;
        // not actually variable for now
        int new_level = L1;// + step_size * ((float)F0_loop_count / (float)step_duration);
        new_level = std::clamp(new_level, 0, 255);
        F0_loop_count++;
        float force = new_level - 127;


        return std::clamp(force/127.0f, -1.0f, 1.0f);
      }
      else if (force_index == 2) {
        // Apply F2
        int direction = D2 ? 1 : -1;
        int step_size = S2 * direction;
        int step_duration = std::clamp(T2, (uint8_t)1, (uint8_t)255);
        int current_level = L2;
        int steps = 0;
        int new_level = L2;// + step_size * ((float)F2_loop_count / (float)step_duration);
        new_level = std::clamp(new_level, 0, 255);
        F2_loop_count++;
        float force = new_level - 127;
        return std::clamp(force/127.0f, -1.0f, 1.0f);
      }
      else {
        return 0;
      }
      }
    case EnumForceType::RAMP:
      ESP_LOGI(TAG, "Force %d: Ramp force", force_index);
      return 0;
    case EnumForceType::SQUARE_WAVE:
      ESP_LOGI(TAG, "Force %d: SquareWave force", force_index);
      return 0;
    case EnumForceType::HI_RES_SPRING:
    {

      uint8_t D1 = param2; // deadband lower
      uint8_t D2 = param3; // deadband upper
      uint8_t K1 = param4 & 0x0F; // spring coefficient low
      uint8_t K2 = (param4 >> 4); // spring coefficient high
      int8_t S1 = param5 & 0x01; // signal direction      // printf(">f%dD1: %d\n", force_index, D1);
      // printf(">f%dD2: %d\n", force_index, D2);
      // printf(">f%dK1: %d\n", force_index, K1);
      // printf(">f%dK2: %d\n", force_index, K2);
      // printf(">f%dS1: %d\n", force_index, S1);
      // printf(">f%dS2: %d\n", force_index, S2);
      // printf(">f%dforce: %f\n", force_index, force); low
      int8_t S2 = (param5 >> 4) & 0x01; // signal direction high
      uint8_t lower_difference = position - D1;
      uint8_t upper_difference = D2 - position;
      int8_t direction_1 = S1 ? 1 : -1;
      int8_t direction_2 = S2 ? 1 : -1;

      float force = 0;
      bool lower = position < D1;
      bool upper = position > D2;
      if (lower) {
        float spring_coef = float(K1)/100.0;
        float offset = position - D1;
        float spring_force = std::min(spring_coef * std::abs((float)(position - D1)), (float)ffb_default_spring_clip);
        force = -spring_force;        
      }
      else if (upper) {
        float spring_coef = float(K2)/100.0;
        float offset = position - D2;
        float spring_force = std::min(spring_coef * std::abs((float)(position - D2)), (float)ffb_default_spring_clip);
        force = spring_force;        
      }
      //ESP_LOGI(TAG, "Force %d: HiResSpring force: D1: %d D2: %d K1: %d K2: %d S1: %d S2: %d force: %f, upper: %d, lower: %d", force_index, D1, D2, K1, K2, S1, S2, force, upper, lower);
      // printf(">f%dD1: %d\n", force_index, D1);
      // printf(">f%dD2: %d\n", force_index, D2);
      // printf(">f%dK1: %d\n", force_index, K1);
      // printf(">f%dK2: %d\n", force_index, K2);
      // printf(">f%dS1: %d\n", force_index, S1);
      // printf(">f%dS2: %d\n", force_index, S2);
      // printf(">f%dforce: %f\n", force_index, force);
      return force;
    }
    case EnumForceType::HI_RES_DAMPER:
    {
      ESP_LOGI(TAG, "Force %d: HiResDamper force", force_index);
      return 0;
    }
  }
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
  }
    for (int i = 0; i < 4; i++) {
    if (ffb_forces_enabled & (1 << i)) {

      force_current += evaluate_force(i, position);
    }
  }
  // static int print_counter = 0;
  // if (print_counter % 100 == 0) {
  //   ESP_LOGI(TAG, "force_current: %d", force_current);
  // }
  if (std::abs(force_current) > 0.05) {
    printf(">force_current:%f\n", force_current);
  }
  return force_current;
}

float FfbController::get_force() {
  printForces();
  return force_current;
} 