#include "FfbController.h"
#include <Arduino.h>
#include "MotorController.h"

FfbController::FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range, MotorController &motorControl)
  : force_current(0x80),
    ffb_forces_en{ 0, 0, 0, 0 },
    motor(motorControl) {
  // Constructor body is now empty since all initialization is done in the initializer list
}

void FfbController::set_default_spring(uint8_t k1, uint8_t k2, uint8_t clip) {
  ffb_default_spring_k1 = k1;
  ffb_default_spring_k2 = k2;
  ffb_default_spring_clip = clip;
}

void FfbController::printForces() {
  static uint32_t last_print = 0;
  if (millis() - last_print < 500) {
    return;
  }
  last_print = millis();
  Serial.println("------ forces -----");
  for (int i = 0; i < 4; i++) {
    if (ffb_forces_en[i]) { 
      Serial.print("F ");
      Serial.print(i);
      Serial.print(": ");
      EnumForceType force_type = (EnumForceType)ffb_forces[i].constant.type;
      switch (force_type) {
        case EnumForceType::CONSTANT:
          Serial.print("Constant");
          break;
        case EnumForceType::SPRING:
          Serial.print("Spring");
          break;
        case EnumForceType::DAMPER:
          Serial.print("Damper");
          break;
        case EnumForceType::AUTO_CNT_SPRING:
          Serial.print("AutoCntSpring");
          break;
        case EnumForceType::SAWTOOTH_UP:
          Serial.print("SawtoothUp");
          break;
        case EnumForceType::SAWTOOTH_DN:
          Serial.print("SawtoothDn");
          break;
        case EnumForceType::TRAPEZOID:
          Serial.print("Trapezoid");
          break;
        case EnumForceType::RECTANGLE:
          Serial.print("Rectangle");
          break;
        case EnumForceType::VARIABLE:
          Serial.print("Variable");
          break;
        case EnumForceType::RAMP:
          Serial.print("Ramp");
          break;
        case EnumForceType::SQUARE_WAVE:
          Serial.print("SquareWave");
          break;
        case EnumForceType::HI_RES_SPRING:
          Serial.print("HiResSpring");
          break;
        case EnumForceType::HI_RES_DAMPER:
          Serial.print("HiResDamper");
          break;
        case EnumForceType::HI_RES_AUTO_CNT_SPRING:
          Serial.print("HiResAutoCntSpring");
          break;
        case EnumForceType::FRICTION:
          Serial.print("Friction");
          break;
        default:
          Serial.print("Unknown");
          break;
      }
      Serial.print(" ");
      Serial.print(ffb_forces[i].constant.param0);
      Serial.print(" ");
      Serial.print(ffb_forces[i].constant.param1);
      Serial.print(" ");
      Serial.print(ffb_forces[i].constant.param2);
      Serial.print(" ");
      Serial.print(ffb_forces[i].constant.param3);
      Serial.println();
    }
  }
  Serial.println();
}

void FfbController::apply_forces(EnumForceType force_type, uint8_t force_mask, uint8_t param0, uint8_t param1, uint8_t param2, uint8_t param3) {
  ffb_forces[force_mask].constant.type = (uint8_t)force_type;
  ffb_forces[force_mask].constant.param0 = param0;
  ffb_forces[force_mask].constant.param1 = param1;
  ffb_forces[force_mask].constant.param2 = param2;
  ffb_forces[force_mask].constant.param3 = param3;
}

float FfbController::coeff_from_table(uint8_t offset) {
// K Value Spring Coeficient
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
 }
}

float FfbController::update(float axis_wheel_value) {
force_current = 0;

// STartign with the spring
// map position to 0-255 as in the logitech protocol
uint8_t position = map(axis_wheel_value*1000, -1000, 1000, 0, 255);

if (ffb_default_spring_on) {
  bool lower = position < dead_band_lower;
  bool upper = position > dead_band_upper;
  if (lower) {
    float spring_coef = coeff_from_table(ffb_default_spring_k1);
    float offset = position - dead_band_lower;
    float spring_force = min(spring_coef * abs(position - dead_band_lower), ffb_default_spring_clip);
    force_current -= spring_force;
  }
  else if (upper) {
    float spring_coef = coeff_from_table(ffb_default_spring_k2);
    float offset = position - dead_band_upper;
    float spring_force = min(spring_coef * abs(position - dead_band_upper), ffb_default_spring_clip);
    force_current += spring_force;
  }



  // bool lower = position < dead_band_lower;
  // bool upper = position > dead_band_upper;
  // if (lower) {
  //   float spring_coef = coeff_from_table(ffb_default_spring_k1);
  //   float offset = position - dead_band_lower;
  //   float spring_force = min(spring_coef * abs(position - dead_band_lower), ffb_default_spring_clip);
  //   force_current += spring_force;
  // }
  // else if (upper) {
  //   float spring_coef = coeff_from_table(ffb_default_spring_k2);
  //   float offset = position - dead_band_upper;
  //   float spring_force = min(spring_coef * abs(position - dead_band_upper), ffb_default_spring_clip);
  //   force_current += spring_force;
  // }
  Serial.print("wheel position: ");
  Serial.print(position);
  Serial.print(" spring force: ");
  Serial.println(force_current);
}
return force_current;
}

float FfbController::get_force() {
  printForces();
  return force_current;
} 