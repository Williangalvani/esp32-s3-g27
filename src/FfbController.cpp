#include "FfbController.h"
#include <Arduino.h>
#include "MotorController.h"

FfbController::FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range, MotorController &motorControl)
  : force_current(0x80),
    ffb_forces_en{ 0, 0, 0, 0 },
    motor(motorControl) {
  // Constructor body is now empty since all initialization is done in the initializer list
}

void FfbController::printForce(uint8_t index, bool enabled, EnumForceType force_type) {
  static uint32_t last_print = 0;
  if (millis() - last_print < 500) {
    return;
  }
  last_print = millis();
  Serial.print("F ");
  Serial.print(index);
  Serial.print(": ");
  if (enabled) {
    Serial.print((uint8_t)force_type);
  } else {
    Serial.print("-");
  }
  Serial.println();
}

void FfbController::apply_force() {
}

// sets force in range 0x00 .. 0xff to force_current
 float FfbController::update(float axis_wheel_value) {
  float f = 0;
  
  // Debug output - print once per second
  static uint32_t last_debug = 0;
  bool should_debug = (millis() - last_debug >= 500);
  if (should_debug) {
    last_debug = millis();
    // Serial.println("Updating force feedback...");
    // Serial.print("Wheel position: ");
    // Serial.println(axis_wheel_value);
  }

  if (ffb_default_spring_on)
  {
    float spring_force = - ((float)ffb_default_spring_k1) * axis_wheel_value;
    f += spring_force;
    
    if (0) {
      Serial.print(">spring: ");
      Serial.println(spring_force);
      Serial.print(">k1: ");
      Serial.println(ffb_default_spring_k1);
    }
  }
  // else if (should_debug) {
  //   Serial.println("Default spring is OFF");
  // }

  for (uint8_t i = 0; i < 4; ++i) {
    bool f_en = ffb_forces_en[i];
    if (!f_en) continue;
    if (i != 0) continue;

    FfbForceType f_entry = ffb_forces[i];
    EnumForceType f_type = (EnumForceType)f_entry.bytes[0];
    // printForce(i, f_en, f_type);
    if (f_type == EnumForceType::CONSTANT) {
      int16_t constant_force = *(&f_entry.constant.f0 + i) - 0x7F;
      f += ((float)constant_force) / 100.0;
      
      if (should_debug) {
        // Serial.print("Constant force: ");
        // Serial.println(constant_force);
      }
    } else if (f_type == EnumForceType::SPRING) {
      if (should_debug) {
        Serial.println("Spring force type enabled but not implemented");
      }
    } else if (f_type == EnumForceType::DAMPER) {
      if (should_debug) {
        Serial.println("Damper force type enabled but not implemented");
      }
    } else if (should_debug) {
      Serial.print("Force type ");
      Serial.print((uint8_t)f_type);
      Serial.println(" enabled but not implemented");
    }
  }

  force_current = f;
  
  // // Debug output
  // if (should_debug) {
  //   Serial.print("Final force value: ");
  //   Serial.print(f);
  //   Serial.print(" (");
  //   Serial.print(f - 0x7f);
  //   Serial.println(")");
  // }
  return f;
}

float FfbController::get_force() {
  return force_current;
} 