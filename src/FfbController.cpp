#include "FfbController.h"
#include <Arduino.h>
#include "MotorController.h"

FfbController::FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range, MotorController &motorControl)
  : force_current(0x80),
    ffb_forces_en{ 0, 0, 0, 0 },
    motor(motorControl) {
  // Constructor body is now empty since all initialization is done in the initializer list
}

void FfbController::apply_force(const FfbRequest &req) {
  Serial.println("apply_force called - processing force feedback command");
  
  EnumFfbCmd f_cmd = (EnumFfbCmd)(req.bytes[0] & 0x0f);

  // force slots
  bool f0_en = req.bytes[0] & 0x10;
  bool f1_en = req.bytes[0] & 0x20;
  bool f2_en = req.bytes[0] & 0x40;
  bool f3_en = req.bytes[0] & 0x80;

  // Add debug prints
  Serial.print("FFB Command: 0x");
  Serial.print(req.bytes[0], HEX);
  Serial.print(" (");
  switch(f_cmd) {
    case EnumFfbCmd::DOWNLOAD_FORCE: Serial.print("DOWNLOAD_FORCE"); break;
    case EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE: Serial.print("DOWNLOAD_AND_PLAY_FORCE"); break;
    case EnumFfbCmd::PLAY_FORCE: Serial.print("PLAY_FORCE"); break;
    case EnumFfbCmd::STOP_FORCE: Serial.print("STOP_FORCE"); break;
    case EnumFfbCmd::DEFAULT_SPRING_ON: Serial.print("DEFAULT_SPRING_ON"); break;
    case EnumFfbCmd::DEFAULT_SPRING_OFF: Serial.print("DEFAULT_SPRING_OFF"); break;
    case EnumFfbCmd::REFRESH_FORCE: Serial.print("REFRESH_FORCE"); break;
    case EnumFfbCmd::SET_DEFAULT_SPRING: Serial.print("SET_DEFAULT_SPRING"); break;
    default: Serial.print("UNKNOWN"); break;
  }
  Serial.println(")");

  // Print which force slots are enabled
  Serial.print("Force slots: ");
  if (f0_en) Serial.print("0 ");
  if (f1_en) Serial.print("1 ");
  if (f2_en) Serial.print("2 ");
  if (f3_en) Serial.print("3 ");
  Serial.println();

  // Print force type when downloading or playing force
  if (f_cmd == EnumFfbCmd::DOWNLOAD_FORCE || 
      f_cmd == EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE ||
      f_cmd == EnumFfbCmd::REFRESH_FORCE) {
    Serial.print("Force type: ");
    EnumForceType force_type = (EnumForceType)req.download_force.force_type.bytes[0];
    switch(force_type) {
      case EnumForceType::CONSTANT: 
      {
        Serial.println("CONSTANT"); 
        int8_t force_value = req.download_and_play_force.force_type.constant.f0 - 0x80;
        Serial.print("Constant force value: ");
        Serial.println(force_value);
      }

        break;
      case EnumForceType::SPRING: Serial.println("SPRING"); break;
      case EnumForceType::DAMPER: Serial.println("DAMPER"); break;
      case EnumForceType::AUTO_CNT_SPRING: Serial.println("AUTO_CNT_SPRING"); break;
      case EnumForceType::SAWTOOTH_UP: Serial.println("SAWTOOTH_UP"); break;
      case EnumForceType::SAWTOOTH_DN: Serial.println("SAWTOOTH_DN"); break;
      case EnumForceType::TRAPEZOID: Serial.println("TRAPEZOID"); break;
      case EnumForceType::RECTANGLE: Serial.println("RECTANGLE"); break;
      case EnumForceType::VARIABLE: Serial.println("VARIABLE"); break;
      case EnumForceType::RAMP: Serial.println("RAMP"); break;
      case EnumForceType::SQUARE_WAVE: Serial.println("SQUARE_WAVE"); break;
      case EnumForceType::HI_RES_SPRING: Serial.println("HI_RES_SPRING"); break;
      case EnumForceType::HI_RES_DAMPER: Serial.println("HI_RES_DAMPER"); break;
      case EnumForceType::HI_RES_AUTO_CNT_SPRING: Serial.println("HI_RES_AUTO_CNT_SPRING"); break;
      case EnumForceType::FRICTION: Serial.println("FRICTION"); break;
      default: 
        Serial.print("UNKNOWN (0x"); 
        Serial.print(req.download_force.force_type.bytes[0], HEX);
        Serial.println(")");
        break;
    }
  }

  if (f_cmd == EnumFfbCmd::DOWNLOAD_FORCE) {
    if (f0_en) {
      ffb_forces[0] = req.download_force.force_type;
      ffb_forces_en[0] = false;
    }
    if (f1_en) {
      ffb_forces[1] = req.download_force.force_type;
      ffb_forces_en[1] = false;
    }
    if (f2_en) {
      ffb_forces[2] = req.download_force.force_type;
      ffb_forces_en[2] = false;
    }
    if (f3_en) {
      ffb_forces[3] = req.download_force.force_type;
      ffb_forces_en[3] = false;
    }
  } else if (f_cmd == EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE) {
    if (f0_en) {
      ffb_forces[0] = req.download_and_play_force.force_type;
      ffb_forces_en[0] = true;
      
      // If this is a constant force, print the value
      if (req.download_and_play_force.force_type.bytes[0] == (uint8_t)EnumForceType::CONSTANT) {
        int8_t force_value = req.download_and_play_force.force_type.constant.f0 - 0x80;
        Serial.print("Setting constant force value: ");
        Serial.println(force_value);
      }
    }
    if (f1_en) {
      ffb_forces[1] = req.download_and_play_force.force_type;
      ffb_forces_en[1] = true;
    }
    if (f2_en) {
      ffb_forces[2] = req.download_and_play_force.force_type;
      ffb_forces_en[2] = true;
    }
    if (f3_en) {
      ffb_forces[3] = req.download_and_play_force.force_type;
      ffb_forces_en[3] = true;
    }
  } else if (f_cmd == EnumFfbCmd::PLAY_FORCE) {
    if (f0_en) { ffb_forces_en[0] = true; }
    if (f1_en) { ffb_forces_en[1] = true; }
    if (f2_en) { ffb_forces_en[2] = true; }
    if (f3_en) { ffb_forces_en[3] = true; }
  } else if (f_cmd == EnumFfbCmd::STOP_FORCE) {
    if (f0_en) { ffb_forces_en[0] = false; }
    if (f1_en) { ffb_forces_en[1] = false; }
    if (f2_en) { ffb_forces_en[2] = false; }
    if (f3_en) { ffb_forces_en[3] = false; }
  } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_ON) {
    ffb_default_spring_on = true;
    Serial.println("Default spring turned ON");
  } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_OFF) {
    ffb_default_spring_on = false;
    Serial.println("Default spring turned OFF");
  } else if (f_cmd == EnumFfbCmd::REFRESH_FORCE) {
    if (f0_en) {
      ffb_forces[0] = req.refresh_force.force_type;
      ffb_forces_en[0] = true;
    }
    if (f1_en) {
      ffb_forces[1] = req.refresh_force.force_type;
      ffb_forces_en[1] = true;
    }
    if (f2_en) {
      ffb_forces[2] = req.refresh_force.force_type;
      ffb_forces_en[2] = true;
    }
    if (f3_en) {
      ffb_forces[3] = req.refresh_force.force_type;
      ffb_forces_en[3] = true;
    }
  } else if (f_cmd == EnumFfbCmd::SET_DEFAULT_SPRING) {
    ffb_default_spring_k1 = req.set_default_spring.k1;
    ffb_default_spring_k2 = req.set_default_spring.k2;
    ffb_default_spring_clip = req.set_default_spring.clip;
    
    Serial.print("Default spring parameters set - k1: ");
    Serial.print(ffb_default_spring_k1);
    Serial.print(", k2: ");
    Serial.print(ffb_default_spring_k2);
    Serial.print(", clip: ");
    Serial.println(ffb_default_spring_clip);
  }
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
    
    if (should_debug) {
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

    if (f_type == EnumForceType::CONSTANT) {
      int16_t constant_force = *(&f_entry.constant.f0 + i) - 0x80;
      f += constant_force;
      
      if (should_debug) {
        Serial.print("Constant force: ");
        Serial.println(constant_force);
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