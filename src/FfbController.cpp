#include "FfbController.h"
#include <Arduino.h>

FfbController::FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range)
  : force_current(0x7f),
    ffb_forces_en{ 0, 0, 0, 0 } {
  axis_wheel_cnt = axis_wheel_center;
  axis_wheel_min = axis_wheel_center - axis_wheel_range / 2;
  axis_wheel_max = axis_wheel_center + axis_wheel_range / 2;
}

void FfbController::apply_force(const FfbRequest &req) {
  EnumFfbCmd f_cmd = (EnumFfbCmd)(req.bytes[0] & 0x0f);

  // force slots
  bool f0_en = req.bytes[0] & 0x10;
  bool f1_en = req.bytes[0] & 0x20;
  bool f2_en = req.bytes[0] & 0x40;
  bool f3_en = req.bytes[0] & 0x80;

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
  } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_OFF) {
    ffb_default_spring_on = false;
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
  }
}

// sets force in range 0x00 .. 0xff to force_current
void FfbController::update(uint16_t axis_wheel_value) {
  int16_t f = 0;

  if (ffb_default_spring_on)
  {
    int16_t k = (axis_wheel_value > axis_wheel_cnt) ? ffb_default_spring_k1 : -ffb_default_spring_k2;
    k *= ffb_default_spring_clip;
    k /= 7;
    f += k * abs(axis_wheel_value - axis_wheel_cnt) * 2 / (axis_wheel_max - axis_wheel_min);
  }

  for (uint8_t i = 0; i < 4; ++i) {
    bool f_en = ffb_forces_en[i];
    if (!f_en) continue;
    if (i != 0) continue;

    FfbForceType f_entry = ffb_forces[i];
    EnumForceType f_type = (EnumForceType)f_entry.bytes[0];

    if (f_type == EnumForceType::CONSTANT) {
      f += *(&f_entry.constant.f0 + i) - 0x7f;
    } else if (f_type == EnumForceType::SPRING) {
    } else if (f_type == EnumForceType::DAMPER) {
    } else if (f_type == EnumForceType::AUTO_CNT_SPRING) {
    } else if (f_type == EnumForceType::SAWTOOTH_UP) {
    } else if (f_type == EnumForceType::SAWTOOTH_DN) {
    } else if (f_type == EnumForceType::TRAPEZOID) {
    } else if (f_type == EnumForceType::RECTANGLE) {
    } else if (f_type == EnumForceType::VARIABLE) {
    } else if (f_type == EnumForceType::RAMP) {
    } else if (f_type == EnumForceType::SQUARE_WAVE) {
    } else if (f_type == EnumForceType::HI_RES_SPRING) {
    } else if (f_type == EnumForceType::HI_RES_DAMPER) {
    } else if (f_type == EnumForceType::HI_RES_AUTO_CNT_SPRING) {
    } else if (f_type == EnumForceType::FRICTION) {
    }
  }

  f += 0x7f;

  if (f < 0x00) f = 0x00;
  if (f > 0xff) f = 0xff;

  force_current = f;
}

uint8_t FfbController::get_force() {
  return force_current;
} 