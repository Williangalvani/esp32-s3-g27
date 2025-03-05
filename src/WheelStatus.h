#pragma once

#include <stdint.h>

// data to send as HID report to host
// union for serialisation/deserialisation
union WheelStatus {
  uint8_t bytes[11];

  struct Status_t {
    uint8_t buttons_0;
    uint8_t buttons_1;
    uint8_t buttons_2;
    uint8_t axis_wheel_lsb6_and_btns2;
    uint8_t axis_wheel_msb;
    int8_t axis_throttle;
    int8_t axis_brake;
    int8_t axis_clutch;
    uint8_t shifter_x;  // not sure what it is, but probably not important
    uint8_t shifter_y;  // not sure what it is, but probably not important
    uint8_t misc;  // not sure what it is, but probably not important

    void init() {
      buttons_0 = 0x08;
      buttons_1 = 0x00;
      buttons_2 = 0x00;
      axis_wheel_lsb6_and_btns2 = 0x0000;
      axis_wheel_msb = 0x00;
      axis_throttle = 0x30;
      axis_brake = 0x90;
      axis_clutch = 0xFF;
      shifter_x = 0x80;
      shifter_y = 0x80;
      misc = 0b10011100; // todo: implement missing from https://gimx.fr/wiki/index.php?title=G27_PS3
    }
    // center_zero == true : v range -1 .. 1
    // center_zero == false : v range 0 .. 1
    void set_axis_wheel_float(float v, bool center_zero) {
      if (center_zero) v = (v + 1) / 2.0f;
      if (v < 0.0f) v = 0.0f;
      if (v > 1.0f) v = 1.0f;

      v = v * 0x3fff;
      set_axis_wheel_14bit((uint16_t)v);
    }

    void set_axis_wheel_14bit(uint16_t v) {
      axis_wheel_lsb6_and_btns2 = (v & 0x003f) << 2;
      axis_wheel_msb = (v >> 6) & 0x00ff;
    }

    uint16_t get_axis_wheel_14bit() {
      uint16_t v = 0x0000;
      v = (axis_wheel_lsb6_and_btns2 >> 2) & 0x003f;
      v |= (((uint16_t)axis_wheel_msb) << 6) & 0x3fc0;
      return v;
    }
  } status;
}; 