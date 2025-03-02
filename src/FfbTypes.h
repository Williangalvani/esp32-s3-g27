#pragma once

#include "Constants.h"
#include <stdint.h>


// ffb commands according to Logitech Force Feedback Protocol V1.6
enum class EnumFfbCmd {
  DOWNLOAD_FORCE = 0x00,
  DOWNLOAD_AND_PLAY_FORCE = 0x01,
  PLAY_FORCE = 0x02,
  STOP_FORCE = 0x03,
  DEFAULT_SPRING_ON = 0x04,
  DEFAULT_SPRING_OFF = 0x05,
  REFRESH_FORCE = 0x0c,
  SET_DEFAULT_SPRING = 0x0e,
};

// ffb force types according to Logitech Force Feedback Protocol V1.6
enum class EnumForceType {
  CONSTANT = 0x00,
  SPRING = 0x01,
  DAMPER = 0x02,
  AUTO_CNT_SPRING = 0x03,
  SAWTOOTH_UP = 0x04,
  SAWTOOTH_DN = 0x05,
  TRAPEZOID = 0x06,
  RECTANGLE = 0x07,
  VARIABLE = 0x08,
  RAMP = 0x09,
  SQUARE_WAVE = 0x0a,
  HI_RES_SPRING = 0x0b,
  HI_RES_DAMPER = 0x0c,
  HI_RES_AUTO_CNT_SPRING = 0x0d,
  FRICTION = 0x0e
};

// union for serialisation/deserialisation
union FfbForceType {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  struct f00_constant_t {
    uint8_t type;
    uint8_t f0;
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
    uint8_t zero;
  } constant;
  // TODO complete list
};

// data from host - force feedback request
// union for serialisation/deserialisation
union FfbRequest {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE];

  struct cmd00_download_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } download_force;

  struct cmd01_download_and_play_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } download_and_play_force;

  struct cmd02_play_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } play_force;

  struct cmd03_stop_force_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } stop_force;

  struct cmd04_default_spring_on_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } default_spring_on;

  struct cmd05_default_spring_off_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } default_spring_off;

  struct cmd0c_refresh_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } refresh_force;

  struct cmd0e_set_default_spring_t {
    uint8_t cmd;
    uint8_t zero;
    uint8_t k1;
    uint8_t k2;
    uint8_t clip;
    uint8_t zeros[2];
  } set_default_spring;
}; 