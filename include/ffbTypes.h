#pragma once

#include <cstdint>

// Define FFB_REQUEST_SIZE here to avoid circular dependencies
#define DEV_FFB_REQUEST_SIZE 19

// ffb commands according to Logitech Force Feedback Protocol V1.6
enum class EnumFfbCmd {
  DOWNLOAD_FORCE = 0x00,
  DOWNLOAD_AND_PLAY_FORCE = 0x01,
  PLAY_FORCE = 0x02,
  STOP_FORCE = 0x03,
  DEFAULT_SPRING_ON = 0x04,
  DEFAULT_SPRING_OFF = 0x05,
  REFRESH_FORCE = 0x0c,
  FIXED_TIME_LOOP = 0x0d,
  SET_DEFAULT_SPRING = 0x0e,
  SET_DEAD_BAND = 0x0f,
  EXTENDED_COMMAND = 0xF8,
};

enum class EnumExtendedCommand : uint8_t {
  CHANGE_MODE_TO_DRIVING_FORCE_PRO = 0xF8,
  CHANGE_WHEEL_RANGE_TO_200_DEGREES = 0x81,
  CHANGE_WHEEL_RANGE_TO_900_DEGREES = 0xF5,
  CHANGE_DEVICE_MODE = 0xF8,
  REVERT_IDENTITY = 0x01,
  SWITCH_TO_G25_IDENTITY_WITH_USB_DETACH = 0xF8,
  SWITCH_TO_G25_IDENTITY_WITHOUT_USB_DETACH = 0xF8,
  SET_RPM_LEDS = 0x80,
  WHEEL_RANGE_CHANGE = 0xF8
};

// ffb force types according to Logitech Force Feedback Protocol V1.6
enum class EnumForceType : uint8_t {
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
  SQUARE_WAVE = 0x0A,
  HI_RES_SPRING = 0x0B,
  HI_RES_DAMPER = 0x0C,
  HI_RES_AUTO_CNT_SPRING = 0x0D,
  FRICTION = 0x0E
};

// union for serialisation/deserialisation
typedef struct {
  uint8_t type;
  uint8_t param0;
  uint8_t param1;
  uint8_t param2;
  uint8_t param3;
  uint8_t zero;
} FfbForceType;

// data from host - force feedback request
// union for serialisation/deserialisation
union FfbRequest {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE];
  
  struct {
    uint8_t cmd;
    uint8_t params[DEV_FFB_REQUEST_SIZE - 1];
  };
};
