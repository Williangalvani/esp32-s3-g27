#include "USB.h"
#include "USBHID.h"
#include <Arduino.h>
#include <ESP32Encoder.h>

/*
  ## VID/PID
    046d  Logitech, Inc.
    c298  Driving Force Pro
    c299  G25 Racing Wheel
    c29b  G27 Racing Wheel
    c24f  G29 Driving Force Racing Wheel [PS3]
    c260  G29 Driving Force Racing Wheel [PS4]
*/

#define DEV_VID (0x046d)
#define DEV_PID (0xc29b)
#define DEV_PRODUCT_NAME "G27_emu"
#define DEV_MANUFACTURER_NAME "MB"
#define DEV_REPORT_ID (0)
#define DEV_REPORT_SIZE (11)
#define DEV_FFB_REQUEST_SIZE (7)


#define DT_REPORT_MS 2  // HID report time period
#define DT_MEASURE_US 800  // based on loop() iteration time measurements for ESP32-S2 @ 240MHz CPU, about worst case

#define DT_REPORT_MS 10  // Increased from 2ms to 10ms to give more processing time
// AS5600 magnetic encoder setup

#define MOT_A 11
#define MOT_B 12

ESP32Encoder encoder;

long last_toggle;


void left() {
    digitalWrite(MOT_A, HIGH);
    digitalWrite(MOT_B, LOW);
}

void right() {
    digitalWrite(MOT_A, LOW);
    digitalWrite(MOT_B, HIGH);
}

void stop() {
    digitalWrite(MOT_A, LOW);
    digitalWrite(MOT_B, LOW);
}

int left_most = 0;
int right_most = 0;
int center = 0;
int range = 0;

void move_to_center() {
    while (abs(encoder.getCount() - center) > 10) {
        if (encoder.getCount() < center) {
            right();
        } else {
            left();
        }
        delay(20);
    }
    stop();
}


void home() {
    USBSerial.println("Homing...");
    left();
    delay(1000);
    int last_count = -100;
    while (encoder.getCount() != last_count) {
        last_count = encoder.getCount();
        delay(100);
    }
    left_most = encoder.getCount();
    right();
    delay(1000);
    while (encoder.getCount() != last_count) {
        last_count = encoder.getCount();
        delay(100);
    }
    right_most = encoder.getCount();
    stop();
    center = (left_most + right_most) / 2;
    USBSerial.println("left_most: " + String(left_most) + " right_most: " + String(right_most) + " center: " + String(center));
    USBSerial.println("range: " + String(range));
    range = (right_most - left_most) / 2;
    move_to_center();
}

float get_position_zero_centered() {
    return ((float)encoder.getCount() - center) / (range);
}

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


// HID descriptor compatible with G27
static const uint8_t hid_report_descriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        //   Logical minimum (0)
  0x25, 0x07,        //   Logical maximum (7)
  0x35, 0x00,        //   Physical minimum (0)
  0x46, 0x3B, 0x01,  //   Physical maximum (315)
  0x65, 0x14,        //   Unit (20)
  0x09, 0x39,        //   Usage (Hat switch)
  0x75, 0x04,        //   Report size (4)
  0x95, 0x01,        //   Report count (1)
  0x81, 0x42,        //   Item
  0x65, 0x00,        //   Unit (0)
  0x25, 0x01,        //   Logical maximum (1)
  0x45, 0x01,        //   Physical maximum (1)
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (Button 1)
  0x29, 0x16,        //   Usage Maximum (Button 22)
  0x75, 0x01,        //   Report size (1)
  0x95, 0x16,        //   Report count (22)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x3F,  //   Logical maximum (16383)
  0x46, 0xFF, 0x3F,  //   Physical maximum (16383)
  0x75, 0x0E,        //   Report size (14)
  0x95, 0x01,        //   Report count (1)
  0x05, 0x01,        //   Usage Page (Generic Desktop)
  0x09, 0x30,        //   Usage (X)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x00,  //   Logical maximum (255)
  0x46, 0xFF, 0x00,  //   Physical maximum (255)
  0x75, 0x08,        //   Report size (8)
  0x95, 0x03,        //   Report count (3)
  0x09, 0x32,        //   Usage (Z)
  0x09, 0x35,        //   Usage (Rz)
  0x09, 0x31,        //   Usage (Y)
  0x81, 0x02,        //   Item
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
  0x95, 0x02,        //   Report count (2)
  0x81, 0x02,        //   Item
  0x95, 0x01,        //   Report count (1)
  0x75, 0x01,        //   Report size (1)
  0x25, 0x01,        //   Logical maximum (1)
  0x45, 0x01,        //   Physical maximum (1)
  0x05, 0x09,        //   Usage Page (Button)
  0x09, 0x17,        //   Usage (Button 23)
  0x81, 0x02,        //   Item
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
  0x95, 0x07,        //   Report count (7)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x00,  //   Logical maximum (255)
  0x46, 0xFF, 0x00,  //   Physical maximum (255)
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x02,        //   Usage (Vendor-defined {ff00:2))
  0x95, 0x07,        //   Report count (7)
  0x75, 0x08,        //   Report size (8)
  0x91, 0x02,        //   Item
  0x95, 0x90,        //   Report count (144)
  0x09, 0x03,        //   Usage (Vendor-defined {ff00:3))
  0xB1, 0x02,        //   Item
  0xC0,              // End Collection
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


void usb_setup() {
 // Set CPU frequency
  setCpuFrequencyMhz(240);
  
  // Initialize Serial for debugging
  USBSerial.begin();
  delay(1000); // Give serial time to initialize
  
  char buf[128];
  snprintf(buf, sizeof(buf), "CPU frequency set to %d MHz\n", getCpuFrequencyMhz());
  USBSerial.print(buf);
  
  USBSerial.println("\n==================");
  USBSerial.println("G27 Wheel Emulator");
  USBSerial.println("==================\n");

  // Initialize USB with correct VID/PID
  USBSerial.println("Initializing USB...");
  USB.VID(DEV_VID);
  USB.PID(DEV_PID);
  USB.manufacturerName(DEV_MANUFACTURER_NAME);
  USB.productName(DEV_PRODUCT_NAME);
  
  if (!USB.begin()) {
    USBSerial.println("USB initialization failed!");
    while (1) {
      delay(1000);
      USBSerial.println("USB init still failed...");
    }
  }
  
  USBSerial.println("USB initialized successfully");
  delay(1000); // Give USB time to stabilize
  USBSerial.begin(115200);  // use only for debug, it interferes with hid data sent to host when used too often; TODO disable later
}


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
    uint8_t axis_throttle;
    uint8_t axis_brake;
    uint8_t axis_clutch;
    uint8_t vendor_specific[3];  // not sure what it is, but probably not important

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


// computing forces to apply to the wheel ffb motor
class FfbController {
public:
  FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range)
    : force_current(0x7f),
      ffb_forces_en{ 0, 0, 0, 0 } {
    axis_wheel_cnt = axis_wheel_center;
    axis_wheel_min = axis_wheel_center - axis_wheel_range / 2;
    axis_wheel_max = axis_wheel_center + axis_wheel_range / 2;
  }

  void apply_force(const FfbRequest &req) {
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
  void update(uint16_t axis_wheel_value) {
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

  uint8_t get_force() {
    return force_current;
  }

  uint16_t axis_wheel_min;
  uint16_t axis_wheel_cnt;
  uint16_t axis_wheel_max;

  uint8_t force_current;

  FfbRequest ffb_request;
  FfbForceType ffb_forces[4];
  bool ffb_forces_en[4];
  bool ffb_default_spring_on;
  uint8_t ffb_default_spring_k1;
  uint8_t ffb_default_spring_k2;
  uint8_t ffb_default_spring_clip;
};

// main wheel class, glues everything together (reading wheel angle, setting ffb force, etc)
// TODO cut out motor handling to another class
// TODO cut out potentiometer reading to another class, make it more modular to support also hall sensor, etc

class WheelController : public USBHIDDevice {
public:
  WheelController()
    : status{},
      ffb_controller(0x1fff, 0x3fff) {
    hid.addDevice(this, sizeof(hid_report_descriptor));
    status.status.buttons_0 = 0x08;
  }

  WheelStatus status;
  FfbController ffb_controller;
  USBHID hid;

  void init() {
    hid.begin();
  }

  void update_ffb() {
  }

  void update_axes() {
    update_axis_wheel();
    
    // Randomly toggle button 0 every ~500ms to verify HID updates
    static uint32_t last_toggle = 0;
    if (millis() - last_toggle >= 500) {
      last_toggle = millis();
      // Randomly set or clear bit 0 of buttons_0
      if (random(2) == 0) {
        status.status.buttons_0 |= 0x01;  // Set bit 0
      } else {
        status.status.buttons_0 &= ~0x01; // Clear bit 0
      }
      
    USBSerial.println(">pos:" + String((int32_t)encoder.getCount()));
    }
  }

  void update_axis_wheel() {
    status.status.set_axis_wheel_float(get_position_zero_centered(), true);
  }

  void sendState() {
    uint8_t ffb_force_req = 0;

    if (hid.ready()) {
      hid.SendReport(DEV_REPORT_ID, status.bytes, sizeof(status.bytes), 0);
    }
  }

  // we set hid report descriptor in this callback, writing it to buffer provided
  uint16_t _onGetDescriptor(uint8_t *buffer) override {
    memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
    return sizeof(hid_report_descriptor);
  }

  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) override {}

  // we get ffb request from host in this callback
  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {
    if (len != sizeof(FfbRequest)) return;
    ffb_controller.apply_force(*((const FfbRequest *)buffer));
    // USBSerial.print("[IN]["); USBSerial.print(report_id); USBSerial.print("] ");
    // for (uint16_t i = 0; i < len; ++i)
    // {
    //   if (buffer[i] < 0x10) USBSerial.print(0);
    //   USBSerial.print(buffer[i], HEX);
    //   USBSerial.print(" ");
    // }
    // USBSerial.println();
  }

  void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {}
};


WheelController whl;

TaskHandle_t TaskSendHidReportsHandle;

void TaskSendHidReports(void *parameter) {
  while (true) {
    static unsigned long time_last_report = millis();
    if (millis() - time_last_report >= DT_REPORT_MS) {
      time_last_report = millis();
      whl.sendState();
    }
    delay(1);
  }
}

void setup() {
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
  encoder.attachHalfQuad(13, 14);
  usb_setup();
  whl.init();
  // HID report update task on core 0 with priority 1 (was stalling when was 0), the rest runs on core 1
  xTaskCreatePinnedToCore(TaskSendHidReports, "TaskSendHidReports", 10000, NULL, 1, &TaskSendHidReportsHandle, 0);
  home();
}


void loop() {
    whl.update_axes();
    whl.update_ffb();
    
  delay(1);
}
