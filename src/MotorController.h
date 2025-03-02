#pragma once

#include <ESP32Encoder.h>
#include <Arduino.h>
#include "Constants.h"

class MotorController {
public:
    MotorController();
    void init();
    void left();
    void right();
    void stop();
    void home();
    void move(float force);
    void move_to_center();
    float get_position_zero_centered();
    int32_t get_encoder_count();
    
private:
    ESP32Encoder encoder;
    int left_most;
    int right_most;
    int center;
    int range;
}; 