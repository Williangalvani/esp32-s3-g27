#pragma once

#include <ESP32Encoder.h>
#include <Arduino.h>
#include "Constants.h"

// PWM configuration for motor control
#define PWM_FREQ 20000  // 20kHz
#define PWM_RESOLUTION 8  // 8-bit resolution (0-255)
#define PWM_CHANNEL_A 0   // LEDC channel for MOT_A
#define PWM_CHANNEL_B 1   // LEDC channel for MOT_B

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