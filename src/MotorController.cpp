#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController() 
    : left_most(0), right_most(0), center(0), range(0) {
}

void MotorController::init() {
    // Configure LEDC channels for PWM control
    ledcSetup(PWM_CHANNEL_A, PWM_FREQ, PWM_RESOLUTION);
    ledcSetup(PWM_CHANNEL_B, PWM_FREQ, PWM_RESOLUTION);
    
    // Attach LEDC channels to motor pins
    ledcAttachPin(MOT_A, PWM_CHANNEL_A);
    ledcAttachPin(MOT_B, PWM_CHANNEL_B);
    
    // Initialize encoder
    encoder.attachHalfQuad(13, 14);
    wheel_range = 900.0;
}

void MotorController::left() {
    ledcWrite(PWM_CHANNEL_A, 255);
    ledcWrite(PWM_CHANNEL_B, 0);
}

void MotorController::right() {
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 255);
}

void MotorController::stop() {
    ledcWrite(PWM_CHANNEL_A, 0);
    ledcWrite(PWM_CHANNEL_B, 0);
}

void MotorController::move(float force) {
    // Constrain force to be between -1 and 1
    force = constrain(force, -1.0, 1.0);
    
    // Calculate PWM value (0-255) based on the absolute value of force
    int pwm = abs(force) * 255;
    int deadband = 150;
    
    // Ensure we don't exceed 255 when adding deadband
    int pwm_with_deadband = min(deadband + pwm, 255);
    
    if (force > 0.01) {
        // Move right with proportional speed
        ledcWrite(PWM_CHANNEL_A, 0);
        ledcWrite(PWM_CHANNEL_B, pwm_with_deadband);
    } else if (force < -0.01) {
        // Move left with proportional speed
        ledcWrite(PWM_CHANNEL_A, pwm_with_deadband);
        ledcWrite(PWM_CHANNEL_B, 0);
    } else {
        // Stop the motor
        ledcWrite(PWM_CHANNEL_A, 0);
        ledcWrite(PWM_CHANNEL_B, 0);
    }
}

void MotorController::move_to_center() {
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

void MotorController::home() {
    Serial.println("Homing...");
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
    range = (right_most - left_most) / 2;
    Serial.println("left_most: " + String(left_most) + " right_most: " + String(right_most) + " center: " + String(center));
    Serial.println("range: " + String(range));
    move_to_center();
}

void MotorController::set_wheel_range(uint16_t new_range) {
    wheel_range = new_range;
    wheel_range_normalized = (float)wheel_range / 900.0;
}

float MotorController::get_position_zero_centered() {
    return -((float)encoder.getCount() - center) / (range);
}

float MotorController::get_position_zero_centered_normalized() {
    return get_position_zero_centered();
}

int32_t MotorController::get_encoder_count() {
    return encoder.getCount();
} 