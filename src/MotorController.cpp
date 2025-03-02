#include "MotorController.h"
#include <Arduino.h>

MotorController::MotorController() 
    : left_most(0), right_most(0), center(0), range(0) {
}

void MotorController::init() {
    pinMode(MOT_A, OUTPUT);
    pinMode(MOT_B, OUTPUT);
    encoder.attachHalfQuad(13, 14);
}

void MotorController::left() {
    digitalWrite(MOT_A, HIGH);
    digitalWrite(MOT_B, LOW);
}

void MotorController::right() {
    digitalWrite(MOT_A, LOW);
    digitalWrite(MOT_B, HIGH);
}

void MotorController::stop() {
    digitalWrite(MOT_A, LOW);
    digitalWrite(MOT_B, LOW);
}

void MotorController::move(float force) {
    if (force > 0.01) {
        right();
    } else if (force < -0.01) {

        left();
    } else {

        stop();
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

float MotorController::get_position_zero_centered() {
    return ((float)encoder.getCount() - center) / (range);
}

int32_t MotorController::get_encoder_count() {
    return encoder.getCount();
} 