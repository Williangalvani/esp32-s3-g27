#include <Arduino.h>
#include <ESP32Encoder.h>


#define USB_VID 0x046d
#define USB_PID 0xc29b
#define USB_MANUFACTURER "MB"
#define USB_PRODUCT "G27_emu"

#include <stdint.h>
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"



#define DEV_VID (0x046d)
#define DEV_PID (0xc29b)
#define DEV_PRODUCT_NAME "G27_emu"
#define DEV_MANUFACTURER_NAME "MB"
#define DEV_REPORT_ID (0)
#define DEV_REPORT_SIZE (11)
#define DEV_FFB_REQUEST_SIZE (7)

#define DT_REPORT_MS 10  // Increased from 2ms to 10ms to give more processing time
// AS5600 magnetic encoder setup


USBCDC USBSerial;


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
    Serial.println("left_most: " + String(left_most) + " right_most: " + String(right_most) + " center: " + String(center));
    move_to_center();
}


void loop() {
    Serial.println(">pos:" + String((int32_t)encoder.getCount()));
}

void setup() {
    pinMode(MOT_A, OUTPUT);
    pinMode(MOT_B, OUTPUT);
    encoder.attachHalfQuad(13, 14);
    Serial.begin(115200);
    Serial.println("Hello");
    // put your setup code here, to run once:
    last_toggle = millis();
    home();
}

