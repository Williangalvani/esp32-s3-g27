#include <Arduino.h>
#include "WheelController.h"
#include "UsbSetup.h"

WheelController whl;
TaskHandle_t TaskSendHidReportsHandle;

void TaskSendHidReports(void *parameter) {
  while (true) {
    static unsigned long time_last_report = millis();
    if (millis() - time_last_report >= DT_REPORT_MS) {
      time_last_report = millis();
      whl.sendState();
    }
    delay(10);
  }
}

void setup() {
  usb_setup();
  whl.init();
  // HID report update task on core 0 with priority 1 (was stalling when was 0), the rest runs on core 1
  xTaskCreatePinnedToCore(TaskSendHidReports, "TaskSendHidReports", 10000, NULL, 1, &TaskSendHidReportsHandle, 0);
  whl.motor_controller.home();
  Serial.println("Setup complete - ready for force feedback");
}

void loop() {
  whl.update_axes();
  whl.update_ffb();
  delay(1);
} 