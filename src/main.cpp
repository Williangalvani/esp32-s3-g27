#include <Arduino.h>
#include "WheelController.h"
#include "UsbSetup.h"

// Uncomment for debug mode, comment out for production (fewer Serial messages)
// #define DEBUG_MODE 

WheelController whl;
TaskHandle_t TaskSendHidReportsHandle;

void TaskSendHidReports(void *parameter) {
  while (true) {
    static unsigned long time_last_report = millis();
    if (millis() - time_last_report >= DT_REPORT_MS) {
      time_last_report = millis();
      // Send data from device to host on IN endpoint (0x81)
      whl.sendState();
    }
    delay(10);
  }
}

void setup() {
  // Initialize USB with custom descriptor
  // This ensures IN endpoint (0x81) comes before OUT endpoint (0x01)
  usb_setup();
  
  // Short delay to ensure USB is properly initialized
  delay(500);
  
  // Initialize the wheel controller
  whl.init();
  
  // Send multiple state reports to establish proper endpoint ordering
  // This helps Windows recognize the correct endpoint order and descriptor
  // Ensure we send reports at proper intervals (based on bInterval=2)
  Serial.println("Sending initial state reports to establish proper endpoint usage...");
  for (int i = 0; i < 5; i++) {
    whl.sendState();
    delay(4); // Interval of 2 corresponds to 4ms (2 * 2ms)
  }
  
  // Home the motor controller after USB and wheel init
  whl.motor_controller.home();
  
  // HID report update task on core 0 with priority 2
  // Higher priority ensures consistent HID report timing
  xTaskCreatePinnedToCore(
    TaskSendHidReports,
    "TaskSendHidReports", 
    10000,              // Stack size  
    NULL,               // Parameters
    2,                  // Priority (higher priority for consistent timing) 
    &TaskSendHidReportsHandle, 
    0                   // Pin to core 0 (keep core 1 for other tasks)
  );
  
  Serial.println("Setup complete, running main loop");
}

void loop() {
  // Update wheel status
  whl.update_axes();
  whl.update_ffb();
  
  // Short delay to avoid flooding the system
  delay(1);
} 