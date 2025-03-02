#include "UsbSetup.h"
#include "USB.h"
#include <Arduino.h>

void usb_setup() {
 // Set CPU frequency
  setCpuFrequencyMhz(240);
  
  // Initialize Serial for debugging
  Serial.begin(115200);
  delay(1000); // Give serial time to initialize
  
  char buf[128];
  snprintf(buf, sizeof(buf), "CPU frequency set to %d MHz\n", getCpuFrequencyMhz());
  Serial.print(buf);
  
  Serial.println("\n==================");
  Serial.println("G27 Wheel Emulator");
  Serial.println("==================\n");

  // Initialize USB with correct VID/PID
  Serial.println("Initializing USB...");
  USB.VID(DEV_VID);
  USB.PID(DEV_PID);
  USB.manufacturerName(DEV_MANUFACTURER_NAME);
  USB.productName(DEV_PRODUCT_NAME);
  
  if (!USB.begin()) {
    Serial.println("USB initialization failed!");
    while (1) {
      delay(1000);
      Serial.println("USB init still failed...");
    }
  }
  
  Serial.println("USB initialized successfully");
  delay(1000); // Give USB time to stabilize
  Serial.begin(115200);  // use only for debug, it interferes with hid data sent to host when used too often; TODO disable later
} 