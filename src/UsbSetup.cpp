#include "UsbSetup.h"
#include "Constants.h"
#include <Arduino.h>
#include <USB.h>
#include "esp_log.h"

// CRITICAL: This is our custom replacement for the TinyUSB descriptor generator
// This ensures the endpoints are defined in the order Windows expects:
// IN endpoint (0x81) FIRST, then OUT endpoint (0x01)
// To handle the linker error, we need to add --allow-multiple-definition to LDFLAGS
extern "C" uint16_t tusb_hid_load_descriptor(uint8_t * dst, uint8_t * itf) {
  // This function is called by TinyUSB to load the HID descriptor
  // We're deliberately replacing the default implementation to fix endpoint order
  
  // Get endpoints from TinyUSB - we'll use endpoint 1 for both directions
  uint8_t ep_in = 1;  // Use endpoint 1 for IN (will become 0x81)
  uint8_t ep_out = 1; // Use endpoint 1 for OUT (will become 0x01)
  
  // String descriptor index
  uint8_t str_index = 0; // Default string index
  
  // Create descriptor with IN endpoint BEFORE OUT endpoint for Windows compatibility
  uint8_t descriptor[32] = {
    // Interface descriptor
    9, 0x04, *itf, 0, 2, 0x03, 0, 0, str_index,
    
    // HID descriptor
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof(hid_report_descriptor), 0,
    
    // CRITICAL: Endpoint IN descriptor (DEVICE TO HOST) MUST COME FIRST for Windows
    7, 0x05, (uint8_t)(0x80 | ep_in), 0x03, 64, 0, 1,
    
    // Endpoint OUT descriptor (HOST TO DEVICE) comes second
    7, 0x05, ep_out, 0x03, 64, 0, 1
  };
  
  // Copy our custom descriptor to the destination buffer
  memcpy(dst, descriptor, sizeof(descriptor));
  
  // Increment the interface number
  *itf += 1;
  
  // Return the descriptor length
  return sizeof(descriptor);
}

void usb_setup() {
  // Set USB Serial for debugging
  Serial.begin(115200);
  Serial.println("Setting up USB with custom descriptor (IN endpoint first)...");
  
  // Set CPU frequency
  setCpuFrequencyMhz(240);
  
  // Note: CDC mode is disabled through build flag ARDUINO_USB_CDC_ON_BOOT=0
  // which we've confirmed is set in platformio.ini
  
  // Initialize USB with correct VID/PID
  USB.VID(DEV_VID);
  USB.PID(DEV_PID);
  USB.manufacturerName(DEV_MANUFACTURER_NAME);
  USB.firmwareVersion(0x1239);
  USB.productName(DEV_PRODUCT_NAME);
  
  // These settings ensure we're using HID class only
  USB.usbClass(0);       // Device class (0 = use interface class)
  USB.usbSubClass(0);    // Device subclass
  USB.usbProtocol(0);    // Device protocol
  
  // CRITICAL: Our custom descriptor implementation ensures:
  // 1. IN endpoint (0x81) is defined BEFORE OUT endpoint (0x01)
  // 2. This matches the descriptor order seen in the working device
  // 3. Windows requires this specific order to work correctly
  
  if (!USB.begin()) {
    Serial.println("USB initialization failed!");
    while (1) {
      delay(1000);
      Serial.println("USB init still failed...");
    }
  }
  
  Serial.println("USB initialized successfully with custom descriptor");
  delay(1000); // Give USB time to stabilize
  
  Serial.println("USB Mode: HID only (CDC disabled via ARDUINO_USB_CDC_ON_BOOT=0)");
  Serial.println("Endpoint order: IN (0x81) first, OUT (0x01) second (Windows compatible)");
} 