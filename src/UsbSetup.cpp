#include "UsbSetup.h"
#include "Constants.h"
#include <Arduino.h>
#include <USB.h>
#include <USBHID.h>
#include "esp_log.h"
#include "common/tusb_common.h"
#include "class/hid/hid_device.h"
#include "device/usbd_pvt.h"
#include "tusb.h"
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
  
  // Define HID report descriptor length (based on reference design)
  uint16_t hid_report_desc_len = 133; // Length from reference design
  
  // Create descriptor with IN endpoint BEFORE OUT endpoint for Windows compatibility
  // Updated to match reference G27 descriptor
  uint8_t descriptor[32] = {
    // Interface descriptor
    9, 0x04, *itf, 0, 2, 0x03, 0, 0, 0,
    
    // HID descriptor - updated to match reference
    9, 0x21, 0x11, 0x01, // bcdHID: 0x0111
    0x21,                // bCountryCode: US (0x21)
    1,                   // bNumDescriptors: 1
    0x22,                // bDescriptorType: HID Report (0x22)
    (uint8_t)(hid_report_desc_len & 0xFF), (uint8_t)((hid_report_desc_len >> 8) & 0xFF), // wDescriptorLength: 133
    
    // CRITICAL: Endpoint IN descriptor (DEVICE TO HOST) MUST COME FIRST for Windows
    7, 0x05, (uint8_t)(0x80 | ep_in), 0x03, 16, 0, 2,  // Updated wMaxPacketSize to 16 and bInterval to 2
    
    // Endpoint OUT descriptor (HOST TO DEVICE) comes second
    7, 0x05, ep_out, 0x03, 16, 0, 2  // Updated wMaxPacketSize to 16 and bInterval to 2
  };
  
  // Copy our custom descriptor to the destination buffer
  memcpy(dst, descriptor, sizeof(descriptor));
  
  // Increment the interface number
  *itf += 1;
  
  // Return the descriptor length
  return sizeof(descriptor);
}

// This is our direct endpoint implementation that doesn't depend on internal TinyUSB structures
extern "C" bool tud_hid_n_report(uint8_t instance, uint8_t report_id, void const* report, uint16_t len)
{
  Serial.println("Custom tud_hid_n_report called");
  
  // Use direct access to the endpoint - don't rely on internal TinyUSB structures
  uint8_t const rhport = 0;
  uint8_t const ep_addr = 0x81; // Hardcoded IN endpoint address
  
  // Create a temporary buffer to hold the report data
  static uint8_t temp_buf[64]; // Make sure this is large enough for your reports
  
  // Format the data with or without report ID
  uint16_t report_len = 0;
  if (report_id) {
    // If using report ID, add it at the beginning
    temp_buf[0] = report_id;
    uint16_t data_len = len > 63 ? 63 : len; // Ensure we don't overflow buffer
    memcpy(temp_buf + 1, report, data_len);
    report_len = data_len + 1;
  } else {
    // If no report ID, just copy the data
    uint16_t data_len = len > 64 ? 64 : len;
    memcpy(temp_buf, report, data_len);
    report_len = data_len;
  }
  
  // Send the report directly using tinyusb's usbd_edpt functions
  if (!usbd_edpt_busy(rhport, ep_addr)) {
    // Only send if endpoint is not busy
    return usbd_edpt_xfer(rhport, ep_addr, temp_buf, report_len);
  }
  
  return false; // Endpoint busy
}

void usb_setup() {
  // Set USB Serial for debugging
  Serial.begin(115200);
  Serial.println("Setting up USB with custom descriptor (IN endpoint first)...");
  
  // Set CPU frequency
  setCpuFrequencyMhz(240);
  
  // Note: CDC mode is disabled through build flag ARDUINO_USB_CDC_ON_BOOT=0
  // which we've confirmed is set in platformio.ini
  
  // Initialize USB with correct VID/PID for G27
  USB.VID(DEV_VID);
  USB.PID(DEV_PID);
  USB.usbAttributes(TUSB_DESC_CONFIG_ATT_SELF_POWERED);
  USB.usbVersion(0x0200);
  USB.usbPower(28);
  USB.manufacturerName(DEV_MANUFACTURER_NAME);
  USB.firmwareVersion(0x1239);
  USB.productName(DEV_PRODUCT_NAME);
  
  // These settings ensure we're using HID class only
  USB.usbClass(0);       // Device class (0 = use interface class)
  USB.usbSubClass(0);    // Device subclass
  USB.usbProtocol(0);    // Device protocol
  
  // CRITICAL: Our custom descriptor implementation ensures:
  // 1. IN endpoint (0x81) is defined BEFORE OUT endpoint (0x01)
  // 2. Country code is set to US (0x21) as in reference
  // 3. Both endpoints use proper wMaxPacketSize (16) and bInterval (2)
  // 4. This matches the descriptor order seen in the working reference device
  // 5. Windows requires this specific order to work correctly
  
  if (!USB.begin()) {
    Serial.println("USB initialization failed!");
    while (1) {
      delay(1000);
      Serial.println("USB init still failed...");
    }
  }
  
  Serial.println("USB initialized successfully with custom descriptor");
  delay(1000); // Give USB time to stabilize
  
  Serial.println("USB Mode: HID only (CDC explicitly disabled)");
  Serial.println("Endpoint order: IN (0x81) first, OUT (0x01) second (Windows compatible)");
  Serial.println("HID descriptor: bcdHID=0x0111, bCountryCode=0x21 (US), wMaxPacketSize=16, bInterval=2");
} 