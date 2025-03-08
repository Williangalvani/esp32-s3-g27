#include "esp_log.h"

// Simple C wrapper for the C++ app_main implementation
// This is needed because ESP-IDF's startup code expects app_main in C

void app_main(void)
{
    // Set up logging
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_write(ESP_LOG_INFO, "wrapper", "Starting G27 Racing Wheel emulation");
    
    // Call the C++ implementation
    extern void cpp_app_main(void);
    cpp_app_main();
} 