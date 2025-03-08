#include "shared_pins.h"
#include "esp_log.h"

static const char* TAG = "SHARED_PINS";

// Define the semaphore
SemaphoreHandle_t shift_clock_semaphore = NULL;

void shared_pins_init() {
    // Create the binary semaphore
    shift_clock_semaphore = xSemaphoreCreateBinary();
    
    if (shift_clock_semaphore == NULL) {
        ESP_LOGE(TAG, "Failed to create shift clock semaphore!");
    } else {
        // Make the semaphore available initially
        xSemaphoreGive(shift_clock_semaphore);
        ESP_LOGI(TAG, "Shift clock semaphore created successfully");
    }
    
    // Configure shared clock pin as output using pin-specific functions
    gpio_set_direction(SHARED_SHIFT_CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(SHARED_SHIFT_CLOCK_PIN, GPIO_FLOATING);
    
    // Set initial state
    gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 0);
    ESP_LOGI(TAG, "Shared shift clock pin initialized individually");
}

bool take_shift_clock(uint32_t timeout_ms) {
    if (shift_clock_semaphore == NULL) {
        ESP_LOGE(TAG, "Shift clock semaphore not initialized!");
        return false;
    }
    
    if (xSemaphoreTake(shift_clock_semaphore, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        ESP_LOGD(TAG, "Acquired shift clock semaphore");
        return true;
    } else {
        ESP_LOGW(TAG, "Failed to acquire shift clock semaphore (timeout)");
        return false;
    }
}

void release_shift_clock() {
    if (shift_clock_semaphore == NULL) {
        ESP_LOGE(TAG, "Shift clock semaphore not initialized!");
        return;
    }
    
    xSemaphoreGive(shift_clock_semaphore);
    ESP_LOGD(TAG, "Released shift clock semaphore");
} 