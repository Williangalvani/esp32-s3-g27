#include "leds.h"
#include <cstring>
#include "esp_rom_sys.h"

// Define LOG TAGs
const char* LedController::TAG = "LED_CTRL";

// Task stack sizes and priorities
#define LED_TASK_STACK_SIZE 2048
#define LED_TASK_PRIORITY 5

// Task periods
#define LED_UPDATE_PERIOD_MS 100     // Update LEDs every 100ms for smoother animation

/*
 * LedController Implementation
 */

LedController::LedController() : 
    initialized(false), 
    led_task_handle(nullptr),
    pattern_counter(0),
    blink_mode(true) {
    memset(led_state, 0, sizeof(led_state));
}

LedController::~LedController() {
    stop_led_task();
}

void LedController::init() {
    if (!initialized) {
        ESP_LOGI(TAG, "Initializing LED controller");
        init_gpio();
        clear_all_leds();
        update_leds();
        initialized = true;
    }
}

void LedController::init_gpio() {
    // Configure LED shift register pins as outputs
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << LED_SHIFT_CLOCK_PIN) | 
                           (1ULL << LED_LATCH_CLOCK_PIN) | 
                           (1ULL << LED_DATA_PIN);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
    
    // Set initial pin states
    gpio_set_level(LED_SHIFT_CLOCK_PIN, 0);
    gpio_set_level(LED_LATCH_CLOCK_PIN, 0);
    gpio_set_level(LED_DATA_PIN, 0);
}

void LedController::set_led(uint8_t led_num, bool state) {
    if (led_num >= 16) {
        ESP_LOGW(TAG, "Invalid LED number: %d", led_num);
        return;
    }
    
    uint8_t byte_idx = led_num / 8;
    uint8_t bit_pos = led_num % 8;
    
    if (state) {
        led_state[byte_idx] |= (1 << bit_pos);
    } else {
        led_state[byte_idx] &= ~(1 << bit_pos);
    }
}

void LedController::set_all_leds(uint16_t led_pattern) {
    led_state[0] = led_pattern & 0xFF;
    led_state[1] = (led_pattern >> 8) & 0xFF;
}

void LedController::clear_all_leds() {
    memset(led_state, 0, sizeof(led_state));
}

void LedController::update_leds() {
    // Pull latch low to begin data transfer
    gpio_set_level(LED_LATCH_CLOCK_PIN, 0);
    
    // Shift out data MSB first (send the second byte first, then the first byte)
    for (int i = 1; i >= 0; i--) {
        for (int bit = 7; bit >= 0; bit--) {
            // Set data bit
            gpio_set_level(LED_DATA_PIN, (led_state[i] >> bit) & 0x01);
            
            // Pulse the shift clock
            gpio_set_level(LED_SHIFT_CLOCK_PIN, 1);
            esp_rom_delay_us(1);  // Short delay
            gpio_set_level(LED_SHIFT_CLOCK_PIN, 0);
            esp_rom_delay_us(1);  // Short delay
        }
    }
    
    // Pulse latch clock to update outputs
    gpio_set_level(LED_LATCH_CLOCK_PIN, 1);
    esp_rom_delay_us(1);  // Short delay
    gpio_set_level(LED_LATCH_CLOCK_PIN, 0);
}

void LedController::led_task_func(void* pvParameters) {
    LedController* led_ctrl = static_cast<LedController*>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(LED_UPDATE_PERIOD_MS);
    
    ESP_LOGI(led_ctrl->TAG, "LED task started");
    
    // We have 10 LEDs, so we'll create a simpler pattern
    // that shifts a single LED back and forth
    const int num_leds = 10;
    int led_position = 0;
    bool direction = true; // true = forward, false = backward
    
    while (true) {
        if (led_ctrl->blink_mode) {
            // Clear all LEDs first
            led_ctrl->clear_all_leds();
            
            // Turn on only the current LED
            led_ctrl->set_led(led_position, true);
            
            // Update position for next time
            if (direction) {
                // Moving forward
                led_position++;
                if (led_position >= num_leds) {
                    // Reached the end, reverse direction
                    direction = false;
                    led_position = num_leds - 2; // Go back to second-to-last position
                    if (led_position < 0) led_position = 0; // Safety check
                }
            } else {
                // Moving backward
                led_position--;
                if (led_position < 0) {
                    // Reached the beginning, reverse direction
                    direction = true;
                    led_position = 1; // Go to second position
                    if (led_position >= num_leds) led_position = num_leds - 1; // Safety check
                }
            }
        }
        
        // Always update LEDs regardless of mode
        led_ctrl->update_leds();
        
        // Wait for next cycle
        vTaskDelay(xDelay);
    }
}

void LedController::start_led_task() {
    while (!initialized) {
        ESP_LOGW(TAG, "Cannot start LED task: controller not initialized... waiting....");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    if (led_task_handle == nullptr) {
        ESP_LOGI(TAG, "Starting LED task");
        xTaskCreate(
            led_task_func,
            "led_task",
            LED_TASK_STACK_SIZE,
            this,
            LED_TASK_PRIORITY,
            &led_task_handle
        );
    } else {
        ESP_LOGW(TAG, "LED task already running");
    }
}

void LedController::stop_led_task() {
    if (led_task_handle != nullptr) {
        ESP_LOGI(TAG, "Stopping LED task");
        vTaskDelete(led_task_handle);
        led_task_handle = nullptr;
    }
}

void LedController::set_blink_mode(bool enable) {
    blink_mode = enable;
    if (!enable) {
        clear_all_leds();
        update_leds();
    }
} 