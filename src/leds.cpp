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
#define SHIFT_CLOCK_TIMEOUT_MS 100   // Timeout for acquiring shift clock semaphore

/*
 * LedController Implementation
 */

LedController::LedController() : 
    initialized(false), 
    led_task_handle(nullptr),
    pattern_counter(0),
    blink_mode(true),
    current_mode(LED_MODE_SINGLE_SCAN) {
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
    // Configure LED pins individually to avoid disturbing other configurations
    
    // Configure LED_LATCH_CLOCK_PIN as output
    gpio_set_direction(LED_LATCH_CLOCK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_LATCH_CLOCK_PIN, GPIO_FLOATING);
    
    // Configure LED_DATA_PIN as output
    gpio_set_direction(LED_DATA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_DATA_PIN, GPIO_FLOATING);
    
    // Set initial pin states
    gpio_set_level(LED_LATCH_CLOCK_PIN, 0);
    gpio_set_level(LED_DATA_PIN, 0);
    
    ESP_LOGI(TAG, "LED GPIO pins initialized individually");
}

void LedController::set_led(uint8_t led_num, bool state) {
    if (led_num >= NUM_LEDS) {
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
    // Acquire the shared shift clock with timeout
    if (!take_shift_clock(SHIFT_CLOCK_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Failed to acquire shift clock semaphore, skipping LED update");
        return;
    }
    
    // Pull latch low to begin data transfer
    gpio_set_level(LED_LATCH_CLOCK_PIN, 0);
    
    // Shift out data MSB first (send the second byte first, then the first byte)
    for (int i = 1; i >= 0; i--) {
        for (int bit = 7; bit >= 0; bit--) {
            // Set data bit
            gpio_set_level(LED_DATA_PIN, (led_state[i] >> bit) & 0x01);
            
            // Pulse the shared shift clock
            gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 1);
            esp_rom_delay_us(1);  // Short delay
            gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 0);
            esp_rom_delay_us(1);  // Short delay
        }
    }
    
    // Pulse latch clock to update outputs
    gpio_set_level(LED_LATCH_CLOCK_PIN, 1);
    esp_rom_delay_us(1);  // Short delay
    gpio_set_level(LED_LATCH_CLOCK_PIN, 0);
    
    // Release the shared shift clock
    release_shift_clock();
}

// Single LED scanning back and forth
void LedController::update_single_scan_pattern() {
    static int led_position = 0;
    static bool direction = true; // true = forward, false = backward
    
    // Clear all LEDs first
    clear_all_leds();
    
    // Turn on only the current LED
    set_led(led_position, true);
    
    // Update position for next time
    if (direction) {
        // Moving forward
        led_position++;
        if (led_position >= NUM_LEDS) {
            // Reached the end, reverse direction
            direction = false;
            led_position = NUM_LEDS - 2; // Go back to second-to-last position
            if (led_position < 0) led_position = 0; // Safety check
        }
    } else {
        // Moving backward
        led_position--;
        if (led_position < 0) {
            // Reached the beginning, reverse direction
            direction = true;
            led_position = 1; // Go to second position
            if (led_position >= NUM_LEDS) led_position = NUM_LEDS - 1; // Safety check
        }
    }
}

// Knight Rider effect (3 LEDs moving)
void LedController::update_knight_rider_pattern() {
    static int led_position = 0;
    static bool direction = true; // true = forward, false = backward
    
    // Clear all LEDs first
    clear_all_leds();
    
    // Turn on 3 LEDs (the main LED and one on each side if possible)
    set_led(led_position, true);
    if (led_position > 0) set_led(led_position - 1, true);
    if (led_position < NUM_LEDS - 1) set_led(led_position + 1, true);
    
    // Update position for next time
    if (direction) {
        // Moving forward
        led_position++;
        if (led_position >= NUM_LEDS - 1) {
            // Reached the end, reverse direction
            direction = false;
        }
    } else {
        // Moving backward
        led_position--;
        if (led_position <= 0) {
            // Reached the beginning, reverse direction
            direction = true;
        }
    }
}

// Binary counting pattern
void LedController::update_binary_count_pattern() {
    // Clear all LEDs first
    clear_all_leds();
    
    // Set LEDs based on counter value (binary representation)
    for (int i = 0; i < NUM_LEDS; i++) {
        if (pattern_counter & (1 << i)) {
            set_led(i, true);
        }
    }
    
    // Increment counter for next time
    pattern_counter = (pattern_counter + 1) % (1 << NUM_LEDS);
}

void LedController::led_task_func(void* pvParameters) {
    LedController* led_ctrl = static_cast<LedController*>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(LED_UPDATE_PERIOD_MS);
    
    ESP_LOGI(led_ctrl->TAG, "LED task started");
    
    while (true) {
        if (led_ctrl->blink_mode) {
            // Update pattern based on current mode
            switch (led_ctrl->current_mode) {
                case LED_MODE_OFF:
                    led_ctrl->clear_all_leds();
                    break;
                
                case LED_MODE_SINGLE_SCAN:
                    led_ctrl->update_single_scan_pattern();
                    break;
                
                case LED_MODE_KNIGHT_RIDER:
                    led_ctrl->update_knight_rider_pattern();
                    break;
                
                case LED_MODE_BINARY_COUNT:
                    led_ctrl->update_binary_count_pattern();
                    break;
                
                case LED_MODE_STATIC:
                    // Do nothing, external code controls the LEDs
                    break;
                
                default:
                    // Invalid mode, default to off
                    led_ctrl->clear_all_leds();
                    break;
            }
        }
        
        // Always update LEDs regardless of mode
        led_ctrl->update_leds();
        
        // Wait for next cycle
        vTaskDelay(xDelay);
    }
}

void LedController::start_led_task() {
    
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

void LedController::set_mode(led_mode_t mode) {
    if (mode < LED_MODE_OFF || mode >= LED_MODE_MAX) {
        ESP_LOGW(TAG, "Invalid LED mode: %d", mode);
        return;
    }
    
    current_mode = mode;
    ESP_LOGI(TAG, "LED mode set to: %d", mode);
    
    // Reset pattern counters when mode changes
    pattern_counter = 0;
}

led_mode_t LedController::get_mode() const {
    return current_mode;
}

void LedController::next_mode() {
    // Cycle to next mode
    led_mode_t next = static_cast<led_mode_t>((current_mode + 1) % LED_MODE_MAX);
    set_mode(next);
} 