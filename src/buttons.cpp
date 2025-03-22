#include "buttons.h"
#include <cstring>
#include "esp_rom_sys.h"

// Define LOG TAG
const char* ButtonController::TAG = "BTN_CTRL";

// Task stack sizes and priorities
#define BUTTON_TASK_STACK_SIZE 4096  // Doubled from 2048 to 4096
#define BUTTON_TASK_PRIORITY 5

// Task periods
#define BUTTON_SCAN_PERIOD_MS 20    // Scan buttons every 20ms
#define SHIFT_CLOCK_TIMEOUT_MS 50   // Timeout for acquiring shift clock semaphore

/*
 * ButtonController Implementation
 */

// Default constructor for backward compatibility
ButtonController::ButtonController() : 
    initialized(false),
    button_state(0),
    last_button_state(0),
    button_task_handle(nullptr),
    pl_pin(DEFAULT_BUTTONS_PL_PIN),
    clk_pin(DEFAULT_BUTTONS_CLK_INH_PIN),
    q7_pin(DEFAULT_BUTTONS_Q7_PIN),
    num_buttons(DEFAULT_NUM_BUTTONS),
    use_as_clock(false), // Default controller uses clk_pin as CLK_INH
    button_callback(nullptr) {
}

ButtonController::ButtonController(gpio_num_t pl_pin, gpio_num_t clk_pin, gpio_num_t q7_pin, uint8_t num_buttons, bool use_as_clock) : 
    initialized(false),
    button_state(0),
    last_button_state(0),
    button_task_handle(nullptr),
    pl_pin(pl_pin),
    clk_pin(clk_pin),
    q7_pin(q7_pin),
    num_buttons(num_buttons > MAX_BUTTONS ? MAX_BUTTONS : num_buttons), // Limit to MAX_BUTTONS
    use_as_clock(use_as_clock),
    button_callback(nullptr) {
}

ButtonController::~ButtonController() {
    stop_button_task();
}

void ButtonController::init() {
    if (!initialized) {
        if (use_as_clock) {
            ESP_LOGI(TAG, "Initializing Button controller with PL:%d, CLK:%d, Q7:%d, buttons:%d", 
                    pl_pin, clk_pin, q7_pin, num_buttons);
        } else {
            ESP_LOGI(TAG, "Initializing Button controller with PL:%d, CLK_INH:%d, Q7:%d, buttons:%d", 
                    pl_pin, clk_pin, q7_pin, num_buttons);
        }
        init_gpio();
        read_buttons(); // Initial read
        last_button_state = button_state;
        initialized = true;
    }
}

void ButtonController::init_gpio() {
    // Configure pins individually to avoid disturbing other configurations
    
    // Configure PL_PIN as output
    ESP_LOGI(TAG, "Configuring PL pin %d as output", pl_pin);
    gpio_config_t pl_config = {
        .pin_bit_mask = (1ULL << pl_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&pl_config);
    
    // Configure CLK pin
    if (use_as_clock) {
        ESP_LOGI(TAG, "Configuring CLK pin %d as output (used as direct clock)", clk_pin);
    } else {
        ESP_LOGI(TAG, "Configuring CLK_INH pin %d as output", clk_pin);
    }
    
    gpio_reset_pin(clk_pin);  // Reset the pin to default state first
    
    gpio_config_t clk_config = {
        .pin_bit_mask = (1ULL << clk_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&clk_config);
    
    // Configure Q7_PIN as input with pull-up
    ESP_LOGI(TAG, "Configuring Q7 pin %d as input with pull-up", q7_pin);
    gpio_config_t q7_config = {
        .pin_bit_mask = (1ULL << q7_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&q7_config);
    
    // Test toggling CLK pin to verify it's working
    if (use_as_clock) {
        ESP_LOGI(TAG, "Testing CLK pin %d - pulsing", clk_pin);
        for (int i = 0; i < 5; i++) {
            gpio_set_level(clk_pin, 0);
            esp_rom_delay_us(10000);  // 10ms delay
            gpio_set_level(clk_pin, 1);
            esp_rom_delay_us(10000);  // 10ms delay
            gpio_set_level(clk_pin, 0);
            esp_rom_delay_us(10000);  // 10ms delay
        }
    } else {
        ESP_LOGI(TAG, "Testing CLK_INH pin %d - setting HIGH/LOW", clk_pin);
        gpio_set_level(clk_pin, 1);  // Set CLK_INH HIGH (inhibit clock)
        esp_rom_delay_us(50000);     // 50ms delay for clear observation
        gpio_set_level(clk_pin, 0);  // Set CLK_INH LOW (enable clock)
        esp_rom_delay_us(50000);     // 50ms delay
    }
    
    // Set initial pin states
    gpio_set_level(pl_pin, 1);     // Start with PL high (shift mode)
    gpio_set_level(clk_pin, 0);    // Start with clock low
    
    // Test toggling PL pin to verify it's working
    for (int i = 0; i < 3; i++) {
        ESP_LOGI(TAG, "Testing PL pin %d toggle: LOW", pl_pin);
        gpio_set_level(pl_pin, 0);
        esp_rom_delay_us(5000);  // 5ms delay
        
        ESP_LOGI(TAG, "Testing PL pin %d toggle: HIGH", pl_pin);
        gpio_set_level(pl_pin, 1);
        esp_rom_delay_us(5000);  // 5ms delay
    }
    
    ESP_LOGI(TAG, "Button GPIO pins initialized for PL:%d, CLK:%d, Q7:%d", pl_pin, clk_pin, q7_pin);
}

void ButtonController::read_buttons() {
    // Save the previous state
    last_button_state = button_state;
    
    // Safety check - ensure we're not reading more buttons than allowed
    if (num_buttons > MAX_BUTTONS) {
        ESP_LOGW(TAG, "Button count exceeds maximum, limiting to %d", MAX_BUTTONS);
        num_buttons = MAX_BUTTONS;
    }
    
    // For custom clock pin, we don't need to acquire the shared clock
    if (!use_as_clock) {
        // Acquire the shared shift clock with timeout
        if (!take_shift_clock(SHIFT_CLOCK_TIMEOUT_MS)) {
            ESP_LOGW(TAG, "Failed to acquire shift clock semaphore, skipping button read");
            return;
        }
    }
    
    // Step 1: Pulse PL low to parallel load the button states
    ESP_LOGV(TAG, "Setting PL pin %d LOW for parallel load", pl_pin);
    gpio_set_level(pl_pin, 0);
    esp_rom_delay_us(50);  // Increased delay for more reliable loading
    
    ESP_LOGV(TAG, "Setting PL pin %d HIGH to capture button state", pl_pin);
    gpio_set_level(pl_pin, 1);
    esp_rom_delay_us(50);  // Increased delay
    
    // If using CLK_INH, make sure it's disabled (set low)
    if (!use_as_clock) {
        gpio_set_level(clk_pin, 0);  // Enable clock (CLK_INH low)
    }
    
    // Step 2: Read num_buttons bits of button data by clocking
    uint16_t new_state = 0;
    
    for (int i = 0; i < num_buttons && i < MAX_BUTTONS; i++) {  // Added safety bound check
        // Read Q7 pin first (before clock edge)
        bool bit_value = gpio_get_level(q7_pin);
        ESP_LOGV(TAG, "Button bit %d read from Q7 pin %d: %d", i, q7_pin, bit_value);
        
        // Shift left and set LSB based on read value
        new_state = (new_state << 1) | (bit_value ? 1 : 0);
        
        // Pulse the appropriate clock pin
        if (use_as_clock) {
            // Using dedicated clock pin
            gpio_set_level(clk_pin, 1);
            esp_rom_delay_us(50);  // Increased delay for more reliable shifting
            gpio_set_level(clk_pin, 0);
            esp_rom_delay_us(50);  // Increased delay
        } else {
            // Using shared clock pin
            gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 1);
            esp_rom_delay_us(50);  // Increased delay for more reliable shifting
            gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 0);
            esp_rom_delay_us(50);  // Increased delay
        }
    }
    
    // Release the shared shift clock if we acquired it
    if (!use_as_clock) {
        release_shift_clock();
    }
    
    // Update the button state
    button_state = new_state;
    
    // Periodically log the button state in hex format
    static int log_counter = 0;
    if (++log_counter % 100 == 0) {  // Reduced logging frequency
        ESP_LOGI(TAG, "Button controller (PL:%d) state: 0x%04x", pl_pin, button_state);
    }
}

bool ButtonController::get_button(uint8_t button_num) {
    if (button_num >= num_buttons) {
        ESP_LOGW(TAG, "Invalid button number: %d", button_num);
        return false;
    }
    
    return (button_state & (1 << button_num)) != 0;
}

uint16_t ButtonController::get_all_buttons() {
    return button_state;
}

bool ButtonController::button_pressed(uint8_t button_num) {
    if (button_num >= num_buttons) {
        return false;
    }
    
    // Check for rising edge (was 0, now 1)
    return ((~last_button_state & button_state) & (1 << button_num)) != 0;
}

bool ButtonController::button_released(uint8_t button_num) {
    if (button_num >= num_buttons) {
        return false;
    }
    
    // Check for falling edge (was 1, now 0)
    return ((last_button_state & ~button_state) & (1 << button_num)) != 0;
}

void ButtonController::button_task_func(void* pvParameters) {
    ButtonController* btn_ctrl = static_cast<ButtonController*>(pvParameters);
    const TickType_t xDelay = pdMS_TO_TICKS(BUTTON_SCAN_PERIOD_MS);
    
    ESP_LOGI(btn_ctrl->TAG, "Button task started");
    
    while (true) {
        // Read button states
        btn_ctrl->read_buttons();
        
        // Wait for next scan
        vTaskDelay(xDelay);
    }
}

void ButtonController::start_button_task() {
    if (button_task_handle == nullptr) {
        // Create a unique task name based on the PL pin number
        char task_name[16];
        snprintf(task_name, sizeof(task_name), "btn_task_%d", pl_pin);
        
        ESP_LOGI(TAG, "Starting Button task with name %s", task_name);
        xTaskCreate(
            button_task_func,
            task_name,
            BUTTON_TASK_STACK_SIZE,
            this,
            BUTTON_TASK_PRIORITY,
            &button_task_handle
        );
    } else {
        ESP_LOGW(TAG, "Button task already running");
    }
}

void ButtonController::stop_button_task() {
    if (button_task_handle != nullptr) {
        ESP_LOGI(TAG, "Stopping Button task");
        vTaskDelete(button_task_handle);
        button_task_handle = nullptr;
    }
}

void ButtonController::register_callback(button_callback_t callback) {
    button_callback = callback;
}

// Simulate button presses for testing (can be controlled by serial input or other means)
void ButtonController::simulate_button_press(uint8_t button_num, bool pressed) {
    if (button_num >= num_buttons) {
        ESP_LOGW(TAG, "Invalid button number for simulation: %d", button_num);
        return;
    }
    
    // Save the previous state
    last_button_state = button_state;
    
    // Update the button state
    if (pressed) {
        button_state |= (1 << button_num);
    } else {
        button_state &= ~(1 << button_num);
    }
    
    // Trigger the callback if registered and button state changed
    if (button_callback != nullptr && ((button_state ^ last_button_state) & (1 << button_num))) {
        button_callback(button_num, pressed);
    }
    
    ESP_LOGI(TAG, "Simulated button %d %s", button_num, pressed ? "pressed" : "released");
} 