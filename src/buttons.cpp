#include "buttons.h"
#include <cstring>
#include "esp_rom_sys.h"

// Define LOG TAG
const char* ButtonController::TAG = "BTN_CTRL";

// Task stack sizes and priorities
#define BUTTON_TASK_STACK_SIZE 2048
#define BUTTON_TASK_PRIORITY 5

// Task periods
#define BUTTON_SCAN_PERIOD_MS 20    // Scan buttons every 20ms
#define SHIFT_CLOCK_TIMEOUT_MS 50   // Timeout for acquiring shift clock semaphore

/*
 * ButtonController Implementation
 */

ButtonController::ButtonController() : 
    initialized(false),
    button_state(0),
    last_button_state(0),
    button_task_handle(nullptr),
    button_callback(nullptr) {
}

ButtonController::~ButtonController() {
    stop_button_task();
}

void ButtonController::init() {
    if (!initialized) {
        ESP_LOGI(TAG, "Initializing Button controller");
        init_gpio();
        read_buttons(); // Initial read
        last_button_state = button_state;
        initialized = true;
    }
}

void ButtonController::init_gpio() {
    // Configure pins individually to avoid disturbing other configurations
    
    // Configure BUTTONS_PL_PIN as output
    gpio_set_direction(BUTTONS_PL_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(BUTTONS_PL_PIN, GPIO_FLOATING);
    
    // Configure BUTTONS_CLK_INH_PIN as output
    gpio_set_direction(BUTTONS_CLK_INH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(BUTTONS_CLK_INH_PIN, GPIO_FLOATING);
    
    // Configure BUTTONS_Q7_PIN as input with pull-up
    gpio_set_direction(BUTTONS_Q7_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUTTONS_Q7_PIN, GPIO_PULLUP_ONLY);
    
    // Set initial pin states
    gpio_set_level(BUTTONS_PL_PIN, 1);       // Start with PL high (shift mode)
    gpio_set_level(BUTTONS_CLK_INH_PIN, 0);  // Clock enabled
    
    ESP_LOGI(TAG, "Button GPIO pins initialized individually");
}

void ButtonController::read_buttons() {
    // Save the previous state
    last_button_state = button_state;
    
    // Acquire the shared shift clock with timeout
    if (!take_shift_clock(SHIFT_CLOCK_TIMEOUT_MS)) {
        ESP_LOGW(TAG, "Failed to acquire shift clock semaphore, skipping button read");
        return;
    }
    
    // Step 1: Pulse PL low to parallel load the button states
    gpio_set_level(BUTTONS_PL_PIN, 0);
    esp_rom_delay_us(5);  // Short delay
    gpio_set_level(BUTTONS_PL_PIN, 1);
    esp_rom_delay_us(5);  // Short delay
    
    // Step 2: Read NUM_BUTTONS bits of button data by clocking
    uint16_t new_state = 0;
    
    // Make sure clock inhibit is disabled
    // gpio_set_level(BUTTONS_CLK_INH_PIN, 0);
    // ESP_LOGI(TAG, "Set clock inhibit to 0");
    for (int i = 0; i < NUM_BUTTONS; i++) {
        // Read Q7 pin first (before clock edge)
        bool bit_value = gpio_get_level(BUTTONS_Q7_PIN);
        
        // Shift left and set LSB based on read value
        new_state = (new_state << 1) | (bit_value ? 1 : 0);
        
        // Pulse the shared clock to shift next bit into Q7
        gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 1);
        esp_rom_delay_us(5);  // Short delay
        gpio_set_level(SHARED_SHIFT_CLOCK_PIN, 0);
        esp_rom_delay_us(5);  // Short delay
    }
    
    // Release the shared shift clock
    release_shift_clock();
    
    // Update the button state
    button_state = new_state;
    
    // // Check for button events and call callback if registered
    // if (button_callback != nullptr) {
    //     uint16_t changed = button_state ^ last_button_state;
    //     for (int i = 0; i < NUM_BUTTONS; i++) {
    //         if (changed & (1 << i)) {
    //             bool pressed = (button_state & (1 << i)) != 0;
    //             // button_callback(i, pressed);
    //         }
    //     }
    // }
    static int16_t print_limiter = 0;
    if (print_limiter % 50 == 0) {
        ESP_LOGI(TAG, "Button state: %d", button_state);
    }
    print_limiter++;
}

bool ButtonController::get_button(uint8_t button_num) {
    if (button_num >= NUM_BUTTONS) {
        ESP_LOGW(TAG, "Invalid button number: %d", button_num);
        return false;
    }
    
    return (button_state & (1 << button_num)) != 0;
}

uint16_t ButtonController::get_all_buttons() {
    return button_state;
}

bool ButtonController::button_pressed(uint8_t button_num) {
    if (button_num >= NUM_BUTTONS) {
        return false;
    }
    
    // Check for rising edge (was 0, now 1)
    return ((~last_button_state & button_state) & (1 << button_num)) != 0;
}

bool ButtonController::button_released(uint8_t button_num) {
    if (button_num >= NUM_BUTTONS) {
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
        ESP_LOGI(TAG, "Starting Button task");
        xTaskCreate(
            button_task_func,
            "button_task",
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