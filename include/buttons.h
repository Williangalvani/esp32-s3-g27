#ifndef BUTTONS_H
#define BUTTONS_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "shared_pins.h" // Include shared pins header

// Maximum number of buttons supported
#define MAX_BUTTONS 16

// Default pin definitions for backward compatibility
#define DEFAULT_BUTTONS_PL_PIN ((gpio_num_t)8)
#define DEFAULT_BUTTONS_CLK_INH_PIN ((gpio_num_t)10)
#define DEFAULT_BUTTONS_Q7_PIN ((gpio_num_t)15)
#define DEFAULT_NUM_BUTTONS 8

// Shifter button pin definitions
#define SHIFTER_BUTTONS_PL_PIN ((gpio_num_t)41)
#define SHIFTER_BUTTONS_CLK_PIN ((gpio_num_t)39)  // This is actually the clock pin, not clock inhibit
#define SHIFTER_BUTTONS_Q7_PIN ((gpio_num_t)40)
#define SHIFTER_NUM_BUTTONS 16

class ButtonController {
private:
    static const char* TAG;
    bool initialized;
    uint16_t button_state;      // Current button state (1 = pressed)
    uint16_t last_button_state; // Previous button state for edge detection
    TaskHandle_t button_task_handle;
    
    // Configurable pins
    gpio_num_t pl_pin;          // SHIFT/LOAD pin (active LOW for parallel load)
    gpio_num_t clk_pin;         // Clock pin (or CLK_INH pin for original controller)
    gpio_num_t q7_pin;          // Serial data output (QH)
    uint8_t num_buttons;        // Number of buttons to read (up to MAX_BUTTONS)
    bool use_as_clock;          // Whether to use clk_pin as clock (true) or as clock inhibit (false)
    
    static void button_task_func(void* pvParameters);
    void init_gpio();
    void read_buttons();

public:
    // Default constructor for backward compatibility
    ButtonController();
    
    // Constructor with configurable pins and button count
    // Added use_clock_pin parameter to specify if clk_pin should be used as clock (true) or clock inhibit (false)
    ButtonController(gpio_num_t pl_pin, gpio_num_t clk_pin, gpio_num_t q7_pin, uint8_t num_buttons = 8, bool use_as_clock = false);
    ~ButtonController();
    
    void init();
    bool get_button(uint8_t button_num);
    uint16_t get_all_buttons();
    bool button_pressed(uint8_t button_num);  // Returns true on button press (rising edge)
    bool button_released(uint8_t button_num); // Returns true on button release (falling edge)
    void start_button_task();
    void stop_button_task();
    
    // Simulate button presses for testing
    void simulate_button_press(uint8_t button_num, bool pressed);

    // Callback registration for button events
    typedef void (*button_callback_t)(uint8_t button_num, bool pressed);
    void register_callback(button_callback_t callback);
    
private:
    button_callback_t button_callback;
};

#endif // BUTTONS_H 