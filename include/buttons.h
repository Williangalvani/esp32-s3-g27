#ifndef BUTTONS_H
#define BUTTONS_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "shared_pins.h" // Include shared pins header

// Define pins as gpio_num_t to avoid type conversion errors
#define BUTTONS_PL_PIN ((gpio_num_t)8)    // SHIFT/LOAD pin (active LOW for parallel load)
// Using shared shift clock (defined in shared_pins.h)
#define BUTTONS_CLK_INH_PIN ((gpio_num_t)10) // Clock inhibit pin (CK INH) - optional, can be tied to GND
#define BUTTONS_Q7_PIN ((gpio_num_t)15)   // Serial data output (QH)

#define SHIFTER_BUTTONS_PL_PIN ((gpio_num_t)41
#define SHIFTER_BUTTONS_CLK_INH_PIN ((gpio_num_t)39)
#define SHIFTER_BUTTONS_Q7_PIN ((gpio_num_t)40)


// Number of buttons to read
#define NUM_BUTTONS 8

class ButtonController {
private:
    static const char* TAG;
    bool initialized;
    uint16_t button_state;      // Current button state (1 = pressed)
    uint16_t last_button_state; // Previous button state for edge detection
    TaskHandle_t button_task_handle;
    
    static void button_task_func(void* pvParameters);
    void init_gpio();
    void read_buttons();

public:
    ButtonController();
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