// led.h

#ifndef LEDS_H
#define LEDS_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

// connector pinout:

// 1 - vcc
// 2- PL (74hc165)
// 3- Latch Clock (HC595AG)
// 4- Shift Clock (HC595AG)
// 5- A (DATA) (74hc165)
// 6- Q7
// 7- GND

// Define pins as gpio_num_t to avoid type conversion errors
#define LED_SHIFT_CLOCK_PIN ((gpio_num_t)17)
#define LED_LATCH_CLOCK_PIN ((gpio_num_t)18)
#define LED_DATA_PIN ((gpio_num_t)16)

// We'll handle buttons later
// #define BUTTONS_PL_PIN ((gpio_num_t)8)
// #define BUTTONS_Q7_PIN ((gpio_num_t)15)
// #define BUTTON_PIN ((gpio_num_t)20)

// This class deals with input/output to talk with g27 leds
// THE HC5595 drives the leds
class LedController {
private:
    static const char* TAG;
    bool initialized;
    uint8_t led_state[2]; // Store state for 16 LEDs (2 bytes)
    TaskHandle_t led_task_handle;
    uint16_t pattern_counter;
    bool blink_mode;
    
    static void led_task_func(void* pvParameters);
    void init_gpio();

public:
    LedController();
    ~LedController();
    
    void init();
    void set_led(uint8_t led_num, bool state);
    void set_all_leds(uint16_t led_pattern);
    void clear_all_leds();
    void update_leds(); // Manual update function
    void start_led_task();
    void stop_led_task();
    void set_blink_mode(bool enable);
};

#endif // LEDS_H

