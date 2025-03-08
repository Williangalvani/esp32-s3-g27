#ifndef SHARED_PINS_H
#define SHARED_PINS_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"

// Define shared pins
#define SHARED_SHIFT_CLOCK_PIN ((gpio_num_t)17) // Shared shift clock pin

// Semaphore to control access to the shared shift clock
extern SemaphoreHandle_t shift_clock_semaphore;

// Initialize the shared pins and semaphore
void shared_pins_init();

// Take the shift clock semaphore with timeout
bool take_shift_clock(uint32_t timeout_ms);

// Release the shift clock semaphore
void release_shift_clock();

#endif // SHARED_PINS_H 