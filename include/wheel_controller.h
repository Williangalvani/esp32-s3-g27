#ifndef WHEEL_CONTROLLER_H
#define WHEEL_CONTROLLER_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/ledc.h"

// Motor pins
#define MOT_A 11
#define MOT_B 12

// Encoder pins
#define ENC_A 13
#define ENC_B 14

// PWM configuration
#define PWM_FREQ        5000    // 5kHz PWM frequency
#define PWM_CHANNEL_A   LEDC_CHANNEL_1
#define PWM_CHANNEL_B   LEDC_CHANNEL_0
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT  // 8-bit resolution (0-255)

// Constants for wheel position
#define WHEEL_CENTER_POS 0        // Center position
#define WHEEL_MIN_POS -32768      // Minimum encoder position
#define WHEEL_MAX_POS 32767       // Maximum encoder position
#define WHEEL_RANGE 900           // Full rotation range in degrees

class WheelController {
private:
    // Encoder related members
    pcnt_unit_handle_t pcnt_unit;
    int16_t current_position;
    int16_t target_position;
    
    // Motor control related members
    bool motor_enabled;
    int8_t motor_power;       // -100 to 100 (percentage)
    
    // Homing related members
    int32_t left_most;        // Leftmost encoder position detected during homing
    int32_t right_most;       // Rightmost encoder position detected during homing
    int32_t center;           // Center position calculated during homing
    int32_t range;            // Half the range of motion in encoder counts
    float wheel_range;        // Wheel rotation range in degrees
    float wheel_range_normalized; // Normalized wheel range (1.0 = 900 degrees)
    
    // Task handle for position monitoring
    TaskHandle_t monitor_task_handle;
    
    // PID control parameters
    float p_gain;             // Proportional gain
    float i_gain;             // Integral gain
    float d_gain;             // Derivative gain
    int32_t integral_error;   // Accumulated error (for I term)
    int16_t previous_error;   // Previous error (for D term)
    int16_t deadzone;         // Error deadzone to reduce jitter
    int32_t max_integral;     // Maximum integral accumulation to prevent windup
    
    // Private methods
    void setup_encoder();
    void setup_motor();
    void set_motor_power(int8_t power);
    int8_t calculate_pid_force();
    
    // Static task function
    static void monitor_task_func(void* pvParameters);
    static void homing_task_func(void* pvParameters);
    
public:
    WheelController();
    ~WheelController();
    
    // Initialize the hardware
    void init();
    
    // Get/Set wheel position
    int32_t get_encoder_count();
    int16_t get_position();
    void set_target_position(int16_t position);
    
    // Position calculation methods
    float get_position_zero_centered();
    float get_position_zero_centered_normalized();
    
    // Force feedback control
    void set_force(int8_t force); // -100 to 100 (percentage)
    void enable_motor(bool enable);
    bool is_motor_enabled() { return motor_enabled; }
    
    // Basic motor control
    void left();   // Turn motor to move wheel left
    void right();  // Turn motor to move wheel right
    void stop();   // Stop the motor
    void move(float force); // Move with proportional force (-1.0 to 1.0)
    
    // Homing functionality
    void home();               // Run the homing procedure
    void move_to_center();     // Move wheel to the center position
    void set_wheel_range(uint16_t new_range); // Set the wheel rotation range in degrees
    
    // PID control parameters
    void set_pid_params(float p, float i, float d);
    
    // Position monitoring
    void start_monitoring();
    void stop_monitoring();
};

#endif // WHEEL_CONTROLLER_H 