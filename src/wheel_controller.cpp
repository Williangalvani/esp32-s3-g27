#include "wheel_controller.h"

static const char *TAG = "wheel_ctrl";

// Constructor
WheelController::WheelController() : 
    pcnt_unit(nullptr), 
    current_position(WHEEL_CENTER_POS), 
    target_position(WHEEL_CENTER_POS),
    motor_enabled(false),
    motor_power(0),
    monitor_task_handle(nullptr),
    p_gain(0.1f),
    i_gain(0.01f),
    d_gain(0.05f),
    integral_error(0),
    previous_error(0),
    deadzone(100),
    max_integral(1000)
{
}

// Destructor
WheelController::~WheelController() {
    // Make sure we clean up
    stop_monitoring();
    
    // Disable motor
    enable_motor(false);
    
    // Clean up PCNT unit if initialized
    if (pcnt_unit != nullptr) {
        pcnt_unit_stop(pcnt_unit);
        pcnt_unit_disable(pcnt_unit);
        pcnt_del_unit(pcnt_unit);
    }
}

// Initialize hardware
void WheelController::init() {
    ESP_LOGI(TAG, "Initializing wheel controller");
    setup_encoder();
    setup_motor();
}

// Set up encoder using ESP32's PCNT (Pulse Counter)
void WheelController::setup_encoder() {
    ESP_LOGI(TAG, "Setting up quadrature encoder on pins %d and %d", ENC_A, ENC_B);
    
    // Configure PCNT unit
    pcnt_unit_config_t unit_config = {
        .low_limit = WHEEL_MIN_POS,
        .high_limit = WHEEL_MAX_POS,
    };
    
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
    
    // Configure PCNT channels
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = ENC_A,
        .level_gpio_num = ENC_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = ENC_B,
        .level_gpio_num = ENC_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
    
    // Set up the channel edges and levels for proper quadrature decoding
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    
    // Enable and start the PCNT unit
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

// Set up motor control pins with PWM
void WheelController::setup_motor() {
    ESP_LOGI(TAG, "Setting up motor control on pins %d and %d", MOT_A, MOT_B);
    
    // Configure LEDC for PWM control
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,  // 8-bit resolution (0-255)
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,  // 5kHz PWM frequency
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));
    
    // Configure LEDC channel for MOT_A
    ledc_channel_config_t ledc_channel_a = {
        .gpio_num = MOT_A,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_a));
    
    // Configure LEDC channel for MOT_B
    ledc_channel_config_t ledc_channel_b = {
        .gpio_num = MOT_B,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
        .flags = {}
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_b));
    
    // Ensure motor is stopped
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

// Read current position from encoder
int16_t WheelController::get_position() {
    int raw_count;
    pcnt_unit_get_count(pcnt_unit, &raw_count);
    current_position = static_cast<int16_t>(raw_count);
    return current_position;
}

// Set target position for position-based force feedback
void WheelController::set_target_position(int16_t position) {
    target_position = position;
    
    // PID control will be done in the monitor task
}

// Set motor force directly (-100 to 100 percentage)
void WheelController::set_force(int8_t force) {
    if (!motor_enabled) {
        return;
    }
    
    motor_power = force;
    set_motor_power(motor_power);
}

// Calculate force using PID control
int8_t WheelController::calculate_pid_force() {
    int16_t current = get_position();
    int16_t error = target_position - current;
    
    // Apply deadzone to reduce jitter when wheel is near target position
    if (abs(error) < deadzone) {
        integral_error = 0;
        previous_error = 0;
        return 0;
    }
    
    // Calculate integral and derivative terms
    integral_error += error;
    
    // Limit integral to prevent windup
    if (integral_error > max_integral) integral_error = max_integral;
    if (integral_error < -max_integral) integral_error = -max_integral;
    
    // Calculate derivative term (change in error)
    int16_t derivative = error - previous_error;
    previous_error = error;
    
    // Calculate PID output
    float output = (p_gain * error) + (i_gain * integral_error) + (d_gain * derivative);
    
    // Clamp output to valid range
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    return (int8_t)output;
}

// Set PID parameters
void WheelController::set_pid_params(float p, float i, float d) {
    p_gain = p;
    i_gain = i;
    d_gain = d;
    
    // Reset error terms when parameters change
    integral_error = 0;
    previous_error = 0;
    
    ESP_LOGI(TAG, "PID parameters updated: P=%.2f, I=%.2f, D=%.2f", p, i, d);
}

// Enable or disable the motor
void WheelController::enable_motor(bool enable) {
    motor_enabled = enable;
    
    if (!enable) {
        // Stop the motor
        set_motor_power(0);
    }
    
    ESP_LOGI(TAG, "Motor %s", enable ? "enabled" : "disabled");
}

// Internal method to apply motor power using PWM
void WheelController::set_motor_power(int8_t power) {
    // Clamp power value
    if (power > 100) power = 100;
    if (power < -100) power = -100;
    
    // Convert -100 to 100 range to 0-255 PWM duty
    uint8_t duty = abs(power) * 255 / 100;
    
    if (power > 0) {
        // Forward: MOT_A PWM, MOT_B 0
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        
        ESP_LOGD(TAG, "Motor forward: power %d, duty %d", power, duty);
    } else if (power < 0) {
        // Reverse: MOT_A 0, MOT_B PWM
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        
        ESP_LOGD(TAG, "Motor reverse: power %d, duty %d", power, duty);
    } else {
        // Stop: both outputs 0
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
        
        ESP_LOGD(TAG, "Motor stop");
    }
}

// Start the position monitoring task
void WheelController::start_monitoring() {
    if (monitor_task_handle != nullptr) {
        ESP_LOGW(TAG, "Monitor task already running");
        return;
    }
    
    ESP_LOGI(TAG, "Starting position monitor task");
    xTaskCreate(monitor_task_func, "wheel_mon", 4096, this, 5, &monitor_task_handle);
}

// Stop the position monitoring task
void WheelController::stop_monitoring() {
    if (monitor_task_handle == nullptr) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping position monitor task");
    vTaskDelete(monitor_task_handle);
    monitor_task_handle = nullptr;
}

// Static task function that calls the object's method
void WheelController::monitor_task_func(void* pvParameters) {
    WheelController* self = static_cast<WheelController*>(pvParameters);
    
    ESP_LOGI(TAG, "Position monitor task started");
    
    // Position monitoring loop
    while (true) {
        // Read current position
        int16_t position = self->get_position();
        
        // Apply force if motor is enabled and we have a target position
        if (self->is_motor_enabled()) {
            int8_t force = self->calculate_pid_force();
            self->set_motor_power(force);
        }
        
        // Log position occasionally (every ~1 second)
        static int count = 0;
        if (++count >= 100) {
            ESP_LOGI(TAG, "Current position: %d, Target: %d", 
                    position, self->target_position);
            count = 0;
        }
        
        // Delay to prevent CPU hogging
        vTaskDelay(pdMS_TO_TICKS(10));
    }
} 