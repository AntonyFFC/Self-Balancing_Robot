#include "motor.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <math.h>

static const char *TAG = "motor";

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_n1        LEDC_CHANNEL_0
#define LEDC_CHANNEL_n2        LEDC_CHANNEL_1
#define LEDC_CHANNEL_n3        LEDC_CHANNEL_2
#define LEDC_CHANNEL_n4        LEDC_CHANNEL_3
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_MAX_DUTY           16383
#define LEDC_FREQUENCY          500

#define LEDC_OUTPUT_IO_1 25
#define LEDC_OUTPUT_IO_2 26
#define LEDC_OUTPUT_IO_3 27
#define LEDC_OUTPUT_IO_4 14

static float min_pwm_percent = 0.0f;  // Minimum PWM percentage for motor movement
static float motor_threshold_percent = 5.0f;  // Minimum PWM threshold to activate motors (stop below this)

esp_err_t motor_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };

    esp_err_t err = ledc_timer_config(&ledc_timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed: %s", esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_1,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel1);
    if (err != ESP_OK) { ESP_LOGE(TAG, "channel1 config failed: %s", esp_err_to_name(err)); return err; }

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_2,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel2);
    if (err != ESP_OK) { ESP_LOGE(TAG, "channel2 config failed: %s", esp_err_to_name(err)); return err; }

    ledc_channel_config_t ledc_channel_3 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n3,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_3,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel_3);
    if (err != ESP_OK) { ESP_LOGE(TAG, "channel3 config failed: %s", esp_err_to_name(err)); return err; }

    ledc_channel_config_t ledc_channel4 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n4,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_4,
        .duty           = 0,
        .hpoint         = 0
    };
    err = ledc_channel_config(&ledc_channel4);
    if (err != ESP_OK) { ESP_LOGE(TAG, "channel4 config failed: %s", esp_err_to_name(err)); return err; }

    ESP_LOGI(TAG, "Motor PWM initialized (SDA=%d SCL=%d @%dHz)", LEDC_OUTPUT_IO_1, LEDC_OUTPUT_IO_2, LEDC_FREQUENCY);
    return ESP_OK;
}

void motor_forward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}

void motor_backward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}

void motor_left_forward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);
}

void motor_left_backward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);
}

void motor_right_forward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}

void motor_right_backward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}

void motor_stop(void)
{
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}


float motor_compensate_deadband(float control_signal, float max_control) {
    float abs_control = fabs(control_signal);
    float min_pwm = min_pwm_percent / 100.0f;
    
    return min_pwm + (abs_control / max_control) * (1.0f - min_pwm);
}

void motor_turn_left(float pwm_ratio)
{
    // Left motor: channels 1/2, Right motor: channels 3/4
    // To turn left: left motor backward, right motor forward
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);

    // left motor backward (channel 2)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);

    // right motor forward (channel 3)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}

void motor_turn_right(float pwm_ratio)
{
    // To turn right: left motor forward, right motor backward
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);

    // left motor forward (channel 1)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2);

    // right motor backward (channel 4)
    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, 0);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, pwm_duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4);
}
