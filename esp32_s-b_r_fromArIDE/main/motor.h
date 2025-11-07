#pragma once

#include "esp_err.h"

esp_err_t motor_init(void);
void motor_forward(float pwm_ratio);
void motor_backward(float pwm_ratio);
void motor_stop(void);
float motor_compensate_deadband(float control_signal, float max_control);
