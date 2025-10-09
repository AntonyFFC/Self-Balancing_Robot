#include "mpu6050_dmp.h"
#include "driver/gpio.h"
#include "esp_log.h"

volatile bool mpuInterrupt = false;
uint8_t mpuIntStatus = 0;
uint16_t fifoCount = 0;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];

double setpoint = 178;
double Kp = 10, Ki = 0, Kd = 0;
double input = 0, output = 0;

// TODO: PID structure and initialization (use ESP-IDF timer/task or suitable PID library)

void app_main() {
    ESP_LOGI("MAIN", "Initializing I2C and MPU6050...");
    // TODO: I2C initialization

    int ret = mpu6050_dmp_initialize();
    if (ret != 0) {
        ESP_LOGE("MAIN", "DMP Initialization failed");
        return;
    }

    mpu6050_set_dmp_enabled(true);
    mpu6050_interrupt_init();

    packetSize = mpu6050_get_fifo_packet_size();

    // TODO: PID initialization here

    while (1) {
        if (!mpuInterrupt) {
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }
        mpuInterrupt = false;

        if (mpu6050_get_int_status(&mpuIntStatus) != 0) continue;
        if (mpu6050_get_fifo_count(&fifoCount) != 0) continue;

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            // TODO: reset FIFO in mpu6050_dmp.c
            ESP_LOGW("MAIN", "FIFO overflow, resetting");
            // mpu6050_reset_fifo();
            continue;
        }
        if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) {
                if (mpu6050_get_fifo_count(&fifoCount) != 0) break;
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            mpu6050_read_fifo_bytes(fifoBuffer, packetSize);

            mpu6050_parse_fifo_packet(fifoBuffer, &q, &gravity, ypr);

            double angle = ypr[1] * 180.0 / M_PI;
            if (angle < 0) angle += 360;
            input = angle;
        }

        // TODO: Call PID Compute based on input, control motors accordingly

        ESP_LOGI("MAIN", "Input: %f, Output: %f", input, output);
    }
}
