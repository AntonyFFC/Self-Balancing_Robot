#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define MPU6050_ADDR             0x68
#define MPU6050_INT_PIN          4
#define MPU6050_DMP_CODE_SIZE    1929
#define MPU6050_DMP_PACKET_SIZE  42

#define MPU6050_USER_CTRL_REG    0x6A
#define MPU6050_INT_STATUS_REG   0x3A
#define MPU6050_FIFO_COUNT_REG   0x72
#define MPU6050_FIFO_R_W_REG     0x74

typedef struct {
  float w, x, y, z;
} Quaternion;

typedef struct {
  float x, y, z;
} VectorFloat;

// Initialization and configuration
esp_err_t mpu6050_init(void);
esp_err_t mpu6050_dmp_initialize(void);
esp_err_t mpu6050_set_sleep_enabled(bool enabled);
esp_err_t mpu6050_set_clock_source(uint8_t source);
esp_err_t mpu6050_set_int_enabled(uint8_t enabled);
esp_err_t mpu6050_set_sample_rate(uint8_t rate);
esp_err_t mpu6050_set_external_frame_sync(uint8_t sync);
esp_err_t mpu6050_set_dlpf_mode(uint8_t mode);
esp_err_t mpu6050_set_gyro_range(uint8_t range);
esp_err_t mpu6050_set_accel_range(uint8_t range);
esp_err_t mpu6050_set_fifo_enabled(bool enabled);
esp_err_t mpu6050_reset(void);
esp_err_t mpu6050_reset_fifo(void);
esp_err_t mpu6050_reset_dmp(void);
esp_err_t mpu6050_set_dmp_enabled(bool enabled);

// DMP specific configuration
esp_err_t mpu6050_set_dmp_config1(uint8_t config);
esp_err_t mpu6050_set_dmp_config2(uint8_t config);
esp_err_t mpu6050_set_otp_bank_valid(bool enabled);
esp_err_t mpu6050_set_motion_detection_threshold(uint8_t threshold);
esp_err_t mpu6050_set_zero_motion_detection_threshold(uint8_t threshold);
esp_err_t mpu6050_set_motion_detection_duration(uint8_t duration);
esp_err_t mpu6050_set_zero_motion_detection_duration(uint8_t duration);

// I2C low-level access helpers
esp_err_t mpu6050_write_bit(uint8_t reg, uint8_t bit_num, bool value);
esp_err_t mpu6050_write_bits(uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t data);
esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data);
esp_err_t mpu6050_read_byte(uint8_t reg, uint8_t *data);
esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *buffer, size_t len);
esp_err_t mpu6050_write_prog_memory_block(const uint8_t *data, uint16_t data_size, uint8_t bank, uint8_t address);

// gyro offset setters
esp_err_t mpu6050_set_x_gyro_offset(int16_t offset);
esp_err_t mpu6050_set_y_gyro_offset(int16_t offset);
esp_err_t mpu6050_set_z_gyro_offset(int16_t offset);
esp_err_t mpu6050_set_z_accel_offset(int16_t offset);


// Interrupt and FIFO handling
// esp_err_t mpu6050_interrupt_init(void);
// esp_err_t mpu6050_get_int_status(uint8_t *status);
esp_err_t mpu6050_get_fifo_count(uint16_t *count);
esp_err_t mpu6050_read_fifo(uint8_t *buf, uint16_t len);
uint16_t mpu6050_get_fifo_packet_size(void);
esp_err_t getFIFOBytes(uint8_t *data, uint8_t length);

// DMP data parsing
int mpu6050_parse_fifo_packet(const uint8_t *fifoBuffer, Quaternion *q, VectorFloat *gravity, float ypr[3]);


#endif // MPU6050_DMP_H
