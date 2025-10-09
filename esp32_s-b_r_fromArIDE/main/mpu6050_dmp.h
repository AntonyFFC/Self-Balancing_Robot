#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
  float w, x, y, z;
} Quaternion;

typedef struct {
  float x, y, z;
} VectorFloat;

// Initialize MPU6050 with DMP; returns 0 on success
int mpu6050_dmp_initialize(void);

// Enable or disable DMP
int mpu6050_set_dmp_enabled(bool enabled);

// Read interrupt status register
int mpu6050_get_int_status(uint8_t *status);

// Read FIFO packet size (usually 42 bytes)
uint16_t mpu6050_get_fifo_packet_size(void);

// Read FIFO count bytes available
int mpu6050_get_fifo_count(uint16_t *count);

// Read FIFO bytes into buffer
int mpu6050_read_fifo_bytes(uint8_t *buf, uint16_t len);

// Process FIFO buffer to extract quaternion, gravity vector, and YPR angles
int mpu6050_parse_fifo_packet(const uint8_t *fifoBuffer, Quaternion *q,
                              VectorFloat *gravity, float ypr[3]);

// Interrupt handler must set this variable externally
extern volatile bool mpuInterrupt;

#endif // MPU6050_DMP_H
