#ifndef I2C_COM_H
#define I2C_COM_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include <stdio.h>


esp_err_t i2c_master_init(void);
esp_err_t i2c_com_read_byte(uint8_t reg, uint8_t *data, size_t len);
esp_err_t i2c_com_write_byte(uint8_t reg, uint8_t *data, size_t len);

esp_err_t i2c_com_write_bit(uint8_t reg, uint8_t bit_num, bool value);
esp_err_t i2c_com_write_bits(uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t data);
esp_err_t i2c_com_write_register(uint8_t reg, uint8_t data);
esp_err_t i2c_com_write_bytes(uint8_t reg, const uint8_t *data, size_t len);
esp_err_t i2c_com_read_register(uint8_t reg, uint8_t *data);
esp_err_t i2c_com_read_bytes(uint8_t reg, uint8_t *buffer, size_t len);
esp_err_t i2c_com_write_word(uint8_t reg_high, int16_t value);


#endif
