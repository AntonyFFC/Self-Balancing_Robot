#include "i2c_com.h"
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#define TAG "I2C_COM"

#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_NUM              0
#define I2C_MASTER_TIMEOUT_MS       100
#define I2C_MASTER_FREQ_HZ          400000   
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

// #define MPU6050_I2C_ADDR (0b1101000 << 1)
#define MPU6050_I2C_ADDR 0x68

esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(i2c_master_port, &conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_param_config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "i2c_driver_install failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "I2C master initialized on port %d (SDA=%d SCL=%d @%dHz)", i2c_master_port, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO, I2C_MASTER_FREQ_HZ);
    }

    return ret;
}


esp_err_t i2c_com_read_byte(uint8_t reg, uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR, &reg, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret == ESP_ERR_TIMEOUT) {
        // ESP_LOGI(TAG, "I2C read timeout, attempting bus recovery");
        #if PYTHON_PLOTTER_DEBUG
        send_i2c_error("READ_TIMEOUT", ret);
        #endif
        vTaskDelay(pdMS_TO_TICKS(2));
    } else if (ret != ESP_OK) {
        #if PYTHON_PLOTTER_DEBUG
        send_i2c_error("READ_ERROR", ret);
        #endif
    }
    
    return ret;
}

esp_err_t i2c_com_write_byte(uint8_t reg, uint8_t *data, size_t len)
{
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(&write_buf[1], data, len);
    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_I2C_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
    if (ret == ESP_ERR_TIMEOUT) {
        // ESP_LOGI(TAG, "I2C write timeout");
        #if PYTHON_PLOTTER_DEBUG
        send_i2c_error("WRITE_TIMEOUT", ret);
        #endif
        vTaskDelay(pdMS_TO_TICKS(2));
    } else if (ret != ESP_OK) {
        #if PYTHON_PLOTTER_DEBUG
        send_i2c_error("WRITE_ERROR", ret);
        #endif
    }
    
    return ret;
}

esp_err_t i2c_com_write_bit(uint8_t reg, uint8_t bit_num, bool value)
{
    uint8_t byte;
    esp_err_t ret = i2c_com_read_byte(reg, &byte, 1);
    if (ret != ESP_OK) return ret;

    if (value)
        byte |= (1 << bit_num);
    else
        byte &= ~(1 << bit_num);

    return i2c_com_write_byte(reg, &byte, 1);
}

esp_err_t i2c_com_write_bits(uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t data)
{
    uint8_t byte;
    esp_err_t ret = i2c_com_read_byte(reg, &byte, 1);
    if (ret != ESP_OK) return ret;

    uint8_t mask = ((1 << length) - 1) << (bit_start - length + 1);
    data <<= (bit_start - length + 1);
    data &= mask;
    byte &= ~mask;
    byte |= data;

    return i2c_com_write_byte(reg, &byte, 1);
}

esp_err_t i2c_com_write_register(uint8_t reg, uint8_t data)
{
    return i2c_com_write_byte(reg, &data, 1);
}

esp_err_t i2c_com_write_bytes(uint8_t reg, const uint8_t *data, size_t len)
{
    return i2c_com_write_byte(reg, data, len);
}

esp_err_t i2c_com_read_register(uint8_t reg, uint8_t* data)
{
    return i2c_com_read_byte(reg, data, 1);
}

esp_err_t i2c_com_read_bytes(uint8_t reg, uint8_t *buffer, size_t len)
{
    return i2c_com_read_byte(reg, buffer, len);
}

esp_err_t i2c_com_write_word(uint8_t reg_high, int16_t value)
{
    esp_err_t ret;
    uint8_t high = (uint8_t)((value >> 8) & 0xFF);
    uint8_t low  = (uint8_t)(value & 0xFF);

    ret = i2c_com_write_register(reg_high, high);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write high byte at 0x%02X", reg_high);
        return ret;
    }

    ret = i2c_com_write_register(reg_high + 1, low);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write low byte at 0x%02X", reg_high + 1);
        return ret;
    }

    return ESP_OK;
}