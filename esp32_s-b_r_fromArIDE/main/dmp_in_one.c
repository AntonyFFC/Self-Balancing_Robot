#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define MPU_INT 4
#define MPU6050_ADDR 0x68
#define MPU6050_USER_CTRL_REG 0x6A
#define MPU6050_USERCTRL_DMP_EN_BIT 6
#define MPU6050_INT_STATUS_REG 0x3A
#define MPU6050_FIFO_COUNT_REG 0x72
#define MPU6050_FIFO_R_W_REG 0x74
#define MPU6050_DMP_CODE_SIZE 1929 // example size; adjust accordingly

static const char* TAG = "MPU6050_DMP";

volatile bool mpuInterrupt = false;
uint16_t packetSize = 42;

// Forward declarations of low-level I2C register functions you need to implement in ESP-IDF
esp_err_t mpu6050_write_bit(uint8_t reg, uint8_t bit_num, bool value);
esp_err_t mpu6050_write_bits(uint8_t reg, uint8_t bit_start, uint8_t length, uint8_t data);
esp_err_t mpu6050_write_byte(uint8_t reg, uint8_t data);
esp_err_t mpu6050_read_byte(uint8_t reg, uint8_t *data);
esp_err_t mpu6050_read_bytes(uint8_t reg, uint8_t *buffer, size_t len);

// Load DMP firmware binary to MPU memory banks - your dmpMemory array must be defined
esp_err_t mpu6050_write_prog_memory_block(const uint8_t *prog, uint16_t size);

// ISR for MPU interrupt pin
static void IRAM_ATTR mpu_isr_handler(void* arg)
{
    mpuInterrupt = true;
}

esp_err_t mpu6050_interrupt_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << MPU_INT),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE
    };
    esp_err_t ret = gpio_config(&io_conf);
    if (ret != ESP_OK) return ret;

    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret;

    ret = gpio_isr_handler_add(MPU_INT, mpu_isr_handler, NULL);
    return ret;
}

// MPU6050 reset
esp_err_t mpu6050_reset()
{
    return mpu6050_write_bit(0x6B, 7, true); // PWR_MGMT_1 device reset bit
}

// Disable sleep mode
esp_err_t mpu6050_set_sleep_enabled(bool enabled)
{
    return mpu6050_write_bit(0x6B, 6, enabled);
}

// Set clock source (Z gyro PLL)
esp_err_t mpu6050_set_clock_source(uint8_t source)
{
    return mpu6050_write_bits(0x6B, 2, 3, source);
}

// Enable interrupts for DMP and FIFO overflow
esp_err_t mpu6050_set_int_enabled(uint8_t enabled)
{
    return mpu6050_write_byte(0x38, enabled);
}

// Set sample rate divider
esp_err_t mpu6050_set_sample_rate(uint8_t rate)
{
    return mpu6050_write_byte(0x19, rate);
}

// Set external frame sync
esp_err_t mpu6050_set_external_frame_sync(uint8_t sync)
{
    return mpu6050_write_bits(0x1A, 5, 3, sync);
}

// Set DLPF bandwidth
esp_err_t mpu6050_set_dlpf_mode(uint8_t mode)
{
    return mpu6050_write_bits(0x1A, 2, 3, mode);
}

// Set gyro full scale range
esp_err_t mpu6050_set_gyro_range(uint8_t range)
{
    return mpu6050_write_bits(0x1B, 4, 2, range);
}

// Enable FIFO
esp_err_t mpu6050_set_fifo_enabled(bool enabled)
{
    return mpu6050_write_bit(0x6A, 6, enabled);
}

// Reset FIFO buffer
esp_err_t mpu6050_reset_fifo()
{
    return mpu6050_write_bit(0x6A, 2, true);
}

// Reset DMP
esp_err_t mpu6050_reset_dmp()
{
    return mpu6050_write_bit(0x6A, 3, true);
}

// Set DMP Enabled / Disabled
esp_err_t mpu6050_set_dmp_enabled(bool enabled)
{
    uint8_t reg_val;
    esp_err_t ret = mpu6050_read_byte(MPU6050_USER_CTRL_REG, &reg_val);
    if (ret != ESP_OK) return ret;
    if (enabled)
        reg_val |= (1 << MPU6050_USERCTRL_DMP_EN_BIT);
    else
        reg_val &= ~(1 << MPU6050_USERCTRL_DMP_EN_BIT);
    return mpu6050_write_byte(MPU6050_USER_CTRL_REG, reg_val);
}

// Get interrupt status
esp_err_t mpu6050_get_int_status(uint8_t *status)
{
    return mpu6050_read_byte(MPU6050_INT_STATUS_REG, status);
}

// Get FIFO count (2 bytes)
esp_err_t mpu6050_get_fifo_count(uint16_t *count)
{
    uint8_t buff[2];
    esp_err_t ret = mpu6050_read_bytes(MPU6050_FIFO_COUNT_REG, buff, 2);
    if (ret == ESP_OK) {
        *count = (buff[0] << 8) | buff[1];
    }
    return ret;
}

// Read FIFO data
esp_err_t mpu6050_read_fifo(uint8_t *buf, uint16_t len)
{
    return mpu6050_read_bytes(MPU6050_FIFO_R_W_REG, buf, len);
}

// DMP initialization based on described steps
esp_err_t mpu6050_dmp_initialize()
{
    esp_err_t ret;

    // Reset device
    ret = mpu6050_reset();
    if (ret != ESP_OK) return ret;
    vTaskDelay(pdMS_TO_TICKS(30));

    // Disable sleep
    ret = mpu6050_set_sleep_enabled(false);
    if (ret != ESP_OK) return ret;

    // Set clock to PLL with Z gyro
    ret = mpu6050_set_clock_source(0x03); // PLL Z Gyro
    if (ret != ESP_OK) return ret;

    // Enable interrupts for FIFO overflow and DMP
    ret = mpu6050_set_int_enabled((1 << 4) | (1 << 5));
    if (ret != ESP_OK) return ret;

    // Set sample rate to 200 Hz (1kHz / (1 + 4) = 200Hz)
    ret = mpu6050_set_sample_rate(4);
    if (ret != ESP_OK) return ret;

    // External frame sync TEMP_OUT_L
    ret = mpu6050_set_external_frame_sync(0x02); // MPU6050_EXT_SYNC_TEMP_OUT_L defined as 0x02
    if (ret != ESP_OK) return ret;

    // Set DLPF bandwidth to 42 Hz
    ret = mpu6050_set_dlpf_mode(3); // MPU6050_DLPF_BW_42 likely 3
    if (ret != ESP_OK) return ret;

    // Set gyro range to 2000 dps
    ret = mpu6050_set_gyro_range(3); // MPU6050_GYRO_FS_2000 likely 3
    if (ret != ESP_OK) return ret;

    // Write DMP firmware to MPU memory banks
    ret = mpu6050_write_prog_memory_block(dmpMemory, MPU6050_DMP_CODE_SIZE);
    if (ret != ESP_OK) return ret;

    // TODO: Set DMP configuration bytes (setDMPConfig1 and setDMPConfig2)

    // TODO: Clear OTP bank valid flag

    // TODO: Setup motion detection thresholds and durations

    // Enable FIFO
    ret = mpu6050_set_fifo_enabled(true);
    if (ret != ESP_OK) return ret;

    // Reset DMP
    ret = mpu6050_reset_dmp();
    if (ret != ESP_OK) return ret;

    // Disable DMP (to be enabled later)
    ret = mpu6050_set_dmp_enabled(false);
    if (ret != ESP_OK) return ret;

    // Reset FIFO and clear interrupts
    ret = mpu6050_reset_fifo();
    if (ret != ESP_OK) return ret;

    uint8_t dummy;
    ret = mpu6050_get_int_status(&dummy);

    packetSize = 42; // Set default packet size

    return ret;
}

// Example processing function called in loop/task when interrupt flag is set
void process_dmp_fifo()
{
    if (!mpuInterrupt)
        return;

    mpuInterrupt = false;

    uint8_t intStatus = 0;
    uint16_t fifoCount = 0;

    if (mpu6050_get_int_status(&intStatus) != ESP_OK) {
        ESP_LOGE(TAG, "MPU Int status fail");
        return;
    }

    if (mpu6050_get_fifo_count(&fifoCount) != ESP_OK) {
        ESP_LOGE(TAG, "MPU FIFO count fail");
        return;
    }

    if ((intStatus & 0x10) || fifoCount == 1024) { // FIFO overflow
        ESP_LOGW(TAG, "FIFO overflow! Resetting FIFO");
        mpu6050_reset_fifo();
        fifoCount = 0;
    }

    if (intStatus & 0x02) { // DMP data ready
        while (fifoCount < packetSize) {
            if (mpu6050_get_fifo_count(&fifoCount) != ESP_OK) {
                ESP_LOGE(TAG, "MPU FIFO count fail inside fifo loop");
                return;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }

        uint8_t fifoBuffer[64] = {0};
        if (mpu6050_read_fifo(fifoBuffer, packetSize) != ESP_OK) {
            ESP_LOGE(TAG, "MPU FIFO read fail");
            return;
        }
        fifoCount -= packetSize;

        // TODO: parse fifoBuffer for quaternion, gravity, and yaw/pitch/roll

        // Example calculation:
        // input = calculated pitch angle from quaternion and gravity vector
    }
}
