#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "mpu6050_dmp.h"

#define PYTHON_PLOTTER_DEBUG           CONFIG_PYTHON_PLOTTER_DEBUG

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "self-balancing-robot";

TaskHandle_t mpu_task_handle = NULL;

static float pid_K = 9.0f;
static float pid_Ti = 900000.0f;
static float pid_Td = 0.0f;
static float setPitch = 178.0f, u = 0.0f;
volatile float pitch = 0.0f;

static volatile bool mpuInterrupt = false;
uint16_t packetSize;
bool dmpReady = false;

// Motor deadband compensation parameters
static float min_pwm_percent = 0.0f;  // Minimum PWM percentage for motor movement
static float motor_threshold_percent = 5.0f;  // Minimum PWM threshold to activate motors (stop below this)

// ============================================================================
// UDP DEBUG SECTION - UDP/Python plotter debug functionality
// ============================================================================
#if PYTHON_PLOTTER_DEBUG

#define DESTINATION_IP "192.168.1.14"
#define DESTINATION_PORT 7777
#define LISTEN_PORT 7778
#define UDP_MSG_MAX_LEN 128
#define UDP_SENDER_TASK_PERIOD_MS 200

int udp_sock;
struct sockaddr_in dest_addr;
int udp_listen_sock;
extern bool wifi_connected;

static uint32_t total_i2c_errors = 0;

static void udp_send_data(const char *data);

static void send_i2c_error(const char* error_type, esp_err_t error_code) {
    char error_msg[UDP_MSG_MAX_LEN];
    total_i2c_errors++;
    snprintf(error_msg, sizeof(error_msg), "I2C_ERROR:%s,code=0x%x,total=%lu\n", 
             error_type, error_code, total_i2c_errors);
    udp_send_data(error_msg);
}

esp_err_t my_udp_init(void) {
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Wi-Fi not connected, delaying UDP init");
        return ESP_ERR_NOT_FOUND;
    }

    udp_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        return ESP_FAIL;
    }

    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DESTINATION_PORT);
    if (inet_pton(AF_INET, DESTINATION_IP, &dest_addr.sin_addr.s_addr) != 1) {
        ESP_LOGE(TAG, "Invalid destination IP: %s", DESTINATION_IP);
        close(udp_sock);
        udp_sock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP socket initialized successfully");
    return ESP_OK;
}

esp_err_t udp_server_init(void) {
    if (!wifi_connected) {
        ESP_LOGE(TAG, "Wi-Fi not connected, delaying UDP server init");
        return ESP_ERR_NOT_FOUND;
    }

    udp_listen_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (udp_listen_sock < 0) {
        ESP_LOGE(TAG, "Unable to create listen socket: errno %d", errno);
        return ESP_FAIL;
    }

    struct sockaddr_in listen_addr;
    listen_addr.sin_family = AF_INET;
    listen_addr.sin_addr.s_addr = INADDR_ANY;
    listen_addr.sin_port = htons(LISTEN_PORT);

    int err = bind(udp_listen_sock, (struct sockaddr *)&listen_addr, sizeof(listen_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        close(udp_listen_sock);
        udp_listen_sock = -1;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "UDP server socket initialized successfully on port %d", LISTEN_PORT);
    return ESP_OK;
}

static void udp_send_data(const char *data) {
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "UDP socket not initialized");
        return;
    }

    int err = sendto(udp_sock, data, strlen(data), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    // if (err < 0) {
    //     ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    // } else {
    //     ESP_LOGI(TAG, "Data sent successfully: %s", data);
    // }
}

void send_initial_pid_values(void) {
    char init_msg[UDP_MSG_MAX_LEN];
    snprintf(init_msg, sizeof(init_msg), "INIT:P=%.3f,I=%.6f,D=%.3f,MP=%.1f,MT=%.1f\n", 
             pid_K, pid_Ti, pid_Td, min_pwm_percent, motor_threshold_percent);
    udp_send_data(init_msg);
    ESP_LOGI(TAG, "Sent initial PID values: %s", init_msg);
}

void parse_pid_command(const char* cmd) {
    float new_P = pid_K, new_I = 1.0f / pid_Ti, new_D = pid_Td;
    float new_min_pwm = min_pwm_percent;
    float new_motor_threshold = motor_threshold_percent;
    bool updated = false;

    if (strstr(cmd, "GET") != NULL) {
        send_initial_pid_values();
        return;
    }
    
    char* p_pos = strstr(cmd, "P=");
    if (p_pos != NULL) {
        new_P = atof(p_pos + 2);
        updated = true;
    }
    
    char* i_pos = strstr(cmd, "I=");
    if (i_pos != NULL) {
        float i_val = atof(i_pos + 2);
        new_I = i_val;
        updated = true;
    }
    
    char* d_pos = strstr(cmd, "D=");
    if (d_pos != NULL) {
        new_D = atof(d_pos + 2);
        updated = true;
    }
    
    char* mp_pos = strstr(cmd, "MP=");
    if (mp_pos != NULL) {
        new_min_pwm = atof(mp_pos + 3);
        updated = true;
    }
    
    char* mt_pos = strstr(cmd, "MT=");
    if (mt_pos != NULL) {
        new_motor_threshold = atof(mt_pos + 3);
        updated = true;
    }
    
    if (updated) {
        pid_K = new_P;
        pid_Ti = new_I;
        pid_Td = new_D;
        min_pwm_percent = new_min_pwm;
        motor_threshold_percent = new_motor_threshold;
        ESP_LOGI(TAG, "Updated - P=%.3f, I=%.6f, D=%.3f, MP=%.1f%%, MT=%.1f%%", 
                 pid_K, pid_Ti, pid_Td, min_pwm_percent, motor_threshold_percent);
    }
}

void init_debug_features(void) {
    ESP_ERROR_CHECK(my_udp_init());
    ESP_ERROR_CHECK(udp_server_init());
    ESP_LOGI(TAG, "UDP debug features enabled");
}

#endif // PYTHON_PLOTTER_DEBUG
// ============================================================================

#define I2C_MASTER_SCL_IO           22      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_TIMEOUT_MS       100
#define I2C_MASTER_FREQ_HZ          400000   
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */ 

// #define ACC_I2C_ADDR (0b1101000 << 1)
#define ACC_I2C_ADDR 0x68          /*!< I2C address of MPU6050 accelerometer/gyroscope NOT SHIFTED, only 7 bit*/
#define ACCEL_START_REG 0x3B
#define GYRO_START_REG 0x43
#define WHO_AM_I_REG 0x75
#define MPU6050_INT_STATUS_REG 0x3A

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
// #define LEDC_OUTPUT_IO_1        33
#define LEDC_CHANNEL_n1          LEDC_CHANNEL_0
// #define LEDC_OUTPUT_IO_2        12
#define LEDC_CHANNEL_n2          LEDC_CHANNEL_1
#define LEDC_CHANNEL_n3          LEDC_CHANNEL_2
#define LEDC_CHANNEL_n4          LEDC_CHANNEL_3
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT
#define LEDC_MAX_DUTY           16383
#define LEDC_DUTY               8191
#define LEDC_FREQUENCY          500 

// #define L298N_ENA_GPIO LEDC_OUTPUT_IO_1
#define LEDC_OUTPUT_IO_1 25
#define LEDC_OUTPUT_IO_2 26
#define LEDC_OUTPUT_IO_3 27
#define LEDC_OUTPUT_IO_4 14
// #define L298N_ENB_GPIO LEDC_OUTPUT_IO_2

#define MPU_INT 4


#define TASK_PERIOD_MS 10 // changed from 10

static void pwm_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    ledc_channel_config_t ledc_channel1 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n1,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_1,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel1));

    ledc_channel_config_t ledc_channel2 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n2,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_2,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel2));

    ledc_channel_config_t ledc_channel_3 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n3,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_3,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_3));

    ledc_channel_config_t ledc_channel4 = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL_n4,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO_4,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel4));
}

static void IRAM_ATTR mpu_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // xTaskNotifyFromISR(mpu_task_handle, 0, eNoAction, &xHigherPriorityTaskWoken);
    vTaskNotifyGiveFromISR(mpu_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

static esp_err_t mpu6050_interrupt_init()
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

    // Install ISR service with default flags
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) return ret; 
    // ESP_ERR_INVALID_STATE means ISR service already installed, can ignore

    // Attach ISR handler for MPU_INT pin
    ret = gpio_isr_handler_add(MPU_INT, mpu_isr_handler, NULL);
    return ret;
}

// static void motor_gpio_init(void)
// {
//     gpio_config_t io_conf = {};
//     io_conf.intr_type = GPIO_INTR_DISABLE;
//     io_conf.mode = GPIO_MODE_OUTPUT;
//     io_conf.pin_bit_mask = (1ULL << L298N_IN1_GPIO) | (1ULL << L298N_IN2_GPIO) | (1ULL << L298N_IN3_GPIO) | (1ULL << L298N_IN4_GPIO);
//     io_conf.pull_down_en = 0;
//     io_conf.pull_up_en = 0;
//     gpio_config(&io_conf);
    
//     gpio_set_level(L298N_IN1_GPIO, 0);
//     gpio_set_level(L298N_IN2_GPIO, 0);
//     gpio_set_level(L298N_IN3_GPIO, 0);
//     gpio_set_level(L298N_IN4_GPIO, 0);
// }

// static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
// {
//     esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, ACC_I2C_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
//     if (ret == ESP_ERR_TIMEOUT) {
//         // ESP_LOGI(TAG, "I2C read timeout, attempting bus recovery");
//         #if PYTHON_PLOTTER_DEBUG
//         send_i2c_error("READ_TIMEOUT", ret);
//         #endif
//         vTaskDelay(pdMS_TO_TICKS(2));
//     } else if (ret != ESP_OK) {
//         #if PYTHON_PLOTTER_DEBUG
//         send_i2c_error("READ_ERROR", ret);
//         #endif
//     }
    
//     return ret;
// }

// static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
// {
//     uint8_t write_buf[2] = {reg_addr, data};
//     esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_NUM, ACC_I2C_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    
//     if (ret == ESP_ERR_TIMEOUT) {
//         // ESP_LOGI(TAG, "I2C write timeout");
//         #if PYTHON_PLOTTER_DEBUG
//         send_i2c_error("WRITE_TIMEOUT", ret);
//         #endif
//         vTaskDelay(pdMS_TO_TICKS(2));
//     } else if (ret != ESP_OK) {
//         #if PYTHON_PLOTTER_DEBUG
//         send_i2c_error("WRITE_ERROR", ret);
//         #endif
//     }
    
//     return ret;
// }

static esp_err_t i2c_master_init(void)
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

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void read3dData(float* x, float* y, float* z, float rangeFactor, uint8_t startReg)
{
    int16_t rawX, rawY, rawZ;
    uint8_t i2c_receive8bit_buf[6];
    const uint8_t bytes_to_receive = 6;
    esp_err_t ret;
    int retry_count = 0;
    const int max_retries = 3;

    do {
        ret = mpu6050_register_read(startReg, i2c_receive8bit_buf, bytes_to_receive);
        if (ret == ESP_OK) {
            break;
        }
        
        if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGI(TAG, "I2C timeout on read attempt %d, retrying...", retry_count + 1);
            vTaskDelay(pdMS_TO_TICKS(1));
        } else {
            ESP_LOGE(TAG, "I2C read error: 0x%x on attempt %d", ret, retry_count + 1);
        }
        
        retry_count++;
    } while (retry_count < max_retries);

    if (ret != ESP_OK) {
        // ESP_LOGE(TAG, "Failed to read sensor data after %d retries, using zeros", max_retries);
        #if PYTHON_PLOTTER_DEBUG
        send_i2c_error("SENSOR_READ_FAILED", ret);
        #endif
        *x = 0.0f;
        *y = 0.0f; 
        *z = 0.0f;
        return;
    }

    rawX = (int16_t)(i2c_receive8bit_buf[0] << 8 | i2c_receive8bit_buf[1]);
    rawY = (int16_t)(i2c_receive8bit_buf[2] << 8 | i2c_receive8bit_buf[3]);
    rawZ = (int16_t)(i2c_receive8bit_buf[4] << 8 | i2c_receive8bit_buf[5]);

    *x = (float)rawX / pow(2, 15) * rangeFactor;
    *y = (float)rawY / pow(2, 15) * rangeFactor;
    *z = (float)rawZ / pow(2, 15) * rangeFactor;
}

void readAccelerometer(float* x, float* y, float* z)
{
	  const float wspolczynnik = 2;
	  read3dData(x, y, z, wspolczynnik, ACCEL_START_REG);
}

void readGyroscope(float* x, float* y, float* z)
{
	const float wspolczynnik = 250.0f;
	read3dData(x, y, z, wspolczynnik, GYRO_START_REG);
}

void calculatePitch(float *pitch, float ax, float ay, float az, float gx, float dt)
{
	const float alpha = 0.85;
	float acc_pitch = atan2(ax, sqrt(ay*ay+az*az)) * 180.0 / M_PI;
    // float acc_pitch = atan2(az, sqrt(ax*ax + ay*ay)) * (180.0 / M_PI); // Alternative vertical formula
	*pitch = alpha * (*pitch + gx * dt) + (1.0f-alpha) * acc_pitch;
}

float PID(float y, float yzad)
{
	const float Tp = TASK_PERIOD_MS / 1000.0f; //czas próbkowania
	const float K = pid_K;
	const float Ti = pid_Ti;
	const float Td = pid_Td;

	static float this_u =  0.0f;

	//bledy:
	static float e = 0.0f;
	static float e_1 = 0.0f;
	static float e_2 = 0.0f;

    static float last_K = 0.0f;
    static float last_Ti = 0.0f;
    static float last_Td = 0.0f;
    
    if (K != last_K || Ti != last_Ti || Td != last_Td) {
        this_u = 0.0f;
        e = 0.0f;
        e_1 = 0.0f;
        e_2 = 0.0f;
        last_K = K;
        last_Ti = Ti;
        last_Td = Td;
        ESP_LOGI(TAG, "PID controller reset due to parameter change");
    }

	const float r2 = (K*Td)/Tp;
	const float r1 = K*((Tp/(2*Ti))-(2*Td/Tp)-1);
	const float r0 = K*(1+(Tp/(2*Ti))+(Td/Tp));

	//aktualizacja bledow:
	e_2 = e_1;
	e_1 = e;
	e = yzad - y;

	this_u = r2*e_2 + r1*e_1 + r0*e + this_u;
	return this_u;
}


float compensate_motor_deadband(float control_signal, float max_control) {
    float abs_control = fabs(control_signal);
    float min_pwm = min_pwm_percent / 100.0f;
    
    return min_pwm + (abs_control / max_control) * (1.0f - min_pwm);
}


void forward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4));
}

void backward(float pwm_ratio)
{
    uint16_t pwm_duty = (uint16_t)(pwm_ratio * LEDC_MAX_DUTY);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, pwm_duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4));
}

void stop()
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n1));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n2, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n2));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n3, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n3));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_n4, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_n4));
}

void calibrate_gyroscope_offset(float* x_offset, float* y_offset, float* z_offset);

// void regular_100Hz_task(void *arg)
// {
//     // float accxf, accyf, acczf;
//     // float gyroxf, gyroyf, gyrozf;
//     TickType_t last_wake_time = xTaskGetTickCount();
//     // float gx_offset, gy_offset, gz_offset;
//     const float max_u = 255.0f;
//     static float pwm_ratio;
//     static uint16_t fifoCount;
//     static uint8_t fifoBuffer[64];
//     static Quaternion q;
//     static VectorFloat gravity;
//     static float ypr[3];
    
//     static uint32_t consecutive_failures = 0;
//     const uint32_t max_consecutive_failures = 10;

//     // calibrate_gyroscope_offset(&gx_offset, &gy_offset, &gz_offset);
//     // gx_offset = -4.084f; // offsets calculated from previous calibrations
//     // gy_offset = -0.798f;
//     // gz_offset = 0.077f;

//     while (true) {
//         if (!dmpReady) {
//             return;
//         }

//         if (mpuInterrupt) {
//             ESP_LOGW(TAG, "MPU interrupt received");
//             mpuInterrupt = false;

//             uint8_t mpuIntStatus;
//             mpu6050_register_read(MPU6050_INT_STATUS_REG, &mpuIntStatus, 1);
//             mpu6050_get_fifo_count(&fifoCount);
//             if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
//                 ESP_LOGW(TAG, "FIFO overflow detected, resetting FIFO");
//                 mpu6050_reset_fifo();
//             }
//             else if (mpuIntStatus & 0x01) {
//                 ESP_LOGW(TAG, "DMP interrupt received");
//                 while(fifoCount < packetSize) {
//                     mpu6050_get_fifo_count(&fifoCount);
//                 }

//                 getFIFOBytes(fifoBuffer, packetSize);
//                 fifoCount -= packetSize;

//                 mpu6050_parse_fifo_packet(fifoBuffer, &q, &gravity, ypr);
//                 pitch = ypr[1] * 180 / M_PI;
//                 if(pitch < 0) {
//                     pitch += 360;
//                 }

//                 // bool sensor_read_success = true;
    
//                 // static float prev_accxf = 0.0f, prev_accyf = 0.0f, prev_acczf = 0.0f;
//                 // static float prev_gyroxf = 0.0f, prev_gyroyf = 0.0f, prev_gyrozf = 0.0f;

//                 // readAccelerometer(&accxf, &accyf, &acczf);
//                 // if (accxf == 0.0f && accyf == 0.0f && acczf == 0.0f) {
//                 //     sensor_read_success = false;
//                 //     accxf = prev_accxf;
//                 //     accyf = prev_accyf;
//                 //     acczf = prev_acczf;
//                 // } else {
//                 //     prev_accxf = accxf;
//                 //     prev_accyf = accyf;
//                 //     prev_acczf = acczf;
//                 // }
                
//                 // readGyroscope(&gyroxf, &gyroyf, &gyrozf);
//                 // if (gyroxf == 0.0f && gyroyf == 0.0f && gyrozf == 0.0f) {
//                 //     sensor_read_success = false;
//                 //     gyroxf = prev_gyroxf;
//                 //     gyroyf = prev_gyroyf;
//                 //     gyrozf = prev_gyrozf;
//                 // } else {
//                 //     prev_gyroxf = gyroxf;
//                 //     prev_gyroyf = gyroyf;
//                 //     prev_gyrozf = gyrozf;
//                 // }

//                 // calculatePitch(&pitch, accxf, accyf, acczf, gyroxf - gx_offset, TASK_PERIOD_MS / 1000.0f);
//             }
//         }

//         // if (!sensor_read_success) {
//         //     consecutive_failures++;
//         //     // ESP_LOGI(TAG, "Sensor read failed, using previous values (%lu consecutive failures)", consecutive_failures);
            
//         //     if (consecutive_failures > max_consecutive_failures) {
//         //         ESP_LOGE(TAG, "Too many sensor failures, stopping robot for safety");
//         //         stop();
//         //         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
//         //         continue;
//         //     }
//         // } else {
//         //     consecutive_failures = 0;
//         // }

//         u = PID(pitch, setPitch);

//         // ESP_LOGW(TAG, "Pitch: %.2f, Setpoint: %.2f", pitch, setPitch);
//         // ESP_LOGW(TAG, "PID output: %.2f", u);

//         if (u > max_u) u = max_u;
//         if (u < -max_u) u = -max_u;

//         float abs_control = fabs(u);
//         pwm_ratio = (abs_control / max_u);
        
//         // Mapping pwm ratio from 0-250 to min pwm percentage to 1.0
//         // pwm_ratio = compensate_motor_deadband(u, max_u);

//         if(pitch>150.0f && pitch < 200) {
//             if(u>0)
//             {
//             forward(pwm_ratio);
//             }
//             else if (u<0)
//             {
//             backward(pwm_ratio);
//             }
//         } else {
//             stop();
//         }

//         // if (consecutive_failures < max_consecutive_failures / 2) {
//         //     if (pwm_ratio < (motor_threshold_percent / 100.0f)) {
//         //         stop();
//         //     } else if (u > 0) {
//         //         backward(pwm_ratio);
//         //     } else {
//         //         forward(pwm_ratio);
//         //     }
//         // } else {
//         //     stop();
//         // }

//         vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
//     }
// }

void regular_100Hz_task(void *arg)
{
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // gpio_intr_disable(MPU_INT);

        uint8_t mpuIntStatus;
        uint16_t fifoCount;
        uint8_t fifoBuffer[64];
        float newPitch;
        static Quaternion q;
        static VectorFloat gravity;
        static float ypr[3];

        mpu6050_register_read(MPU6050_INT_STATUS_REG, &mpuIntStatus, 1);
        mpu6050_get_fifo_count(&fifoCount);

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            ESP_LOGW(TAG, "FIFO overflow detected, resetting FIFO");
            mpu6050_reset_fifo();
            // gpio_intr_enable(MPU_INT);
            continue;
        }

        if (mpuIntStatus & 0x02) {
            while (fifoCount < packetSize) {
                mpu6050_get_fifo_count(&fifoCount);
            }

            getFIFOBytes(fifoBuffer, packetSize);
            mpu6050_parse_fifo_packet(fifoBuffer, &q, &gravity, ypr);

            newPitch = ypr[1] * 180 / M_PI;
            if (newPitch < 0) newPitch += 360;

            pitch = newPitch;
        }
        // gpio_intr_enable(MPU_INT);
    }
}


void regulator_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const float max_u = 255.0f;
    

    while (true)
    {
        if (!dmpReady) {
            return;
        }

        float local_pitch = pitch;
        u = PID(local_pitch, setPitch);

        if (u > max_u) u = max_u;
        if (u < -max_u) u = -max_u;

        float abs_control = fabs(u);
        float pwm_ratio;
        pwm_ratio = (abs_control / max_u);

        if(local_pitch>150.0f && local_pitch < 200) {
            if(u>0)
            {
            forward(pwm_ratio);
            }
            else if (u<0)
            {
            backward(pwm_ratio);
            }
        } else {
            stop();
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

void udp_sender_task(void *arg)
{
    char msg[UDP_MSG_MAX_LEN];
    float latest_pitch, latest_setPitch, latest_u;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        latest_pitch = pitch;
        latest_setPitch = setPitch;
        latest_u = u;
        snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f\n", latest_pitch, latest_setPitch, latest_u);
        udp_send_data(msg);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UDP_SENDER_TASK_PERIOD_MS));
    }
}

void udp_receiver_task(void *arg)
{
    char rx_buffer[128];
    struct sockaddr_in source_addr;
    socklen_t socklen = sizeof(source_addr);
    
    while (true) {
        if (udp_listen_sock < 0) {
            ESP_LOGE(TAG, "UDP listen socket not initialized, retrying...");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        
        int len = recvfrom(udp_listen_sock, rx_buffer, sizeof(rx_buffer) - 1, 0, 
                        (struct sockaddr *)&source_addr, &socklen);
        
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        
        rx_buffer[len] = 0;
        
        char addr_str[128];
        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        
        ESP_LOGI(TAG, "Received %d bytes from %s: %s", len, addr_str, rx_buffer);
        parse_pid_command(rx_buffer);
    }
}

void calibrate_gyroscope_offset(float* x_offset, float* y_offset, float* z_offset)
{
    float x, y, z;
    const int samples = 10000;
    float sum_x = 0, sum_y = 0, sum_z = 0;

    for (int i = 0; i < samples; ++i) {
        readGyroscope(&x, &y, &z);
        sum_x += x;
        sum_y += y;
        sum_z += z;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    *x_offset = sum_x / samples;
    *y_offset = sum_y / samples;
    *z_offset = sum_z / samples;

    ESP_LOGI(TAG, "Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f", *x_offset, *y_offset, *z_offset);
}

void app_main(void)
{
    // uint8_t i2c_receive_buf[6];
    // uint8_t i2c_transmit_buf[6];

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    // ESP_ERROR_CHECK(mpu6050_register_read(WHO_AM_I_REG, i2c_receive_buf, 1));
    // ESP_LOGI(TAG, "WHO_AM_I = %X", i2c_receive_buf[0]);

    // uint8_t PWR_MGMT_1_reg = 0x6B;
    // i2c_transmit_buf[0] = 0b00000000;
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(PWR_MGMT_1_reg, i2c_transmit_buf[0]));
    // ESP_LOGI(TAG, "PWR_MGMT_1 set to 0x%02X", i2c_transmit_buf[0]);

    // uint8_t SMPRT_DIV_reg = 0x19;
    // i2c_transmit_buf[0] = 0x07;
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(SMPRT_DIV_reg, i2c_transmit_buf[0]));
    // ESP_LOGI(TAG, "SMPRT_DIV set to 0x%02X", i2c_transmit_buf[0]);

    // uint8_t GYRO_CONFIG_reg = 0x1B;
    // i2c_transmit_buf[0] = 0x00;
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(GYRO_CONFIG_reg, i2c_transmit_buf[0]));
    // ESP_LOGI(TAG, "GYRO_CONFIG set to 0x%02X", i2c_transmit_buf[0]);

    // uint8_t ACCEL_CONFIG_reg = 0x1C;
    // i2c_transmit_buf[0] = 0x00;
    // ESP_ERROR_CHECK(mpu6050_register_write_byte(ACCEL_CONFIG_reg, i2c_transmit_buf[0]));
    // ESP_LOGI(TAG, "ACCEL_CONFIG set to 0x%02X", i2c_transmit_buf[0]);
    mpu6050_init();

    uint8_t devStatus;
    devStatus = mpu6050_dmp_initialize();
    mpu6050_set_x_gyro_offset(220);
    mpu6050_set_y_gyro_offset(76);
    mpu6050_set_z_gyro_offset(-85);
    mpu6050_set_z_accel_offset(1688);
    if (devStatus == 0) {
        mpu6050_set_dmp_enabled(true);

        xTaskCreate(regular_100Hz_task, "100Hz_task", 4096, NULL, 10, &mpu_task_handle);
        esp_err_t ret = mpu6050_interrupt_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize MPU6050 interrupt GPIO");
        }
        uint8_t mpuIntStatus;
        mpu6050_register_read(MPU6050_INT_STATUS_REG, &mpuIntStatus, 1);

        dmpReady = true;
        packetSize = mpu6050_get_fifo_packet_size();
        ESP_LOGI(TAG, "DMP ready! Waiting for first interrupt...");

        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        wifi_init_sta();
        while (!wifi_connected) {
            ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
        
    #if PYTHON_PLOTTER_DEBUG
        init_debug_features();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Small delay to ensure UDP is ready
        send_initial_pid_values();
    #else
        ESP_LOGI(TAG, "UDP debug disabled");
    #endif

        pwm_init();
        // motor_gpio_init();

        // udp_queue = xQueueCreate(UDP_QUEUE_LEN, UDP_MSG_MAX_LEN);
        // if (udp_queue == NULL) {
        //     ESP_LOGI(TAG, "Failed to create UDP queue");
        // }
        xTaskCreate(regulator_task, "regulator_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "DMP Initialization failed (code %d)", devStatus);
    }
    
#if PYTHON_PLOTTER_DEBUG
    xTaskCreate(udp_sender_task, "udp_sender_task", 4096, NULL, 3, NULL);
    xTaskCreate(udp_receiver_task, "udp_receiver_task", 4096, NULL, 3, NULL);
#endif
}