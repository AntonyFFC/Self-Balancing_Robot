#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "wifi.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "cJSON.h"

#define DESTINATION_IP "192.168.1.21"
#define DESTINATION_PORT 7777

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static const char *TAG = "self-balancing-robot";

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

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          12 // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_14_BIT // Set duty resolution to 14 bits
#define LEDC_MAX_DUTY           16383 // Maximum duty cycle value
#define LEDC_DUTY               8191 // Set duty to 50%. ((2 ** 14) - 1) * 50% = 8191
#define LEDC_FREQUENCY          500 // Frequency in Hertz. Set frequency at 500 Hz

#define L298N_ENA_GPIO LEDC_OUTPUT_IO
#define L298N_IN1_GPIO 14
#define L298N_IN2_GPIO 27

#define UDP_QUEUE_LEN 10
#define UDP_MSG_MAX_LEN 128

static QueueHandle_t udp_queue;

// static float pitch = 0.0f, setPitch = 0.0f, u;

#define TASK_PERIOD_MS 100   /*!< Task period in milliseconds */
#define UDP_SENDER_TASK_PERIOD_MS 100 /*!< UDP sender task period in milliseconds */

int udp_sock;
struct sockaddr_in dest_addr;

extern bool wifi_connected;

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

static void udp_send_data(const char *data) {
    if (udp_sock < 0) {
        ESP_LOGE(TAG, "UDP socket not initialized");
        return;
    }

    int err = sendto(udp_sock, data, strlen(data), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err < 0) {
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    } else {
        ESP_LOGI(TAG, "Data sent successfully: %s", data);
    }
}

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

    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
}

static esp_err_t mpu6050_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_MASTER_NUM, ACC_I2C_ADDR, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static esp_err_t mpu6050_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    int ret;
    uint8_t write_buf[2] = {reg_addr, data};

    ret = i2c_master_write_to_device(I2C_MASTER_NUM, ACC_I2C_ADDR, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

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
	  int16_t rawX,rawY,rawZ;
	  uint8_t i2c_receive8bit_buf[6];
	  const uint8_t bytes_to_receive = 6;

    ESP_ERROR_CHECK(mpu6050_register_read(startReg, i2c_receive8bit_buf, bytes_to_receive));
	  rawX = (int16_t)(i2c_receive8bit_buf[0]<<8 | i2c_receive8bit_buf[1]);
	  rawY = (int16_t)(i2c_receive8bit_buf[2]<<8 | i2c_receive8bit_buf[3]);
	  rawZ = (int16_t)(i2c_receive8bit_buf[4]<<8 | i2c_receive8bit_buf[5]);

	  *x = (float)rawX/pow(2,15)*rangeFactor;
	  *y = (float)rawY/pow(2,15)*rangeFactor;
	  *z = (float)rawZ/pow(2,15)*rangeFactor;
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
	const float alpha = 0.95;
	float acc_pitch = atan2(ax, sqrt(ay*ay+az*az)) * 180.0 / M_PI;
    // float acc_pitch = atan2(az, sqrt(ax*ax + ay*ay)) * (180.0 / M_PI); // Alternative vertical formula
	*pitch = alpha * (*pitch + gx * dt) + (1.0f-alpha) * acc_pitch;
}

float PID(float y, float yzad)
{
	const float Tp = 0.1f; //czas próbkowania
	const float K =  2.5f;
	const float Ti = 900000.0f;
	const float Td = 0.0f;

	static float u =  0.0f;

	//bledy:
	static float e = 0.0f;
	static float e_1 = 0.0f;
	static float e_2 = 0.0f;

	const float r2 = (K*Td)/Tp;
	const float r1 = K*((Tp/(2*Ti))-(2*Td/Tp)-1);
	const float r0 = K*(1+(Tp/(2*Ti))+(Td/Tp));

	//aktualizacja bledow:
	e_2 = e_1;
	e_1 = e;
	e = yzad - y;

	u = r2*e_2 + r1*e_1 + r0*e + u;
	return u;
}


void forward(uint16_t pwm)
{
    gpio_set_level(L298N_IN1_GPIO, 1);
    gpio_set_level(L298N_IN2_GPIO, 0);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void backward(uint16_t pwm)
{
    gpio_set_level(L298N_IN1_GPIO, 0);
    gpio_set_level(L298N_IN2_GPIO, 1);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pwm));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void stop()
{
    gpio_set_level(L298N_IN1_GPIO, 0);
    gpio_set_level(L298N_IN2_GPIO, 0);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL));
}

void calibrate_gyroscope_offset(float* x_offset, float* y_offset, float* z_offset);

void regular_100Hz_task(void *arg)
{
    float accxf, accyf, acczf;
    float gyroxf, gyroyf, gyrozf;
    static float pitch = 0.0f, setPitch = 0.0f, u;
    TickType_t last_wake_time = xTaskGetTickCount();
    float gx_offset, gy_offset, gz_offset;
    const float max_u = 250.0f;
    static uint16_t pwm;
    char msg[UDP_MSG_MAX_LEN];

    calibrate_gyroscope_offset(&gx_offset, &gy_offset, &gz_offset);

    while (true) {
        readAccelerometer(&accxf, &accyf, &acczf);
		readGyroscope(&gyroxf, &gyroyf, &gyrozf);
        // printf("%0.2f,%0.2f,%0.2f\n", accxf, accyf, acczf);

        calculatePitch(&pitch, accxf, accyf, acczf, gyroxf - gx_offset, TASK_PERIOD_MS / 1000.0f);
        snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f\n", pitch, setPitch, u);
        xQueueSend(udp_queue, msg, 0);

        u = PID(pitch, setPitch);
		if (u > max_u) u = max_u;
		if (u < -max_u) u = -max_u;
        pwm = (uint16_t)((fabs(u)/max_u)*LEDC_MAX_DUTY);

        if (u > 5.0)
		{
			forward(pwm);
            ESP_LOGI(TAG, "Forward with duty cycle: %4d", pwm);
        }
        else if (u < -5.0)
        {
            backward(pwm);
            ESP_LOGI(TAG, "Backward with duty cycle: %4d", pwm);
        }
        else
        {
            stop();
            ESP_LOGI(TAG, "Stop");
		}

        // ESP_LOGI(TAG, "Pitch: %3.2f, SetPitch: %3.2f, Control Signal: %3.2f", pitch, setPitch, u);
        // ESP_LOGI(TAG, "Pitch: %3.2f", pitch);
        // ESP_LOGI(TAG, "ACCEL xf: %3.2f g, yf: %3.2f g, zf: %3.2f g", accxf, accyf, acczf);
        // ESP_LOGI(TAG, "GYRO xf: %3.2f g, yf: %3.2f g, zf: %3.2f g", gyroxf, gyroyf, gyrozf);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

void udp_sender_task(void *arg)
{
    char msg[UDP_MSG_MAX_LEN];

    // float latest_pitch, latest_setPitch, latest_u;
    // TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        // latest_pitch = pitch;
        // latest_setPitch = setPitch;
        // latest_u = u;
        // snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f\n", latest_pitch, latest_setPitch, latest_u);
        // udp_send_data(msg);
        // vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UDP_SENDER_TASK_PERIOD_MS));
        if (xQueueReceive(udp_queue, msg, portMAX_DELAY)) {
            udp_send_data(msg);
        }
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
    uint8_t i2c_receive_buf[6];
    uint8_t i2c_transmit_buf[6];

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    ESP_ERROR_CHECK(mpu6050_register_read(WHO_AM_I_REG, i2c_receive_buf, 1));
    ESP_LOGI(TAG, "WHO_AM_I = %X", i2c_receive_buf[0]);

    uint8_t PWR_MGMT_1_reg = 0x6B;
    i2c_transmit_buf[0] = 0b00000000;
    ESP_ERROR_CHECK(mpu6050_register_write_byte(PWR_MGMT_1_reg, i2c_transmit_buf[0]));
    ESP_LOGI(TAG, "PWR_MGMT_1 set to 0x%02X", i2c_transmit_buf[0]);

    uint8_t SMPRT_DIV_reg = 0x19;
    i2c_transmit_buf[0] = 0x07;
    ESP_ERROR_CHECK(mpu6050_register_write_byte(SMPRT_DIV_reg, i2c_transmit_buf[0]));
    ESP_LOGI(TAG, "SMPRT_DIV set to 0x%02X", i2c_transmit_buf[0]);

    uint8_t GYRO_CONFIG_reg = 0x1B;
    i2c_transmit_buf[0] = 0x00;
    ESP_ERROR_CHECK(mpu6050_register_write_byte(GYRO_CONFIG_reg, i2c_transmit_buf[0]));
    ESP_LOGI(TAG, "GYRO_CONFIG set to 0x%02X", i2c_transmit_buf[0]);

    uint8_t ACCEL_CONFIG_reg = 0x1C;
    i2c_transmit_buf[0] = 0x00;
    ESP_ERROR_CHECK(mpu6050_register_write_byte(ACCEL_CONFIG_reg, i2c_transmit_buf[0]));
    ESP_LOGI(TAG, "ACCEL_CONFIG set to 0x%02X", i2c_transmit_buf[0]);

    esp_err_t ret = nvs_flash_init();
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
    ESP_ERROR_CHECK(my_udp_init());

    pwm_init();

    udp_queue = xQueueCreate(UDP_QUEUE_LEN, UDP_MSG_MAX_LEN);
    if (udp_queue == NULL) {
        ESP_LOGI(TAG, "Failed to create UDP queue");
    }

    xTaskCreate(regular_100Hz_task, "100Hz_task", 4096, NULL, 5, NULL);
    xTaskCreate(udp_sender_task, "udp_sender_task", 4096, NULL, 3, NULL);

}