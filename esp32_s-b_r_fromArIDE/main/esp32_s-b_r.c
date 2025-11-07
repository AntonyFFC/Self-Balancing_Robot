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
#include <fcntl.h>
#include "lwip/netdb.h"
#include "nvs_flash.h"
#include "cJSON.h"
#include "mpu6050.h"
#include "i2c_com.h"
#include "motor.h"

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
// ============================================================================
// UDP DEBUG SECTION - UDP/Python plotter debug functionality
// ============================================================================
#if PYTHON_PLOTTER_DEBUG

#define DESTINATION_IP "192.168.1.14"
#define DESTINATION_PORT 7777
#define LISTEN_PORT 7778
#define UDP_MSG_MAX_LEN 128
#define UDP_SENDER_TASK_PERIOD_MS 300

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

    /* Make socket non-blocking and set a small send timeout so that
       sendto() cannot block indefinitely and interfere with real-time
       control. */
    int flags = fcntl(udp_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(udp_sock, F_SETFL, flags | O_NONBLOCK);
    }
    struct timeval tv = {0, 100000}; /* 100 ms */
    setsockopt(udp_sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

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

    /* Make listen socket non-blocking and give it a small recv timeout to
       avoid blocking the receiver task in case of network issues. */
    int flags = fcntl(udp_listen_sock, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(udp_listen_sock, F_SETFL, flags | O_NONBLOCK);
    }
    struct timeval rtv = {0, 200000}; /* 200 ms */
    setsockopt(udp_listen_sock, SOL_SOCKET, SO_RCVTIMEO, &rtv, sizeof(rtv));

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
    if (err < 0) {
        if (errno == EWOULDBLOCK || errno == EAGAIN) {
            /* socket would block - skip this sample to avoid blocking the
               control loop */
            return;
        }
        ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
    }
}

void send_initial_pid_values(void) {
    char init_msg[UDP_MSG_MAX_LEN];
    snprintf(init_msg, sizeof(init_msg), "INIT:P=%.3f,I=%.6f,D=%.3f\n", 
             pid_K, pid_Ti, pid_Td);
    udp_send_data(init_msg);
    ESP_LOGI(TAG, "Sent initial PID values: %s", init_msg);
}

void parse_pid_command(const char* cmd) {
    float new_P = pid_K, new_I = 1.0f / pid_Ti, new_D = pid_Td;
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
    
    if (updated) {
        pid_K = new_P;
        pid_Ti = new_I;
        pid_Td = new_D;
        ESP_LOGI(TAG, "Updated - P=%.3f, I=%.6f, D=%.3f", 
                 pid_K, pid_Ti, pid_Td);
    }
}

void init_debug_features(void) {
    ESP_ERROR_CHECK(my_udp_init());
    ESP_ERROR_CHECK(udp_server_init());
    ESP_LOGI(TAG, "UDP debug features enabled");
}

#endif // PYTHON_PLOTTER_DEBUG
// ============================================================================

#define MPU_INT 4
#define TASK_PERIOD_MS 10 // changed from 10


void IRAM_ATTR mpu_isr_handler(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(mpu_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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

void regular_100Hz_task(void *arg)
{
    for(;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        uint8_t mpuIntStatus;
        uint16_t fifoCount;
        uint8_t fifoBuffer[64];
        float newPitch;
        static Quaternion q;
        static VectorFloat gravity;
        static float ypr[3];

        mpu6050_get_int_status(&mpuIntStatus);
        mpu6050_get_fifo_count(&fifoCount);

        if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
            ESP_LOGW(TAG, "FIFO overflow detected, resetting FIFO");
            mpu6050_reset_fifo();
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
                motor_forward(pwm_ratio);
            }
            else if (u<0)
            {
                motor_backward(pwm_ratio);
            }
        } else {
            motor_stop();
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}

#if PYTHON_PLOTTER_DEBUG

void udp_sender_task(void *arg)
{
    char msg[UDP_MSG_MAX_LEN];
    float latest_pitch, latest_setPitch, latest_u;
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        if (udp_sock < 0) {
            /* Socket isn't ready yet - wait a bit and retry. Use a longer
               delay so this low-priority background task doesn't spin. */
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

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

#endif // PYTHON_PLOTTER_DEBUG

// void calibrate_gyroscope_offset(float* x_offset, float* y_offset, float* z_offset)
// {
//     float x, y, z;
//     const int samples = 10000;
//     float sum_x = 0, sum_y = 0, sum_z = 0;

//     for (int i = 0; i < samples; ++i) {
//         readGyroscope(&x, &y, &z);
//         sum_x += x;
//         sum_y += y;
//         sum_z += z;
//         vTaskDelay(pdMS_TO_TICKS(2));
//     }

//     *x_offset = sum_x / samples;
//     *y_offset = sum_y / samples;
//     *z_offset = sum_z / samples;

//     ESP_LOGI(TAG, "Gyro offsets: X=%.3f, Y=%.3f, Z=%.3f", *x_offset, *y_offset, *z_offset);
// }

void app_main(void)
{

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");

    uint8_t who_am_i;
    esp_err_t ret = mpu6050_get_who_am_i(&who_am_i);
    ESP_LOGI(TAG, "WHO_AM_I = %X", who_am_i);

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
        ret = mpu6050_interrupt_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize MPU6050 interrupt GPIO");
        }
        uint8_t mpuIntStatus;
        mpu6050_get_int_status(&mpuIntStatus);

        dmpReady = true;
        packetSize = mpu6050_get_fifo_packet_size();
        ESP_LOGI(TAG, "DMP ready! Waiting for first interrupt...");

        ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        
        
    #if PYTHON_PLOTTER_DEBUG
        // wifi_init_sta();
        // while (!wifi_connected) {
        //     ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
        //     vTaskDelay(1000 / portTICK_PERIOD_MS);
        // }

        // init_debug_features();
        // vTaskDelay(pdMS_TO_TICKS(1000)); // Small delay to ensure UDP is ready
        // send_initial_pid_values();
    #else
        ESP_LOGI(TAG, "UDP debug disabled");
    #endif

    ESP_ERROR_CHECK(motor_init());

        xTaskCreate(regulator_task, "regulator_task", 4096, NULL, 5, NULL);
    } else {
        ESP_LOGE(TAG, "DMP Initialization failed (code %d)", devStatus);
    }
    
#if PYTHON_PLOTTER_DEBUG
    // xTaskCreate(udp_sender_task, "udp_sender_task", 4096, NULL, 3, NULL);
    // xTaskCreate(udp_receiver_task, "udp_receiver_task", 4096, NULL, 3, NULL);
#endif
}