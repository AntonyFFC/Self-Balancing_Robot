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
#include "mpu6050.h"
#include "i2c_com.h"
#include "motor.h"

#define PYTHON_PLOTTER_DEBUG           CONFIG_PYTHON_PLOTTER_DEBUG

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DRIVE_ANGLE_OFFSET 5.0f
#define SPEED_SLEW_RATE 1.0f
#define TURN_OFFSET 70.0f
#define TURN_SLEW_RATE 10.0f

static volatile float drive_angle_offset = DRIVE_ANGLE_OFFSET;
static volatile float speed_slew_rate = SPEED_SLEW_RATE;
static volatile float turn_offset_val = TURN_OFFSET;
static volatile float turn_slew_rate = TURN_SLEW_RATE;

#define STARTUP_STABLE_THRESHOLD_DEG 3.0f
#define STARTUP_STABLE_COUNT 30
#define STARTUP_SAMPLE_INTERVAL_MS 20

static const char *TAG = "self-balancing-robot";

TaskHandle_t mpu_task_handle = NULL;

static float pid_K = 9.0f;
static float pid_1Ti = 0.0f;
static float pid_Td = 0.0f;
static float setPitch = 177.0f, u = 0.0f;
volatile float pitch = 0.0f;
static float upright_pitch = 177.0f;

static volatile bool motors_enabled = false;
static volatile bool request_pid_reset = false;

static volatile bool mpuInterrupt = false;
uint16_t packetSize;
bool dmpReady = false;

SemaphoreHandle_t pitch_mutex;
SemaphoreHandle_t u_mutex;
SemaphoreHandle_t move_mutex;
SemaphoreHandle_t setPitch_mutex;

typedef enum {
    MOVE_STOP = 0,
    MOVE_FORWARD,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} move_cmd_t;

static volatile move_cmd_t manual_move = MOVE_STOP;
// ============================================================================
// UDP DEBUG SECTION
// ============================================================================
#if PYTHON_PLOTTER_DEBUG

// #define DESTINATION_IP "192.168.1.161" //14 - pc //161 - phone
#define DESTINATION_PORT 7777
#define LISTEN_PORT 7778
#define UDP_MSG_MAX_LEN 128
#define UDP_SENDER_TASK_PERIOD_MS 100

int udp_sock;
struct sockaddr_in dest_addr;
static bool dest_addr_known = false;
int udp_listen_sock;
extern bool wifi_connected;

static uint32_t total_i2c_errors = 0;

static void udp_send_data(const char *data);

static void __attribute__((unused)) send_i2c_error(const char* error_type, esp_err_t error_code) {
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

    // Adres docelowy zapisany po otrzymaniu komendy CONNECT
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DESTINATION_PORT);
    dest_addr_known = false;

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
    if (!dest_addr_known) {
        ESP_LOGW(TAG, "UDP destination not known yet; not sending");
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
    snprintf(init_msg, sizeof(init_msg), "INIT:Kp=%.3f,1/Ti=%.6f,Td=%.3f,UPRIGHT=%.3f\n", 
             pid_K, pid_1Ti, pid_Td, upright_pitch);
    udp_send_data(init_msg);
    ESP_LOGI(TAG, "Sent initial PID values: %s", init_msg);
}

void parse_pid_command(const char* cmd) {
    float new_P = pid_K, new_I = pid_1Ti, new_D = pid_Td;
    float new_up = upright_pitch;
    bool updated = false;

    if (strstr(cmd, "GET") != NULL) {
        send_initial_pid_values();
        return;
    }
    
    char* p_pos = strstr(cmd, "Kp=");
    if (p_pos != NULL) {
        new_P = atof(p_pos + 3);
        updated = true;
    }
    
    char* i_pos = strstr(cmd, "1/Ti=");
    if (i_pos != NULL) {
        float i_val = atof(i_pos + 5);
        new_I = i_val;
        updated = true;
    }
    
    char* d_pos = strstr(cmd, "Td=");
    if (d_pos != NULL) {
        new_D = atof(d_pos + 3);
        updated = true;
    }

    char* up_pos = strstr(cmd, "UPRIGHT=");
    if (up_pos == NULL) up_pos = strstr(cmd, "UPRIGHT:");
    if (up_pos != NULL) {
        char *sep = strchr(up_pos, '=');
        if (sep == NULL) sep = strchr(up_pos, ':');
        if (sep != NULL) {
            new_up = atof(sep + 1);
            updated = true;
        }
    }
    
    if (updated) {
        pid_K = new_P;
        pid_1Ti = new_I;
        pid_Td = new_D;
        if (new_up != upright_pitch) {
            upright_pitch = new_up;
            if (xSemaphoreTake(setPitch_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                setPitch = upright_pitch;
                xSemaphoreGive(setPitch_mutex);
            } else {
                ESP_LOGW(TAG, "Could not obtain setPitch_mutex to update upright pitch");
            }
        }
        ESP_LOGI(TAG, "Updated - Kp=%.3f, 1/Ti=%.6f, Td=%.3f, Upright=%.3f", 
                 pid_K, pid_1Ti, pid_Td, upright_pitch);
    }
}

void parse_move_command(const char* cmd) {
    const char *pos = strstr(cmd, "MOVE:");
    if (!pos) return;

    pos += strlen("MOVE:");
    char token[32];
    int i = 0;
    while (*pos && i < (int)sizeof(token)-1) {
        if (*pos == '\r' || *pos == '\n') break;
        token[i++] = *pos++;
    }
    token[i] = '\0';

    move_cmd_t new_cmd = MOVE_STOP;
    if (strcasecmp(token, "FORWARD") == 0) new_cmd = MOVE_FORWARD;
    else if (strcasecmp(token, "BACKWARD") == 0) new_cmd = MOVE_BACKWARD;
    else if (strcasecmp(token, "LEFT") == 0) new_cmd = MOVE_LEFT;
    else if (strcasecmp(token, "RIGHT") == 0) new_cmd = MOVE_RIGHT;
    else if (strcasecmp(token, "STOP") == 0) new_cmd = MOVE_STOP;
    else {
        ESP_LOGW(TAG, "Unknown MOVE token: %s", token);
        return;
    }

    if (xSemaphoreTake(move_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        manual_move = new_cmd;
        xSemaphoreGive(move_mutex);
        ESP_LOGI(TAG, "Manual move set to %s", token);
    } else {
        ESP_LOGW(TAG, "Could not obtain move_mutex to set manual move");
    }
}

void parse_manual_settings(const char* cmd) {
    const char *pos = strstr(cmd, "MAN_SET:");
    if (!pos) return;

    char *dao_pos = strstr(cmd, "DAO=");
    char *ssr_pos = strstr(cmd, "SSR=");
    char *to_pos = strstr(cmd, "TO=");
    char *tsr_pos = strstr(cmd, "TSR=");

    bool updated = false;
    if (dao_pos) {
        float v = atof(dao_pos + 4);
        drive_angle_offset = v;
        updated = true;
    }
    if (ssr_pos) {
        float v = atof(ssr_pos + 4);
        speed_slew_rate = v;
        updated = true;
    }
    if (to_pos) {
        float v = atof(to_pos + 3);
        turn_offset_val = v;
        updated = true;
    }
    if (tsr_pos) {
        float v = atof(tsr_pos + 4);
        turn_slew_rate = v;
        updated = true;
    }

    if (updated) {
        ESP_LOGI(TAG, "Updated manual settings: DAO=%.3f, SSR=%.3f, TO=%.3f, TSR=%.3f",
                 drive_angle_offset, speed_slew_rate, turn_offset_val, turn_slew_rate);
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
#define TASK_PERIOD_MS 10 //10

void update_ramped_speed(float *current_speed, float target_speed, float slew_rate, float dt)
{
    if (*current_speed < target_speed) {
        *current_speed += slew_rate * dt;
        if (*current_speed > target_speed) {
            *current_speed = target_speed;
        }
    } else if (*current_speed > target_speed) {
        *current_speed -= slew_rate * dt;
        if (*current_speed < target_speed) {
            *current_speed = target_speed;
        }
    }
}

void update_ramped_turn(float *current_turn, float target_turn, float slew_rate, float dt)
{
    if (*current_turn < target_turn) {
        *current_turn += slew_rate * dt;
        if (*current_turn > target_turn) {
            *current_turn = target_turn;
        }
    } else if (*current_turn > target_turn) {
        *current_turn -= slew_rate * dt;
        if (*current_turn < target_turn) {
            *current_turn = target_turn;
        }
    }
}

static void wait_for_stable_pitch_and_enable(void)
{
    ESP_LOGI(TAG, "Waiting for stable pitch around %.2f deg...", upright_pitch);
    int consecutive = 0;
    while (consecutive < STARTUP_STABLE_COUNT) {
        float sample = 0.0f;
        if (xSemaphoreTake(pitch_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            sample = pitch;
            xSemaphoreGive(pitch_mutex);
        } else {
            vTaskDelay(pdMS_TO_TICKS(STARTUP_SAMPLE_INTERVAL_MS));
            continue;
        }

        float diff = fabsf(sample - upright_pitch);
        if (diff <= STARTUP_STABLE_THRESHOLD_DEG) {
            consecutive++;
        } else {
            if (consecutive > 0) {
                ESP_LOGI(TAG, "Stability broken (diff %.2f), resetting counter", diff);
            }
            consecutive = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(STARTUP_SAMPLE_INTERVAL_MS));
    }

    motors_enabled = true;
    request_pid_reset = true;
    ESP_LOGI(TAG, "Pitch stable — motors enabled");
}

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
	const float _1Ti = pid_1Ti;
	const float Td = pid_Td;

	static float this_u =  0.0f;

	//bledy:
	static float e = 0.0f, e_1 = 0.0f, e_2 = 0.0f;

    static float last_K = 0.0f, last_1Ti = 0.0f, last_Td = 0.0f;
    
    if (K != last_K || _1Ti != last_1Ti || Td != last_Td || request_pid_reset) {
        request_pid_reset = false;
        this_u = 0.0f;
        e = 0.0f;
        e_1 = 0.0f;
        e_2 = 0.0f;
        last_K = K;
        last_1Ti = _1Ti;
        last_Td = Td;
        ESP_LOGI(TAG, "PID controller reset due to parameter change");
    }

	const float r2 = (K*Td)/Tp;
	const float r1 = K*(((Tp/2)*_1Ti)-(2*Td/Tp)-1);
	const float r0 = K*(1+((Tp/2)*_1Ti)+(Td/Tp));

	//aktualizacja bledow:
	e_2 = e_1;
	e_1 = e;
	e = yzad - y;

	this_u = r2*e_2 + r1*e_1 + r0*e + this_u;
	return this_u;
}

void Data_Acquisition_task(void *arg)
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

        if (mpu6050_check_fifo_oflow(mpuIntStatus, fifoCount)) {
            ESP_LOGW(TAG, "FIFO overflow detected, resetting FIFO");
            mpu6050_reset_fifo();
            continue;
        }

        if (mpu6050_check_dmp_status(mpuIntStatus)) {
            while (fifoCount < packetSize) {
                mpu6050_get_fifo_count(&fifoCount);
            }

            mpu6050_get_fifo_bytes(fifoBuffer, packetSize);
            mpu6050_parse_fifo_packet(fifoBuffer, &q, &gravity, ypr);

            newPitch = ypr[1] * 180 / M_PI;
            if (newPitch < 0) newPitch += 360;
            // ESP_LOGW(TAG, "Pitch: %.2f", newPitch);

            if (xSemaphoreTake(pitch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
                pitch = newPitch;
                xSemaphoreGive(pitch_mutex);
            }
        }
    }
}


void Balance_Control_task(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    const float max_u = 255.0f;
    
    while (true)
    {
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));

        if (!dmpReady) continue;
 
        float local_pitch;
        if (xSemaphoreTake(pitch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_pitch = pitch;
            xSemaphoreGive(pitch_mutex);
        } else {
            continue;
        }

        float local_setPitch;
        if (xSemaphoreTake(setPitch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_setPitch = setPitch;
            xSemaphoreGive(setPitch_mutex);
        } else {
            continue;
        }

        move_cmd_t local_move;
        if (xSemaphoreTake(move_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            local_move = manual_move;
            xSemaphoreGive(move_mutex);
        } else {
            continue;
        }

        static float turnOffset = 0.0f;

        if (local_move == MOVE_STOP)
        {
            local_setPitch = upright_pitch;
            turnOffset = 0.0f;
        } else if (local_move == MOVE_FORWARD) {
            update_ramped_speed(&local_setPitch, upright_pitch - drive_angle_offset, speed_slew_rate, TASK_PERIOD_MS / 1000.0f);
        } else if (local_move == MOVE_BACKWARD) {
            update_ramped_speed(&local_setPitch, upright_pitch + drive_angle_offset, speed_slew_rate, TASK_PERIOD_MS / 1000.0f);
        } else if (local_move == MOVE_LEFT) {
            update_ramped_turn(&turnOffset, -turn_offset_val, turn_slew_rate, TASK_PERIOD_MS / 1000.0f);
            local_setPitch = upright_pitch;
        } else if (local_move == MOVE_RIGHT) {
            update_ramped_turn(&turnOffset, turn_offset_val, turn_slew_rate, TASK_PERIOD_MS / 1000.0f);
            local_setPitch = upright_pitch;
        }
        
        float local_u = PID(local_pitch, local_setPitch);

        float left_u = local_u + turnOffset;
        float right_u = local_u - turnOffset;

        if (left_u > max_u) left_u = max_u;
        if (left_u < -max_u) left_u = -max_u;
        if (right_u > max_u) right_u = max_u;
        if (right_u < -max_u) right_u = -max_u;

        float left_abs_control = fabs(left_u);
        float right_abs_control = fabs(right_u);
        float left_pwm_ratio = (left_abs_control / max_u);
        float right_pwm_ratio = (right_abs_control / max_u);
        
        if (motors_enabled && local_pitch>150.0f && local_pitch < 200) {
            if(left_u>0)
            {
                motor_left_forward(left_pwm_ratio);
            }
            else if (left_u<0)
            {
                motor_left_backward(left_pwm_ratio);
            }

            if(right_u>0)
            {
                motor_right_forward(right_pwm_ratio);
            }
            else if (right_u<0)
            {
                motor_right_backward(right_pwm_ratio);
            }
        } else {
            motor_stop();
        }

        if (xSemaphoreTake(u_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            u = local_u;
            xSemaphoreGive(u_mutex);
        } else {
            continue;
        }

        if (xSemaphoreTake(setPitch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            setPitch = local_setPitch;
            xSemaphoreGive(setPitch_mutex);
        } else {
            continue;
        }
    }
}

#if PYTHON_PLOTTER_DEBUG

void udp_sender_task(void *arg)
{
    char msg[UDP_MSG_MAX_LEN/2];
    TickType_t last_wake_time = xTaskGetTickCount();

    while (true) {
        float latest_pitch, latest_setPitch, latest_u;

        if (xSemaphoreTake(pitch_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            latest_pitch = pitch;
            xSemaphoreGive(pitch_mutex);
        } else {
            continue;
        }

        if (xSemaphoreTake(setPitch_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            latest_setPitch = setPitch;
            xSemaphoreGive(setPitch_mutex);
        } else {
            continue;
        }

        if (xSemaphoreTake(u_mutex, pdMS_TO_TICKS(5)) == pdTRUE)
        {
            latest_u = u;
            xSemaphoreGive(u_mutex);
        } else {
            continue;
        }

        snprintf(msg, sizeof(msg), "%.2f,%.2f,%.2f\n", latest_pitch, latest_setPitch, latest_u);
        udp_send_data(msg);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(UDP_SENDER_TASK_PERIOD_MS));
    }
}

void udp_receiver_task(void *arg)
{
    char rx_buffer[UDP_MSG_MAX_LEN/2];
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


        if (strstr(rx_buffer, "CONNECT") != NULL) {
            dest_addr = source_addr;
            dest_addr.sin_port = htons(DESTINATION_PORT);
            dest_addr_known = true;
            ESP_LOGI(TAG, "CONNECT received — remote set to %s, sending INIT", addr_str);
            send_initial_pid_values();
        } else if (!dest_addr_known) {
            ESP_LOGI(TAG, "Ignoring packet before CONNECT from %s", addr_str);
        } else if (strstr(rx_buffer, "MOVE:") != NULL) {
            parse_move_command(rx_buffer);
        } else if (strstr(rx_buffer, "MAN_SET:") != NULL) {
            parse_manual_settings(rx_buffer);
        } else {
            parse_pid_command(rx_buffer);
        }
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
    ESP_ERROR_CHECK(motor_init());
    motor_stop();

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
        pitch_mutex = xSemaphoreCreateMutex();
        u_mutex = xSemaphoreCreateMutex();
        move_mutex = xSemaphoreCreateMutex();
        setPitch_mutex = xSemaphoreCreateMutex();

        mpu6050_set_dmp_enabled(true);

        xTaskCreatePinnedToCore(Data_Acquisition_task, "Data_Acquisition_task", 4096, NULL, 20, &mpu_task_handle,1);
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

        wifi_init_ap();
        while (!wifi_connected) {
            ESP_LOGI(TAG, "Waiting for Wi-Fi connection...");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }

        init_debug_features();
        vTaskDelay(pdMS_TO_TICKS(1000));
        send_initial_pid_values();
    #else
        ESP_LOGI(TAG, "UDP debug disabled");
    #endif
        wait_for_stable_pitch_and_enable();

        xTaskCreatePinnedToCore(Balance_Control_task, "Balance_Control_task", 4096, NULL, 10, NULL,1);
    } else {
        ESP_LOGE(TAG, "DMP Initialization failed (code %d)", devStatus);
    }
    
#if PYTHON_PLOTTER_DEBUG
    xTaskCreatePinnedToCore(udp_sender_task, "udp_sender_task", 4096, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(udp_receiver_task, "udp_receiver_task", 4096, NULL, 4, NULL, 0);
#endif
}