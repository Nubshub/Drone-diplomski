#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "comm.h"
#include "control.h"
#include "soc.h"

#define TELEMETRY_DIV        10        /* 200 Hz / 10 = 20 Hz */
#define TELEMETRY_SERIAL     0
#define TELE_LINE_LEN        200
#define FAILSAFE_TIMEOUT_US  500000
#define MAX_I2C_ERRORS       20
#define TUNE_AXIS_HELLO      255

typedef struct { char line[TELE_LINE_LEN]; } tele_msg_t;

static portMUX_TYPE pkt_spinlock  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE tele_spinlock = portMUX_INITIALIZER_UNLOCKED;
static volatile int64_t last_pkt_us = 0;

static QueueHandle_t      tele_queue = NULL;
static struct sockaddr_in tele_dest;
static volatile bool      tele_dest_valid = false;

static void vMotor_control(void *pvParametars)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    Motor_thrust_t motors = {0};
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    uint8_t buffer[14];
    int64_t last_us = esp_timer_get_time();
    uint32_t i2c_err_cnt = 0;
    uint32_t tele_cnt = 0;
    tele_msg_t msg;

    while (1)
    {
        int64_t now_us = esp_timer_get_time();
        float dt = (float)(now_us - last_us) * 1e-6f;
        last_us = now_us;
        dt = CLAMP(dt, 0.001f, 0.02f);

        crtp_packet_t local_pkt;
        taskENTER_CRITICAL(&pkt_spinlock);
        local_pkt = pkt;
        taskEXIT_CRITICAL(&pkt_spinlock);

        if (now_us - last_pkt_us > FAILSAFE_TIMEOUT_US) {
            local_pkt.thrust = 0;
        }

        esp_err_t err = i2c_master_transmit_receive(mpu6050_handle, &reg, 1, buffer, sizeof(buffer), -1);
        if (err != ESP_OK) {
            if (++i2c_err_cnt >= MAX_I2C_ERRORS) {
                for (int i = 0; i < 4; i++) motor_thrust(0, i);
            }
            vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
            continue;
        }
        i2c_err_cnt = 0;

        parse_mpu6050_data(&mpu6050_raw, buffer);
        compl_filter(&mpu6050_raw, &angles, dt, ALPHA);

        float roll_rate  = roll_rate_dps(&mpu6050_raw);
        float pitch_rate = pitch_rate_dps(&mpu6050_raw);
        float yaw_rate   = yaw_rate_dps(&mpu6050_raw);

        uint16_t thrust = (local_pkt.thrust > 1) ? local_pkt.thrust : 0;
        motors = pid_control(local_pkt.roll, local_pkt.pitch, local_pkt.yaw,
                             angles.roll, angles.pitch,
                             roll_rate, pitch_rate, yaw_rate,
                             thrust, dt);

        motor_thrust(motors.m1, MOTOR_FRONT_LEFT);
        motor_thrust(motors.m2, MOTOR_FRONT_RIGHT);
        motor_thrust(motors.m3, MOTOR_BACK_LEFT);
        motor_thrust(motors.m4, MOTOR_BACK_RIGHT);

        if (++tele_cnt >= TELEMETRY_DIV) {
            tele_cnt = 0;
            snprintf(msg.line, sizeof(msg.line),
                "T,%lld,%.2f,%.1f,%.2f,%.1f,%.0f,%.0f,%.0f,%.0f,%.1f,%.2f,%.1f,%.0f,%.0f,%.0f,%.0f,%u,%u,%u,%u,%u\n",
                now_us / 1000, dt * 1000.0f,
                local_pkt.roll,  angles.roll,  roll_rate,
                roll_pid.p_out,  roll_pid.i_out,  roll_pid.d_out,  roll_pid.out,
                local_pkt.pitch, angles.pitch, pitch_rate,
                pitch_pid.p_out, pitch_pid.i_out, pitch_pid.d_out, pitch_pid.out,
                thrust, motors.m1, motors.m2, motors.m3, motors.m4);
            if (tele_queue) xQueueSend(tele_queue, &msg, 0);
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(5));
    }
}

static void vTelemetry(void *pvParametars)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    tele_msg_t msg;

    while (1) {
        if (xQueueReceive(tele_queue, &msg, portMAX_DELAY) != pdTRUE) continue;

        struct sockaddr_in dest;
        bool valid;
        taskENTER_CRITICAL(&tele_spinlock);
        dest  = tele_dest;
        valid = tele_dest_valid;
        taskEXIT_CRITICAL(&tele_spinlock);

        if (valid && sock >= 0) {
            sendto(sock, msg.line, strlen(msg.line), 0, (struct sockaddr *)&dest, sizeof(dest));
        }
#if TELEMETRY_SERIAL
        printf("%s", msg.line);
#endif
    }
}

static void apply_tuning(const crtp_packet_t *t)
{
    PID_t *targets[3] = { &roll_pid, &pitch_pid, &yaw_pid };
    int axis = t->thrust;

    if (axis == TUNE_AXIS_HELLO) {
        printf("TUNE hello\n");
        return;
    }
    if (axis == 3) {
        roll_pid.kp = pitch_pid.kp = t->roll;
        roll_pid.ki = pitch_pid.ki = t->pitch;
        roll_pid.kd = pitch_pid.kd = t->yaw;
        pid_reset(&roll_pid);
        pid_reset(&pitch_pid);
    } else if (axis >= 0 && axis <= 2) {
        targets[axis]->kp = t->roll;
        targets[axis]->ki = t->pitch;
        targets[axis]->kd = t->yaw;
        pid_reset(targets[axis]);
    }
    printf("TUNE axis=%d Kp=%.1f Ki=%.1f Kd=%.1f\n", axis, t->roll, t->pitch, t->yaw);
}

static void vUdp_server(void *pvParametars)
{
    struct sockaddr_in server_addr, client_addr;
    socklen_t socklen = sizeof(client_addr);
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);

    UDP_server_init(&server_addr, &client_addr, &socklen, &sock);

    while (1) {
        socklen = sizeof(client_addr);
        len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0,
                       (struct sockaddr *)&client_addr, &socklen);
        if (len < (int)sizeof(crtp_packet_t)) {
            continue;
        }

        crtp_packet_t tmp;
        memcpy(&tmp, rx_buffer, sizeof(crtp_packet_t));

        if (tmp.port == CRTP_PORT_TUNING) {
            struct sockaddr_in d = client_addr;
            d.sin_port = htons(TELEMETRY_PORT);
            taskENTER_CRITICAL(&tele_spinlock);
            tele_dest = d;
            tele_dest_valid = true;
            taskEXIT_CRITICAL(&tele_spinlock);

            apply_tuning(&tmp);
            continue;
        }

        taskENTER_CRITICAL(&pkt_spinlock);
        pkt = tmp;
        last_pkt_us = esp_timer_get_time();
        taskEXIT_CRITICAL(&pkt_spinlock);
    }
}

void app_main(void)
{
    i2c_master_init();
    configure_mpu6050();
    calibrate_mpu6050();
    timer_config();
    wifi_init();

    tele_queue = xQueueCreate(8, sizeof(tele_msg_t));

    if (xTaskCreate(vUdp_server, "vUdp_server", 4096, NULL, 2, NULL) != pdPASS)
        printf("Error: Failed to create vUdp_server task!\n");

    if (xTaskCreatePinnedToCore(vTelemetry, "vTelemetry", 4096, NULL, 1, NULL, 0) != pdPASS)
        printf("Error: Failed to create vTelemetry task!\n");

    if (xTaskCreatePinnedToCore(vMotor_control, "vMotor_control", 4096, NULL, 5, NULL, 1) != pdPASS)
        printf("Error: Failed to create vMotor_control task!\n");

    printf("End app_main.\n");
}