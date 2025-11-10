#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
#include <stdio.h>
#include "servo.h"

#define SERVO_PIN 16
#define TRIG_PIN 10
#define ECHO_PIN 11
#define PIN_MOTOR_LEFT 2
#define PIN_MOTOR_RIGHT 3

#define QUEUE_SIZE 180
#define OBSTACLE_THRESHOLD_CM 20.0
#define ECHO_TIMEOUT_US 20000
#define DISTANCE_FAIL_SAFE_CM 400.0
#define TASK_DELAY_MS 25
#define SCAN_SERVO_DELAY_MS 25
#define TURN_DELAY_MS 500
#define DEFAULT_SERVO_ANGLE 90
#define SCAN_START_ANGLE 0
#define SCAN_END_ANGLE 180

QueueHandle_t distanceQueue;
SemaphoreHandle_t xDistanceMutex;
servo_t myServo;

TaskHandle_t moveTaskHandle = NULL;
TaskHandle_t scanTaskHandle = NULL;
TaskHandle_t decisionTaskHandle = NULL;
TaskHandle_t measureTaskHandle = NULL;

volatile bool obstacle_detected = false;
volatile float last_distance = 100.0;

void setup_sensor(void) {
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);

    gpio_init(PIN_MOTOR_LEFT);
    gpio_set_dir(PIN_MOTOR_LEFT, GPIO_OUT);
    gpio_init(PIN_MOTOR_RIGHT);
    gpio_set_dir(PIN_MOTOR_RIGHT, GPIO_OUT);
}

void check_for_obstacle(void) {
    xSemaphoreTake(xDistanceMutex, portMAX_DELAY);
    if (last_distance < OBSTACLE_THRESHOLD_CM) {
        obstacle_detected = true;
    } else {
        obstacle_detected = false;
    }
    xSemaphoreGive(xDistanceMutex);
}

void vTaskMeasureDistance(void *pvParameters) {
    uint32_t start_time, end_time, pulse_duration;
    float measured_distance;

    while (1) {
        printf("Verificando se tem obstaculo\n");
        gpio_put(TRIG_PIN, 0);
        sleep_us(2);
        gpio_put(TRIG_PIN, 1);
        sleep_us(10);
        gpio_put(TRIG_PIN, 0);

        uint32_t timeout = ECHO_TIMEOUT_US;
        while (gpio_get(ECHO_PIN) == 0 && timeout > 0) {
            timeout--;
            sleep_us(1);
        }

        if (timeout == 0) {
            printf("Erro: Timeout ao esperar o início do pulso de ECHO\n");
            measured_distance = DISTANCE_FAIL_SAFE_CM;
        } else {
            start_time = time_us_32();
            timeout = ECHO_TIMEOUT_US;
            while (gpio_get(ECHO_PIN) == 1 && timeout > 0) {
                timeout--;
                sleep_us(1);
            }

            if (timeout == 0) {
                printf("Erro: Timeout ao esperar o fim do pulso de ECHO\n");
                measured_distance = DISTANCE_FAIL_SAFE_CM;
            } else {
                end_time = time_us_32();
                pulse_duration = end_time - start_time;

                if (pulse_duration > 0) {
                    measured_distance = (pulse_duration * 0.034) / 2;
                    printf("Distância medida: %.2f cm\n", measured_distance);
                } else {
                    printf("Erro: Duração do pulso inválida\n");
                    measured_distance = DISTANCE_FAIL_SAFE_CM;
                }
            }
        }

        xSemaphoreTake(xDistanceMutex, portMAX_DELAY);
        last_distance = measured_distance;
        xSemaphoreGive(xDistanceMutex);
        
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}

void vTaskMove(void *pvParameters) {
    bool is_obstacle;

    while (1) {
        servoWrite(&myServo, DEFAULT_SERVO_ANGLE);
        check_for_obstacle();

        gpio_put(PIN_MOTOR_LEFT, 1);
        gpio_put(PIN_MOTOR_RIGHT, 1);

        xSemaphoreTake(xDistanceMutex, portMAX_DELAY);
        is_obstacle = obstacle_detected;
        xSemaphoreGive(xDistanceMutex);

        if (is_obstacle) {
            gpio_put(PIN_MOTOR_LEFT, 0);
            gpio_put(PIN_MOTOR_RIGHT, 0);
            printf("Obstáculo detectado, parando o carrinho...\n");

            xTaskNotifyGive(scanTaskHandle);
            vTaskSuspend(NULL);
        }
        vTaskDelay(pdMS_TO_TICKS(TASK_DELAY_MS));
    }
}

void vTaskScan(void *pvParameters) {
    uint8_t angle = SCAN_START_ANGLE;
    float distance;
    bool is_obstacle;

    while (1) {
        xSemaphoreTake(xDistanceMutex, portMAX_DELAY);
        is_obstacle = obstacle_detected;
        xSemaphoreGive(xDistanceMutex);

        if(!is_obstacle){
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        servoWrite(&myServo, angle);
        vTaskDelay(pdMS_TO_TICKS(SCAN_SERVO_DELAY_MS));
        
        xSemaphoreTake(xDistanceMutex, portMAX_DELAY);
        distance = last_distance;
        xSemaphoreGive(xDistanceMutex);
        
        angle++;
        if (xQueueSend(distanceQueue, &distance, pdMS_TO_TICKS(10)) != pdTRUE) {
            printf("Erro ao enviar distância para a fila.\n");
        }
        if(angle == SCAN_END_ANGLE){
            angle = SCAN_START_ANGLE;
            xTaskNotifyGive(decisionTaskHandle);
        }
    }
}

void vTaskDecision(void *pvParameters) {
    float distance;
    float left_distance = 9999, right_distance = 9999;
    uint8_t scan_count = 0;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        while (xQueueReceive(distanceQueue, &distance, 0) == pdTRUE) {
            if (right_distance > distance && scan_count <= (SCAN_END_ANGLE / 2)) {
                right_distance = distance;
            } else if(left_distance > distance){
                left_distance = distance;
            }
            scan_count++;
        }
        
        if (right_distance > left_distance) {
            printf("Virar à direita.\n");
            gpio_put(PIN_MOTOR_LEFT, 1);
            gpio_put(PIN_MOTOR_RIGHT, 0);
            vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        } else {
            printf("Virar à esquerda.\n");
            gpio_put(PIN_MOTOR_LEFT, 0);
            gpio_put(PIN_MOTOR_RIGHT, 1);
            vTaskDelay(pdMS_TO_TICKS(TURN_DELAY_MS));
        }

        xSemaphoreTake(xDistanceMutex, portMAX_DELAY);
        obstacle_detected = false;
        xSemaphoreGive(xDistanceMutex);
        
        vTaskResume(moveTaskHandle);

        left_distance = 9999;
        right_distance = 9999;
        scan_count = 0;
    }
}

int main() {
    stdio_init_all();
    setup_sensor();

    servoAttach(&myServo, SERVO_PIN);
    servoWrite(&myServo, DEFAULT_SERVO_ANGLE);

    distanceQueue = xQueueCreate(QUEUE_SIZE, sizeof(float));
    if (distanceQueue == NULL) {
        printf("Erro ao criar a fila.\n");
        return -1;
    }
    
    xDistanceMutex = xSemaphoreCreateMutex();
    if (xDistanceMutex == NULL) {
        printf("Erro ao criar o mutex.\n");
        return -1;
    }

    xTaskCreate(vTaskMove, "Move Task", 256, NULL, 2, &moveTaskHandle);
    xTaskCreate(vTaskScan, "Scan Task", 256, NULL, 1, &scanTaskHandle);
    xTaskCreate(vTaskDecision, "Decision Task", 256, NULL, 1, &decisionTaskHandle);
    xTaskCreate(vTaskMeasureDistance, "Measure Task", 256, NULL, 3, &measureTaskHandle);

    vTaskStartScheduler();

    while (1) {
    }

    return 0;
}