/*
 * opi_control_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "cmsis_os.h"
#include "stdio.h"
#include "string.h"
#include "main.h"
#include "opi_control_task.h"
#include "pid_task.h"


extern UART_HandleTypeDef huart3;
extern DriveMotor leftMotor;
extern DriveMotor rightMotor;
extern TrimmerMotor trimmerMotor;

char message[64];

int8_t getMotorValue(DriveMotor motor) {
    if (motor.direction == FORWARD) return motor.percent;
    if (motor.direction == BACKWARD) return -motor.percent;
    return 0;
}

void prepareMotorsJson() {
    const int8_t l = getMotorValue(leftMotor);
    const int8_t r = getMotorValue(rightMotor);
    const uint8_t t = trimmerMotor.enabled;

    sprintf(message, "{\"type\":\"motors\",\"data\":{\"l\":%i,\"r\":%i,\"t\":%u}}\n", l, r, t);
}

/**
 * @brief Function implementing the opiControlTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartOpiControlTask(void const *argument) {
    const TickType_t xIntervalMs = 200;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        prepareMotorsJson();
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)message, strlen(message));
    }
}
