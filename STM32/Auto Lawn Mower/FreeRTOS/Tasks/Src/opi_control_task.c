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
extern MotorPwm leftMotorControl;
extern MotorPwm rightMotorControl;
extern bool trimmerMotorEnabled;

char message[64];

int8_t getMotorValue(MotorPwm motor) {
    if (motor.direction == FORWARD) return motor.percent;
    if (motor.direction == BACKWARD) return -motor.percent;
    return 0;
}

void prepareMotorsJson() {
    const int8_t l = getMotorValue(leftMotorControl);
    const int8_t r = getMotorValue(rightMotorControl);
    const uint8_t t = trimmerMotorEnabled;

    sprintf(message, "{\"type\":\"motors\",\"data\":{\"l\":%i,\"r\":%i,\"t\":%u}}\n", l, r, t);
}

/**
 * @brief Function implementing the opiControlTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartOpiControlTask(void const *argument) {
    const TickType_t xIntervalMs = 100;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        prepareMotorsJson();
        HAL_UART_Transmit_DMA(&huart3, (uint8_t *)message, strlen(message));
    }
}
