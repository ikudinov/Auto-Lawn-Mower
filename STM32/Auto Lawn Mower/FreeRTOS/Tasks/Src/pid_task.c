/*
 * pid_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "cmsis_os.h"
#include "math.h"

#include "pid_task.h"
#include "setup_queues.h"
#include "rc_task.h"

MotorPwm leftMotorControl = { STOP, 0 };
MotorPwm rightMotorControl = { STOP, 0 };

uint16_t TRIMMER_TIMER_STEP = 1500;
bool trimmerMotorEnabled = false;
uint16_t trimmerTimerCounter = 0;

/**
 * @brief Write all PWM/GPIO values to HAL
 * @retval None
 */
void WriteMotorsHAL() {
    uint16_t timerValue;

    // Left motor
    timerValue = 65535 * leftMotorControl.percent / 100;
    if (leftMotorControl.direction == STOP) {
        TIM1->CCR3 = 0;
        TIM1->CCR4 = 0;
    }
    if (leftMotorControl.direction == FORWARD) {
        TIM1->CCR3 = 0;
        TIM1->CCR4 = timerValue;
    }
    if (leftMotorControl.direction == BACKWARD) {
        TIM1->CCR3 = timerValue;
        TIM1->CCR4 = 0;
    }

    // Right motor
    timerValue = 65535 * rightMotorControl.percent / 100;
    if (rightMotorControl.direction == STOP) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
    }
    if (rightMotorControl.direction == FORWARD) {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = timerValue;
    }
    if (rightMotorControl.direction == BACKWARD) {
        TIM1->CCR1 = timerValue;
        TIM1->CCR2 = 0;
    }

    // Trimmer
    trimmerTimerCounter = trimmerMotorEnabled ? fmin(65535, trimmerTimerCounter + TRIMMER_TIMER_STEP) : 0;
    TIM2->CCR1 = trimmerTimerCounter;
}

/**
 * @brief Compare MotorPwm structs
 * @retval Bool
 */
bool isMotorPwnEqual(MotorPwm a, MotorPwm b) {
    return a.direction == b.direction && a.percent == b.percent;
}

/**
 * @brief Motor control by RC receiver
 * @param event: OS Main Event data
 * @retval None
 */
void HandleRcControlMessage(osEvent event) {
    RcControlMessage *message = (RcControlMessage*) event.value.p;
    bool needWriteHal = false;

    if (!isMotorPwnEqual(leftMotorControl, message->leftMotor)) {
        leftMotorControl.direction = message->leftMotor.direction;
        leftMotorControl.percent = message->leftMotor.percent;
        needWriteHal = true;
    }

    if (!isMotorPwnEqual(rightMotorControl, message->rightMotor)) {
        rightMotorControl.direction = message->rightMotor.direction;
        rightMotorControl.percent = message->rightMotor.percent;
        needWriteHal = true;
    }

    if ((trimmerMotorEnabled != message->trimmerMotor) || (trimmerMotorEnabled && trimmerTimerCounter < 65535)) {
        trimmerMotorEnabled = message->trimmerMotor;
        needWriteHal = true;
    }

    if (needWriteHal) {
        WriteMotorsHAL();
    }

    osMailFree(rcControlQueueHandle, message);
}

/**
 * @brief Function implementing the pidTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartPidTask(void const *argument) {
    const TickType_t xIntervalMs = 25;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        // Try to get RC control message
        osEvent rcControlEvent = osMailGet(rcControlQueueHandle, 1);
        if (rcControlEvent.status == osEventMail) {
            HandleRcControlMessage(rcControlEvent);
        }
    }
}
