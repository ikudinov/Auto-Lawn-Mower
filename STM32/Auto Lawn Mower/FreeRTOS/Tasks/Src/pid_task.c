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
#include "encoder_task.h"
#include "overload_task.h"

#define TASK_INTERVAL 25
#define TRIMMER_TIMER_STEP 3000
#define TRIMMER_DISABLE_DELAY 40 // 40 * 25ms = 1000ms
#define TRIMMER_GPIO_REG TIM2->CCR1
#define MAX_TIMER_VALUE 65535


DriveMotor leftMotor, rightMotor;
TrimmerMotor trimmerMotor = { 0, 0, 0 };

/**
 * @brief
 * @retval
 */
void HandleDriveMotorRcMessage(DriveMotor *motor, MotorPwm *message) {
    uint16_t timerValue;
    bool needWriteHal;

    needWriteHal = motor->direction != message->direction || motor->percent != message->percent;
    if (!needWriteHal) return;

    motor->direction = message->direction;
    motor->percent = message->percent;

    timerValue = MAX_TIMER_VALUE * motor->percent / 100;
    if (motor->direction == STOP) {
        *motor->timerCcrFwd = 0;
        *motor->timerCcrBack = 0;
    }
    if (motor->direction == FORWARD) {
        *motor->timerCcrFwd = 0;
        *motor->timerCcrBack = timerValue;
    }
    if (motor->direction == BACKWARD) {
        *motor->timerCcrFwd = timerValue;
        *motor->timerCcrBack = 0;
    }
}

/**
 * @brief
 * @retval
 */
void HandleTrimmerMotorRcMessage(TrimmerMotor *motor, bool enabled) {
    bool needWriteHal;

    needWriteHal = motor->enabled != enabled || (enabled && motor->timerCounter < MAX_TIMER_VALUE);
    if (!needWriteHal) return;

    // Delay after trimmer overload
    if (motor->disableCounter > 0 && enabled) {
        motor->disableCounter--;
        return;
    }

    motor->enabled = enabled;
    motor->timerCounter = enabled ? fmin(MAX_TIMER_VALUE, motor->timerCounter + TRIMMER_TIMER_STEP) : 0;

    TRIMMER_GPIO_REG = motor->timerCounter;
}

/**
 * @brief Motor control by RC receiver
 * @param event: OS Main Event data
 * @retval None
 */
void HandleRcControlMessage(osEvent event) {
    RcControlMessage *message = (RcControlMessage*) event.value.p;

    HandleDriveMotorRcMessage(&leftMotor, &message->leftMotor);
    HandleDriveMotorRcMessage(&rightMotor, &message->rightMotor);
    HandleTrimmerMotorRcMessage(&trimmerMotor, message->trimmerMotor);

    osMailFree(rcControlQueueHandle, message);
}

/**
 * @brief Motor overload event
 * @param event: OS Main Event data
 * @retval None
 */
void HandleOverloadMessage(osEvent event) {
    OverloadMessage *message = (OverloadMessage*) event.value.p;

    // Trimmer OFF
    if (message->trimmerMotorOverload && trimmerMotor.enabled && trimmerMotor.timerCounter >= MAX_TIMER_VALUE) {
        trimmerMotor.enabled = false;
        trimmerMotor.disableCounter = TRIMMER_DISABLE_DELAY;
        trimmerMotor.timerCounter = 0;
        TRIMMER_GPIO_REG = 0;
    }

    osMailFree(overloadQueueHandle, message);
}

/**
 * @brief Motor register initialization
 * @param motor: Drive motor structure
 * @param ccrFwd: Forward PWM GPIO
 * @param ccrBack: Backward PWM GPIO
 * @retval None
 */
void InitMotor(DriveMotor *motor, volatile uint32_t *ccrFwd, volatile uint32_t *ccrBack) {
    motor->timerCcrFwd = ccrFwd;
    motor->timerCcrBack = ccrBack;
    motor->direction = STOP;
    motor->percent = 0;
}

/**
 * @brief Function implementing the pidTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartPidTask(void const *argument) {
    const TickType_t xIntervalMs = TASK_INTERVAL;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    InitMotor(&leftMotor, &TIM1->CCR3, &TIM1->CCR4);
    InitMotor(&rightMotor, &TIM1->CCR1, &TIM1->CCR2);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        // Try to get RC control message
        osEvent rcControlEvent = osMailGet(rcControlQueueHandle, 0);
        if (rcControlEvent.status == osEventMail) {
            HandleRcControlMessage(rcControlEvent);
        }

        // Try to get RC control message
        osEvent overloadEvent = osMailGet(overloadQueueHandle, 0);
        if (overloadEvent.status == osEventMail) {
            HandleOverloadMessage(overloadEvent);
        }
    }
}
