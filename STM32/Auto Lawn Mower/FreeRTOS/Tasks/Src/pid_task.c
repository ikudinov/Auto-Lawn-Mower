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

#define TRIMMER_TIMER_STEP 1500
#define MAX_TIMER_VALUE 65535
#define PID_K_P 1.11
#define PID_K_I 2.12
#define PID_K_D 10.2
#define PWM_TO_PULSE_KOEFF 123


DriveMotor leftMotor, rightMotor;
TrimmerMotor trimmerMotor = { 0, 0 };

uint8_t CalcPidValue(DriveMotor * motor) {
//    pwm * PWM_TO_PULSE_KOEFF;
//
//    double error, pTerm, iTerm, dTerm, pidValue;
//    uint16_t setPulseFreq, pwmValue;
//
//    setPulseFreq = this->pwmToPulseFreq(setPwm);
//    error = setPulseFreq - actualPulseFreq;
//    this->errorTotal += error;
//
//    pTerm = (double)(this->Kp * error);
//    iTerm = (double)(this->Ki * this->errorTotal);
//    dTerm = (double)(this->Kd * (error - this->errorPrev));
//    pidValue = pTerm + iTerm + dTerm;
//    this->errorPrev = error;
//
//    // Limit output between  MIN_PWM & MAX_PWM
//    pwmValue = fmax(MIN_PWM, fmin(MAX_PWM, pidValue));


    return 0;
}

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

    motor->enabled = enabled;
    motor->timerCounter = enabled ? fmin(MAX_TIMER_VALUE, motor->timerCounter + TRIMMER_TIMER_STEP) : 0;

    TIM2->CCR1 = motor->timerCounter;
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
 *
 */
void InitMotor(DriveMotor *motor, volatile uint32_t *ccrFwd, volatile uint32_t *ccrBack) {
    motor->timerCcrFwd = ccrFwd;
    motor->timerCcrBack = ccrBack;
    motor->direction = STOP;
    motor->percent = 0;
    motor->pid.Kp = PID_K_P;
    motor->pid.Ki = PID_K_I;
    motor->pid.Kd = PID_K_D;
    motor->pid.errorPrev = 0;
    motor->pid.errorTotal = 0;
}

/**
 * @brief Function implementing the pidTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartPidTask(void const *argument) {
    const TickType_t xIntervalMs = 25;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    InitMotor(&leftMotor, &TIM1->CCR3, &TIM1->CCR4);
    InitMotor(&rightMotor, &TIM1->CCR1, &TIM1->CCR2);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        // Try to get RC control message
        osEvent rcControlEvent = osMailGet(rcControlQueueHandle, 1);
        if (rcControlEvent.status == osEventMail) {
            HandleRcControlMessage(rcControlEvent);
        }
    }
}
