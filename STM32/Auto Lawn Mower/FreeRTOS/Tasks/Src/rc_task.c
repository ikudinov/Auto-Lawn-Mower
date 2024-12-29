/*
 * rc_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "math.h"
#include "stdio.h"
#include "cmsis_os.h"

#include "rc_task.h"
#include "setup_queues.h"


#define IMPULSE_ARR_LEN 6

typedef struct {
    GPIO_PinState pinState;
    uint32_t highIntTs; // Cycle time last interrupt low -> high
    uint8_t pwmImpulseIdx;
    uint32_t pwmImpulseArr[IMPULSE_ARR_LEN];
} RcChannel;

RcChannel leftMotor;
RcChannel rightMotor;
RcChannel trimmerMotor;


/**
 * @brief  Fill RC channel struct with initial values
 */
void InitRcChannel(RcChannel * rcChannel) {
    rcChannel->pinState = GPIO_PIN_RESET;
    rcChannel->highIntTs = 0;
    rcChannel->pwmImpulseIdx = 0;
    for (uint8_t idx = 0; idx < IMPULSE_ARR_LEN; idx++) {
        rcChannel->pwmImpulseArr[idx] = 0;
    }
}

/**
 * @brief  Push impulse value to AVG array
 */
void AddImpulseCycles(RcChannel * rcChannel, uint32_t impulseCycles) {
    if (rcChannel->pwmImpulseIdx == IMPULSE_ARR_LEN - 1) {
        rcChannel->pwmImpulseIdx = 0;
    } else {
        rcChannel->pwmImpulseIdx++;
    }

    rcChannel->pwmImpulseArr[rcChannel->pwmImpulseIdx] = impulseCycles;
}

/**
 * @brief  Calc simple average impulse cycles
 */
uint32_t CalcAvgImpulseCycles(RcChannel rcChannel) {
    uint32_t avg = 0;

    for (uint8_t idx = 0; idx < IMPULSE_ARR_LEN; idx++) {
        avg += rcChannel.pwmImpulseArr[idx] / IMPULSE_ARR_LEN;
    }

    return avg;
}

/**
 * @brief  Calc pin state change
 * @param  argument: Not used
 * @retval None
 */
void HandleInterrupt(RcChannel * rcChannel, uint16_t pinNum, uint32_t currTs) {
    GPIO_PinState pinState;
    uint32_t impulseCycles;

    if (__HAL_GPIO_EXTI_GET_IT(pinNum) == RESET)
        return;

    __HAL_GPIO_EXTI_CLEAR_IT(pinNum);

    pinState = HAL_GPIO_ReadPin(MOTOR_PORT, pinNum);

    // Low -> High
    if (rcChannel->pinState == GPIO_PIN_RESET && pinState == GPIO_PIN_SET) {
        rcChannel->highIntTs = currTs;
    }

    // High -> Low
    if (rcChannel->pinState == GPIO_PIN_SET && pinState == GPIO_PIN_RESET) {
        impulseCycles = rcChannel->highIntTs >= currTs ? 0 : currTs - rcChannel->highIntTs;
        AddImpulseCycles(rcChannel, impulseCycles);
    }

    rcChannel->pinState = pinState;
}

/**
 * @brief  PWM value calculation
 * @param  argument: RC channel info
 * @retval None
 */
MotorPwm CalcPwmPercent(RcChannel rcChannel) {
    int64_t oneMsCycles;
    double pwmPercent;
    MotorPwm motorPwm = { STOP, 0 };
    uint32_t pwmImpulseCycles;

    pwmImpulseCycles = CalcAvgImpulseCycles(rcChannel);

    if (!pwmImpulseCycles)
        return motorPwm; // STOP Dir

    oneMsCycles = SystemCoreClock / 1000;

    // Too short impulse
    if (pwmImpulseCycles < oneMsCycles / 2)
        return motorPwm; // STOP Dir

    // Impulse shorter than 1ms but not too short
    if (pwmImpulseCycles < oneMsCycles) {
        motorPwm.direction = BACKWARD;
        motorPwm.percent = 100;
        return motorPwm;
    }

    pwmPercent = (pwmImpulseCycles - oneMsCycles) * 200 / oneMsCycles;
    pwmPercent = fmin(200, fabs(pwmPercent)); // limit 0 - 100

    // PWM 48-52% is zero speed
    if (pwmPercent >= 95 && pwmPercent <= 105) {
        return motorPwm; // STOP Dir
    }

    // Backward
    if (pwmPercent < 95) {
        motorPwm.direction = BACKWARD;
        motorPwm.percent = 100.0 - pwmPercent;
    }

    // Backward
    if (pwmPercent > 105) {
        motorPwm.direction = FORWARD;
        motorPwm.percent = pwmPercent - 100;
    }

    return motorPwm;
}

/**
 * @brief  Calculation on/off channel
 * @param  argument: RC channel info
 * @retval None
 */
uint8_t CalcOnOff(RcChannel rcChannel) {
    int64_t enableTresholdCycles = SystemCoreClock * 17 / 10000;
    uint32_t pwmImpulseCycles = CalcAvgImpulseCycles(rcChannel);

    return pwmImpulseCycles >= enableTresholdCycles;
}

/**
 * @brief  Interrupt callback
 * @param  argument: Not used
 * @retval None
 */
void EXTI9_5_IRQHandler(void) {
    const uint32_t ts = DWT->CYCCNT;

    HandleInterrupt(&leftMotor, MOTOR_LEFT_PIN, ts);
    HandleInterrupt(&rightMotor, MOTOR_RIGHT_PIN, ts);
    HandleInterrupt(&trimmerMotor, MOTOR_TRIMMER_PIN, ts);
}

/**
 * @brief  Function implementing the rcTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartRcTask(void const *argument) {
    const TickType_t xIntervalMs = 50;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    RcControlMessage *message;

    InitRcChannel(&leftMotor);
    InitRcChannel(&rightMotor);
    InitRcChannel(&trimmerMotor);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        message = osMailAlloc(rcControlQueueHandle, xIntervalMs);
        message->trimmerMotor = CalcOnOff(trimmerMotor);
        message->leftMotor = CalcPwmPercent(leftMotor);
        message->rightMotor = CalcPwmPercent(rightMotor);

        if (osMailPut(rcControlQueueHandle, message) != osOK) {
            osMailFree(rcControlQueueHandle, message);
        }
    }
}
