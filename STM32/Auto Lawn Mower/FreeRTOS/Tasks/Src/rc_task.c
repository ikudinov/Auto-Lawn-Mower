/*
 * rc_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "stdbool.h"
#include "math.h"
#include "stdio.h"

#include "cmsis_os.h"
#include "rc_task.h"

typedef enum {
    PIN_STATE_LOW = -1,
    PIN_STATE_NO_CHANGE = 0,
    PIN_STATE_HIGH = 1,
} PinState;

PinState leftMotorPinState = PIN_STATE_LOW;
PinState rightMotorPinState = PIN_STATE_LOW;
PinState trimmerMotorPinState = PIN_STATE_LOW;

uint32_t leftMotorHighIntTs = 0;
uint32_t rightMotorHighIntTs = 0;
uint32_t trimmerMotorHighIntTs = 0;

uint8_t leftMotorPwmPercent = 0;
uint8_t rightMotorPwmPercent = 0;
uint8_t trimmerMotorPwmPercent = 0;

/**
 * @brief  Calc pin state change
 * @param  argument: Not used
 * @retval None
 */
PinState GetPinInterruptValue(uint16_t pin, PinState prevPinState) {
    if (__HAL_GPIO_EXTI_GET_IT(pin) == RESET) {
        return PIN_STATE_NO_CHANGE;
    }

    __HAL_GPIO_EXTI_CLEAR_IT(pin);

    bool bPinState = HAL_GPIO_ReadPin(MOTOR_PORT, pin);

    // Low -> High
    if (prevPinState == PIN_STATE_LOW && bPinState == GPIO_PIN_SET) {
        return PIN_STATE_HIGH;
    }

    // High -> Low
    if (prevPinState == PIN_STATE_HIGH && bPinState == GPIO_PIN_RESET) {
        return PIN_STATE_LOW;
    }

    return PIN_STATE_NO_CHANGE;
}

/**
 * @brief  PWM value calculation
 * @param  argument: Not used
 * @retval None
 */
uint8_t calcPwmPercent(uint32_t highTs, uint32_t lowTs) {
    if (highTs >= lowTs)
        return 0;

    int64_t diffTicks = lowTs - highTs;
    int64_t oneMsTicks = SystemCoreClock / 1000;

    if (diffTicks < oneMsTicks)
        return 0;

    int64_t pwmPercent = (diffTicks - oneMsTicks) * 100 / oneMsTicks;

    return fmin(100, fabs(pwmPercent));
}

/**
 * @brief  Interrupt callback
 * @param  argument: Not used
 * @retval None
 */
void EXTI9_5_IRQHandler(void) {
    uint32_t ts = DWT->CYCCNT;

    /* Left motor */
    PinState leftMotorNewState = GetPinInterruptValue(MOTOR_LEFT_PIN, leftMotorPinState);
    if (leftMotorNewState == PIN_STATE_HIGH) {
        leftMotorHighIntTs = ts;
        return;
    }
    if (leftMotorNewState == PIN_STATE_LOW) {
        leftMotorPwmPercent = calcPwmPercent(leftMotorHighIntTs, ts);
        return;
    }

    /* Right motor */
    PinState rightMotorNewState = GetPinInterruptValue(MOTOR_RIGHT_PIN, rightMotorPinState);
    if (rightMotorNewState == PIN_STATE_HIGH) {
        rightMotorHighIntTs = ts;
        return;
    }
    if (rightMotorNewState == PIN_STATE_LOW) {
        rightMotorPwmPercent = calcPwmPercent(rightMotorHighIntTs, ts);
        return;
    }

    /* Trimmer motor */
    PinState trimmerMotorNewState = GetPinInterruptValue(MOTOR_TRIMMER_PIN, trimmerMotorPinState);
    if (trimmerMotorNewState == PIN_STATE_HIGH) {
        trimmerMotorHighIntTs = ts;
        return;
    }
    if (trimmerMotorNewState == PIN_STATE_LOW) {
        trimmerMotorPwmPercent = calcPwmPercent(trimmerMotorHighIntTs, ts);
        return;
    }
}

/**
 * @brief  Function implementing the rcTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartRcTask(void const *argument) {
    const TickType_t xIntervalMs = 500;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);
        printf("left = %u", leftMotorPwmPercent);
    }
}
