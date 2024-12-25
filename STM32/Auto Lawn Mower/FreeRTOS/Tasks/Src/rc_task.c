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


typedef struct {
    GPIO_PinState pinState;
    uint32_t highIntTs; // Cycle time last interrupt low -> high
    uint32_t pwmImpulseCycles; // Cycle period of impulse
} RcChannel;

RcChannel leftMotor = { GPIO_PIN_RESET, 0, 0 };
RcChannel rightMotor = { GPIO_PIN_RESET, 0, 0 };
RcChannel trimmerMotor = { GPIO_PIN_RESET, 0, 0 };


/**
 * @brief  Calc pin state change
 * @param  argument: Not used
 * @retval None
 */
void HandleInterrupt(RcChannel * rcChannel, uint16_t pinNum, uint32_t currTs) {
    if (__HAL_GPIO_EXTI_GET_IT(pinNum) == RESET)
        return;

    __HAL_GPIO_EXTI_CLEAR_IT(pinNum);

    GPIO_PinState pinState = HAL_GPIO_ReadPin(MOTOR_PORT, pinNum);

    // Low -> High
    if (rcChannel->pinState == GPIO_PIN_RESET && pinState == GPIO_PIN_SET) {
        rcChannel->highIntTs = currTs;
        rcChannel->pwmImpulseCycles = 0;
    }

    // High -> Low
    if (rcChannel->pinState == GPIO_PIN_SET && pinState == GPIO_PIN_RESET) {
        rcChannel->pwmImpulseCycles = rcChannel->highIntTs >= currTs ? 0 : currTs - rcChannel->highIntTs;
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

    if (!rcChannel.pwmImpulseCycles)
        return motorPwm; // STOP Dir

    oneMsCycles = SystemCoreClock / 1000;

    // Too short impulse
    if (rcChannel.pwmImpulseCycles < oneMsCycles / 2)
        return motorPwm; // STOP Dir

    // Impulse shorter than 1ms but not too short
    if (rcChannel.pwmImpulseCycles < oneMsCycles) {
        motorPwm.direction = BACKWARD;
        motorPwm.percent = 100;
        return motorPwm;
    }

    pwmPercent = (rcChannel.pwmImpulseCycles - oneMsCycles) * 100 / oneMsCycles;
    pwmPercent = fmin(100, fabs(pwmPercent)); // limit 0 - 100

    // PWM 48-52% is zero speed
    if (pwmPercent >= 45 && pwmPercent <= 55) {
        return motorPwm; // STOP Dir
    }

    // Backward
    if (pwmPercent < 45) {
        motorPwm.direction = BACKWARD;
        motorPwm.percent = (50.0 - pwmPercent) * 2.0;
    }

    // Backward
    if (pwmPercent > 55) {
        motorPwm.direction = FORWARD;
        motorPwm.percent = (pwmPercent - 50.0) * 2.0;
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

    return rcChannel.pwmImpulseCycles >= enableTresholdCycles;
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
    const TickType_t xIntervalMs = 100;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    RcControlMessage *message;

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        message = osMailAlloc(rcControlQueueHandle, xIntervalMs);
        message->trimmerMotor = SET;
        message->leftMotor = CalcPwmPercent(leftMotor);
        message->rightMotor = CalcPwmPercent(rightMotor);

        if (osMailPut(rcControlQueueHandle, message) != osOK) {
            osMailFree(rcControlQueueHandle, message);
        }
    }
}
