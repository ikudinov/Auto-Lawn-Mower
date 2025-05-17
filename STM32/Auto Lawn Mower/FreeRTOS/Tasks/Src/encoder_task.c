/*
 * encoder_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "cmsis_os.h"
#include "stm32f1xx_hal.h"
#include "encoder_task.h"

#define ENCODER_PORT GPIOB
#define LEFT_ENCODER_PIN GPIO_PIN_0
#define RIGHT_ENCODER_PIN GPIO_PIN_1

/**
 *
 */
void reset(uint16_t pinNum) {
    if (__HAL_GPIO_EXTI_GET_IT(pinNum) == RESET)
        return;

    __HAL_GPIO_EXTI_CLEAR_IT(pinNum);
}

/**
 *
 */
void EXTI0_IRQHandler(void) {
    reset(LEFT_ENCODER_PIN);
}

/**
 *
 */
void EXTI1_IRQHandler(void) {
    reset(RIGHT_ENCODER_PIN);
}

/**
 * @brief Function implementing the encoderTask thread.
 * @param argument: Not used
 * @retval None
 */
void StartEncoderTask(void const *argument) {
    const TickType_t xIntervalMs = 200;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);
    }
}
