/*
 * overload_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "overload_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "setup_queues.h"

#define ADC_HANDLER hadc1
#define TRIMMER_TRESHOLD 800
#define SMA_LENGTH 2

uint32_t smaValues[SMA_LENGTH];
uint8_t smaIndex = 0;

/**
 * @brief Read ADC channel value
 * @retval ADC value
 */
uint32_t ReadAdcValue() {
    uint32_t adcValue = 0;

    if (HAL_ADC_Start(&ADC_HANDLER) != HAL_OK)
        return 0;
    if (HAL_ADC_PollForConversion(&ADC_HANDLER, 10) != HAL_OK)
        return 0;

    adcValue = HAL_ADC_GetValue(&ADC_HANDLER);

    HAL_ADC_Stop(&ADC_HANDLER);

    return adcValue;
}

/**
 * @brief Calc avg value for ADC
 * @retval AVG value
 */
uint32_t CalcSmaValue(uint32_t value) {
    uint32_t avg = 0;

    smaIndex = smaIndex == SMA_LENGTH - 1 ? 0 : smaIndex + 1;
    smaValues[smaIndex] = value;

    for (uint8_t idx = 0; idx < SMA_LENGTH; idx++) {
        avg += smaValues[idx] / SMA_LENGTH;
    }

    return avg;
}


/**
 * @brief Check trimmer motor overcurrent
 * @retval None
 */
void CheckTrimmerOverload() {
    uint32_t value, avgValue = 0;
    OverloadMessage *message;

    value = ReadAdcValue();
    avgValue = CalcSmaValue(value);

    if (avgValue <= TRIMMER_TRESHOLD) {
        message = osMailAlloc(overloadQueueHandle, QUEUE_SEND_TIMEOUT);
        message->leftMotorOverload = 0;
        message->rightMotorOverload = 0;
        message->trimmerMotorOverload = 1;

        if (osMailPut(overloadQueueHandle, message) != osOK) {
            osMailFree(overloadQueueHandle, message);
        }
    }
}

/**
 * @brief  Function implementing the overloadTask thread.
 * @param  argument: Not used
 * @retval None
 */
void StartOverloadTask(void const *argument) {
    const TickType_t xIntervalMs = 200;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        CheckTrimmerOverload();
    }
}
