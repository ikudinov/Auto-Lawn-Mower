/*
 * overload_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "overload_task.h"
#include "cmsis_os.h"
#include "main.h"
#include "stdio.h"
#include "stm32f1xx_hal.h"
#include "setup_queues.h"

#define ADC_TRIMMER_CHANNEL ADC_CHANNEL_4
#define ADC_HANDLE &hadc1
#define TRIMMER_TRESHOLD 3000
#define QUEUE_SEND_TIMEOUT 100

int adcTrimmerValue = 0;

/**
 * @brief Read ADC channel value
 * @param channel: channel number
 * @retval ADC value
 */
uint32_t ReadAdcValue(uint32_t channel) {
    uint32_t adcValue = 0;
    ADC_ChannelConfTypeDef adcCfg = { 0 };

    adcCfg.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    adcCfg.Channel = channel;

    if (HAL_ADC_ConfigChannel(ADC_HANDLE, &adcCfg) != HAL_OK)
        return 0;
    if (HAL_ADC_Start(ADC_HANDLE) != HAL_OK)
        return 0;
    if (HAL_ADC_PollForConversion(ADC_HANDLE, 2) != HAL_OK)
        return 0;

    adcValue = HAL_ADC_GetValue(ADC_HANDLE);

    HAL_ADC_Stop(ADC_HANDLE);

    return adcValue;
}

/**
 * @brief Check trimmer motor overcurrent
 * @retval None
 */
void CheckTrimmerOverload() {
    uint32_t value = 0;
    OverloadMessage *message;

    value = ReadAdcValue(ADC_TRIMMER_CHANNEL);

    printf("ADC %lu", value);

    if (value >= TRIMMER_TRESHOLD) {
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
    const TickType_t xIntervalMs = 250;
    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xIntervalMs);

        CheckTrimmerOverload();
    }
}
