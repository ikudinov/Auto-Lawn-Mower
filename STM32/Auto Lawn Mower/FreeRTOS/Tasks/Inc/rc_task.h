/*
 * rc_task.h
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */

#ifndef TASKS_INC_RC_TASK_H_
#define TASKS_INC_RC_TASK_H_

#include "stm32f1xx_hal.h"
#include "tasks_common.h"

#define IMPULSE_ARR_LEN 6

typedef struct {
    MotorPwm leftMotor;
    MotorPwm rightMotor;
    uint8_t trimmerMotor;
} RcControlMessage;

typedef struct {
    GPIO_PinState pinState;
    uint32_t highIntTs; // Cycle time last interrupt low -> high
    uint8_t pwmImpulseIdx;
    uint32_t pwmImpulseArr[IMPULSE_ARR_LEN];
} RcChannel;

void StartRcTask(void const * argument);


#endif /* TASKS_INC_RC_TASK_H_ */
