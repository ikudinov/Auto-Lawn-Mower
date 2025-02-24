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


#define MOTOR_PORT GPIOB
#define MOTOR_LEFT_PIN GPIO_PIN_7
#define MOTOR_RIGHT_PIN GPIO_PIN_8
#define MOTOR_TRIMMER_PIN GPIO_PIN_9

typedef struct {
    MotorPwm leftMotor;
    MotorPwm rightMotor;
    uint8_t trimmerMotor;
} RcControlMessage;


void StartRcTask(void const * argument);


#endif /* TASKS_INC_RC_TASK_H_ */
