/*
 * overload_task.h
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */

#ifndef TASKS_INC_OVERLOAD_TASK_H_
#define TASKS_INC_OVERLOAD_TASK_H_

#include "stdint.h"

typedef struct {
    uint8_t leftMotorOverload;
    uint8_t rightMotorOverload;
    uint8_t trimmerMotorOverload;
} OverloadMessage;

void StartOverloadTask(void const * argument);


#endif /* TASKS_INC_OVERLOAD_TASK_H_ */
