/*
 * pid_task.h
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */

#ifndef TASKS_INC_PID_TASK_H_
#define TASKS_INC_PID_TASK_H_

#include "stdbool.h"
#include "tasks_common.h"


extern DriveMotor leftMotor;
extern DriveMotor rightMotor;
extern TrimmerMotor trimmerMotor;

void StartPidTask(void const * argument);


#endif /* TASKS_INC_PID_TASK_H_ */
