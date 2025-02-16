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


extern MotorPwm leftMotorControl;
extern MotorPwm rightMotorControl;
extern bool trimmerMotorEnabled;

void StartPidTask(void const * argument);


#endif /* TASKS_INC_PID_TASK_H_ */
