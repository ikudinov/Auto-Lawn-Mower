/*
 * tasks_common.h
 *
 *  Created on: Dec 25, 2024
 *      Author: ivankudinov
 */

#ifndef TASKS_INC_TASKS_COMMON_H_
#define TASKS_INC_TASKS_COMMON_H_

typedef enum
{
    STOP = 0,
    BACKWARD = 1,
    FORWARD = 2,
} Direction;

typedef struct {
    Direction direction;
    uint8_t percent;
} MotorPwm;

#endif /* TASKS_INC_TASKS_COMMON_H_ */
