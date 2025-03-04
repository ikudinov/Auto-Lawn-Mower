/*
 * tasks_common.h
 *
 *  Created on: Dec 25, 2024
 *      Author: ivankudinov
 */

#ifndef TASKS_INC_TASKS_COMMON_H_
#define TASKS_INC_TASKS_COMMON_H_

#include "stdbool.h"

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

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    double errorTotal;
    double errorPrev;
} PID;

typedef struct {
    Direction direction;
    uint8_t percent;
    volatile uint32_t * timerCcrFwd;
    volatile uint32_t * timerCcrBack;
    PID pid;
} DriveMotor;

typedef struct {
    bool enabled;
    uint16_t timerCounter;
} TrimmerMotor;

#endif /* TASKS_INC_TASKS_COMMON_H_ */
