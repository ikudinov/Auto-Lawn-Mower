/*
 * setup_queues.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */

#include "setup_queues.h"
#include "cmsis_os.h"

osMessageQId encoderQueueHandle;
osMessageQId motorCurrQueueHandle;


void vSetupAllQueues()
{
	osMessageQDef(encoderQueue, 4, uint16_t);
	encoderQueueHandle = osMessageCreate(osMessageQ(encoderQueue), NULL);

	osMessageQDef(motorCurrQueue, 4, uint16_t);
	motorCurrQueueHandle = osMessageCreate(osMessageQ(motorCurrQueue), NULL);
}
