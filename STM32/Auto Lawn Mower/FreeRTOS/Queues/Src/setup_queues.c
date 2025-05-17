/*
 * setup_queues.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */

#include "cmsis_os.h"
#include "setup_queues.h"
#include "rc_task.h"
#include "overload_task.h"

osMailQId rcControlQueueHandle;
osMailQId overloadQueueHandle;

void vSetupAllQueues()
{
	osMailQDef(rcControlQueue, 2, RcControlMessage);
	rcControlQueueHandle = osMailCreate(osMailQ(rcControlQueue), NULL);

    osMailQDef(overloadQueue, 2, OverloadMessage);
    overloadQueueHandle = osMailCreate(osMailQ(overloadQueue), NULL);
}
