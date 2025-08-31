/*
 * setup_queues.h
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */

#ifndef QUEUES_INC_SETUP_QUEUES_H_
#define QUEUES_INC_SETUP_QUEUES_H_

#include "cmsis_os.h"

#define QUEUE_SEND_TIMEOUT 200

extern osMailQId rcControlQueueHandle;
extern osMailQId overloadQueueHandle;

void vSetupAllQueues();


#endif /* QUEUES_INC_SETUP_QUEUES_H_ */
