/*
 * status_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "../Inc/status_task.h"

#include "cmsis_os.h"


/**
* @brief Function implementing the sendStatusTask thread.
* @param argument: Not used
* @retval None
*/
void StartSendStatusTask(void const * argument)
{
  for(;;)
  {
	vTaskDelay(1);
  }
}
