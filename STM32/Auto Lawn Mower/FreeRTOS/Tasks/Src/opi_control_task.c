/*
 * opi_control_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "opi_control_task.h"

#include "cmsis_os.h"


/**
* @brief Function implementing the opiControlTask thread.
* @param argument: Not used
* @retval None
*/
void StartOpiControlTask(void const * argument)
{
  for(;;)
  {
	vTaskDelay(1);
  }
}
