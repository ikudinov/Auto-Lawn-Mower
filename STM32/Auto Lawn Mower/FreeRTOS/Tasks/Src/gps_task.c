/*
 * gps_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "gps_task.h"

#include "cmsis_os.h"


/**
  * @brief  Function implementing the gpsTask thread.
  * @param  argument: Not used
  * @retval None
  */
void StartGpsTask(void const * argument)
{
  for(;;)
  {
	vTaskDelay(1);
  }
}
