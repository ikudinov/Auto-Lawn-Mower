/*
 * pid_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "../Inc/pid_task.h"

#include "cmsis_os.h"


/**
* @brief Function implementing the pidTask thread.
* @param argument: Not used
* @retval None
*/
void StartPidTask(void const * argument)
{
  /* USER CODE BEGIN StartPidTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartPidTask */
}
