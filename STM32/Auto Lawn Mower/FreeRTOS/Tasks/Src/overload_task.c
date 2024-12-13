/*
 * overload_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "overload_task.h"

#include "cmsis_os.h"


/**
  * @brief  Function implementing the overloadTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartOverloadTask */
void StartOverloadTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}
