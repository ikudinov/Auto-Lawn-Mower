/*
 * encoder_task.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "encoder_task.h"

#include "cmsis_os.h"


/**
* @brief Function implementing the encoderTask thread.
* @param argument: Not used
* @retval None
*/
void StartEncoderTask(void const * argument)
{
  for(;;)
  {
    vTaskDelay(1);
  }
}
