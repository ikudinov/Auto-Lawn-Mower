/*
 * setup_tasks.c
 *
 *  Created on: Dec 13, 2024
 *      Author: ivankudinov
 */
#include "setup_tasks.h"

#include "cmsis_os.h"

#include "encoder_task.h"
#include "gps_task.h"
#include "opi_control_task.h"
#include "overload_task.h"
#include "pid_task.h"
#include "status_task.h"
#include "rc_task.h"


osThreadId overloadTaskHandle;
osThreadId encoderTaskHandle;
osThreadId pidTaskHandle;
osThreadId gpsTaskHandle;
osThreadId sendStatusTaskHandle;
osThreadId opiControlTaskHandle;
osThreadId rcTaskHandle;


void vSetupAllTasks()
{
	osThreadDef(overloadTask, StartOverloadTask, osPriorityNormal, 0, 128);
	overloadTaskHandle = osThreadCreate(osThread(overloadTask), NULL);

	osThreadDef(encoderTask, StartEncoderTask, osPriorityNormal, 0, 128);
	encoderTaskHandle = osThreadCreate(osThread(encoderTask), NULL);

	osThreadDef(pidTask, StartPidTask, osPriorityNormal, 0, 128);
	pidTaskHandle = osThreadCreate(osThread(pidTask), NULL);

	osThreadDef(gpsTask, StartGpsTask, osPriorityNormal, 0, 128);
	gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

	osThreadDef(sendStatusTask, StartSendStatusTask, osPriorityNormal, 0, 128);
	sendStatusTaskHandle = osThreadCreate(osThread(sendStatusTask), NULL);

	osThreadDef(opiControlTask, StartOpiControlTask, osPriorityIdle, 0, 128);
	opiControlTaskHandle = osThreadCreate(osThread(opiControlTask), NULL);

	osThreadDef(rcTask, StartRcTask, osPriorityIdle, 0, 128);
	rcTaskHandle = osThreadCreate(osThread(rcTask), NULL);
}
